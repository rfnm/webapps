#include "receiver.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace lorad {

// sum of |out[i]|^2 and the argmax bin over an FFT magnitude spectrum. NEON on AArch64
// (norm + sum + argmax vectorized; the tie-break keeps the lowest index so the result
// is bit-identical to the scalar reference). The scalar fallback is the reference that
// the x86 host selftest exercises.
static inline int fft_argmax(const cf *out, int n, float &best_p_out, float &sum_out) {
	int best = 0;
	float best_p = 0, sum = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
	const float *of = reinterpret_cast<const float *>(out);
	float32x4_t vsum = vdupq_n_f32(0), vmax = vdupq_n_f32(0);
	uint32_t seed[4] = { 0, 1, 2, 3 };
	uint32x4_t vcur = vld1q_u32(seed), vidx = vdupq_n_u32(0), four = vdupq_n_u32(4);
	int i = 0;
	for(; i + 4 <= n; i += 4) {
		float32x4x2_t ov = vld2q_f32(of + 2 * i);
		float32x4_t p = vaddq_f32(vmulq_f32(ov.val[0], ov.val[0]), vmulq_f32(ov.val[1], ov.val[1]));
		vsum = vaddq_f32(vsum, p);
		uint32x4_t gt = vcgtq_f32(p, vmax);
		vmax = vbslq_f32(gt, p, vmax);
		vidx = vbslq_u32(gt, vcur, vidx);
		vcur = vaddq_u32(vcur, four);
	}
	float32x2_t s2 = vadd_f32(vget_low_f32(vsum), vget_high_f32(vsum));
	sum = vget_lane_f32(vpadd_f32(s2, s2), 0);
	float mx[4];
	uint32_t ix[4];
	vst1q_f32(mx, vmax);
	vst1q_u32(ix, vidx);
	for(int k = 0; k < 4; k++) {
		if(mx[k] > best_p || (mx[k] == best_p && (int)ix[k] < best)) {
			best_p = mx[k];
			best = (int)ix[k];
		}
	}
	for(; i < n; i++) {
		float p = of[2 * i] * of[2 * i] + of[2 * i + 1] * of[2 * i + 1];
		sum += p;
		if(p > best_p) {
			best_p = p;
			best = i;
		}
	}
#else
	for(int i = 0; i < n; i++) {
		float p = std::norm(out[i]);
		sum += p;
		if(p > best_p) {
			best_p = p;
			best = i;
		}
	}
#endif
	best_p_out = best_p;
	sum_out = sum;
	return best;
}

// Detection tuning. The hop grid is N/4 samples; a preamble candidate needs DETECT_HOPS
// windows agreeing on the dechirped bin. Real preambles read a peak ratio of only 2..6, so the
// agreement carries the decision, the ratio only drops flat windows, and a few outliers are
// forgiven. The sync word and the header checksum reject what gets through.
static const int DETECT_HOPS = 8;        // 2 full symbols
static const int DETECT_MISS = 3;        // outlier windows forgiven inside a run
static const float DETECT_SNR = 1.5f;    // peak power over mean floor, linear
static const float SFD_SNR = 2.5f;       // same, for the downchirps
static const int SYNC_TOL = 2;           // bins of tolerance on the sync word symbols
static const int MAX_SFD_HUNT = 140;     // hops after a candidate: a 32 symbol MeshCore preamble

struct channel_receiver::sf_state {
	int sf;
	int n;
	fft_plan fft;
	std::vector<cf> upconj;   // conj(base upchirp): dechirps upchirps
	std::vector<cf> up;       // base upchirp: dechirps downchirps
	uint64_t next_hop = 0;    // next absolute sample index to analyze

	// detector history
	int run_len = 0;
	int run_bin = -1;
	int run_miss = 0;
	uint64_t cand_pos = 0;    // window start of the first agreeing hop
	int cand_bin = 0;
	bool pending = false;     // candidate frozen waiting for more samples

	sf_state(int sf_) : sf(sf_), n(1 << sf_), fft(1u << sf_) {
		upconj.resize(n);
		up.resize(n);
		for(int i = 0; i < n; i++) {
			double ph = 2.0 * M_PI * ((double)i * i / (2.0 * n) - (double)i / 2.0);
			up[i] = cf((float)cos(ph), (float)sin(ph));
			upconj[i] = std::conj(up[i]);
		}
	}
};

channel_receiver::channel_receiver(double freq_hz, uint8_t sync, unsigned sf_mask, double bw_hz,
                                   std::function<void(const rx_packet &)> on_packet, int os)
	: freq_hz_(freq_hz), sync_(sync), bw_hz_(bw_hz), os_(os), on_packet_(std::move(on_packet)) {
	for(int sf = 7; sf <= 12; sf++) {
		if(sf_mask & (1u << sf)) {
			sfs_.emplace_back(new sf_state(sf));
		}
	}
}

channel_receiver::~channel_receiver() = default;

void channel_receiver::trim() {
	// keep enough history for the largest SF to re-read preamble + sync + SFD + slack
	uint64_t keep_from = total_in_;
	for(auto &st : sfs_) {
		uint64_t need = (uint64_t)st->n * 20;
		uint64_t from = total_in_ > need ? total_in_ - need : 0;
		uint64_t anchor = st->run_len ? st->cand_pos : st->next_hop;
		if(anchor < from) {
			from = anchor;
		}
		if(from < keep_from) {
			keep_from = from;
		}
	}
	if(keep_from > ring_base_ + 65536) {
		size_t drop = (size_t)(keep_from - ring_base_);
		ring_.erase(ring_.begin(), ring_.begin() + drop);
		if(os_ > 1) {
			ring_hi_.erase(ring_hi_.begin(), ring_hi_.begin() + (long)(drop * (size_t)os_));
		}
		ring_base_ = keep_from;
	}
}

void channel_receiver::feed(const cf *samples, size_t count) {
	if(os_ > 1) {
		ring_hi_.insert(ring_hi_.end(), samples, samples + count);
		for(size_t i = (size_t)((os_ - hi_total_ % os_) % os_); i < count; i += (size_t)os_) {
			ring_.push_back(samples[i]);
			total_in_++;
		}
		hi_total_ += count;
	} else {
		ring_.insert(ring_.end(), samples, samples + count);
		total_in_ += count;
	}
	for(auto &st : sfs_) {
		scan(*st);
	}
	trim();
}

// dechirp window at absolute pos with conj(up) or up basis, FFT, return argmax;
// optionally report peak and floor power in dB
static int window_peak(const fft_plan &fft, const cf *base,
                       const cf *x, int n, float *peak_db, float *noise_db, float *peak_lin_out) {
	static thread_local std::vector<cf> buf, out;
	buf.resize(n);
	out.resize(n);
#if defined(__aarch64__) || defined(__ARM_NEON)
	// dechirp premultiply x[i]*base[i], 4 complex lanes at a time
	const float *xf = reinterpret_cast<const float *>(x);
	const float *bf = reinterpret_cast<const float *>(base);
	float *of = reinterpret_cast<float *>(buf.data());
	int i = 0;
	for(; i + 4 <= n; i += 4) {
		float32x4x2_t xv = vld2q_f32(xf + 2 * i);
		float32x4x2_t bv = vld2q_f32(bf + 2 * i);
		float32x4x2_t r;
		r.val[0] = vsubq_f32(vmulq_f32(xv.val[0], bv.val[0]), vmulq_f32(xv.val[1], bv.val[1]));
		r.val[1] = vaddq_f32(vmulq_f32(xv.val[0], bv.val[1]), vmulq_f32(xv.val[1], bv.val[0]));
		vst2q_f32(of + 2 * i, r);
	}
	for(; i < n; i++) {
		buf[i] = x[i] * base[i];
	}
#else
	for(int i = 0; i < n; i++) {
		buf[i] = x[i] * base[i];
	}
#endif
	fft.run(buf.data(), out.data());
	float best_p = 0, sum = 0;
	int best = fft_argmax(out.data(), n, best_p, sum);
	float floor_p = (sum - best_p) / (float)(n - 1);
	if(floor_p <= 0) {
		floor_p = 1e-20f;
	}
	if(peak_db) {
		*peak_db = 10.0f * log10f(best_p + 1e-20f);
	}
	if(noise_db) {
		*noise_db = 10.0f * log10f(floor_p);
	}
	if(peak_lin_out) {
		*peak_lin_out = best_p / floor_p;
	}
	return best;
}

void channel_receiver::scan(sf_state &st) {
	int hop = st.n / 4;
	// a candidate frozen on insufficient data resumes before any new scanning; the
	// detector state (cand_pos, run_bin) is untouched so the retry is deterministic
	if(st.pending) {
		if(!try_frame(st)) {
			return;
		}
		st.pending = false;
		st.run_len = 0;
	}
	while(st.next_hop + st.n <= avail()) {
		if(st.next_hop < skip_until_) {
			st.next_hop += hop;
			st.run_len = 0;
			continue;
		}
		float ratio = 0;
		int bin = window_peak(st.fft, st.upconj.data(), at(st.next_hop), st.n, nullptr, nullptr, &ratio);
		// a window starting d samples past a symbol boundary peaks at bin +d, so each
		// hop advances the raw bin by N/4; remove the window position (mod N) so a
		// steady preamble gives one constant value across the whole run
		bin = (bin - (int)(st.next_hop % (uint64_t)st.n) + st.n) % st.n;
		bool hit = ratio > DETECT_SNR;
		int d = std::abs(bin - st.run_bin);
		if(d > st.n / 2) {
			d = st.n - d;
		}
		if(hit && st.run_len > 0 && d <= 1) {
			st.run_len++;
			st.run_bin = bin;
			st.run_miss = 0;
		} else if(st.run_len > 0 && st.run_miss < DETECT_MISS) {
			st.run_miss++;
		} else if(hit) {
			st.run_len = 1;
			st.run_miss = 0;
			st.run_bin = bin;
			st.cand_pos = st.next_hop;
		} else {
			st.run_len = 0;
			st.run_miss = 0;
		}
		st.next_hop += hop;

		if(st.run_len >= DETECT_HOPS) {
			cand_count_++;
			if(!try_frame(st)) {
				st.pending = true;
				return;
			}
			st.run_len = 0;
		}
	}
}

// demod a single data symbol at absolute pos with CFO derotation
int channel_receiver::demod(const sf_state &st, uint64_t pos, float cfo_bins, float *peak_db, float *noise_db) const {
	return demod_window(st, at(pos), 1, cfo_bins, peak_db, noise_db);
}

// demod the window whose start sits at fractional sample pos, rounded to 1/os_. Starting r
// samples late moves the dechirped tone up r bins, so the rounding goes into the derotation.
int channel_receiver::demod_fine(const sf_state &st, double pos, float tone_bins, float *peak_db, float *noise_db) const {
	long long hi = llround(pos * os_);
	float r = (float)((double)hi / os_ - pos);
	const cf *x = &ring_hi_[(size_t)(hi - (long long)(ring_base_ * (uint64_t)os_))];
	return demod_window(st, x, os_, tone_bins + r, peak_db, noise_db);
}

int channel_receiver::demod_window(const sf_state &st, const cf *x, int stride, float cfo_bins, float *peak_db, float *noise_db) const {
	static thread_local std::vector<cf> buf, out;
	int n = st.n;
	buf.resize(n);
	out.resize(n);
	float w = -2.0f * (float)M_PI * cfo_bins / (float)n;
	// derotate by a constant complex step per sample (rotor recurrence) instead of a
	// per-sample cosf/sinf; renormalize periodically to bound magnitude drift. A
	// per-symbol phase restart is fine because only |FFT| is used.
	cf step((float)cos(w), (float)sin(w));
	cf r(1.0f, 0.0f);
	for(int i = 0; i < n; i++) {
		buf[i] = x[(size_t)i * stride] * st.upconj[i] * r;
		r *= step;
		if((i & 0xFF) == 0xFF) {
			float m = std::abs(r);
			if(m > 0) {
				r /= m;
			}
		}
	}
	st.fft.run(buf.data(), out.data());
	float best_p = 0, sum = 0;
	int best = fft_argmax(out.data(), n, best_p, sum);
	if(peak_db) {
		*peak_db = 10.0f * log10f(best_p + 1e-20f);
	}
	if(noise_db) {
		float f = (sum - best_p) / (float)(n - 1);
		*noise_db = 10.0f * log10f(f > 0 ? f : 1e-20f);
	}
	return best;
}

static int signed_bin(int bin, int n) {
	return bin >= n / 2 ? bin - n : bin;
}

// Total frequency offset of upchirps on the final symbol grid, in bins, including its
// fraction: argmax alone leaves up to half a bin, which at low BW is enough to push peaks
// onto the neighbouring bin. The grid-aligned preamble windows right before the sync word
// give a steady tone, whose phase advance across one symbol is the fraction. Returns
// false when the preamble is no longer buffered.
bool channel_receiver::fine_offset(const sf_state &st, uint64_t sfd_start, float &bins) const {
	static const int WINS = 4;
	const int n = st.n;
	const uint64_t first = (uint64_t)(WINS + 2) * n;
	if(sfd_start < first + ring_base_) {
		return false;
	}
	static thread_local std::vector<cf> buf, spec;
	static thread_local std::vector<float> power;
	buf.resize(n);
	spec.resize((size_t)n * WINS);
	power.assign(n, 0.0f);
	for(int w = 0; w < WINS; w++) {
		const cf *x = at(sfd_start - first + (uint64_t)w * n);
		for(int i = 0; i < n; i++) {
			buf[i] = x[i] * st.upconj[i];
		}
		st.fft.run(buf.data(), spec.data() + (size_t)w * n);
		for(int i = 0; i < n; i++) {
			power[i] += std::norm(spec[(size_t)w * n + i]);
		}
	}
	int peak = 0;
	for(int i = 1; i < n; i++) {
		if(power[i] > power[peak]) {
			peak = i;
		}
	}
	cf acc(0, 0);
	for(int w = 0; w + 1 < WINS; w++) {
		acc += std::conj(spec[(size_t)w * n + peak]) * spec[(size_t)(w + 1) * n + peak];
	}
	bins = (float)signed_bin(peak, n) + std::arg(acc) / (2.0f * (float)M_PI);
	return true;
}

// Pick the frame's sub-sample timing: the correct phase makes the wrap's phase step vanish
// and the symbol peaks tallest. The preamble has no use here, its chirps have no wrap inside
// the window, so score the first interleaving block, whose symbols wrap all over the place.
// Whole-sample error is the grid's job, hence the +-0.5 sample range. Fills timings, best
// first: a frame between two candidates can rank them wrongly, so the caller lets the payload
// CRC have the last word. False: more data needed.
bool channel_receiver::search_timing(const sf_state &st, uint64_t sfd_start, uint64_t data_start, float tone, double slip, std::vector<float> &timings) const {
	static const int SYMS = 8;
	const int n = st.n;
	if(data_start + (uint64_t)(SYMS + 1) * n > avail()) {
		return false;
	}
	std::vector<std::pair<float, float>> ranked;
	for(int k = 1 - os_ / 2; k <= os_ / 2; k++) {
		float d = (float)k / (float)os_, score = 0;
		for(int s = 0; s < SYMS; s++) {
			uint64_t pos = data_start + (uint64_t)s * n;
			float pk = 0;
			demod_fine(st, (double)pos + d - slip * (double)(pos - sfd_start), tone + d, &pk, nullptr);
			score += pk;
		}
		ranked.push_back({ score, d });
	}
	std::sort(ranked.begin(), ranked.end(), [](const std::pair<float, float> &a, const std::pair<float, float> &b) { return a.first > b.first; });
	timings.clear();
	for(const auto &r : ranked) {
		timings.push_back(r.second);
	}
	return true;
}

bool channel_receiver::try_frame(sf_state &st) {
	int n = st.n;
	int hop = n / 4;

	// hunt forward from the candidate for the SFD: first hop window where the
	// down-dechirped peak clearly beats the up-dechirped peak
	uint64_t p = st.next_hop;
	uint64_t sfd_win = 0;
	int k_down = -1;
	uint64_t best_win = 0;
	float best_score = 1.5f;     // weakest down/up contrast that still counts as an SFD
	for(int i = 0; i < MAX_SFD_HUNT; i++, p += hop) {
		if(p + n > avail()) {
			return false; // not enough data yet; scan() freezes the candidate
		}
		float up_ratio = 0, down_ratio = 0;
		window_peak(st.fft, st.upconj.data(), at(p), n, nullptr, nullptr, &up_ratio);
		window_peak(st.fft, st.up.data(), at(p), n, nullptr, nullptr, &down_ratio);
		if(down_ratio > SFD_SNR && down_ratio > 2.0f * up_ratio) {
			sfd_win = p + hop; // one hop further in, safely inside the two downchirps
			if(sfd_win + n > avail()) {
				return false;
			}
			k_down = window_peak(st.fft, st.up.data(), at(sfd_win), n, nullptr, nullptr, &down_ratio);
			break;
		}
		if(down_ratio / (up_ratio + 1e-6f) > best_score) {
			best_score = down_ratio / (up_ratio + 1e-6f);
			best_win = p;
		}
	}
	if(k_down < 0 && best_win) {
		sfd_win = best_win + hop;
		if(sfd_win + n > avail()) {
			return false;
		}
		float down_ratio = 0;
		k_down = window_peak(st.fft, st.up.data(), at(sfd_win), n, nullptr, nullptr, &down_ratio);
	}
	if(k_down < 0) {
		return true; // no SFD in range: false alarm, give up on this candidate
	}

	// st.run_bin is window-compensated; restore the bin as seen at the cand_pos window
	int k_up = (st.run_bin + (int)(st.cand_pos % (uint64_t)n)) % n;
	// k_down was measured a non-integer number of symbols past cand_pos; shift it onto
	// the cand_pos grid so both measurements reference the same timing offset
	k_down = (k_down + (int)((sfd_win - st.cand_pos) % (uint64_t)n)) % n;

	// k_up = sto + cfo, k_down = cfo - sto (bins, wrapped)
	float su = (float)signed_bin(k_up, n);
	float sd = (float)signed_bin(k_down, n);
	float cfo = (su + sd) / 2.0f;
	float sto = (su - sd) / 2.0f;
	// the pair is ambiguous by n/2; pick the representation with the smaller CFO
	if(fabsf(cfo) > (float)n / 4) {
		cfo += cfo > 0 ? -(float)n / 2 : (float)n / 2;
		sto += sto > 0 ? -(float)n / 2 : (float)n / 2;
	}

	// windows start sto samples after a symbol boundary, so boundaries sit at
	// cand_pos - sto (mod N); +N keeps the grid start past the candidate window
	uint64_t grid = st.cand_pos + (uint64_t)n - (int64_t)lroundf(sto);

	// walk the grid to find the SFD start: the first grid symbol at/after the
	// candidate where down beats up
	uint64_t sym = grid;
	uint64_t sfd_start = 0;
	const int sync_hi = (int)(((sync_ >> 4) & 0xF) << 3), sync_lo = (int)((sync_ & 0xF) << 3);
	auto sync_near = [&](int got, int want) {
		int alt = want >= 64 ? ((want - 128) % n + n) % n : want; // signed nibble, see below
		int d = std::abs(got - want), e = std::abs(got - alt);
		return std::min(std::min(d, n - d), std::min(e, n - e)) <= SYNC_TOL;
	};
	for(int i = 0; i < MAX_SFD_HUNT + 8; i++, sym += n) {
		if(sym + 3 * (uint64_t)n > avail()) {
			return false;
		}
		float upr = 0, dnr = 0;
		{
			// the energy test below trips inside a weak preamble, and the sync word is a known
			// pattern anyway
			float pk, nf;
			if(sync_near(demod(st, sym, cfo, &pk, &nf), sync_hi) && sync_near(demod(st, sym + n, cfo, &pk, &nf), sync_lo)) {
				sfd_start = sym + 2 * (uint64_t)n;
				break;
			}
		}
		window_peak(st.fft, st.upconj.data(), at(sym), n, nullptr, nullptr, &upr);
		window_peak(st.fft, st.up.data(), at(sym), n, nullptr, nullptr, &dnr);
		if(dnr > upr) {
			sfd_start = sym;
			break;
		}
	}
	if(!sfd_start) {
		return true;
	}

	// validate the sync word: the two symbols right before the SFD
	if(sfd_start < (uint64_t)2 * n + ring_base_) {
		return true; // not enough history to check the sync word - treat as a false alarm
	}
	if(os_ > 1) {
		fine_offset(st, sfd_start, cfo);
	}
	{
		int exp_hi = (int)(((sync_ >> 4) & 0xF) << 3);
		int exp_lo = (int)((sync_ & 0xF) << 3);
		float pk, nf;
		int s1 = demod(st, sfd_start - 2 * (uint64_t)n, cfo, &pk, &nf);
		int s2 = demod(st, sfd_start - (uint64_t)n, cfo, &pk, &nf);
		auto near = [&](int got, int want) {
			int d = std::abs(got - want);
			return std::min(d, n - d) <= SYNC_TOL;
		};
		// a nibble above 7 is signed on air: Meshtastic's 0x2B sends the low symbol at -40, not +88
		auto on_air = [&](int want) { return want >= 64 ? ((want - 128) % n + n) % n : want; };
		if(!near(s1, exp_hi) && near(s1, on_air(exp_hi))) {
			exp_hi = on_air(exp_hi);
		}
		if(!near(s2, exp_lo) && near(s2, on_air(exp_lo))) {
			exp_lo = on_air(exp_lo);
		}
		if(!near(s1, exp_hi) || !near(s2, exp_lo)) {
			return true; // different network or a false alarm
		}
		// fine_offset() knows the fraction, but its FFT peak can sit one bin from the true
		// tone, which leaves the whole offset one bin out. The sync symbols are known values,
		// so when both agree they settle the integer part.
		int e1 = signed_bin(((s1 - exp_hi) % n + n) % n, n), e2 = signed_bin(((s2 - exp_lo) % n + n) % n, n);
		if(os_ > 1 && e1 == e2) {
			cfo += (float)e1;
		}
	}

	// data starts 2.25 symbols after the SFD
	uint64_t data_start = sfd_start + 2 * (uint64_t)n + n / 4;

	// LDRO is on for symbols of 16 ms and longer, as the radios' automatic setting does:
	// SF12 at 250 kHz, SF11 at 125 kHz, SF10 at 62.5 kHz
	bool ldro = (double)n / bw_hz_ >= 0.016;
	// A LoRa radio derives carrier and chirp clock from one crystal, so a carrier offset of
	// x ppm also shortens the symbols by x ppm: this many samples of slip per sample, about
	// one whole sample by the end of a long BW 62.5 frame at 10 ppm
	const double slip = (double)cfo * bw_hz_ / n / freq_hz_;
	std::vector<float> timings(1, 0.0f);
	if(os_ > 1 && !search_timing(st, sfd_start, data_start, cfo, slip, timings)) {
		return false;
	}
	// header symbols are reduced rate, so a real frame reads 1 mod 4 on all eight: a majority on
	// another residue is the carrier estimate one bin out
	{
		if(data_start + 8 * (uint64_t)n + 4 > avail()) {
			return false;
		}
		int votes[4] = { 0, 0, 0, 0 };
		float pk = 0, nf = 0;
		for(int i = 0; i < 8; i++) {
			uint64_t q = data_start + (uint64_t)i * n;
			int raw = os_ > 1 ? demod_fine(st, (double)q + timings[0], cfo + timings[0], &pk, &nf) : demod(st, q, cfo, &pk, &nf);
			votes[(raw + 3) & 3]++;
		}
		if(votes[1] >= 6) {
			cfo += 1.0f;
		} else if(votes[3] >= 6) {
			cfo -= 1.0f;
		}
	}
	frame_decoder dec(st.sf, ldro);
	float snr_sum = 0;
	int snr_cnt = 0;
	uint64_t pos = data_start;
	bool have = false, runaway = false;
	for(float timing : timings) {
		frame_decoder cand(st.sf, ldro);
		float sum = 0, pk = 0, nf = 0;
		int cnt = 0, guard = 0;
		uint64_t p = data_start;
		runaway = false;
		while(true) {
			const uint64_t margin = os_ > 1 ? 4 : 0; // room for the timing and slip offsets
			if(p + n + margin > avail()) {
				return false; // wait for more samples; scan() freezes the candidate
			}
			int raw = os_ > 1 ? demod_fine(st, (double)p + timing - slip * (double)(p - sfd_start), cfo + timing, &pk, &nf)
			                  : demod(st, p, cfo, &pk, &nf);
			sum += pk - nf;
			cnt++;
			p += n;
			if(!cand.feed((uint16_t)raw)) {
				break;
			}
			// bound the decode: a small cap until the header parses, then exactly what the
			// header-declared frame needs (a fixed 520-symbol cap silently dropped max-length
			// SF7 CR4/8 frames, which run ~600 symbols)
			if(++guard > (cand.header_ready() ? cand.symbols_needed() + 8 : 32)) {
				runaway = true; // header never parsed, abandon
				break;
			}
		}
		if(runaway || !cand.header_ready() || !cand.done()) {
			continue;
		}
		if(!have || cand.crc_ok()) {
			dec = cand;
			snr_sum = sum;
			snr_cnt = cnt;
			pos = p;
			have = true;
		}
		if(cand.crc_ok()) {
			break;
		}
	}
	if(!have) {
		if(!runaway) {
			skip_until_ = sfd_start + 3 * (uint64_t)n;
		}
		return true;
	}

	rx_packet pkt;
	pkt.freq_hz = freq_hz_;
	pkt.bw_hz = bw_hz_;
	pkt.sf = st.sf;
	pkt.cr = dec.header().cr;
	pkt.crc_present = dec.header().has_crc;
	pkt.crc_ok = dec.crc_ok();
	pkt.fec_clean = dec.fec_clean();
	pkt.payload = dec.payload();
	pkt.sample_pos = st.cand_pos;
	// report SNR relative to the in-band noise floor estimate of the FFT
	pkt.snr_db = snr_cnt ? (snr_sum / snr_cnt - 10.0f * log10f((float)n)) : 0;
	on_packet_(pkt);

	// resume scanning after the frame
	skip_until_ = pos;
	for(auto &other : sfs_) {
		other->run_len = 0;
		other->pending = false;
		if(other->next_hop < pos) {
			other->next_hop = pos;
		}
	}
	return true;
}

} // namespace lorad

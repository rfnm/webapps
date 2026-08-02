#include "receiver.h"

#include <cmath>
#include <cstring>

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

// Detection tuning. The hop grid is N/4 samples; a preamble candidate needs
// DETECT_HOPS consecutive windows agreeing on the dechirped bin.
static const int DETECT_HOPS = 8;        // 2 full symbols
static const float DETECT_SNR = 8.0f;    // peak power over mean floor, linear
static const int SYNC_TOL = 2;           // bins of tolerance on the sync word symbols
static const int MAX_SFD_HUNT = 24;      // hops to find the SFD after a candidate

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

channel_receiver::channel_receiver(double freq_hz, uint8_t sync, unsigned sf_mask, bool bw250,
                                   std::function<void(const rx_packet &)> on_packet)
	: freq_hz_(freq_hz), sync_(sync), bw250_(bw250), on_packet_(std::move(on_packet)) {
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
		ring_base_ = keep_from;
	}
}

void channel_receiver::feed(const cf *samples, size_t count) {
	ring_.insert(ring_.end(), samples, samples + count);
	total_in_ += count;
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
		if(hit && st.run_len > 0) {
			int d = std::abs(bin - st.run_bin);
			if(d > st.n / 2) {
				d = st.n - d;
			}
			if(d <= 1) {
				st.run_len++;
			} else {
				st.run_len = 1;
				st.cand_pos = st.next_hop;
			}
			st.run_bin = bin;
		} else if(hit) {
			st.run_len = 1;
			st.run_bin = bin;
			st.cand_pos = st.next_hop;
		} else {
			st.run_len = 0;
		}
		st.next_hop += hop;

		if(st.run_len >= DETECT_HOPS) {
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
	static thread_local std::vector<cf> buf, out;
	int n = st.n;
	buf.resize(n);
	out.resize(n);
	const cf *x = at(pos);
	float w = -2.0f * (float)M_PI * cfo_bins / (float)n;
	// derotate by a constant complex step per sample (rotor recurrence) instead of a
	// per-sample cosf/sinf; renormalize periodically to bound magnitude drift. A
	// per-symbol phase restart is fine because only |FFT| is used.
	cf step((float)cos(w), (float)sin(w));
	cf r(1.0f, 0.0f);
	for(int i = 0; i < n; i++) {
		buf[i] = x[i] * st.upconj[i] * r;
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

bool channel_receiver::try_frame(sf_state &st) {
	int n = st.n;
	int hop = n / 4;

	// hunt forward from the candidate for the SFD: first hop window where the
	// down-dechirped peak clearly beats the up-dechirped peak
	uint64_t p = st.next_hop;
	uint64_t sfd_win = 0;
	int k_down = -1;
	for(int i = 0; i < MAX_SFD_HUNT; i++, p += hop) {
		if(p + n > avail()) {
			return false; // not enough data yet; scan() freezes the candidate
		}
		float up_ratio = 0, down_ratio = 0;
		window_peak(st.fft, st.upconj.data(), at(p), n, nullptr, nullptr, &up_ratio);
		window_peak(st.fft, st.up.data(), at(p), n, nullptr, nullptr, &down_ratio);
		if(down_ratio > DETECT_SNR && down_ratio > 3.0f * up_ratio) {
			sfd_win = p + hop; // one hop further in, safely inside the two downchirps
			if(sfd_win + n > avail()) {
				return false;
			}
			k_down = window_peak(st.fft, st.up.data(), at(sfd_win), n, nullptr, nullptr, &down_ratio);
			break;
		}
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
	for(int i = 0; i < MAX_SFD_HUNT + 8; i++, sym += n) {
		if(sym + n > avail()) {
			return false;
		}
		float upr = 0, dnr = 0;
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
		if(!near(s1, exp_hi) || !near(s2, exp_lo)) {
			return true; // different network or a false alarm
		}
	}

	// data starts 2.25 symbols after the SFD
	uint64_t data_start = sfd_start + 2 * (uint64_t)n + n / 4;

	bool ldro = bw250_ ? st.sf >= 12 : st.sf >= 11;
	frame_decoder dec(st.sf, ldro);
	float snr_sum = 0, pk = 0, nf = 0;
	int snr_cnt = 0;
	uint64_t pos = data_start;
	int guard = 0;
	while(true) {
		if(pos + n > avail()) {
			return false; // wait for more samples; scan() freezes the candidate
		}
		int raw = demod(st, pos, cfo, &pk, &nf);
		snr_sum += pk - nf;
		snr_cnt++;
		pos += n;
		if(!dec.feed((uint16_t)raw)) {
			break;
		}
		// bound the decode: a small cap until the header parses, then exactly what the
		// header-declared frame needs (a fixed 520-symbol cap silently dropped max-length
		// SF7 CR4/8 frames, which run ~600 symbols)
		if(++guard > (dec.header_ready() ? dec.symbols_needed() + 8 : 32)) {
			return true; // runaway (header never parsed), abandon
		}
	}
	if(!dec.header_ready() || !dec.done()) {
		skip_until_ = sfd_start + 3 * (uint64_t)n;
		return true;
	}

	rx_packet pkt;
	pkt.freq_hz = freq_hz_;
	pkt.bw_hz = bw250_ ? 250000.0 : 125000.0;
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

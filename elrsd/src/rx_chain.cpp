// NEEDS: src/dsp.cpp
// elrsd: RX signal chain - css_modem, frame_sync, burst-gated rx_chain,
// rate_index_from_duration. Mirrors a Python reference implementation (1123/1181 on the SF6
// capture) step for step No ELRS bit decoding here.
#include "rx_chain.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>

namespace elrsd {

// ====================================================================== css_modem =====
css_modem::css_modem(int sf, int os, double bw_hz)
    : sf_(sf), n_(1 << sf), os_(os < 1 ? 1 : os), bw_(bw_hz), fft_((unsigned)((1 << sf) * (os < 1 ? 1 : os))) {
	const int N = n_ * os_;
	up_.resize(N);
	down_.resize(N);
	upconj_.resize(N);
	// base up-chirp: phase = 2*pi*(k^2 / (2*M*os^2) - k / (2*os)) -> sweeps -bw/2 .. +bw/2
	for(int k = 0; k < N; k++) {
		double ph = 2.0 * M_PI * ((double)k * k / (2.0 * n_ * os_ * os_) - (double)k / (2.0 * os_));
		up_[k] = cf((float)cos(ph), (float)sin(ph));
		down_[k] = std::conj(up_[k]);
		upconj_[k] = down_[k];
	}
}

// dechirp with `ref`, FFT, fold the os aliases (sum of magnitudes like the Python _fold),
// argmax over n bins. peak = |X|^2 at the argmax, noise = mean |X|^2 of the other bins.
static int fold_argmax(const fft_plan &fft, const cf *x, const cf *ref, int n, int os, float *peak, float *noise, float *frac = nullptr, float *peak3 = nullptr) {
	const int N = n * os;
	thread_local std::vector<cf> t, X;
	thread_local std::vector<float> mag;
	if((int)t.size() < N) {
		t.resize(N);
		X.resize(N);
	}
	if((int)mag.size() < n) mag.resize(n);
	for(int i = 0; i < N; i++) t[i] = x[i] * ref[i];
	fft.run(t.data(), X.data());
	for(int k = 0; k < n; k++) {
		float m = std::abs(X[k]);
		for(int j = 1; j < os; j++) m += std::abs(X[k + j * n]);
		mag[k] = m;
	}
	int best = 0;
	float bm = mag[0];
	double sum2 = 0;
	for(int k = 0; k < n; k++) {
		if(mag[k] > bm) {
			bm = mag[k];
			best = k;
		}
		sum2 += (double)mag[k] * mag[k];
	}
	if(peak) *peak = bm * bm;
	if(noise) *noise = n > 1 ? (float)((sum2 - (double)bm * bm) / (n - 1)) : 0.0f;
	if(peak3) {
		float a = mag[(best + n - 1) % n], c = mag[(best + 1) % n];
		*peak3 = a * a + bm * bm + c * c;
	}
	if(frac) {
		// Jacobsen estimator on the complex bins of the dominant alias (exact for a
		// rectangular window): d = -Re[(X+ - X-) / (2 X0 - X+ - X-)]
		int K = best;
		float am = std::abs(X[best]);
		for(int j = 1; j < os; j++) {
			float m = std::abs(X[best + j * n]);
			if(m > am) {
				am = m;
				K = best + j * n;
			}
		}
		cf x0 = X[K], xp = X[(K + 1) % N], xm = X[(K + N - 1) % N];
		cf den = 2.0f * x0 - xp - xm;
		float f = std::norm(den) > 0 ? -std::real((xp - xm) / den) : 0.0f;
		*frac = std::max(-0.5f, std::min(0.5f, f));
	}
	return best;
}

int css_modem::demod_symbol(const cf *x, float *peak, float *noise) const {
	return fold_argmax(fft_, x, upconj_.data(), n_, os_, peak, noise);
}

int css_modem::demod_symbol_down(const cf *x, float *peak, float *noise) const {
	return fold_argmax(fft_, x, up_.data(), n_, os_, peak, noise);
}

int css_modem::demod_symbol_frac(const cf *x, float *frac, float *peak, float *noise) const {
	return fold_argmax(fft_, x, upconj_.data(), n_, os_, peak, noise, frac);
}

float css_modem::peak_power3(const cf *x, bool down) const {
	float p3 = 0;
	fold_argmax(fft_, x, down ? up_.data() : upconj_.data(), n_, os_, nullptr, nullptr, nullptr, &p3);
	return p3;
}

void css_modem::demod(const cf *x, size_t count, std::vector<uint16_t> &out, std::vector<float> *snr_db) const {
	const int N = n_ * os_;
	out.clear();
	if(snr_db) snr_db->clear();
	for(size_t i = 0; i < count; i++) {
		float pk = 0, nz = 0;
		int s = demod_symbol(x + i * N, &pk, &nz);
		out.push_back((uint16_t)s);
		if(snr_db) snr_db->push_back(nz > 0 ? 10.0f * log10f(pk / nz) : 0.0f);
	}
}

void css_modem::rotate_cfo(cf *x, size_t count, double cfo_hz, double fs_hz, double phase0) {
	double w = 2.0 * M_PI * cfo_hz / fs_hz;
	for(size_t i = 0; i < count; i++) {
		double ph = phase0 + w * (double)i;
		x[i] *= cf((float)cos(ph), (float)sin(ph));
	}
}

// ====================================================================== frame_sync ====
static const float MIN_PREAMBLE_SNR_DB = 6.0f;
frame_sync::frame_sync(const rate_params &rp, int os, bool refine) : rp_(rp), modem_(rp.sf, os), refine_(refine) {}

std::vector<double> frame_sync::candidate_offsets() const {
	return {rp_.frame_offset_symbols, rp_.frame_offset_symbols - 0.25, rp_.frame_offset_symbols + 0.25};
}

namespace {
// longest run of consecutive preamble argmaxes over sy[0..n_pre) (Python: j in 1..n_pre-1,
// run reset to 1 on a change). Unlike Python the run tolerates a +-1 bin wobble (circular):
// a tone sitting on a half bin alternates between two neighbours, which the plain equality
// test would read as "no preamble". Returns the run length and the majority value of the run.
int longest_run(const std::vector<uint16_t> &sy, int n_pre, int n, int &value) {
	int run = 1, mx = 1, end = 0;
	for(int j = 1; j < n_pre && j < (int)sy.size(); j++) {
		int d = ((int)sy[j] - (int)sy[j - 1] + n) % n;
		run = (d <= 1 || d == n - 1) ? run + 1 : 1;
		if(run > mx) {
			mx = run;
			end = j;
		}
	}
	// majority over the run [end - mx + 1, end]
	int a = end - mx + 1;
	int best = sy.empty() ? 0 : sy[a], bc = 0;
	for(int i = a; i <= end; i++) {
		int c = 0;
		for(int k = a; k <= end; k++) c += sy[k] == sy[i];
		if(c > bc) {
			bc = c;
			best = sy[i];
		}
	}
	value = best;
	return mx;
}
inline bool near_bin(int a, int b, int n) {
	int d = (a - b + n) % n;
	return d <= 1 || d == n - 1;
}
int wrap_signed(int b, int n) { return b >= n / 2 ? b - n : b; }
} // namespace

frame_sync_result frame_sync::run(const cf *x_in, size_t count, std::vector<uint16_t> &symbols) const {
	frame_sync_result r;
	r.sf = rp_.sf;
	symbols.clear();
	const int N = modem_.samples_per_symbol();
	const int n = modem_.n();
	const int n_pre = rp_.preamble_len;
	const int n_sym = rp_.n_sym;
	if(count < (size_t)((n_pre + 2) * N)) return r;

	// --- narrow-band envelope onset (32-sample blocks, thr = min + 0.3 * (p95 - min)) ---
	const size_t B = 32;
	size_t nb = count / B;
	if(nb < 2) return r;
	std::vector<float> env(nb);
	for(size_t b = 0; b < nb; b++) {
		float s = 0;
		for(size_t i = 0; i < B; i++) s += std::abs(x_in[b * B + i]);
		env[b] = s / (float)B;
	}
	std::vector<float> tmp(env);
	size_t i95 = (size_t)std::min<double>((double)(nb - 1), std::ceil(0.95 * (double)(nb - 1)));
	std::nth_element(tmp.begin(), tmp.begin() + i95, tmp.end());
	float p95 = tmp[i95];
	float mn = *std::min_element(env.begin(), env.end());
	// (Python: 0.3 and the first block; 0.4 + two consecutive blocks keeps a noise block from
	// starting the scan a symbol early on weak bursts - the scan then covers the true start)
	float thr = mn + 0.4f * (p95 - mn);
	size_t p0 = 0;
	bool have = false;
	for(size_t b = 0; b + 1 < nb; b++) {
		if(env[b] > thr && env[b + 1] > thr) {
			p0 = b * B;
			have = true;
			break;
		}
	}
	if(!have) return r;

	// --- optional CFO refinement pass works on a rotated copy ---
	std::vector<cf> rot;
	const cf *x = x_in;

	// --- shift scan -------------------------------------------------------------------
	// A preamble of identical up-chirps carries no sub-symbol timing: a window shifted by any
	// delta still dechirps to one clean tone (the bin moves, `ref` absorbs it), so neither the
	// run length nor the preamble peak power can rank shifts. The frame EDGE can: at the true
	// alignment the window n_pre+1 holds the whole last sync-word up-chirp and window n_pre+2
	// the whole first SFD down-chirp; shifted early or late, one of the two gets split with a
	// chirp of the other direction and its peak falls as (1 - delta)^2. J(sh) = P_up(n_pre+1)
	// + P_down(n_pre+2) is therefore unimodal in the shift. Stage 1 evaluates J on the coarse
	// grid (2 FFTs per shift), stage 2 checks the preamble run (Python's criterion, +-1 bin
	// tolerant) on the best J shifts, stage 3 refines J at 1-sample steps around the winner.
	auto edge_score = [&](const cf *xx, size_t p) -> double {
		if(p + (size_t)(n_pre + 4) * N > count) return -1.0;
		// both sync-word windows (up) and both full SFD windows (down): late or early by a
		// fraction delta, three of the four fall as (1 - delta)^2 - steeper than one pair
		double pu = modem_.peak_power3(xx + p + (size_t)(n_pre + 0) * N, false) + modem_.peak_power3(xx + p + (size_t)(n_pre + 1) * N, false);
		double pd = modem_.peak_power3(xx + p + (size_t)(n_pre + 2) * N, true) + modem_.peak_power3(xx + p + (size_t)(n_pre + 3) * N, true);
		return pu + pd;
	};
	auto run_at = [&](const cf *xx, size_t p, int &val, float &snr) -> int {
		std::vector<uint16_t> sy;
		std::vector<float> ss;
		val = 0;
		snr = 0;
		if(p + (size_t)(n_pre + 2) * N > count) return 0;
		modem_.demod(xx + p, (size_t)n_pre, sy, &ss);
		int run = longest_run(sy, n_pre, n, val);
		float acc = 0;
		int cnt = 0;
		for(int j = 0; j < n_pre; j++) {
			if(near_bin(sy[j], val, n)) {
				acc += ss[j];
				cnt++;
			}
		}
		snr = cnt ? acc / cnt : 0;
		return run;
	};
	auto scan = [&](const cf *xx, int &best_run, int &best_sh, int &best_val, float &snr) {
		best_run = 0;
		best_sh = 0;
		best_val = 0;
		snr = 0;
		const int step = std::max(N / 32, 1);
		std::vector<std::pair<double, int>> cand;   // (J, sh)
		for(int sh = 0; sh < N; sh += step) {
			double j = edge_score(xx, p0 + (size_t)sh);
			if(j < 0) break;
			cand.push_back({j, sh});
		}
		if(cand.empty()) return;
		std::sort(cand.begin(), cand.end(), [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; });
		double best_j = -1;
		for(size_t i = 0; i < cand.size() && i < 4; i++) {
			int val;
			float sn;
			int run = run_at(xx, p0 + (size_t)cand[i].second, val, sn);
			if(run > best_run) {
				best_run = run;
				best_sh = cand[i].second;
				best_val = val;
				snr = sn;
				best_j = cand[i].first;
			}
			if(run >= n_pre - 4) break;   // candidates are J-sorted: the first valid one wins
		}
		if(best_run < n_pre - 4) return;
		// fine pass on J at 1-sample steps (os 4: 0.25 bin), run must stay valid
		const int fine = std::max(1, step / 8);
		int fine_sh = best_sh;
		for(int d = -(step - 1); d <= step - 1; d += fine) {
			if(d == 0) continue;
			long sh = (long)best_sh + d;
			if(sh < 0) continue;
			double j = edge_score(xx, p0 + (size_t)sh);
			if(j > best_j) {
				best_j = j;
				fine_sh = (int)sh;
			}
		}
		if(fine_sh != best_sh) {
			int val;
			float sn;
			int run = run_at(xx, p0 + (size_t)fine_sh, val, sn);
			if(run >= n_pre - 4) {
				best_run = run;
				best_sh = fine_sh;
				best_val = val;
				snr = sn;
			}
		}
	};
	int best_run, best_sh, best_val;
	float snr;
	scan(x, best_run, best_sh, best_val, snr);
	r.preamble_run = best_run;
	r.snr_db = snr;
	if(best_run < n_pre - 4) return r;
	// tone quality gate: an equal-argmax run is also produced by ANY periodic waveform (e.g. a
	// conjugated preamble = down-chirps, whose up-dechirp is a flat double-slope chirp with a
	// stable but meaningless argmax). A real preamble tone stands >= 6 dB above the mean of
	// the other bins even at the weakest gated burst (Python never needed this: its data had
	// the right sign; auto_conj relies on it).
	if(snr < MIN_PREAMBLE_SNR_DB) return r;

	size_t pre_start = p0 + (size_t)best_sh;
	// SFD statistics (never used to reject; the ELRS sync word is): last preamble symbol
	// up-chirp bin vs the down-chirp bin at the SFD position
	{
		size_t sfd = pre_start + (size_t)(n_pre + 2) * N;
		if(sfd + (size_t)N <= count) {
			float pu = 0, nu = 0, pd = 0, nd = 0;
			int bu = modem_.demod_symbol(x + sfd, &pu, &nu);
			int bd = modem_.demod_symbol_down(x + sfd, &pd, &nd);
			r.sfd_bin_up = bu;
			r.sfd_bin_down = bd;
			r.sfd_seen = pd > pu;
		}
	}
	if(refine_ && r.sfd_seen) {
		// lorad-style: up bin = cfo + sto, down bin = cfo - sto (bins) -> cfo = (up + down) / 2
		int u = wrap_signed(best_val, n), d = wrap_signed(r.sfd_bin_down, n);
		double cfo_bins = 0.5 * (u + d);
		if(std::fabs(cfo_bins) < n / 4.0 && cfo_bins != 0.0) {
			r.cfo_hz = cfo_bins * ELRS_BW_HZ / n;
			rot.assign(x_in, x_in + count);
			css_modem::rotate_cfo(rot.data(), count, -r.cfo_hz, modem_.os() * ELRS_BW_HZ);
			x = rot.data();
			int r2, s2, v2;
			float q2;
			scan(x, r2, s2, v2, q2);
			if(r2 >= n_pre - 4) {
				best_run = r2;
				best_sh = s2;
				best_val = v2;
				snr = q2;
				pre_start = p0 + (size_t)best_sh;
				r.preamble_run = best_run;
			} else {
				x = x_in;   // refinement made it worse: keep the raw pass
				r.cfo_hz = 0;
			}
		}
	}
	size_t pay = pre_start + (size_t)llround(rp_.frame_offset_symbols * N);
	r.preamble_start = pre_start;
	r.payload_start = pay;
	r.ref = best_val;
	r.snr_db = snr;
	if(pay + (size_t)n_sym * N > count) return r;   // frame truncated: not found

	// Fractional bin of the preamble tone (circular mean over the run symbols) -> rotate the
	// window by -frac bins so that ref and payload round from an integer position (two
	// passes). Same correction for both, so the (s - ref) convention is untouched; r.cfo_hz
	// reports the total shift. ref = majority of the preamble symbols under that correction.
	const size_t lo = pre_start, hi = pay + (size_t)n_sym * N;
	std::vector<cf> w(x + lo, x + hi);
	for(int pass = 0; pass < 2; pass++) {
		double cs = 0, sn = 0;
		for(int j = 0; j < n_pre; j++) {
			float fr = 0;
			int b = modem_.demod_symbol_frac(w.data() + (size_t)j * N, &fr);
			if(!near_bin(b, best_val, n)) continue;
			cs += cos(2.0 * M_PI * fr);
			sn += sin(2.0 * M_PI * fr);
		}
		double frac = (cs != 0 || sn != 0) ? atan2(sn, cs) / (2.0 * M_PI) : 0.0;
		if(std::fabs(frac) < 0.01) break;
		double f_hz = frac * ELRS_BW_HZ / n;
		css_modem::rotate_cfo(w.data(), w.size(), -f_hz, modem_.os() * ELRS_BW_HZ);
		r.cfo_hz += f_hz;
	}
	{
		std::vector<int> cnt(n, 0);
		for(int j = 0; j < n_pre; j++) cnt[modem_.demod_symbol(w.data() + (size_t)j * N)]++;
		int ref = 0;
		for(int k = 1; k < n; k++) if(cnt[k] > cnt[ref]) ref = k;
		r.ref = ref;
	}
	std::vector<float> ps;
	modem_.demod(w.data() + (pay - lo), (size_t)n_sym, symbols, &ps);
	r.found = true;
	return r;
}

// ====================================================================== rate detect ===
int rate_index_from_duration(double duration_s) {
	for(int idx : rate_indices_verified()) {
		const rate_params *rp = rate_by_index(idx);
		if(!rp) continue;
		double b = rp->burst_s();
		if(std::fabs(duration_s - b) <= 0.2 * b) return idx;
	}
	return 0;
}

// ====================================================================== rx_chain ======
// extraction filter: 32 taps per polyphase branch, generic cutoff 0.45 * out_rate (= 1.46 MHz
// at OS 4: flat over the +-406 kHz chirp like scipy's resample_poly in the Python reference,
// aliases from > 2.9 MHz are down > 40 dB; at OS 1 it is the 366 kHz anti-alias filter).
// Adjacent-channel energy is spread by the dechirp, it needs no sharp channel filter.
static const int RS_TAPS = 32;
static const double RS_CUTOFF_HZ = 0;

struct rx_chain::impl {
	rx_chain_config cfg;                         // runtime copy (knobs may change)
	rx_chain_config *pub = nullptr;              // rx_chain::cfg_ (config() view), kept in step under the knob mutex
	std::mutex knob;
	std::function<void(const rx_burst &)> cb;

	// raw ring (int16 pairs)
	std::vector<int16_t> ring;
	uint64_t ring_len = 0;                       // complex samples
	uint64_t ring_end = 0;                       // absolute index of the next sample to write
	uint64_t ring_valid_from = 0;
	bool have_data = false;

	// device timestamp annotation
	uint64_t ts_index = 0;
	int64_t ts_ns = -1;

	// envelope + floor
	size_t blk;
	envelope_i16 env;
	uint64_t env_next_start = 0;
	std::vector<uint32_t> env_out;
	std::vector<uint32_t> hist;
	size_t hist_pos = 0, hist_fill = 0;
	double floor_level = 0;
	int since_floor = 0;
	double margin;

	// burst state machine
	bool in_burst = false, skip_until_off = false;
	uint64_t burst_onset = 0;
	size_t burst_blocks = 0;
	double burst_env_sum = 0;

	struct pending_burst {
		uint64_t onset, end, need_end;
		double dur_s, env_mean;
		int rate_index;
	};
	std::vector<pending_burst> pend;

	// hint
	double hint_t0 = 0, hint_t1 = 0;
	int hint_ch = -1;

	// per-rate frame syncs (built lazily under the knob mutex)
	std::map<int, std::unique_ptr<frame_sync>> fsync;
	fft_plan chfft;
	std::vector<float> hann;

	rx_chain::stats st;
	double busy_s = 0;
	std::chrono::steady_clock::time_point cpu_t0;
	bool cpu_started = false;

	explicit impl(const rx_chain_config &c)
	    : cfg(c), blk((size_t)std::max(1.0, std::round(c.env_block_s * c.fs_hz))), env(blk), chfft(16384) {
		env.set_dc_tracking(true);
		ring_len = (uint64_t)std::max(1.0, std::round(c.ring_s * c.fs_hz));
		ring.assign((size_t)ring_len * 2, 0);
		size_t hist_len = (size_t)std::max(64.0, std::round(1.0 / c.env_block_s));   // ~1 s of blocks
		hist.assign(hist_len, 0);
		margin = 64.0 * (double)blk;
		// offline-sweep overrides (replay tuning); do not affect the on-board defaults unless set
		if(const char *e = getenv("ELRSD_ENV_ENTER")) cfg.env_thresh_factor = atof(e);
		if(const char *e = getenv("ELRSD_ENV_SUSTAIN")) cfg.env_sustain_factor = atof(e);
		if(cfg.env_sustain_factor > cfg.env_thresh_factor) cfg.env_sustain_factor = cfg.env_thresh_factor;
		hann.resize(16384);
		for(int i = 0; i < 16384; i++) hann[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / 16383.0f);
	}

	frame_sync &sync_for(int idx) {
		auto it = fsync.find(idx);
		if(it != fsync.end()) return *it->second;
		const rate_params *rp = rate_by_index(idx);
		if(!rp) rp = rate_by_index(6);
		auto fs = std::make_unique<frame_sync>(*rp, cfg.os, cfg.refine);
		frame_sync &ref = *fs;
		fsync[idx] = std::move(fs);
		return ref;
	}

	void update_floor(uint32_t v) {
		hist[hist_pos] = v;
		hist_pos = (hist_pos + 1) % hist.size();
		if(hist_fill < hist.size()) hist_fill++;
		if(++since_floor >= 128 || hist_fill < 64 || floor_level <= 0) {
			since_floor = 0;
			std::vector<uint32_t> t(hist.begin(), hist.begin() + hist_fill);
			size_t k = (size_t)(0.2 * (double)(hist_fill - 1));
			std::nth_element(t.begin(), t.begin() + k, t.end());
			floor_level = (double)t[k];
		}
	}

	void reset_stream(uint64_t index) {
		env.reset();
		env_next_start = index;
		in_burst = false;
		skip_until_off = false;
		pend.clear();
		ring_end = index;
		ring_valid_from = index;
	}

	// wrap-aware copy of [start, start+n) from the ring
	void ring_copy(uint64_t start, size_t n, int16_t *dst) const {
		size_t pos = (size_t)(start % ring_len);
		size_t first = (size_t)std::min<uint64_t>(n, ring_len - pos);
		memcpy(dst, &ring[pos * 2], first * 4);
		if(first < n) memcpy(dst + first * 2, &ring[0], (n - first) * 4);
	}

	// channel estimate from the burst middle: 16384-point Hann FFT, peak, centroid +-0.6 MHz
	double estimate_offset_hz(uint64_t onset, uint64_t end, bool conj, double &quality) {
		const size_t L = 16384;
		uint64_t dur = end - onset;
		uint64_t start = dur > L ? onset + (dur - L) / 2 : onset;
		if(start < ring_valid_from) start = ring_valid_from;
		std::vector<int16_t> raw(L * 2);
		ring_copy(start, L, raw.data());
		std::vector<cf> in(L), X(L);
		const float qs = conj ? -1.0f : 1.0f;
		// DC removed (a raw DC bin would win the peak search on the RFNM capture)
		double si = 0, sq = 0;
		for(size_t i = 0; i < L; i++) {
			si += raw[2 * i];
			sq += raw[2 * i + 1];
		}
		const float mi = (float)(si / (double)L), mq = (float)(sq / (double)L);
		for(size_t i = 0; i < L; i++) in[i] = cf((raw[2 * i] - mi) * hann[i], (raw[2 * i + 1] - mq) * qs * hann[i]);
		chfft.run(in.data(), X.data());
		std::vector<float> P(L);
		size_t kp = 0;
		float pm = -1;
		double tot = 0;
		for(size_t k = 0; k < L; k++) {
			P[k] = std::norm(X[k]);
			tot += P[k];
			if(P[k] > pm) {
				pm = P[k];
				kp = k;
			}
		}
		const double df = cfg.fs_hz / (double)L;
		int W = (int)(0.6e6 / df);
		double num = 0, den = 0;
		for(int d = -W; d <= W; d++) {
			long k = ((long)kp + d) % (long)L;
			if(k < 0) k += L;
			double f = (k < (long)L / 2) ? k * df : ((double)k - (double)L) * df;
			num += f * P[k];
			den += P[k];
		}
		quality = tot > 0 ? den / tot : 0;   // fraction of the power inside the +-0.6 MHz window
		return den > 0 ? num / den : 0;
	}

	static int channel_of(double f_hz) {
		double x = (f_hz - FHSS_FREQ_START_HZ) / FHSS_FREQ_STEP_HZ;
		int ch = (int)llround(x);
		if(ch < 0 || ch >= FHSS_NUM_CHANNELS) return -1;
		if(std::fabs(x - ch) > 0.5) return -1;
		return ch;
	}

	// mix + resample one raw segment to os * bw, DC removed, conj optional
	void extract(const int16_t *raw, size_t n, double offset_hz, bool conj, std::vector<cf> &out) {
		// rational ratio out/in = (os * bw) / fs
		uint64_t a = (uint64_t)llround(cfg.os * ELRS_BW_HZ), b = (uint64_t)llround(cfg.fs_hz);
		uint64_t g = std::gcd(a, b);
		resampler rs((unsigned)(a / g), (unsigned)(b / g), cfg.fs_hz, offset_hz, RS_TAPS, RS_CUTOFF_HZ);
		double si = 0, sq = 0;
		for(size_t i = 0; i < n; i++) {
			si += raw[2 * i];
			sq += raw[2 * i + 1];
		}
		const float mi = (float)(si / (double)n), mq = (float)(sq / (double)n);
		const float scale = 1.0f / 32768.0f, qs = conj ? -scale : scale;
		std::vector<cf> t(4096);
		out.clear();
		out.reserve(n * (a / g) / (b / g) + 16);
		for(size_t off = 0; off < n; off += t.size()) {
			size_t m = std::min(t.size(), n - off);
			for(size_t i = 0; i < m; i++) t[i] = cf((raw[2 * (off + i)] - mi) * scale, (raw[2 * (off + i) + 1] - mq) * qs);
			rs.process(t.data(), m, out);
		}
	}

	void process_pending(const pending_burst &pb) {
		// snapshot the knobs (never hold the mutex into the callback)
		bool conj, auto_conj;
		int forced;
		double h0, h1;
		int hch;
		{
			std::lock_guard<std::mutex> g(knob);
			conj = cfg.conj;
			auto_conj = cfg.auto_conj;
			forced = cfg.rate_index;
			h0 = hint_t0;
			h1 = hint_t1;
			hch = hint_ch;
		}
		int idx = forced ? forced : pb.rate_index;
		const rate_params *rp = rate_by_index(idx);
		if(!rp) {
			std::lock_guard<std::mutex> g(knob);
			st.bursts_no_rate++;
			return;
		}
		const double fs = cfg.fs_hz;
		const uint64_t pre = (uint64_t)llround(cfg.pre_s * fs);
		uint64_t seg_start = pb.onset > pre ? pb.onset - pre : 0;
		double t_sym = rp->t_sym_s();
		size_t seg_len = (size_t)llround((rp->preamble_len + 8 + rp->n_sym + 6) * t_sym * fs) + (size_t)pre;
		if(seg_start < ring_valid_from || seg_start + seg_len > ring_end) {
			std::lock_guard<std::mutex> g(knob);
			st.ring_overruns++;
			return;
		}
		rx_burst b;
		b.sample_start = pb.onset;
		b.t_start_s = (double)pb.onset / fs;
		b.ts_ns = ts_ns >= 0 ? ts_ns + (int64_t)llround(((double)pb.onset - (double)ts_index) / fs * 1e9) : -1;
		b.duration_s = pb.dur_s;
		b.rate_index = idx;
		b.channel_predicted = (hch >= 0 && b.t_start_s >= h0 && b.t_start_s < h1) ? hch : -1;
		b.rssi_db = floor_level > 0 ? 10.0f * log10f((float)(pb.env_mean / floor_level)) : 0.0f;
		b.snr_db = 0;

		std::vector<int16_t> raw(seg_len * 2);
		ring_copy(seg_start, seg_len, raw.data());
		frame_sync &fsy = sync_for(idx);
		std::vector<cf> nb;
		std::vector<uint16_t> syms;
		auto attempt = [&](bool cj) -> bool {
			double q = 0;
			double off = estimate_offset_hz(pb.onset, pb.end, cj, q);
			int ch = channel_of(cfg.fc_hz + off);
			double f_demod = cfg.fc_hz + off;
			int ch_used = ch;
			if(b.channel_predicted >= 0) {
				double fp = FHSS_FREQ_START_HZ + b.channel_predicted * FHSS_FREQ_STEP_HZ;
				if(ch < 0 || std::fabs(cfg.fc_hz + off - fp) <= 1.0e6) ch_used = b.channel_predicted;
			}
			if(ch_used >= 0) f_demod = FHSS_FREQ_START_HZ + ch_used * FHSS_FREQ_STEP_HZ;
			b.freq_offset_hz = off;
			b.channel = ch_used;
			b.conj = cj;
			extract(raw.data(), seg_len, f_demod - cfg.fc_hz, cj, nb);
			b.sync = fsy.run(nb.data(), nb.size(), syms);
			return b.sync.found;
		};
		bool found = attempt(conj);
		if(!found && auto_conj) {
			if(attempt(!conj)) {
				found = true;
				std::lock_guard<std::mutex> g(knob);
				cfg.conj = !conj;   // sticky: the other sign decodes
				if(pub) pub->conj = cfg.conj;
			}
		}
		b.symbols = found ? syms : std::vector<uint16_t>();
		b.snr_db = b.sync.snr_db;
		{
			std::lock_guard<std::mutex> g(knob);
			st.demod_attempts++;
			if(found) st.demod_found++;
			st.last_channel = b.channel;
		}
		if(cb) cb(b);
	}

	void feed(const int16_t *iq, size_t count, uint64_t sample_index, int64_t tsns) {
		auto t0 = std::chrono::steady_clock::now();
		if(!cpu_started) {
			cpu_started = true;
			cpu_t0 = t0;
		}
		if(!have_data || sample_index != ring_end) {
			have_data = true;
			reset_stream(sample_index);
		}
		if(tsns >= 0) {
			ts_ns = tsns;
			ts_index = sample_index;
		}
		// ring write (wrap-aware); blocks larger than the ring keep only their tail
		if(count > 0) {
			const int16_t *src = iq;
			size_t n = count;
			if(n > ring_len) {
				src += 2 * (n - ring_len);
				n = (size_t)ring_len;
			}
			size_t pos = (size_t)((ring_end + (count - n)) % ring_len);
			size_t first = (size_t)std::min<uint64_t>(n, ring_len - pos);
			memcpy(&ring[pos * 2], src, first * 4);
			if(first < n) memcpy(&ring[0], src + first * 2, (n - first) * 4);
			ring_end += count;
			if(ring_end > ring_len) ring_valid_from = std::max(ring_valid_from, ring_end - ring_len);
		}
		// envelope blocks + burst state machine. The knob mutex is held for this loop (it
		// touches st.* and cfg.rate_index and never calls out); released before any on_burst.
		env_out.clear();
		env.process(iq, count, env_out);
		std::unique_lock<std::mutex> lk(knob);
		st.samples += count;
		for(size_t i = 0; i < env_out.size(); i++) {
			uint32_t v = env_out[i];
			uint64_t s = env_next_start + (uint64_t)i * blk;
			// the floor history takes off-burst blocks only (percentile stays honest at any
			// duty cycle); a too-long "burst" (skip_until_off) feeds it again so a raised
			// noise floor is followed instead of counted as an endless burst
			if(!in_burst) update_floor(v);
			// Schmitt trigger: enter high, sustain low (hysteresis). In a burst the constant-
			// envelope signal may hover between the two factors; without the lower sustain gate
			// a marginal-SNR burst chops into sub-1 ms fragments -> false too_short rejects.
			double thr_hi = floor_level * cfg.env_thresh_factor + margin;
			double thr_lo = floor_level * cfg.env_sustain_factor + margin;
			bool on = (double)v > (in_burst ? thr_lo : thr_hi);
			if(skip_until_off) {
				if(!on) skip_until_off = false;
				continue;
			}
			if(!in_burst) {
				if(on) {
					in_burst = true;
					burst_onset = s;
					burst_blocks = 1;
					burst_env_sum = v;
				}
				continue;
			}
			if(on) {
				burst_blocks++;
				burst_env_sum += v;
				if((double)burst_blocks * blk / cfg.fs_hz > cfg.max_burst_s) {
					st.bursts++;
					st.bursts_too_long++;
					in_burst = false;
					skip_until_off = true;
				}
				continue;
			}
			// burst end
			in_burst = false;
			st.bursts++;
			double dur = (double)burst_blocks * blk / cfg.fs_hz;
			if(dur < cfg.min_burst_s) {
				st.bursts_too_short++;
				continue;
			}
			pending_burst pb;
			pb.onset = burst_onset;
			pb.end = burst_onset + (uint64_t)burst_blocks * blk;
			pb.dur_s = dur;
			pb.env_mean = burst_env_sum / (double)burst_blocks;
			int forced = cfg.rate_index;
			pb.rate_index = forced ? forced : rate_index_from_duration(dur);
			const rate_params *rp = rate_by_index(pb.rate_index);
			if(!rp) {
				st.bursts_no_rate++;
				continue;
			}
			uint64_t pre = (uint64_t)llround(cfg.pre_s * cfg.fs_hz);
			uint64_t seg_start = pb.onset > pre ? pb.onset - pre : 0;
			pb.need_end = seg_start + (uint64_t)llround((rp->preamble_len + 8 + rp->n_sym + 6) * rp->t_sym_s() * cfg.fs_hz) + pre;
			pend.push_back(pb);
		}
		env_next_start += (uint64_t)env_out.size() * blk;
		lk.unlock();

		// demodulate every pending burst whose segment is complete in the ring
		size_t done = 0;
		for(; done < pend.size(); done++) {
			if(pend[done].need_end > ring_end) break;
			process_pending(pend[done]);
		}
		if(done) pend.erase(pend.begin(), pend.begin() + done);

		// cpu accounting (1 s IIR of the busy fraction)
		auto t1 = std::chrono::steady_clock::now();
		busy_s += std::chrono::duration<double>(t1 - t0).count();
		double wall = std::chrono::duration<double>(t1 - cpu_t0).count();
		if(wall >= 1.0) {
			double frac = busy_s / wall;
			std::lock_guard<std::mutex> g(knob);
			st.cpu_busy = st.cpu_busy > 0 ? 0.5 * st.cpu_busy + 0.5 * frac : frac;
			busy_s = 0;
			cpu_t0 = t1;
		}
	}
};

rx_chain::rx_chain(const rx_chain_config &cfg) : p_(new impl(cfg)), cfg_(cfg) { p_->pub = &cfg_; }
rx_chain::~rx_chain() { delete p_; }

void rx_chain::feed(const int16_t *iq, size_t count, uint64_t sample_index, int64_t ts_ns) {
	p_->feed(iq, count, sample_index, ts_ns);
}

void rx_chain::set_on_burst(std::function<void(const rx_burst &)> cb) {
	p_->cb = std::move(cb);
}

void rx_chain::set_channel_hint(double t0_s, double t1_s, int channel) {
	std::lock_guard<std::mutex> g(p_->knob);
	p_->hint_t0 = t0_s;
	p_->hint_t1 = t1_s;
	p_->hint_ch = (channel >= 0 && channel < FHSS_NUM_CHANNELS) ? channel : -1;
}

void rx_chain::set_rate_index(int idx) {
	std::lock_guard<std::mutex> g(p_->knob);
	p_->cfg.rate_index = (idx == 0 || rate_by_index(idx)) ? idx : 0;
	cfg_.rate_index = p_->cfg.rate_index;
}

void rx_chain::set_conj(bool conj) {
	std::lock_guard<std::mutex> g(p_->knob);
	p_->cfg.conj = conj;
	cfg_.conj = conj;
}

void rx_chain::get_stats(stats &out) const {
	std::lock_guard<std::mutex> g(p_->knob);
	out = p_->st;
	out.floor_level = p_->floor_level;
}

} // namespace elrsd

// cellsync: acquisition + per-cell tracking. See tracker.h for the model.
#include "tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <cstring>

namespace cellsync {

// ---- small helpers -----------------------------------------------------------------

static inline const cf &bin_at(const cf *bins, int rel) {
	return bins[(rel + FFT_N) % FFT_N];
}

// 62-bin extraction in sequence order n=0..61 (k=-31..-1, +1..+31)
static void take62(const cf *bins, cf *out) {
	for(int n = 0; n < 62; n++) {
		int k = n < 31 ? n - 31 : n - 30;
		out[n] = bin_at(bins, k);
	}
}

static void ifft(const fft_plan &f, const cf *in, cf *out) {
	unsigned n = f.size();
	std::vector<cf> tmp(in, in + n);
	for(auto &v : tmp) {
		v = std::conj(v);
	}
	f.run(tmp.data(), out);
	for(unsigned i = 0; i < n; i++) {
		out[i] = std::conj(out[i]) * (1.0f / n);
	}
}

// element-wise complex product out[i] = a[i] * b[i] (NEON 4-wide on aarch64; the
// scalar loop stays the x86 A/B reference)
static void cmul_arrays(cf *out, const cf *a, const cf *b, size_t n) {
#ifdef __aarch64__
	const float *ap = reinterpret_cast<const float *>(a);
	const float *bp = reinterpret_cast<const float *>(b);
	float *op = reinterpret_cast<float *>(out);
	size_t i = 0;
	for(; i + 4 <= n; i += 4) {
		float32x4x2_t av = vld2q_f32(ap + 2 * i);
		float32x4x2_t bv = vld2q_f32(bp + 2 * i);
		float32x4x2_t ov;
		ov.val[0] = vfmsq_f32(vmulq_f32(av.val[0], bv.val[0]), av.val[1], bv.val[1]);
		ov.val[1] = vfmaq_f32(vmulq_f32(av.val[0], bv.val[1]), av.val[1], bv.val[0]);
		vst2q_f32(op + 2 * i, ov);
	}
	for(; i < n; i++) {
		out[i] = a[i] * b[i];
	}
#else
	for(size_t i = 0; i < n; i++) {
		out[i] = a[i] * b[i];
	}
#endif
}

// all 336 SSS hypotheses, generated once
static const std::vector<std::vector<float>> &sss_cache(int nid2) {
	static std::vector<std::vector<float>> cache[3];
	if(cache[nid2].empty()) {
		cache[nid2].resize(336);
		for(int nid1 = 0; nid1 < 168; nid1++) {
			for(int half = 0; half < 2; half++) {
				cache[nid2][nid1 * 2 + half] = sss_seq(nid1, nid2, half);
			}
		}
	}
	return cache[nid2];
}

// coherent SSS metrics for all 336 (nid1, half) hypotheses, PSS-equalized;
// returns false when the symbol carries no usable energy
static bool sss_metrics(const fft_plan &f128, const cf *pss_time_sym, const cf *sss_time_sym, int nid2, float out[336]) {
	std::vector<cf> pb(FFT_N), sb(FFT_N);
	f128.run(pss_time_sym, pb.data());
	f128.run(sss_time_sym, sb.data());
	cf xp[62], xs[62], eq[62];
	take62(pb.data(), xp);
	take62(sb.data(), xs);
	std::vector<cf> pref = pss_freq(nid2);
	float pw = 0;
	for(int n = 0; n < 62; n++) {
		eq[n] = xs[n] * std::conj(xp[n]) * pref[n];   // = Xs * conj(H)
		pw += std::norm(eq[n]);
	}
	if(pw <= 0) {
		return false;
	}
	const auto &cache = sss_cache(nid2);
#ifdef __aarch64__
	// real-weight complex dot: deinterleave eq once, then 15x4 fused MACs per
	// hypothesis + a 2-lane scalar tail (62 = 15*4 + 2)
	float er[62], ei[62];
	for(int n = 0; n < 62; n++) {
		er[n] = eq[n].real();
		ei[n] = eq[n].imag();
	}
	for(int h = 0; h < 336; h++) {
		const float *d = cache[h].data();
		float32x4_t cr = vdupq_n_f32(0), ci = vdupq_n_f32(0);
		for(int n = 0; n < 60; n += 4) {
			float32x4_t dv = vld1q_f32(d + n);
			cr = vfmaq_f32(cr, dv, vld1q_f32(er + n));
			ci = vfmaq_f32(ci, dv, vld1q_f32(ei + n));
		}
		float sr = vaddvq_f32(cr) + d[60] * er[60] + d[61] * er[61];
		float si = vaddvq_f32(ci) + d[60] * ei[60] + d[61] * ei[61];
		out[h] = (sr * sr + si * si) / (pw * 62.0f);
	}
#else
	for(int h = 0; h < 336; h++) {
		cf corr(0, 0);
		const float *d = cache[h].data();
		for(int n = 0; n < 62; n++) {
			corr += eq[n] * d[n];
		}
		out[h] = std::norm(corr) / (pw * 62.0f);
	}
#endif
	return true;
}

// ---- per-cell tracker ------------------------------------------------------------------

class cell_tracker {
public:
	cell_tracker(sync_engine *e, int pci, int nid2, int half_at_pss, uint64_t t_pss, double t_origin_s)
		: eng_(e), pci_(pci), nid2_(nid2), half_(half_at_pss), t_f_((double)t_pss), t0_((double)t_pss),
		  origin_s_(t_origin_s) {
		// CRS positions/labels are identical for every cell bandwidth (m' = f(rel) only),
		// so the canonical nrb=6 layout descrambles any cell - cache both pilot symbols
		cpos_[0] = crs_positions(pci_, 6, 0, -36, 36);
		cpos_[1] = crs_positions(pci_, 6, 4, -36, 36);
	}

	int pci() const { return pci_; }
	int nid2() const { return nid2_; }
	double next_pss() const { return t_f_; }
	bool lost() const { return miss_ >= 20 || bad_; }

	// process the PSS occurrence expected at t_f_ plus the half-frame of CRS behind it;
	// caller guarantees ring data through t_f_ + SEARCH + FFT_N
	void step();

	// under eng_->meas_mtx_
	void export_meas(cell_meas &m, bool consume);
	void reset_phase_chains();

private:
	void crs_halfframe(double hf_base, int h0);
	void crs_symbol(double slot_base, int ns, int l);

	sync_engine *eng_;
	int pci_, nid2_, half_;
	double t_f_;                   // expected abs position of next PSS data start
	double t0_;                    // first PSS position (drift series origin)
	double origin_s_;
	uint64_t n_hf_ = 0;            // PSS occurrences seen
	int miss_ = 0;
	float snr_ = 0;                // smoothed normalized corr snr (linear)

	// timing-drift series: 1-second buckets of (t_sec, pos_err_samples)
	double bucket_sum_ = 0;
	int bucket_cnt_ = 0;
	std::deque<std::pair<double, double>> drift_;
	uint64_t drift_epoch_seen_ = 0;               // engine drift_epoch_ we last synced to

	// canonical pilot layout (bandwidth-invariant) + wide/narrow occupancy accumulators
	std::vector<crs_pos> cpos_[2];
	double edge_pw_ = 0, mid_pw_ = 0;
	int occ_syms_ = 0;

	// CRS cross-slot chains, one per pilot symbol index (l=0 -> 0, l=4 -> 1)
	cf prev_v_[2][16];
	int prev_ns_[2] = { -99, -99 };
	int prev_cnt_[2] = { 0, 0 };
	cf acc_vec_ = cf(0, 0);        // sum of per-pair correlations (angle = fine CFO phase step)
	double acc_absd_ = 0;
	uint64_t pairs_ = 0;

	// PCI confirmation: a provisionally-accepted wrong PCI descrambles CRS into noise,
	// so its pair coherence stays near zero - kill it instead of feeding the loop
	cf conf_vec_ = cf(0, 0);
	double conf_absd_ = 0;
	uint64_t conf_pairs_ = 0;
	bool confirmed_ = false;
	bool bad_ = false;

	cf cp_acc_ = cf(0, 0);         // decaying CP autocorrelation (wrap disambiguation)

	friend class sync_engine;
};

static constexpr int SEARCH = 24;
// acquisition window: 4 frames = 8 PSS occurrences, so a weak cell detected on only
// half its occurrences still forms a min_train-sized train with SSS to accumulate
static constexpr int ACQ_WIN = 4 * FRAME_SAMP;

void cell_tracker::step() {
	sync_engine *e = eng_;
	// local correlation around the predicted position
	uint64_t w0 = (uint64_t)llround(t_f_) - SEARCH;
	int nlag = 2 * SEARCH + 1;
	const std::vector<cf> &tpl = e->pss_t_[nid2_];
	float best = 0;
	int besti = 0;
	std::vector<float> met(nlag);
#ifdef __aarch64__
	// conj-template dot over 128 complex, 4-wide: the steady-state tracker cost
	// scales with tracked cells (49 lags x 128 taps per half-frame per cell)
	float tr[FFT_N], ti[FFT_N];
	for(int m = 0; m < FFT_N; m++) {
		tr[m] = tpl[m].real();
		ti[m] = tpl[m].imag();
	}
	for(int i = 0; i < nlag; i++) {
		const float *xp = reinterpret_cast<const float *>(e->at(w0 + i));
		float32x4_t cr = vdupq_n_f32(0), ci = vdupq_n_f32(0), pv = vdupq_n_f32(0);
		for(int m = 0; m < FFT_N; m += 4) {
			float32x4x2_t xv = vld2q_f32(xp + 2 * m);
			float32x4_t trv = vld1q_f32(tr + m), tiv = vld1q_f32(ti + m);
			cr = vfmaq_f32(vfmaq_f32(cr, xv.val[0], trv), xv.val[1], tiv);
			ci = vfmaq_f32(vfmsq_f32(ci, xv.val[0], tiv), xv.val[1], trv);
			pv = vfmaq_f32(vfmaq_f32(pv, xv.val[0], xv.val[0]), xv.val[1], xv.val[1]);
		}
		float pw = vaddvq_f32(pv);
		float re = vaddvq_f32(cr), im = vaddvq_f32(ci);
		met[i] = pw > 0 ? (re * re + im * im) / pw : 0;
		if(met[i] > best) {
			best = met[i];
			besti = i;
		}
	}
#else
	for(int i = 0; i < nlag; i++) {
		const cf *x = e->at(w0 + i);
		cf corr(0, 0);
		float pw = 0;
		for(int m = 0; m < FFT_N; m++) {
			corr += x[m] * std::conj(tpl[m]);
			pw += std::norm(x[m]);
		}
		met[i] = pw > 0 ? std::norm(corr) / pw : 0;
		if(met[i] > best) {
			best = met[i];
			besti = i;
		}
	}
#endif
	double hf_base = t_f_ - 832.0 - HALF_FRAME_SAMP;   // previous, fully-buffered half-frame
	int h0_prev = half_ ? 0 : 10;                       // slots of the *previous* occurrence

	if(best < 0.008f || besti == 0 || besti == nlag - 1) {
		miss_++;
	} else {
		miss_ = 0;
		float d = 0;
		float den = met[besti - 1] - 2 * met[besti] + met[besti + 1];
		if(den < 0) {
			d = 0.5f * (met[besti - 1] - met[besti + 1]) / den;
		}
		double p_meas = (double)w0 + besti + d;
		double err = p_meas - t_f_;
		t_f_ += 0.1 * err;                              // slow timing follow
		snr_ = 0.95f * snr_ + 0.05f * (best / (1.0f - best));

		// drift series: nominal-removed position, bucketed per second. A DCO actuation
		// (engine drift_epoch_ bumped) restarts the window: a slope regressed across an
		// actuation step under-reports the new pull by the pre-step fraction of the window.
		uint64_t ep = eng_->drift_epoch_.load(std::memory_order_relaxed);
		if(ep != drift_epoch_seen_) {
			std::lock_guard<std::mutex> lk(eng_->meas_mtx_);   // export_meas regresses drift_
			drift_.clear();
			bucket_sum_ = 0;
			bucket_cnt_ = 0;
			drift_epoch_seen_ = ep;
		}
		double x = p_meas - (t0_ + (double)n_hf_ * HALF_FRAME_SAMP);
		bucket_sum_ += x;
		bucket_cnt_++;
		if(bucket_cnt_ >= 200) {                        // 1 s of half-frames
			drift_.emplace_back(origin_s_ + (double)n_hf_ / 200.0, bucket_sum_ / bucket_cnt_);
			bucket_sum_ = 0;
			bucket_cnt_ = 0;
			if(drift_.size() > 600) {
				drift_.pop_front();
			}
		}
		crs_halfframe(hf_base, h0_prev);
	}
	t_f_ += HALF_FRAME_SAMP;
	half_ ^= 1;
	n_hf_++;
}

void cell_tracker::crs_halfframe(double hf_base, int h0) {
	if(hf_base < (double)eng_->base_ + 1) {
		return;
	}
	cp_acc_ *= 0.95f;   // ~0.1 s memory: enough averaging to disambiguate +-1 kHz wraps
	for(int j = 0; j < 10; j++) {
		int ns = (h0 + j) % 20;
		double slot_base = hf_base + j * (double)SLOT_SAMP;
		// continuous CP autocorrelation: an unambiguous (+-7.5 kHz) but coarse view of
		// the residual CFO, used to pick the right 2 kHz cycle of the CRS phase slope
		uint64_t sb = (uint64_t)llround(slot_base);
		if(sb >= eng_->base_ && sb + SLOT_SAMP + FFT_N < eng_->avail()) {
			for(int l = 0; l < SYMS_PER_SLOT; l++) {
				uint64_t s = sb + (l == 0 ? 0 : 138 + (l - 1) * 137);
				const cf *x = eng_->at(s);
				for(int i = 3; i < cp_len(l); i++) {
					cp_acc_ += x[i + FFT_N] * std::conj(x[i]);
				}
			}
		}
		crs_symbol(slot_base, ns, 0);
		crs_symbol(slot_base, ns, 4);
	}
}

void cell_tracker::crs_symbol(double slot_base, int ns, int l) {
	sync_engine *e = eng_;
	uint64_t start = (uint64_t)llround(slot_base) + sym_data_off(l);
	if(start < e->base_ || start + FFT_N > e->avail()) {
		return;
	}
	cf bins[FFT_N];
	e->fft128_.run(e->at(start), bins);

	// wide/narrow: energy beyond the center 6 RB vs inside it (informational only)
	for(int rel = 41; rel <= 62; rel += 3) {
		edge_pw_ += std::norm(bin_at(bins, rel)) + std::norm(bin_at(bins, -rel));
	}
	for(int rel = 5; rel <= 26; rel += 3) {
		mid_pw_ += std::norm(bin_at(bins, rel)) + std::norm(bin_at(bins, -rel));
	}
	occ_syms_++;

	// descramble with the bandwidth-invariant labels and correlate against the previous slot
	int li = l == 0 ? 0 : 1;
	const auto &pos = cpos_[li];
	int np = std::min((int)pos.size(), 16);
	cf v[16], ref[16];
	crs_seq(pci_, ns, l, pos[0].mprime, np, ref);   // m' runs consecutively in-window
	for(int i = 0; i < np; i++) {
		v[i] = bin_at(bins, pos[i].rel) * std::conj(ref[i]);
	}
	if(prev_ns_[li] == (ns + 19) % 20 && prev_cnt_[li] == np) {
		cf d(0, 0);
		for(int i = 0; i < np; i++) {
			d += v[i] * std::conj(prev_v_[li][i]);
		}
		std::lock_guard<std::mutex> lk(eng_->meas_mtx_);
		acc_vec_ += d;
		acc_absd_ += std::abs(d);
		pairs_++;
		if(!confirmed_ && !bad_) {
			conf_vec_ += d;
			conf_absd_ += std::abs(d);
			if(++conf_pairs_ >= 300) {
				double coh = std::abs(conf_vec_) / std::max(conf_absd_, 1e-12);
				if(coh < 0.15) {
					bad_ = true;
					if(eng_->verbose_) {
						printf("cellsync: PCI %d failed CRS confirmation (coherence %.2f) - dropping\n", pci_, coh);
						fflush(stdout);
					}
				} else {
					confirmed_ = true;
				}
			}
		}
	}
	for(int i = 0; i < np; i++) {
		prev_v_[li][i] = v[i];
	}
	prev_ns_[li] = ns;
	prev_cnt_[li] = np;
}

void cell_tracker::export_meas(cell_meas &m, bool consume) {
	m.pci = pci_;
	m.wide = occ_syms_ >= 100 && mid_pw_ > 0 && edge_pw_ / mid_pw_ > 0.25;
	m.snr_db = 10.0f * log10f(std::max(snr_, 1e-4f));
	double fine = 0;
	double coh = 0;
	if(pairs_ > 0 && std::abs(acc_vec_) > 0) {
		fine = std::arg(acc_vec_) / (2.0 * M_PI * 5e-4);   // phase per 0.5 ms, +-1 kHz window
		coh = std::abs(acc_vec_) / std::max(acc_absd_, 1e-12);
		// the CRS slope is precise but 2 kHz-cyclic; the CP view is coarse but
		// unambiguous - place the fine estimate on the CP's cycle. Only once the CP
		// accumulator has matured (~0.2 s): an early noisy cycle pick walks the loop
		// away in 2 kHz steps.
		if(n_hf_ >= 40 && std::abs(cp_acc_) > 0) {
			double cp_cfo = std::arg(cp_acc_) / (2.0 * M_PI) * SCS;
			fine += 2000.0 * llround((cp_cfo - fine) / 2000.0);
		}
	}
	m.cfo_hz = eng_->coarse_cfo_ + fine;
	m.offset_ppb = -m.cfo_hz / eng_->f_lo_ * 1e9;
	m.slots = pairs_;
	m.weight = (double)pairs_ * std::min((double)snr_, 100.0) * coh;
	m.age_s = miss_ * 0.005;

	// timing-drift slope over the bucket series (needs >= 30 s of track)
	m.tim_valid = false;
	if(drift_.size() >= 30) {
		double n = (double)drift_.size(), sx = 0, sy = 0, sxx = 0, sxy = 0;
		for(const auto &p : drift_) {
			sx += p.first;
			sy += p.second;
			sxx += p.first * p.first;
			sxy += p.first * p.second;
		}
		double den = n * sxx - sx * sx;
		if(den > 0) {
			double slope = (n * sxy - sx * sy) / den;    // samples per second
			m.tim_ppb = slope / FS_SYNC * 1e9;
			m.tim_valid = true;
		}
	}
	if(consume) {
		acc_vec_ = cf(0, 0);
		acc_absd_ = 0;
		pairs_ = 0;
	}
}

void cell_tracker::reset_phase_chains() {
	prev_ns_[0] = prev_ns_[1] = -99;
	acc_vec_ = cf(0, 0);
	acc_absd_ = 0;
	pairs_ = 0;
}

// ---- engine ---------------------------------------------------------------------------

sync_engine::sync_engine(double f_lo_hz, bool verbose)
	: f_lo_(f_lo_hz), verbose_(verbose), fft512_(512), fft128_(FFT_N) {
	for(int nid2 = 0; nid2 < 3; nid2++) {
		pss_t_.push_back(pss_time(nid2));
		std::vector<cf> padded(512, cf(0, 0));
		std::copy(pss_t_[nid2].begin(), pss_t_[nid2].end(), padded.begin());
		std::vector<cf> tf(512);
		fft512_.run(padded.data(), tf.data());
		for(auto &v : tf) {
			v = std::conj(v);
		}
		pss_f512_.push_back(std::move(tf));
	}
}

sync_engine::~sync_engine() = default;

void sync_engine::flush_acquire() {
	// caller = the feed thread, so acq_/cells_/tracking_ need no lock (same rule as
	// feed/process_half_frames); only the measurement snapshot is shared
	acq_.clear();
	cells_.clear();
	tracking_ = false;
	coarse_cfo_ = 0;      // same rule as the all-cells-lost path: a stale coarse guess
	                      // derotates the real cell straight out of the detection grid
	next_acq_ = 0;
	next_augment_ = 0;
	reset_drift();
	std::lock_guard<std::mutex> lk(meas_mtx_);
	snapshot_.clear();
}

void sync_engine::reset(double f_lo_hz) {
	flush_acquire();
	f_lo_ = f_lo_hz;
	base_ += ring_.size();
	ring_.clear();
	proc_ = base_;
	nco_phase_ = 0;
	nco_step_cached_ = 0;
	nco_cur_ = cf(1.0f, 0.0f);
	nco_k_ = 0;
	pci_bl_.clear();      // a different carrier: its PCI verdicts do not transfer
}

void sync_engine::feed(const cf *x, size_t n) {
	size_t w0 = ring_.size();
	ring_.insert(ring_.end(), x, x + n);
	total_in_ += n;
	// common derotation by the engine coarse CFO (phase-continuous across updates).
	// Phasor recurrence, not per-sample cos/sin: at 1.92 MSps the libm pair here was
	// a second whole-core-class cost on the A53 (see the nco class note). Re-seeded
	// from the exact double phase every 4096 samples and on any coarse-CFO change.
	if(coarse_cfo_ != 0) {
		double step = -2.0 * M_PI * coarse_cfo_ / FS_SYNC;
		if(step != nco_step_cached_) {
			nco_step_cached_ = step;
			nco_stepper_ = cf((float)cos(step), (float)sin(step));
			nco_cur_ = cf((float)cos(nco_phase_), (float)sin(nco_phase_));
			nco_k_ = 0;
		}
		for(size_t i = w0; i < ring_.size(); i++) {
			ring_[i] *= nco_cur_;
			nco_cur_ *= nco_stepper_;
			nco_phase_ += step;
			if(++nco_k_ >= 4096) {
				nco_phase_ = std::fmod(nco_phase_, 2.0 * M_PI);
				nco_cur_ = cf((float)cos(nco_phase_), (float)sin(nco_phase_));
				nco_k_ = 0;
			}
		}
	}

	if(!tracking_) {
		// stride >> window: the A53 cannot correlate every sample in real time, and the
		// class accumulators make skipped stretches free - lock latency stays seconds.
		// The stride is paced by the MEASURED round cost: the round runs inline on the
		// capture thread, so its wall time backs up the transport queue (~1.1 s deep)
		// and anything past that is dropped + glued. 2x cost caps acquire duty at 50%
		// and keeps the stream real-time on the A53 (receipted 2026-07-21: fixed
		// 0.16 s stride vs ~1.2 s rounds = permanent overrun, 22% duty, no lock ever).
		if(avail() >= next_acq_ && avail() >= base_ + 2 * FRAME_SAMP + 1000) {
			auto tr0 = std::chrono::steady_clock::now();
			acquire();
			proc_ = avail();
			double cost_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - tr0).count();
			double stride_s = std::max(0.16, 2.0 * cost_s);
			next_acq_ = avail() + (uint64_t)(stride_s * FS_SYNC);
		}
	} else {
		process_half_frames();
		if(total_in_ >= next_augment_) {
			next_augment_ = total_in_ + (uint64_t)(2.5 * FS_SYNC);
			if((int)cells_.size() < max_cells) {
				augment_scan();
			}
		}
		fold_fine_into_coarse();
	}
	trim();
}

void sync_engine::process_half_frames() {
	bool progressed = true;
	while(progressed) {
		progressed = false;
		for(auto &c : cells_) {
			while((double)avail() > c->next_pss() + SEARCH + FFT_N + 4 && !c->lost()) {
				c->step();
				progressed = true;
			}
		}
		cells_.erase(std::remove_if(cells_.begin(), cells_.end(),
		                            [&](const std::unique_ptr<cell_tracker> &c) {
			if(c->lost()) {
				// a CRS-coherence kill means the provisional PCI was WRONG - blacklist
				// it so re-acquisition explores the next candidate instead of looping
				if(c->bad_) {
					pci_bl_.emplace_back(c->pci(), total_in_);
				}
				if(verbose_) {
					printf("cellsync: lost PCI %d%s\n", c->pci(), c->bad_ ? " (CRS reject, blacklisted)" : "");
				}
			}
			return c->lost();
		}), cells_.end());
		if(cells_.empty()) {
			tracking_ = false;
			// a dead first cell's coarse guess must not poison re-acquisition: left in
			// place it compounds across CRS-kill retries (observed walking to +100 kHz)
			// and derotates the real cell straight out of the detection grid
			coarse_cfo_ = 0;
			acq_.clear();
			return;
		}
	}
}

void sync_engine::xcorr_win3(uint64_t abs0, size_t n, std::vector<float> c[3]) {
	// normalized |corr|^2 in [0,1] for lags abs0 .. abs0+n-128, all three templates.
	// One forward FFT and one power prefix per block serve every template.
	size_t nl = n - FFT_N + 1;
	for(int t = 0; t < 3; t++) {
		c[t].assign(nl, 0.0f);
	}
	std::vector<double> pw(n + 1, 0.0);
	const cf *x = at(abs0);
	for(size_t i = 0; i < n; i++) {
		pw[i + 1] = pw[i] + std::norm(x[i]);
	}
	std::vector<cf> blk(512), X(512), P(512), y(512);
	for(size_t off = 0; off < nl; off += 385) {
		size_t take = std::min<size_t>(512, n - off);
		std::copy(x + off, x + off + take, blk.begin());
		std::fill(blk.begin() + take, blk.end(), cf(0, 0));
		fft512_.run(blk.data(), X.data());
		size_t nvalid = std::min<size_t>(385, nl - off);
		for(int t = 0; t < 3; t++) {
			cmul_arrays(P.data(), X.data(), pss_f512_[t].data(), 512);
			ifft(fft512_, P.data(), y.data());
			for(size_t i = 0; i < nvalid; i++) {
				double p = pw[off + i + FFT_N] - pw[off + i];
				c[t][off + i] = p > 1e-9 ? (float)(std::norm(y[i]) / p) : 0.0f;
			}
		}
	}
}

// cluster per-nid2 correlation peaks into 5 ms-periodic trains; the ring window
// [w0, w0+wlen) must already be derotated to the frequency variant fg under test
void sync_engine::collect_trains(uint64_t w0, size_t wlen, double fg, std::vector<train> &out) {
	static constexpr int64_t TOL = 4;            // residue tolerance: sub-ppm drift + multipath jitter
	static constexpr size_t SLOT_PKS = 8;        // strongest local maxima kept per nid2 per 5 ms slot:
	                                             // a global cap lets the noise tail crowd out a weak
	                                             // cell's true peaks, a per-slot cap cannot
	std::vector<float> c3[3];
	xcorr_win3(w0, wlen, c3);
	struct pk {
		uint64_t pos;
		float met;
		bool used;
	};
	std::vector<pk> pks, kept;
	for(int nid2 = 0; nid2 < 3; nid2++) {
		std::vector<float> &c = c3[nid2];
		pks.clear();
		// every local maximum above threshold: no blanket skip after a hit - a weak
		// spike 40 samples ahead of a true peak must not shadow it (the per-slot cap
		// below, not a skip, is what bounds the noise flood)
		for(size_t i = 1; i + 1 < c.size(); i++) {
			if(c[i] > pss_thr && c[i] >= c[i - 1] && c[i] >= c[i + 1]) {
				pks.push_back({ w0 + i, c[i], false });
			}
		}
		std::sort(pks.begin(), pks.end(), [](const pk &a, const pk &b) { return a.met > b.met; });
		kept.clear();
		{
			size_t nslots = wlen / HALF_FRAME_SAMP + 1;
			std::vector<size_t> per_slot(nslots, 0);
			for(const pk &p : pks) {
				size_t sl = (size_t)(p.pos - w0) / HALF_FRAME_SAMP;
				if(per_slot[sl] < SLOT_PKS) {
					per_slot[sl]++;
					kept.push_back(p);
				}
			}
		}
		pks.swap(kept);                          // met-sorted order is preserved
		// greedy: strongest unused peak seeds a train; members are the strongest unused
		// peak per 5 ms slot whose residue matches the seed within TOL
		for(size_t s = 0; s < pks.size(); s++) {
			if(pks[s].used) {
				continue;
			}
			std::vector<size_t> members;
			std::vector<int64_t> mslot;
			for(size_t p = 0; p < pks.size(); p++) {
				if(pks[p].used) {
					continue;
				}
				int64_t d = (int64_t)pks[p].pos - (int64_t)pks[s].pos;
				int64_t k = (int64_t)llround((double)d / HALF_FRAME_SAMP);
				int64_t res = d - k * HALF_FRAME_SAMP;
				if(res > TOL || res < -TOL) {
					continue;
				}
				bool taken = false;
				for(int64_t ks : mslot) {
					if(ks == k) {
						taken = true;
						break;
					}
				}
				if(!taken) {                     // met-sorted, so the first claimant is the strongest
					members.push_back(p);
					mslot.push_back(k);
				}
			}
			if((int)members.size() >= min_train) {
				train t;
				t.nid2 = nid2;
				t.fg = fg;
				t.score = 0;
				for(size_t p : members) {
					pks[p].used = true;
					t.score += pks[p].met;
					t.occ.push_back(pks[p].pos);
				}
				std::sort(t.occ.begin(), t.occ.end());
				out.push_back(std::move(t));
			} else {
				pks[s].used = true;              // isolated peak: consumed, never a train
			}
		}
	}
}

// class accumulation: each (nid2, variant, 5 ms residue) candidate integrates
// PSS-equalized SSS metrics across acquire windows until the PCI decision separates
// statistically. CFO: no CP estimate here - it is unusable at deep-indoor SNR (and a
// PSS split-half phase is biased by the ZC time-frequency ambiguity). Instead every
// class accumulates ACQ_NF hypotheses tiling the variant's +-7.5 kHz cell, and the
// winning hypothesis IS the coarse CFO (quantized 2.5 kHz; the tracker's CP-guided
// fold absorbs the residual).
bool sync_engine::pci_blacklisted(int pci) {
	for(size_t i = 0; i < pci_bl_.size();) {
		if(total_in_ - pci_bl_[i].second > (uint64_t)(300.0 * FS_SYNC)) {
			pci_bl_.erase(pci_bl_.begin() + i);
		} else {
			i++;
		}
	}
	for(const auto &b : pci_bl_) {
		if(b.first == pci) {
			return true;
		}
	}
	return false;
}

size_t sync_engine::find_class(const train &t) const {
	for(size_t i = 0; i < acq_.size(); i++) {
		if(acq_[i].nid2 != t.nid2 || std::fabs(acq_[i].fg - t.fg) > 1.0) {
			continue;
		}
		uint64_t d = t.occ[0] > acq_[i].anchor ? t.occ[0] - acq_[i].anchor : acq_[i].anchor - t.occ[0];
		uint64_t r = d % HALF_FRAME_SAMP;
		if(r < 150 || r > HALF_FRAME_SAMP - 150) {
			return i;
		}
	}
	return acq_.size();
}

// detection only CREATES candidate classes - it never gates accumulation. A weak
// cell's train re-forms in maybe half the windows, but once its residue is known,
// every 5 ms grid position carries SSS evidence worth folding
void sync_engine::touch_class(const train &t) {
	size_t ci = find_class(t);
	if(ci == acq_.size()) {
		if(acq_.size() >= 24) {
			size_t oldest = 0;
			for(size_t i = 1; i < acq_.size(); i++) {
				if(acq_[i].last_train < acq_[oldest].last_train) {
					oldest = i;
				}
			}
			acq_.erase(acq_.begin() + oldest);
		}
		acq_class fresh;
		fresh.nid2 = t.nid2;
		fresh.fg = t.fg;
		fresh.anchor = t.occ[0];
		fresh.refpos = t.occ[0];
		fresh.prof.assign((size_t)ACQ_NF * ACQ_NPROF, 0.0f);
		fresh.comb.assign((size_t)ACQ_NF * 336, 0.0);
		acq_.push_back(std::move(fresh));
		ci = acq_.size() - 1;
	}
	acq_[ci].last_train = total_in_;
	acq_[ci].trains_seen++;
}

// fold every unprocessed 5 ms grid position of this class into its per-hypothesis SSS
// accumulators, then decide. The grid extrapolates from the newest refined position
// (refpos), so reference drift is followed and detection misses cost nothing.
bool sync_engine::fold_class(size_t ci, bool quiet) {
	acq_class &cl = acq_[ci];
	int mx = 0;
	for(int m = 0; m < ACQ_NF; m++) {
		mx = std::max(mx, cl.fused[m]);
	}
	if(mx >= 250) {
		cl.dead = true;                          // undecidable this deep (phantom or below
		return false;                            // the floor) - stop burning cycles on it
	}
	// alignment: detection anchors can sit hundreds of samples off the cell (ZC
	// ambiguity ridges), and at deep-indoor SNR a per-fold peak pick over a wide lag
	// range loses to noise maxima. So each fold ADDS its |corr|^2 profile to the
	// class accumulator and takes the SSS at the ACCUMULATED argmax - noncoherent
	// integration converges onto the true path within a few folds and stays there
	static constexpr int PRE = ACQ_REF + 137 + 8;             // SSS symbol reach at bi = -ACQ_REF
	static constexpr int POST = ACQ_REF + FFT_N + 64 + 8;
	uint64_t lo = base_ + PRE;
	if(cl.folded_until + 1 > lo) {
		lo = cl.folded_until + 1;
	}
	if(lo + POST > avail()) {
		return false;
	}
	const size_t seglen = PRE + POST;
	std::vector<cf> w;
	std::vector<float> pw(seglen + 1);
	float m1[336];
	const uint64_t ref0 = cl.refpos;
	int64_t k = (int64_t)llround(((double)lo - (double)ref0) / HALF_FRAME_SAMP) - 1;
	for(;; k++) {
		int64_t pos = (int64_t)ref0 + k * HALF_FRAME_SAMP;
		if(pos < (int64_t)lo) {
			continue;
		}
		if(pos + POST > (int64_t)avail()) {
			break;
		}
		int64_t j = (int64_t)llround((double)(pos - (int64_t)cl.anchor) / HALF_FRAME_SAMP);
		int flip = (int)(j & 1);                 // SSS half alternates every 5 ms
		for(int m = 0; m < ACQ_NF; m++) {
			double f = cl.fg + (m - ACQ_NF / 2) * 2500.0;
			w.assign(at(pos - PRE), at(pos - PRE) + seglen);
			// phasor recurrence: the per-sample sincos was measurable on the A53
			double step = -2.0 * M_PI * f / FS_SYNC;
			cf rot(1, 0);
			const cf rstep((float)cos(step), (float)sin(step));
			for(auto &v : w) {
				v *= rot;
				rot *= rstep;
			}
			pw[0] = 0;
			for(size_t i = 0; i < seglen; i++) {
				pw[i + 1] = pw[i] + std::norm(w[i]);
			}
			// FFT overlap-save correlation over the alignment span (~6x cheaper on the
			// A53 than the direct form; identical math to xcorr_win)
			float met[ACQ_NPROF];
			{
				std::vector<cf> blk(512), X(512), y(512);
				const std::vector<cf> &tf = pss_f512_[cl.nid2];
				for(int off = PRE - ACQ_REF; off <= PRE + ACQ_REF; off += 385) {
					size_t take = std::min<size_t>(512, seglen - (size_t)off);
					std::copy(w.begin() + off, w.begin() + off + take, blk.begin());
					std::fill(blk.begin() + take, blk.end(), cf(0, 0));
					fft512_.run(blk.data(), X.data());
					cmul_arrays(X.data(), X.data(), tf.data(), 512);
					ifft(fft512_, X.data(), y.data());
					int nv = std::min((int)take - FFT_N + 1, std::min(385, PRE + ACQ_REF - off + 1));
					for(int i2 = 0; i2 < nv; i2++) {
						float p = pw[off + i2 + FFT_N] - pw[off + i2];
						met[off + i2 - (PRE - ACQ_REF)] = p > 1e-9f ? std::norm(y[i2]) / p : 0;
					}
				}
			}
			float *pr = &cl.prof[(size_t)m * ACQ_NPROF];
			int bi = 0;
			float bacc = -1;
			for(int i = -ACQ_REF; i <= ACQ_REF; i++) {
				pr[i + ACQ_REF] += met[i + ACQ_REF];
				if(pr[i + ACQ_REF] > bacc) {
					bacc = pr[i + ACQ_REF];
					bi = i;
				}
			}
			if(!sss_metrics(fft128_, &w[PRE + bi], &w[PRE + bi - 137], cl.nid2, m1)) {
				continue;
			}
			double *acc = &cl.comb[(size_t)m * 336];
			for(int h = 0; h < 336; h++) {
				acc[h] += m1[h ^ flip];          // fold to anchor parity
			}
			cl.fused[m]++;
			cl.lastp[m] = (double)pos + bi;
			cl.lastj[m] = j;
		}
		cl.folded_until = (uint64_t)pos;
	}

	// decision on the accumulated averages. The metric floor is NOT 1/62: the
	// PSS-equalized product noise is heavy-tailed and floors near 2/62, and the floor
	// shifts with channel statistics - so the signal is measured as the EXCESS of a
	// hypothesis over the median of its own 336-row, and the margin is taken on
	// excesses against the m-family-DISJOINT runner-up (sharing m0 or m1 with the
	// truth structurally scores ~half the true excess)
	int best_m = -1, best_h = -1;
	double best_sig = -1e9, best_v = 0, best_med = 0;
	std::vector<double> row(336);
	for(int m = 0; m < ACQ_NF; m++) {
		if(cl.fused[m] < 2) {
			continue;
		}
		for(int h = 0; h < 336; h++) {
			row[h] = cl.comb[(size_t)m * 336 + h] / cl.fused[m];
		}
		std::vector<double> srt(row);
		std::nth_element(srt.begin(), srt.begin() + 168, srt.end());
		double med = srt[168];
		for(int h = 0; h < 336; h++) {
			if(pci_blacklisted(3 * (h / 2) + cl.nid2)) {
				continue;
			}
			if(row[h] - med > best_sig) {
				best_sig = row[h] - med;
				best_v = row[h];
				best_med = med;
				best_m = m;
				best_h = h;
			}
		}
	}
	if(best_m < 0) {
		return false;
	}
	int used = cl.fused[best_m];
	double sig = best_sig;
	// family-disjoint runner-up excess in the winning hypothesis
	int m0b, m1b;
	sss_m0m1(best_h / 2, &m0b, &m1b);
	double rn = 0;
	for(int h = 0; h < 336; h++) {
		if(h / 2 == best_h / 2) {
			continue;
		}
		int m0h, m1h;
		sss_m0m1(h / 2, &m0h, &m1h);
		if(m0h == m0b || m0h == m1b || m1h == m0b || m1h == m1b) {
			continue;
		}
		double v = cl.comb[(size_t)best_m * 336 + h] / used - best_med;
		if(v > rn) {
			rn = v;
		}
	}
	// three tiers, all provisional (CRS descrambling coherence downstream confirms or
	// kills, and a killed PCI is blacklisted so the retry explores the next candidate):
	//   strong    - clean cell, decide fast
	//   weak      - solid statistical separation
	//   desperate - deep-fringe: after long accumulation take the best candidate and
	//               let the CRS arbiter settle what SSS statistics cannot
	bool strong = used >= 4 && sig >= 0.10;
	bool weak = used >= min_occ && sig >= sss_accept && (rn <= 0 || sig >= sss_margin * rn);
	bool desperate = used >= 500 && sig >= sss_accept && (rn <= 0 || sig >= 1.05 * rn);
	if(!strong && !weak && !desperate) {
		if(used >= 600 && sig < sss_accept) {
			cl.dead = true;                      // hundreds of folds and no excess: phantom
		}
		if(verbose_ && used >= min_occ && !quiet) {
			printf("cellsync: class nid2 %d fg %+.0f x%d held: sig %.4f vs disjoint %.4f (floor %.4f)\n",
			       cl.nid2, cl.fg, used, sig, rn, best_med);
		}
		return false;
	}

	int pci = 3 * (best_h / 2) + cl.nid2;
	for(const auto &c : cells_) {
		if(c->pci() == pci) {
			cl.dead = true;                      // another variant/phantom of a tracked cell
			return false;
		}
	}

	double cfo_cand = cl.fg + (best_m - ACQ_NF / 2) * 2500.0;
	bool first = cells_.empty();
	if(first) {
		coarse_cfo_ += cfo_cand;
	}
	// start tracking at the first PSS occurrence beyond the still-unprocessed data;
	// best_h & 1 is the parity at the class anchor, lastp sits lastj half-frames later
	double best_p = cl.lastp[best_m];
	int half_ref = ((best_h & 1) + (int)(cl.lastj[best_m] & 1)) & 1;
	uint64_t start_at = avail() + 500;
	uint64_t nskip = (start_at > (uint64_t)best_p) ? ((start_at - (uint64_t)best_p) / HALF_FRAME_SAMP + 1) : 1;
	double t_next = best_p + (double)nskip * HALF_FRAME_SAMP;
	int half_at_next = (half_ref + (int)nskip) & 1;
	int nid2 = cl.nid2;
	cells_.push_back(std::unique_ptr<cell_tracker>(
		new cell_tracker(this, pci, nid2, half_at_next, (uint64_t)t_next, (double)total_in_ / FS_SYNC)));
	if(!quiet || verbose_) {
		printf("cellsync: cell PCI %d (nid2 %d) cfo %+.1f Hz sss %.2f (sig %.4f) x%d pos %llu%s\n",
		       pci, nid2, coarse_cfo_ + (first ? 0.0 : cfo_cand), best_v, sig, used,
		       (unsigned long long)best_p, first ? " [coarse ref]" : "");
	}
	fflush(stdout);
	// residue/frequency bases just moved (first cell), or this cell's other-variant
	// classes are now stale either way - restart accumulation clean
	acq_.clear();
	return true;
}

// class bookkeeping per scan round: age out candidates whose trains stopped appearing
// (5 s while acquiring at the 0.2 s stride, 30 s under the 10 s augment cadence), then
// register this round's trains. Creation budget: 2 new classes per nid2 per round,
// deduped per (nid2, variant) residue - a wrong-root ZC cross-ambiguity phantom can
// outscore the true root at a nearby residue, and variants are DISTINCT classes, so
// neither cross-nid2 nor cross-variant dedupe is allowed
void sync_engine::select_classes(std::vector<train> &trains) {
	double horizon = tracking_ ? 30.0 : 5.0;
	for(size_t i = 0; i < acq_.size();) {
		if(total_in_ - acq_[i].last_train > (uint64_t)(horizon * FS_SYNC)) {
			acq_.erase(acq_.begin() + i);
		} else {
			i++;
		}
	}
	std::sort(trains.begin(), trains.end(), [](const train &a, const train &b) { return a.score > b.score; });
	int fresh_nid2[3] = { 0, 0, 0 };
	std::vector<std::pair<double, uint64_t>> tried[3];
	for(const auto &t : trains) {
		bool known = find_class(t) < acq_.size();
		if(!known && fresh_nid2[t.nid2] >= 2) {
			continue;
		}
		bool dup = false;
		for(const auto &tp : tried[t.nid2]) {
			if(std::fabs(tp.first - t.fg) > 1.0) {
				continue;
			}
			uint64_t d = t.occ[0] > tp.second ? t.occ[0] - tp.second : tp.second - t.occ[0];
			if(d % HALF_FRAME_SAMP < 150 || d % HALF_FRAME_SAMP > HALF_FRAME_SAMP - 150) {
				dup = true;
				break;
			}
		}
		if(dup) {
			continue;
		}
		// a tracked cell owns its residue class within its own nid2
		for(const auto &c : cells_) {
			if(c->nid2() != t.nid2) {
				continue;
			}
			uint64_t cp_ = (uint64_t)llround(c->next_pss());
			uint64_t d = t.occ[0] > cp_ ? t.occ[0] - cp_ : cp_ - t.occ[0];
			if(d % HALF_FRAME_SAMP < 150 || d % HALF_FRAME_SAMP > HALF_FRAME_SAMP - 150) {
				dup = true;
				break;
			}
		}
		if(dup) {
			continue;
		}
		tried[t.nid2].emplace_back(t.fg, t.occ[0]);
		if(!known) {
			fresh_nid2[t.nid2]++;
		}
		touch_class(t);
	}
}

// fold every established class (seen in at least 2 rounds - one-shot noise trains
// never earn fold time), stopping if an accept clears the accumulator set
void sync_engine::fold_all(bool quiet) {
	for(size_t i = 0; i < acq_.size(); i++) {
		if(acq_[i].dead || acq_[i].trains_seen < 2) {
			continue;
		}
		if(fold_class(i, quiet)) {
			return;
		}
		if((int)cells_.size() >= max_cells) {
			return;
		}
	}
}

void sync_engine::acquire() {
	acq_attempts_++;
	uint64_t w0 = avail() > (uint64_t)ACQ_WIN + base_ ? avail() - ACQ_WIN : base_;
	size_t wlen = (size_t)(avail() - w0);
	// 15 kHz detection spacing (the ZC ridge keeps detection alive between points, and
	// each class scans fg +-7.5 kHz in fine steps anyway); +-45 kHz total soaks up LO
	// placement error on top of reference error - the bench B13 capture sat at -37 kHz.
	// The wide-CFO wings alternate between rounds: the class accumulators integrate
	// ACROSS rounds anyway, so visiting a wing every second round costs one round of
	// latency on far-CFO cells only - and it cuts the per-round wall time ~40%, which
	// runs inline on the capture thread against a ~1.1 s transport queue (A53 budget).
	// fg=0 rides EVERY round: post-cal references sit within the +-7.5 kHz class scan,
	// so the common case keeps full-rate confirmation.
	static const double FGRID_A[] = { 0, 15000, -15000 };
	static const double FGRID_B[] = { 0, 30000, -30000, 45000, -45000 };

	std::vector<train> trains;
	std::vector<cf> save(at(w0), at(w0) + wlen);   // raw window (ring gets re-derotated per fgrid)
	const bool odd = (acq_attempts_ & 1) != 0;
	const double *fgs = odd ? FGRID_B : FGRID_A;
	size_t nfg = odd ? sizeof(FGRID_B) / sizeof(double) : sizeof(FGRID_A) / sizeof(double);
	double t_derot = 0, t_coll = 0;
	auto pnow = []() { return std::chrono::steady_clock::now(); };
	for(size_t gi = 0; gi < nfg; gi++) {
		double fg = fgs[gi];
		auto tp0 = pnow();
		// derotate the saved window into the ring region in place (phasor recurrence -
		// the per-sample sincos pair here was ~0.2 s of every round on the A53)
		double step = -2.0 * M_PI * fg / FS_SYNC;
		cf *dst = &ring_[(size_t)(w0 - base_)];
		cf rot(1.0f, 0.0f);
		const cf rstep((float)cos(step), (float)sin(step));
		double ph = 0;
		for(size_t i = 0; i < wlen; i++) {
			dst[i] = save[i] * rot;
			rot *= rstep;
			ph += step;
			if((i & 4095) == 4095) {
				rot = cf((float)cos(ph), (float)sin(ph));
			}
		}
		auto tp1 = pnow();
		collect_trains(w0, wlen, fg, trains);
		t_derot += std::chrono::duration<double>(tp1 - tp0).count();
		t_coll += std::chrono::duration<double>(pnow() - tp1).count();
	}
	// restore the raw window
	std::copy(save.begin(), save.end(), ring_.begin() + (size_t)(w0 - base_));

	auto tp2 = pnow();
	select_classes(trains);
	auto tp3 = pnow();
	fold_all(false);
	auto tp4 = pnow();
	static const bool acqprof = getenv("CELLSYNC_ACQPROF") != nullptr;
	if(acqprof) {
		fprintf(stderr, "acqprof: derot %.1f ms collect %.1f ms select %.1f ms fold %.1f ms (nfg %zu)\n",
		        t_derot * 1e3, t_coll * 1e3,
		        std::chrono::duration<double>(tp3 - tp2).count() * 1e3,
		        std::chrono::duration<double>(tp4 - tp3).count() * 1e3, nfg);
	}
	tracking_ = !cells_.empty();
}

void sync_engine::augment_scan() {
	// stream is already derotated to ~0 CFO: one variant over the whole retained ring
	// (6-8 frames in steady tracking); same-reference cells sit within the fg=0 cell
	uint64_t w0 = base_;
	size_t wlen = (size_t)(avail() - w0);
	if(wlen < (size_t)(2 * HALF_FRAME_SAMP + 2000)) {
		return;
	}
	std::vector<train> trains;
	collect_trains(w0, wlen, 0.0, trains);
	select_classes(trains);
	fold_all(true);
}

void sync_engine::fold_fine_into_coarse() {
	if(cells_.empty()) {
		return;
	}
	std::lock_guard<std::mutex> lk(meas_mtx_);
	cell_tracker *best = cells_[0].get();
	for(auto &c : cells_) {
		if(c->snr_ > best->snr_) {
			best = c.get();
		}
	}
	// folding retunes every tracker: demand a mature CP view, plenty of pairs, and a
	// cooldown so a bad early estimate can't ratchet the coarse NCO away in 2 kHz steps
	if(best->pairs_ < 500 || best->n_hf_ < 40 || std::abs(best->acc_vec_) == 0 ||
	   total_in_ - last_fold_in_ < (uint64_t)(0.5 * FS_SYNC)) {
		return;
	}
	double fine = std::arg(best->acc_vec_) / (2.0 * M_PI * 5e-4);
	if(std::abs(best->cp_acc_) > 0) {
		double cp_cfo = std::arg(best->cp_acc_) / (2.0 * M_PI) * SCS;
		fine += 2000.0 * llround((cp_cfo - fine) / 2000.0);
	}
	if(std::fabs(fine) > 200.0) {
		last_fold_in_ = total_in_;
		coarse_cfo_ += fine;
		for(auto &c : cells_) {
			c->reset_phase_chains();
		}
		if(verbose_) {
			printf("cellsync: folded %+.1f Hz into coarse (now %+.1f Hz)\n", fine, coarse_cfo_);
			fflush(stdout);
		}
	}
}

std::vector<cell_meas> sync_engine::cells() {
	std::lock_guard<std::mutex> lk(meas_mtx_);
	std::vector<cell_meas> out;
	for(auto &c : cells_) {
		cell_meas m;
		c->export_meas(m, false);
		out.push_back(m);
	}
	return out;
}

std::vector<cell_meas> sync_engine::take_measurements() {
	std::lock_guard<std::mutex> lk(meas_mtx_);
	std::vector<cell_meas> out;
	for(auto &c : cells_) {
		cell_meas m;
		c->export_meas(m, true);
		out.push_back(m);
	}
	return out;
}

void sync_engine::trim() {
	uint64_t keep = proc_;
	for(const auto &c : cells_) {
		uint64_t need = (uint64_t)std::max(0.0, c->next_pss() - 832.0 - 2.0 * HALF_FRAME_SAMP - 100.0);
		keep = std::min(keep, need);
	}
	if(!tracking_) {
		// ACQ_WIN plus enough history for the earliest occurrence's CP-CFO slots
		keep = avail() > (uint64_t)(ACQ_WIN + 4700) + base_ ? avail() - ACQ_WIN - 4700 : base_;
	}
	if(keep > base_ + 4096) {
		size_t drop = (size_t)(keep - base_);
		ring_.erase(ring_.begin(), ring_.begin() + drop);
		base_ = keep;
	}
	// hard cap: a stalled consumer must not grow the ring unboundedly
	if(ring_.size() > (size_t)8 * FRAME_SAMP) {
		size_t drop = ring_.size() - 6 * FRAME_SAMP;
		ring_.erase(ring_.begin(), ring_.begin() + drop);
		base_ += drop;
		if(proc_ < base_) {
			proc_ = base_;
		}
	}
}

} // namespace cellsync

// elrsd: DSP primitives. fft_plan and ddc are lorad's (rfnm-webapps/lorad/src/fft.h,
// channelizer.h) re-homed into namespace elrsd; the resampler and the int16 envelope are new.
// The scalar paths are the numeric reference (x86 selftest); the NEON paths follow lorad's
// proven op-sequences so they stay bit-identical under -fcx-limited-range.
#include "dsp.h"

#include <cmath>
#include <cstring>
#include <algorithm>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#define ELRSD_NEON 1
#endif

namespace elrsd {

// ---------------------------------------------------------------- fft_plan (lorad) ----
fft_plan::fft_plan(unsigned n) : n_(n) {
	log2n_ = 0;
	while((1u << log2n_) < n) {
		log2n_++;
	}
	tw_.resize(n / 2);
	for(unsigned k = 0; k < n / 2; k++) {
		double a = -2.0 * M_PI * k / n;
		tw_[k] = cf((float)cos(a), (float)sin(a));
	}
	rev_.resize(n);
	for(unsigned i = 0; i < n; i++) {
		unsigned r = 0;
		for(unsigned b = 0; b < log2n_; b++) {
			r = (r << 1) | ((i >> b) & 1);
		}
		rev_[i] = r;
	}
	// per-stage contiguous twiddles (unit-stride loads for the NEON butterfly)
	stage_off_.assign(log2n_ + 1, 0);
	for(unsigned s = 1; s <= log2n_; s++) {
		stage_off_[s] = (unsigned)stw_.size();
		unsigned half = 1u << (s - 1);
		unsigned step = n_ / (1u << s);
		for(unsigned j = 0; j < half; j++) {
			stw_.push_back(tw_[j * step]);
		}
	}
}

void fft_plan::run(const cf *in, cf *out) const {
	for(unsigned i = 0; i < n_; i++) {
		out[i] = in[rev_[i]];
	}
	for(unsigned s = 1; s <= log2n_; s++) {
		unsigned m = 1u << s;
		unsigned half = m >> 1;
		const cf *stw = stw_.data() + stage_off_[s];
		for(unsigned k = 0; k < n_; k += m) {
			cf *a = out + k;
			cf *b = out + k + half;
			unsigned j = 0;
#ifdef ELRSD_NEON
			for(; j + 4 <= half; j += 4) {
				float32x4x2_t u = vld2q_f32(reinterpret_cast<const float *>(a + j));
				float32x4x2_t v = vld2q_f32(reinterpret_cast<const float *>(b + j));
				float32x4x2_t w = vld2q_f32(reinterpret_cast<const float *>(stw + j));
				float32x4_t tre = vsubq_f32(vmulq_f32(w.val[0], v.val[0]), vmulq_f32(w.val[1], v.val[1]));
				float32x4_t tim = vaddq_f32(vmulq_f32(w.val[0], v.val[1]), vmulq_f32(w.val[1], v.val[0]));
				float32x4x2_t r0, r1;
				r0.val[0] = vaddq_f32(u.val[0], tre);
				r0.val[1] = vaddq_f32(u.val[1], tim);
				r1.val[0] = vsubq_f32(u.val[0], tre);
				r1.val[1] = vsubq_f32(u.val[1], tim);
				vst2q_f32(reinterpret_cast<float *>(a + j), r0);
				vst2q_f32(reinterpret_cast<float *>(b + j), r1);
			}
#endif
			for(; j < half; j++) {
				cf t = stw[j] * b[j];
				cf u = a[j];
				a[j] = u + t;
				b[j] = u - t;
			}
		}
	}
}

// ---------------------------------------------------------------- ddc (lorad) ---------
ddc::ddc(double in_rate, double out_rate, double offset_hz) {
	decim_ = (int)llround(in_rate / out_rate);
	if(decim_ < 1) decim_ = 1;
	phase_ = cf(1.0f, 0.0f);
	double w = -2.0 * M_PI * offset_hz / in_rate;
	rot_ = cf((float)cos(w), (float)sin(w));
	rot2_ = rot_ * rot_;
	rot3_ = rot2_ * rot_;
	rot4_ = rot2_ * rot2_;
	int taps = 4 * decim_ | 1;
	taps_.resize(taps);
	double fc = 0.45 * out_rate / in_rate;
	double sum = 0;
	for(int i = 0; i < taps; i++) {
		double m = i - (taps - 1) / 2.0;
		double s = m == 0 ? 2.0 * fc : sin(2.0 * M_PI * fc * m) / (M_PI * m);
		double win = 0.54 - 0.46 * cos(2.0 * M_PI * i / (taps - 1));
		taps_[i] = (float)(s * win);
		sum += taps_[i];
	}
	for(auto &t : taps_) {
		t = (float)(t / sum);
	}
	nt_ = (int)taps_.size();
	buf_.assign((size_t)nt_ * 2, cf(0, 0));
	w_ = nt_ - 1;
	cnt_ = 0;
}

void ddc::reset() {
	std::fill(buf_.begin(), buf_.end(), cf(0, 0));
	w_ = nt_ - 1;
	cnt_ = 0;
	phase_ = cf(1.0f, 0.0f);
}

// newest-first window dotted with the (symmetric) taps in natural order
static inline cf fir_dot_nf(const cf *win, const float *taps, int nt) {
#ifdef ELRSD_NEON
	float32x4_t ar = vdupq_n_f32(0), ai = vdupq_n_f32(0);
	const float *wf = reinterpret_cast<const float *>(win);
	int t = 0;
	for(; t + 4 <= nt; t += 4) {
		float32x4x2_t v = vld2q_f32(wf + 2 * t);
		float32x4_t tp = vld1q_f32(taps + t);
		ar = vmlaq_f32(ar, v.val[0], tp);
		ai = vmlaq_f32(ai, v.val[1], tp);
	}
	float32x2_t sr = vadd_f32(vget_low_f32(ar), vget_high_f32(ar));
	float32x2_t si = vadd_f32(vget_low_f32(ai), vget_high_f32(ai));
	float re = vget_lane_f32(vpadd_f32(sr, sr), 0);
	float im = vget_lane_f32(vpadd_f32(si, si), 0);
	for(; t < nt; t++) {
		re += win[t].real() * taps[t];
		im += win[t].imag() * taps[t];
	}
	return cf(re, im);
#else
	float re = 0, im = 0;
	for(int t = 0; t < nt; t++) {
		re += win[t].real() * taps[t];
		im += win[t].imag() * taps[t];
	}
	return cf(re, im);
#endif
}

void ddc::process(const cf *in, size_t count, std::vector<cf> &out) {
	for(size_t i = 0; i < count; i++) {
		cf x = in[i] * phase_;
		phase_ *= rot_;
		buf_[w_] = x;
		buf_[w_ + nt_] = x;
		if(++cnt_ == decim_) {
			cnt_ = 0;
			out.push_back(fir_dot_nf(&buf_[w_], taps_.data(), nt_));
		}
		w_ = w_ == 0 ? nt_ - 1 : w_ - 1;
	}
	float m = std::abs(phase_);
	if(m > 0) {
		phase_ /= m;
	}
}

// ---------------------------------------------------------------- resampler -----------
// Polyphase interpolation-by-up / decimation-by-down with a windowed-sinc prototype of
// up * taps_per_branch taps, cutoff 0.45 * out_rate. Output m uses branch p = (m*down) % up
// on the newest-first history: y = sum_k h[p + k*up] * x[n-k], n = floor(m*down/up).
// The NCO rotates at the input rate; its phase is re-derived from a double accumulator
// every 1024 samples so a long burst segment (100k samples) keeps phase accuracy.
resampler::resampler(unsigned up, unsigned down, double in_rate, double offset_hz, int taps_per_branch, double cutoff_hz)
    : up_(up), down_(down), in_rate_(in_rate) {
	if(up_ < 1) up_ = 1;
	if(down_ < 1) down_ = 1;
	if(taps_per_branch < 4) taps_per_branch = 4;
	out_rate_ = in_rate_ * (double)up_ / (double)down_;
	int L = taps_per_branch;
	int ntot = (int)up_ * L;
	std::vector<double> proto(ntot);
	// cutoff relative to the interpolated rate in_rate * up
	if(cutoff_hz <= 0) cutoff_hz = 0.45 * out_rate_;
	if(cutoff_hz > 0.5 * out_rate_) cutoff_hz = 0.5 * out_rate_;
	double fc = cutoff_hz / (in_rate_ * (double)up_);
	double centre = (ntot - 1) / 2.0;
	for(int i = 0; i < ntot; i++) {
		double m = i - centre;
		double s = (fabs(m) < 1e-9) ? 2.0 * fc : sin(2.0 * M_PI * fc * m) / (M_PI * m);
		double win = 0.54 - 0.46 * cos(2.0 * M_PI * i / (ntot - 1));   // Hamming
		proto[i] = s * win;
	}
	double sum = 0;
	for(double v : proto) sum += v;
	// unit DC gain per branch: the interpolated (zero-stuffed) signal carries 1/up of the
	// energy per sample, so the prototype gets gain up
	double g = (double)up_ / sum;
	taps_.assign((size_t)ntot, 0.0f);
	for(unsigned p = 0; p < up_; p++) {
		for(int k = 0; k < L; k++) {
			int idx = (int)p + k * (int)up_;
			taps_[p * L + k] = (float)(proto[idx] * g);
		}
	}
	hist_.assign((size_t)L * 2, cf(0, 0));
	double w = -2.0 * M_PI * offset_hz / in_rate_;
	rot_ = cf((float)cos(w), (float)sin(w));
	phase_ = cf(1.0f, 0.0f);
	offset_hz_d_ = offset_hz;
	reset();
}

void resampler::reset() {
	std::fill(hist_.begin(), hist_.end(), cf(0, 0));
	phase_acc_ = 0;
	phase_ = cf(1.0f, 0.0f);
	nco_n_ = 0;
	hw_ = (int)(hist_.size() / 2) - 1;
}

void resampler::process(const cf *in, size_t count, std::vector<cf> &out) {
	const int L = (int)(hist_.size() / 2);
	const float *taps = taps_.data();
	for(size_t i = 0; i < count; i++) {
		// NCO: rotate, and every 1024 samples re-derive the phase from the exact sample count
		cf x = in[i] * phase_;
		phase_ *= rot_;
		if((++nco_n_ & 1023) == 0) {
			double ph = -2.0 * M_PI * offset_hz_d_ * (double)nco_n_ / in_rate_;
			phase_ = cf((float)cos(ph), (float)sin(ph));
		}
		hist_[hw_] = x;
		hist_[hw_ + L] = x;
		// emit every output whose branch falls on this input sample
		while(phase_acc_ < up_) {
			out.push_back(fir_dot_nf(&hist_[hw_], taps + (size_t)phase_acc_ * L, L));
			phase_acc_ += down_;
		}
		phase_acc_ -= up_;
		hw_ = hw_ == 0 ? L - 1 : hw_ - 1;
	}
}

void resampler::process_i16(const int16_t *iq, size_t count, std::vector<cf> &out, bool conj) {
	const float scale = 1.0f / 32768.0f;
	const float qs = conj ? -scale : scale;
	std::vector<cf> tmp;
	const size_t CH = 4096;
	tmp.resize(CH);
	for(size_t off = 0; off < count; off += CH) {
		size_t n = std::min(CH, count - off);
		for(size_t i = 0; i < n; i++) {
			tmp[i] = cf(iq[2 * (off + i)] * scale, iq[2 * (off + i) + 1] * qs);
		}
		process(tmp.data(), n, out);
	}
}

// ---------------------------------------------------------------- envelope_i16 --------
envelope_i16::envelope_i16(size_t block_len) : blk_(block_len ? block_len : 1) {}

void envelope_i16::reset() {
	fill_ = 0;
	acc_ = 0;
	sum_i_ = sum_q_ = 0;
	dc_locked_ = false;
	dc_i_ = dc_q_ = 0;
}

void envelope_i16::set_dc_tracking(bool on) {
	dc_track_ = on;
	if(!on) dc_i_ = dc_q_ = 0;
	dc_locked_ = false;
}

static inline int16_t sat16(int32_t v) { return (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); }

// sum of saturate(|saturate(v - dc)|) over n2 int16 values (interleaved I,Q; dc alternates
// dci, dcq) plus the raw sums of I and Q (for DC tracking)
static inline uint64_t abs_sum_dc_i16(const int16_t *v, size_t n2, int16_t dci, int16_t dcq, int64_t &si, int64_t &sq) {
	uint64_t s = 0;
	size_t i = 0;
#ifdef ELRSD_NEON
	const int16_t dcv[8] = {dci, dcq, dci, dcq, dci, dcq, dci, dcq};
	int16x8_t dc = vld1q_s16(dcv);
	int32x4_t acc = vdupq_n_s32(0), sacc = vdupq_n_s32(0);
	size_t cnt = 0;
	for(; i + 8 <= n2; i += 8) {
		int16x8_t x = vld1q_s16(v + i);
		acc = vpadalq_s16(acc, vqabsq_s16(vqsubq_s16(x, dc)));
		sacc = vaddq_s32(sacc, vmovl_s16(vget_low_s16(x)));
		sacc = vaddq_s32(sacc, vmovl_s16(vget_high_s16(x)));
		if(++cnt == 4096) {
			s += (uint64_t)vaddvq_s32(acc);
			si += (int64_t)vgetq_lane_s32(sacc, 0) + vgetq_lane_s32(sacc, 2);
			sq += (int64_t)vgetq_lane_s32(sacc, 1) + vgetq_lane_s32(sacc, 3);
			acc = vdupq_n_s32(0);
			sacc = vdupq_n_s32(0);
			cnt = 0;
		}
	}
	s += (uint64_t)vaddvq_s32(acc);
	si += (int64_t)vgetq_lane_s32(sacc, 0) + vgetq_lane_s32(sacc, 2);
	sq += (int64_t)vgetq_lane_s32(sacc, 1) + vgetq_lane_s32(sacc, 3);
#endif
	for(; i < n2; i += 2) {
		int32_t a = v[i], b = v[i + 1];
		si += a;
		sq += b;
		int32_t da = sat16(a - dci), db = sat16(b - dcq);
		s += (uint64_t)sat16(da < 0 ? -da : da) + (uint64_t)sat16(db < 0 ? -db : db);
	}
	return s;
}

void envelope_i16::process(const int16_t *iq, size_t count, std::vector<uint32_t> &out) {
	size_t i = 0;
	while(i < count) {
		size_t room = blk_ - fill_;
		size_t n = std::min(room, count - i);
		if(dc_track_) {
			acc_ += abs_sum_dc_i16(iq + 2 * i, 2 * n, (int16_t)dc_i_, (int16_t)dc_q_, sum_i_, sum_q_);
		} else {
			int64_t si = 0, sq = 0;
			acc_ += abs_sum_dc_i16(iq + 2 * i, 2 * n, 0, 0, si, sq);
		}
		fill_ += n;
		i += n;
		if(fill_ == blk_) {
			out.push_back(acc_ > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)acc_);
			if(dc_track_) {
				int mi = (int)(sum_i_ / (int64_t)blk_), mq = (int)(sum_q_ / (int64_t)blk_);
				if(!dc_locked_) {
					dc_i_ = mi;
					dc_q_ = mq;
					dc_locked_ = true;
				} else {
					dc_i_ += (mi - dc_i_) / 32;
					dc_q_ += (mq - dc_q_) / 32;
				}
			}
			acc_ = 0;
			fill_ = 0;
			sum_i_ = sum_q_ = 0;
		}
	}
}

} // namespace elrsd

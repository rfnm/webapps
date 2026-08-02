// aisd: NCO + FIR decimator (same design as lorad's). One instance shifts an AIS
// channel from the capture center to baseband and decimates the capture rate down to
// the demod rate (~100 kHz). The FIR only computes at the output rate. The down-mix and
// FIR are NEON-optimized on AArch64 with a scalar fallback that is the numeric
// reference (bit-identical to the scalar path, which is what the x86 host selftest runs).
#pragma once

#include <cmath>
#include <vector>
#include "dsp.h"

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace aisd {

class ddc {
public:
	// in_rate -> out_rate must divide evenly; offset_hz is the channel offset from
	// the capture center (positive == channel above center)
	ddc(double in_rate, double out_rate, double offset_hz) {
		decim_ = (int)llround(in_rate / out_rate);
		phase_ = cf(1.0f, 0.0f);
		double w = -2.0 * M_PI * offset_hz / in_rate;
		rot_ = cf((float)cos(w), (float)sin(w));
		rot2_ = rot_ * rot_;
		rot3_ = rot2_ * rot_;
		rot4_ = rot2_ * rot2_; // 4-step rotor for the NEON down-mix

		// windowed-sinc lowpass, cutoff at 0.45 * out_rate, 4 * decim taps
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
		// double-length ring: each new sample is written at w_ and w_+nt_ so the FIR
		// always reads nt_ contiguous, in-order (newest-first) samples with no wrap and
		// no per-sample modulo. Symmetric (linear-phase) taps make the newest-first
		// window dotted with taps_ in natural order equal to the convolution.
		buf_.assign((size_t)nt_ * 2, cf(0, 0));
		w_ = nt_ - 1;
	}

	// feed input samples, append output samples to out
	void process(const cf *in, size_t count, std::vector<cf> &out) {
		size_t i = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
		// NCO down-mix 4 samples per vector; keep 4 phases coherent by advancing the
		// lane vector by rot^4 each group. phase_ stays the scalar truth for the tail.
		if(count >= 4) {
			float pr[4] = { phase_.real(), (phase_ * rot_).real(), (phase_ * rot2_).real(), (phase_ * rot3_).real() };
			float pi[4] = { phase_.imag(), (phase_ * rot_).imag(), (phase_ * rot2_).imag(), (phase_ * rot3_).imag() };
			float32x4_t phr = vld1q_f32(pr), phi = vld1q_f32(pi);
			float32x4_t r4r = vdupq_n_f32(rot4_.real()), r4i = vdupq_n_f32(rot4_.imag());
			for(; i + 4 <= count; i += 4) {
				float32x4x2_t v = vld2q_f32(reinterpret_cast<const float *>(in + i));
				float32x4_t xr = vsubq_f32(vmulq_f32(v.val[0], phr), vmulq_f32(v.val[1], phi));
				float32x4_t xi = vaddq_f32(vmulq_f32(v.val[0], phi), vmulq_f32(v.val[1], phr));
				float xrs[4], xis[4];
				vst1q_f32(xrs, xr);
				vst1q_f32(xis, xi);
				for(int k = 0; k < 4; k++) {
					push_sample(cf(xrs[k], xis[k]), out);
				}
				float32x4_t nphr = vsubq_f32(vmulq_f32(phr, r4r), vmulq_f32(phi, r4i));
				float32x4_t nphi = vaddq_f32(vmulq_f32(phr, r4i), vmulq_f32(phi, r4r));
				phr = nphr;
				phi = nphi;
			}
			phase_ = cf(vgetq_lane_f32(phr, 0), vgetq_lane_f32(phi, 0));
		}
#endif
		for(; i < count; i++) {
			cf x = in[i] * phase_;
			phase_ *= rot_;
			push_sample(x, out);
		}
		// renormalize the NCO once per block to stop amplitude drift
		float m = std::abs(phase_);
		if(m > 0) {
			phase_ /= m;
		}
	}

private:
	// store one down-mixed sample into the ring and, on the decimation phase, emit the
	// FIR output. Shared by the NEON and scalar down-mix loops so both keep identical
	// ring/decimation bookkeeping.
	inline void push_sample(cf x, std::vector<cf> &out) {
		buf_[w_] = x;
		buf_[w_ + nt_] = x;
		if(++dphase_ == decim_) {
			dphase_ = 0;
			out.push_back(fir_dot(&buf_[w_]));
		}
		w_ = w_ == 0 ? nt_ - 1 : w_ - 1;
	}

	// dot the newest-first window (nt_ contiguous cf) with taps_ in natural order
	inline cf fir_dot(const cf *win) const {
#if defined(__aarch64__) || defined(__ARM_NEON)
		float32x4_t ar = vdupq_n_f32(0), ai = vdupq_n_f32(0);
		const float *wf = reinterpret_cast<const float *>(win);
		int t = 0;
		for(; t + 4 <= nt_; t += 4) {
			float32x4x2_t v = vld2q_f32(wf + 2 * t); // val[0]=re lanes, val[1]=im lanes
			float32x4_t tp = vld1q_f32(&taps_[t]);
			ar = vmlaq_f32(ar, v.val[0], tp);
			ai = vmlaq_f32(ai, v.val[1], tp);
		}
		float32x2_t sr = vadd_f32(vget_low_f32(ar), vget_high_f32(ar));
		float32x2_t si = vadd_f32(vget_low_f32(ai), vget_high_f32(ai));
		float re = vget_lane_f32(vpadd_f32(sr, sr), 0);
		float im = vget_lane_f32(vpadd_f32(si, si), 0);
		for(; t < nt_; t++) {
			re += win[t].real() * taps_[t];
			im += win[t].imag() * taps_[t];
		}
		return cf(re, im);
#else
		float re = 0, im = 0;
		for(int t = 0; t < nt_; t++) {
			re += win[t].real() * taps_[t];
			im += win[t].imag() * taps_[t];
		}
		return cf(re, im);
#endif
	}

	int decim_ = 1;
	int dphase_ = 0;
	int nt_ = 0;
	cf phase_, rot_, rot2_, rot3_, rot4_;
	std::vector<float> taps_;
	std::vector<cf> buf_; // length 2*nt_ double-write ring
	int w_ = 0;
};

} // namespace aisd

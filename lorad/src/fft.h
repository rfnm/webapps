// lorad: minimal iterative radix-2 complex FFT, power-of-2 sizes only.
// Self-contained on purpose: lorad must not pull external DSP dependencies.
// The butterfly is NEON-vectorized on AArch64 (the i.MX8MP/A53 target); the scalar path is the exact
// numeric reference (x86) and the fallback for small stages. Per-stage contiguous twiddle tables make
// the NEON loads unit-stride. Under -fcx-limited-range the complex butterfly op-sequence is identical
// to std::complex, so the vector path is bit-for-bit equal to scalar (the LoRa selftest stays exact).
#pragma once

#include <complex>
#include <vector>
#include <cmath>
#include <cstdint>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace lorad {

typedef std::complex<float> cf;

class fft_plan {
public:
	explicit fft_plan(unsigned n) : n_(n) {
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
		// per-stage contiguous twiddles: stage s uses tw_[j*step], step=n/2^s, j in [0,half) -> pack
		// them unit-stride so the NEON butterfly can vld2q them (total size == n-1).
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

	unsigned size() const {
		return n_;
	}

	// in-place DIT after bit-reverse copy; caller provides out buffer of n elements
	void run(const cf *in, cf *out) const {
		for(unsigned i = 0; i < n_; i++) {
			out[i] = in[rev_[i]];
		}
		for(unsigned s = 1; s <= log2n_; s++) {
			unsigned m = 1u << s;
			unsigned half = m >> 1;
			const cf *stw = stw_.data() + stage_off_[s];
			for(unsigned k = 0; k < n_; k += m) {
				cf *a = out + k;          // out[k + j]
				cf *b = out + k + half;   // out[k + j + half]
				unsigned j = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
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

private:
	unsigned n_;
	unsigned log2n_;
	std::vector<cf> tw_;
	std::vector<cf> stw_;              // per-stage contiguous twiddles (unit-stride for NEON)
	std::vector<unsigned> stage_off_;  // start of each stage's twiddles in stw_
	std::vector<unsigned> rev_;
};

} // namespace lorad

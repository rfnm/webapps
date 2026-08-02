// cellsync: minimal iterative radix-2 complex FFT, power-of-2 sizes only (shared lorad lineage).
// Self-contained on purpose: cellsyncd must not pull external DSP dependencies.
#pragma once

#include <complex>
#include <vector>
#include <cmath>
#include <cstdint>

#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace cellsync {

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
		// per-stage contiguous twiddles: the butterfly loop reads tw_[j*step], a
		// strided gather the NEON path can't load - repacked once here (sum over
		// stages = n-1 entries, trivial memory) so every stage streams contiguously
		stage_tw_.resize(log2n_);
		for(unsigned s = 1; s <= log2n_; s++) {
			unsigned half = 1u << (s - 1);
			unsigned step = n_ >> s;
			stage_tw_[s - 1].resize(half);
			for(unsigned j = 0; j < half; j++) {
				stage_tw_[s - 1][j] = tw_[j * step];
			}
		}
		rev_.resize(n);
		for(unsigned i = 0; i < n; i++) {
			unsigned r = 0;
			for(unsigned b = 0; b < log2n_; b++) {
				r = (r << 1) | ((i >> b) & 1);
			}
			rev_[i] = r;
		}
	}

	unsigned size() const {
		return n_;
	}

	// in-place DIT after bit-reverse copy; caller provides out buffer of n elements.
	// aarch64: stages with >=4 butterflies run 4-wide NEON on deinterleaved planes
	// (x86 keeps the scalar loop as the A/B reference - house perf pattern).
	void run(const cf *in, cf *out) const {
		for(unsigned i = 0; i < n_; i++) {
			out[i] = in[rev_[i]];
		}
		for(unsigned s = 1; s <= log2n_; s++) {
			unsigned m = 1u << s;
			unsigned half = m >> 1;
			const cf *st = stage_tw_[s - 1].data();
#ifdef __aarch64__
			if(half >= 4) {
				const float *tp = reinterpret_cast<const float *>(st);
				for(unsigned k = 0; k < n_; k += m) {
					float *lo = reinterpret_cast<float *>(out + k);
					float *hi = reinterpret_cast<float *>(out + k + half);
					for(unsigned j = 0; j < half; j += 4) {
						float32x4x2_t w = vld2q_f32(tp + 2 * j);
						float32x4x2_t b = vld2q_f32(hi + 2 * j);
						float32x4x2_t a = vld2q_f32(lo + 2 * j);
						float32x4_t tr = vfmsq_f32(vmulq_f32(w.val[0], b.val[0]), w.val[1], b.val[1]);
						float32x4_t ti = vfmaq_f32(vmulq_f32(w.val[0], b.val[1]), w.val[1], b.val[0]);
						float32x4x2_t u, v;
						u.val[0] = vaddq_f32(a.val[0], tr);
						u.val[1] = vaddq_f32(a.val[1], ti);
						v.val[0] = vsubq_f32(a.val[0], tr);
						v.val[1] = vsubq_f32(a.val[1], ti);
						vst2q_f32(lo + 2 * j, u);
						vst2q_f32(hi + 2 * j, v);
					}
				}
				continue;
			}
#endif
			for(unsigned k = 0; k < n_; k += m) {
				for(unsigned j = 0; j < half; j++) {
					cf t = st[j] * out[k + j + half];
					cf u = out[k + j];
					out[k + j] = u + t;
					out[k + j + half] = u - t;
				}
			}
		}
	}

private:
	unsigned n_;
	unsigned log2n_;
	std::vector<cf> tw_;
	std::vector<std::vector<cf>> stage_tw_;
	std::vector<unsigned> rev_;
};

} // namespace cellsync

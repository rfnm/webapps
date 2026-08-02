// cellsync: streaming NCO mixer + polyphase rational L/M resampler (windowed-sinc).
// Used as 2.4 MSps capture -> NCO(-offset) -> 4/5 -> 1.92 MSps; generic L/M so the
// selftest can also run the inverse (5/4) to exercise the exact daemon path.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "fft.h"

namespace cellsync {

class nco {
public:
	nco(double freq_hz, double fs) : ph_(0), step_(-2.0 * M_PI * freq_hz / fs),
		stepper_((float)cos(step_), (float)sin(step_)), cur_(1.0f, 0.0f), k_(0) {}

	// multiply buf by e^{-j2pi f t}: shifts a component at +f down to DC.
	// Phasor recurrence, ONE complex multiply per sample: the old per-sample libm
	// cos()+sin() pair was ~0.3 us/sample on the A53 - at MSps rates that alone ate
	// most of a core (the receipted 22%-of-real-time cellsyncd starvation). The
	// float phasor drifts ~1e-7 rad/step in phase and ~1e-7 in magnitude, so it is
	// re-seeded from the exact double phase accumulator every 4096 samples (two
	// transcendentals per 4096 instead of two per sample); worst-case phase error
	// between reseeds is < 0.03 deg, three orders below anything LTE sync can see.
	void mix(cf *buf, size_t n) {
		for(size_t i = 0; i < n; i++) {
			buf[i] *= cur_;
			cur_ *= stepper_;
			if(++k_ >= 4096) {
				ph_ = std::fmod(ph_ + step_ * 4096.0, 2.0 * M_PI);
				cur_ = cf((float)cos(ph_), (float)sin(ph_));
				k_ = 0;
			}
		}
	}

private:
	double ph_, step_;
	cf stepper_, cur_;
	size_t k_;
};

// polyphase rational resampler: y runs at fs * L / M.
// Derivation: v = x upsampled by L, y[j] = (h * v)[jM] = sum_p h[phi + Lp] x[n - p]
// with n = floor(jM/L), phi = jM mod L - so per input sample n we emit every output
// whose source index lands on n (acc_ tracks jM - nL).
class resampler {
public:
	resampler(int L, int M, int taps_per_phase = 12) : L_(L), M_(M), P_(taps_per_phase), acc_(0), wp_(0) {
		int n = L_ * P_;
		h_.resize(n);
		// windowed-sinc prototype at the interpolated rate fs*L, cutoff below the
		// narrower of input/output Nyquist with a guard (Blackman window)
		double fc = 0.45 / std::max(L_, M_);
		for(int i = 0; i < n; i++) {
			double x = i - (n - 1) / 2.0;
			double s = x == 0 ? 2 * fc : sin(2 * M_PI * fc * x) / (M_PI * x);
			double w = 0.42 - 0.5 * cos(2 * M_PI * i / (n - 1)) + 0.08 * cos(4 * M_PI * i / (n - 1));
			h_[i] = (float)(s * w * L_);
		}
		buf_.assign(2 * P_, cf(0, 0));   // double-write ring: contiguous window always exists
	}

	// feed n input samples, append outputs to out (caller reserves)
	void process(const cf *x, size_t n, std::vector<cf> &out) {
		for(size_t i = 0; i < n; i++) {
			buf_[wp_] = x[i];
			buf_[wp_ + P_] = x[i];
			const cf *w = &buf_[wp_ + 1];   // w[0..P-1] = oldest..newest
			while(acc_ < L_) {
				cf y(0, 0);
				const float *hp = &h_[acc_];
				for(int p = 0; p < P_; p++) {
					y += w[P_ - 1 - p] * hp[(size_t)L_ * p];
				}
				out.push_back(y);
				acc_ += M_;
			}
			acc_ -= L_;
			wp_ = (wp_ + 1) % P_;
		}
	}

private:
	int L_, M_, P_;
	int acc_, wp_;
	std::vector<float> h_;
	std::vector<cf> buf_;
};

} // namespace cellsync

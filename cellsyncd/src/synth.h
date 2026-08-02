// cellsync: LTE FDD downlink synthesizer - SELFTEST ONLY, never linked into the daemon
// path. Generates the center window of a cell (PSS/SSS/CRS + QPSK filler PDSCH) at
// 1.92 MSps, then models the receive impairments the estimator must undo: a static
// 2-tap channel, carrier offset, sampling-clock offset (both from the same synthetic
// reference error, matching the real board), and AWGN.
#pragma once

#include <algorithm>
#include <cstring>
#include <random>
#include <vector>

#include "lte_seq.h"

namespace cellsync {

class dl_synth {
public:
	dl_synth(int nid_cell, int nrb, uint32_t seed = 1) : nid_(nid_cell), nrb_(nrb), rng_(seed), fft_(FFT_N) {}

	// n_frames of ideal baseband at 1.92 MSps, RMS-normalized
	std::vector<cf> frames(int n_frames) {
		std::vector<cf> out;
		out.reserve((size_t)n_frames * FRAME_SAMP);
		for(int f = 0; f < n_frames; f++) {
			for(int sf = 0; sf < 10; sf++) {
				for(int sl = 0; sl < 2; sl++) {
					int ns = 2 * sf + sl;
					for(int l = 0; l < SYMS_PER_SLOT; l++) {
						emit_symbol(out, ns, l);
					}
				}
			}
		}
		float e = 0;
		for(const auto &v : out) {
			e += std::norm(v);
		}
		float g = 1.0f / sqrtf(e / out.size());
		for(auto &v : out) {
			v *= g;
		}
		return out;
	}

	int pci() const { return nid_; }

private:
	void emit_symbol(std::vector<cf> &out, int ns, int l) {
		cf bins[FFT_N];
		std::fill(bins, bins + FFT_N, cf(0, 0));
		int edge = std::min(6 * nrb_, FFT_N / 2 - 1);
		std::uniform_int_distribution<int> qp(0, 3);
		const float A = 0.70710678f;
		for(int rel = -edge; rel <= edge; rel++) {
			if(rel == 0) {
				continue;
			}
			int q = qp(rng_);
			bins[(rel + FFT_N) % FFT_N] = cf(A * (q & 1 ? 1 : -1), A * (q & 2 ? 1 : -1));
		}
		// CRS overwrite data at pilot positions
		if(l == 0 || l == 4) {
			for(const auto &p : crs_positions(nid_, nrb_, l, -edge, edge)) {
				cf r;
				crs_seq(nid_, ns, l, p.mprime, 1, &r);
				bins[(p.rel + FFT_N) % FFT_N] = r;
			}
		}
		// sync signals in slots 0 and 10 (FDD, normal CP): SSS at l=5, PSS at l=6,
		// occupying the center 62 with 5 empty guards each side
		if((ns == 0 || ns == 10) && (l == 5 || l == 6)) {
			for(int rel = -36; rel <= 36; rel++) {
				bins[(rel + FFT_N) % FFT_N] = cf(0, 0);
			}
			if(l == 6) {
				std::vector<cf> d = pss_freq(nid_ % 3);
				cf mapped[FFT_N];
				map62(d.data(), mapped);
				for(int i = 0; i < FFT_N; i++) {
					if(mapped[i] != cf(0, 0)) {
						bins[i] = mapped[i];
					}
				}
			} else {
				std::vector<float> d = sss_seq(nid_ / 3, nid_ % 3, ns == 0 ? 0 : 1);
				for(int n = 0; n < 62; n++) {
					int k = n < 31 ? n - 31 : n - 30;
					bins[(k + FFT_N) % FFT_N] = cf(d[n], 0);
				}
			}
		}
		// IFFT + CP
		cf td[FFT_N], tmp[FFT_N];
		for(int i = 0; i < FFT_N; i++) {
			tmp[i] = std::conj(bins[i]);
		}
		fft_.run(tmp, td);
		for(int i = 0; i < FFT_N; i++) {
			td[i] = std::conj(td[i]) * (1.0f / FFT_N);
		}
		int cp = cp_len(l);
		for(int i = FFT_N - cp; i < FFT_N; i++) {
			out.push_back(td[i]);
		}
		out.insert(out.end(), td, td + FFT_N);
	}

	int nid_, nrb_;
	std::mt19937 rng_;
	fft_plan fft_;
};

// ---- receive-side impairments ---------------------------------------------------------

inline void apply_channel(std::vector<cf> &x, float a2, float phase2, int delay2) {
	std::vector<cf> y(x.size());
	cf tap2 = std::polar(a2, phase2);
	for(size_t i = 0; i < x.size(); i++) {
		y[i] = x[i] + (i >= (size_t)delay2 ? x[i - delay2] * tap2 : cf(0, 0));
	}
	x.swap(y);
}

inline void apply_cfo(std::vector<cf> &x, double f_hz, double fs) {
	double step = 2.0 * M_PI * f_hz / fs;
	double ph = 0;
	for(auto &v : x) {
		v *= cf((float)cos(ph), (float)sin(ph));
		ph += step;
		if(ph > M_PI) {
			ph -= 2 * M_PI;
		}
		if(ph < -M_PI) {
			ph += 2 * M_PI;
		}
	}
}

// receiver clock fast by eps -> events span MORE receiver samples: output[n] = x(n/(1+eps))
inline std::vector<cf> apply_sco(const std::vector<cf> &x, double eps) {
	std::vector<cf> y;
	y.reserve(x.size());
	double step = 1.0 / (1.0 + eps);
	double t = 2.0;
	while(t < (double)x.size() - 3.0) {
		int i = (int)t;
		double mu = t - i;
		// cubic Lagrange on x[i-1..i+2]
		cf xm1 = x[i - 1], x0 = x[i], x1 = x[i + 1], x2 = x[i + 2];
		double c0 = -mu * (mu - 1) * (mu - 2) / 6.0;
		double c1 = (mu + 1) * (mu - 1) * (mu - 2) / 2.0;
		double c2 = -mu * (mu + 1) * (mu - 2) / 2.0;
		double c3 = mu * (mu + 1) * (mu - 1) / 6.0;
		y.push_back(xm1 * (float)c0 + x0 * (float)c1 + x1 * (float)c2 + x2 * (float)c3);
		t += step;
	}
	return y;
}

inline void add_awgn(std::vector<cf> &x, float snr_db, uint32_t seed) {
	std::mt19937 rng(seed);
	float sigma = sqrtf(0.5f * powf(10.0f, -snr_db / 10.0f));   // signal RMS is 1
	std::normal_distribution<float> nd(0.0f, sigma);
	for(auto &v : x) {
		v += cf(nd(rng), nd(rng));
	}
}

} // namespace cellsync

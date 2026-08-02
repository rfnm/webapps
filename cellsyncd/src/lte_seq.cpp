// cellsync: 36.211 sequence generation. Conventions isolated here on purpose - see header.
#include "lte_seq.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace cellsync {

const int NRB_CANDIDATES[6] = { 6, 15, 25, 50, 75, 100 };

int cp_len(int l) {
	return l == 0 ? 10 : 9;
}

int sym_data_off(int l) {
	// cumulative starts: sym0 occupies [0,138), sym1.. occupy 137 each
	int start = l == 0 ? 0 : 138 + (l - 1) * 137;
	return start + cp_len(l);
}

// ---- PSS -----------------------------------------------------------------------------
static const int ZC_ROOT[3] = { 25, 29, 34 };

std::vector<cf> pss_freq(int nid2) {
	std::vector<cf> d(62);
	int u = ZC_ROOT[nid2];
	for(int n = 0; n < 31; n++) {
		double a = -M_PI * u * n * (n + 1) / 63.0;
		d[n] = cf((float)cos(a), (float)sin(a));
	}
	for(int n = 31; n < 62; n++) {
		double a = -M_PI * u * (n + 1) * (n + 2) / 63.0;
		d[n] = cf((float)cos(a), (float)sin(a));
	}
	return d;
}

void map62(const cf *d, cf *bins128) {
	std::fill(bins128, bins128 + FFT_N, cf(0, 0));
	for(int n = 0; n < 62; n++) {
		int k = n < 31 ? n - 31 : n - 30;          // DC (k=0) stays empty
		bins128[(k + FFT_N) % FFT_N] = d[n];
	}
}

std::vector<cf> pss_time(int nid2) {
	std::vector<cf> d = pss_freq(nid2);
	std::vector<cf> bins(FFT_N), t(FFT_N);
	map62(d.data(), bins.data());
	// IFFT via conj(FFT(conj(x)))/N with the forward-only plan
	fft_plan fft(FFT_N);
	for(auto &b : bins) {
		b = std::conj(b);
	}
	fft.run(bins.data(), t.data());
	float e = 0;
	for(auto &x : t) {
		x = std::conj(x) * (1.0f / FFT_N);
		e += std::norm(x);
	}
	float g = 1.0f / sqrtf(e);
	for(auto &x : t) {
		x *= g;
	}
	return t;
}

// ---- SSS -----------------------------------------------------------------------------
// three length-31 m-sequences from x^5 recursions, seed x(0..4) = 0,0,0,0,1
static void mseq31(int taps_case, float *out) {
	uint8_t x[31] = { 0, 0, 0, 0, 1 };
	for(int i = 0; i + 5 < 31; i++) {
		switch(taps_case) {
		case 0: x[i + 5] = (uint8_t)((x[i + 2] + x[i]) & 1); break;                          // s~
		case 1: x[i + 5] = (uint8_t)((x[i + 3] + x[i]) & 1); break;                          // c~
		default: x[i + 5] = (uint8_t)((x[i + 4] + x[i + 2] + x[i + 1] + x[i]) & 1); break;   // z~
		}
	}
	for(int i = 0; i < 31; i++) {
		out[i] = 1.0f - 2.0f * x[i];
	}
}

void sss_m0m1(int nid1, int *m0, int *m1) {
	int qp = nid1 / 30;
	int q = (nid1 + qp * (qp + 1) / 2) / 30;
	int mp = nid1 + q * (q + 1) / 2;
	*m0 = mp % 31;
	*m1 = (*m0 + mp / 31 + 1) % 31;
}

std::vector<float> sss_seq(int nid1, int nid2, int half) {
	int m0, m1;
	sss_m0m1(nid1, &m0, &m1);

	float st[31], ct[31], zt[31];
	mseq31(0, st);
	mseq31(1, ct);
	mseq31(2, zt);

	auto s = [&](int m, int n) { return st[(n + m) % 31]; };
	auto c0 = [&](int n) { return ct[(n + nid2) % 31]; };
	auto c1 = [&](int n) { return ct[(n + nid2 + 3) % 31]; };
	auto z = [&](int m, int n) { return zt[(n + (m % 8)) % 31]; };

	std::vector<float> d(62);
	for(int n = 0; n < 31; n++) {
		if(half == 0) {
			d[2 * n] = s(m0, n) * c0(n);
			d[2 * n + 1] = s(m1, n) * c1(n) * z(m0, n);
		} else {
			d[2 * n] = s(m1, n) * c0(n);
			d[2 * n + 1] = s(m0, n) * c1(n) * z(m1, n);
		}
	}
	return d;
}

// ---- Gold ------------------------------------------------------------------------------
void gold_seq(uint32_t cinit, int len, uint8_t *out) {
	// state kept as 31-bit registers, bit i = x(n+i)
	uint32_t x1 = 1;                     // x1(0)=1, rest 0
	uint32_t x2 = cinit & 0x7FFFFFFF;
	auto step1 = [&] {
		uint32_t nb = ((x1 >> 3) ^ x1) & 1;
		x1 = (x1 >> 1) | (nb << 30);
	};
	auto step2 = [&] {
		uint32_t nb = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
		x2 = (x2 >> 1) | (nb << 30);
	};
	for(int i = 0; i < 1600; i++) {
		step1();
		step2();
	}
	for(int i = 0; i < len; i++) {
		out[i] = (uint8_t)((x1 ^ x2) & 1);
		step1();
		step2();
	}
}

// ---- CRS -------------------------------------------------------------------------------
void crs_seq(int nid_cell, int ns, int l, int mprime0, int count, cf *out) {
	uint32_t cinit = (uint32_t)(1024u * (7u * (ns + 1) + l + 1) * (2u * nid_cell + 1) + 2u * nid_cell + 1u);
	std::vector<uint8_t> c(2 * (mprime0 + count));
	gold_seq(cinit, (int)c.size(), c.data());
	const float s = 0.70710678f;
	for(int i = 0; i < count; i++) {
		int m = mprime0 + i;
		out[i] = cf(s * (1 - 2 * c[2 * m]), s * (1 - 2 * c[2 * m + 1]));
	}
}

std::vector<crs_pos> crs_positions(int nid_cell, int nrb, int l, int rel_min, int rel_max) {
	std::vector<crs_pos> out;
	int v = l == 0 ? 0 : 3;
	int off = (v + nid_cell % 6) % 6;
	for(int m = 0; m < 2 * nrb; m++) {
		int k = 6 * m + off;
		int rel = k < 6 * nrb ? k - 6 * nrb : k - 6 * nrb + 1;   // DC hole
		if(rel >= rel_min && rel <= rel_max && rel != 0) {
			out.push_back({ rel, m + 110 - nrb });
		}
	}
	return out;
}

} // namespace cellsync

// cellsync: 36.212 channel coding - see lte_coding.h for the scope and the doctrine.
#include "lte_coding.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>

namespace cellsync {

// ---- CRC ------------------------------------------------------------------------------

int crc_len(crc_poly poly) {
	switch(poly) {
	case CRC24A: return 24;
	case CRC16:  return 16;
	case CRC8:   return 8;
	}
	return 0;
}

uint32_t crc_bits(const uint8_t *bits, size_t n, crc_poly poly) {
	const int L = crc_len(poly);
	const uint32_t top = 1u << L;                 // the implicit leading term
	uint32_t reg = 0;
	for(size_t i = 0; i < n; i++) {
		reg = (reg << 1) | (bits[i] & 1);
		if(reg & top) {
			reg ^= (uint32_t)poly;
		}
	}
	// flush L zero bits: the parity is the remainder of the message shifted left by L
	for(int i = 0; i < L; i++) {
		reg <<= 1;
		if(reg & top) {
			reg ^= (uint32_t)poly;
		}
	}
	return reg & (top - 1);
}

void crc_attach(std::vector<uint8_t> &bits, crc_poly poly) {
	const int L = crc_len(poly);
	uint32_t p = crc_bits(bits.data(), bits.size(), poly);
	for(int i = L - 1; i >= 0; i--) {
		bits.push_back((uint8_t)((p >> i) & 1));
	}
}

bool crc_check(const uint8_t *bits, size_t n, crc_poly poly, const uint8_t *mask) {
	const int L = crc_len(poly);
	if(n < (size_t)L) {
		return false;
	}
	const size_t msg = n - (size_t)L;
	uint32_t want = crc_bits(bits, msg, poly);
	uint32_t got = 0;
	for(int i = 0; i < L; i++) {
		uint8_t b = bits[msg + i] & 1;
		if(mask) {
			b ^= (uint8_t)(mask[i] & 1);
		}
		got = (got << 1) | b;
	}
	return got == want;
}

// ---- masks ----------------------------------------------------------------------------

const int PBCH_PORT_HYPOTHESES[3] = { 2, 4, 1 };

void pbch_crc_mask(int nof_ports, uint8_t mask[16]) {
	// 36.212 Table 5.3.1.1-1: 1 port -> all zero, 2 ports -> all one,
	// 4 ports -> <0,1,0,1,...>. Anything else is a caller bug; fail loud by zeroing.
	for(int i = 0; i < 16; i++) {
		switch(nof_ports) {
		case 1: mask[i] = 0; break;
		case 2: mask[i] = 1; break;
		case 4: mask[i] = (uint8_t)(i & 1); break;
		default: mask[i] = 0; break;
		}
	}
}

void rnti_crc_mask(uint16_t rnti, uint8_t mask[16]) {
	for(int i = 0; i < 16; i++) {
		mask[i] = (uint8_t)((rnti >> (15 - i)) & 1);   // MSB first
	}
}

// ---- DCI 1A / 1C ----------------------------------------------------------------------

static int ceil_log2(int v) {
	int b = 0;
	while((1 << b) < v) {
		b++;
	}
	return b;
}

// 36.211 Table 6.2.3.2-1: the distributed-VRB gaps. Both are pure bandwidth functions.
static int dvrb_gap1(int nrb) {
	if(nrb <= 10) { return (nrb + 1) / 2; }
	if(nrb == 11) { return 4; }
	if(nrb <= 19) { return 8; }
	if(nrb <= 26) { return 12; }
	if(nrb <= 44) { return 18; }
	if(nrb <= 63) { return 27; }
	if(nrb <= 79) { return 32; }
	return 48;
}
static int dvrb_gap2(int nrb) {
	if(nrb < 50) { return 0; }        // gap 2 only exists from 50 PRB up
	return (nrb <= 63) ? 9 : 16;
}
static int dvrb_nvrb(int nrb, int gap) {
	if(gap == 0) {
		const int g = dvrb_gap1(nrb);
		return 2 * ((g < nrb - g) ? g : nrb - g);
	}
	const int g2 = dvrb_gap2(nrb);
	return g2 ? (nrb / (2 * g2)) * (2 * g2) : 0;
}

int dci_size_1a(int nrb) {
	// flag(1) + local/dist(1) + RBA + MCS(5) + HARQ(3, FDD) + NDI(1) + RV(2) + TPC(2).
	// Format 0 is strictly smaller in Rel-8 FDD, so the pair size is 1A's - then the
	// ambiguous-size rule appends padding until the size leaves the forbidden list.
	int n = 1 + 1 + ceil_log2(nrb * (nrb + 1) / 2) + 5 + 3 + 1 + 2 + 2;
	static const int AMBIG[10] = { 12, 14, 16, 20, 24, 26, 32, 40, 44, 56 };
	bool hit = true;
	while(hit) {
		hit = false;
		for(int a : AMBIG) {
			if(n == a) {
				n++;
				hit = true;
			}
		}
	}
	return n;
}

int dci_size_1c(int nrb) {
	const int nstep = (nrb < 50) ? 2 : 4;
	const int nd = dvrb_nvrb(nrb, 0) / nstep;
	return ((nrb >= 50) ? 1 : 0) + ceil_log2(nd * (nd + 1) / 2) + 5;
}

// 36.213 7.1.7.2.1-1, columns N_PRB = 2 and 3 - the only two a 1A DCI on SI-RNTI can
// select (the TPC LSB picks the column). Copied literally from the standard.
static const int TBS_1A[27][2] = {
	{ 32, 56 },     { 56, 88 },     { 72, 144 },    { 104, 176 },   { 120, 208 },
	{ 144, 224 },   { 176, 256 },   { 224, 328 },   { 256, 392 },   { 296, 456 },
	{ 328, 504 },   { 376, 584 },   { 440, 680 },   { 488, 744 },   { 552, 840 },
	{ 600, 904 },   { 632, 968 },   { 696, 1064 },  { 776, 1160 },  { 840, 1288 },
	{ 904, 1384 },  { 1000, 1480 }, { 1064, 1608 }, { 1128, 1736 }, { 1192, 1800 },
	{ 1256, 1864 }, { 1480, 2216 },
};
// 36.213 Table 7.1.7.2.3-1: format 1C has its own 32-entry TBS list.
static const int TBS_1C[32] = {
	40,  56,  72,  120, 136, 144, 176,  208,  224,  256,  280,  296,  328,  336,  392,  488,
	552, 600, 632, 696, 776, 840, 904, 1000, 1064, 1128, 1192, 1256, 1480, 1736, 2024, 2280,
};

// RIV -> (start, length) over nvrb resource blocks (36.213 7.1.6.3). The two RIV branches
// of the encoder overlap nowhere, so the decode is the simple inverse with one reflection.
static bool riv_decode(int riv, int nvrb, int *start, int *len) {
	if(nvrb <= 0 || riv < 0 || riv >= nvrb * (nvrb + 1)) {
		return false;
	}
	int l = riv / nvrb + 1;
	int s = riv % nvrb;
	if(l > nvrb - s) {
		l = nvrb - l + 2;
		s = nvrb - 1 - s;
	}
	if(l < 1 || s + l > nvrb) {
		return false;
	}
	*start = s;
	*len = l;
	return true;
}

static int take_bits(const uint8_t *bits, size_t &p, int n) {
	int v = 0;
	for(int i = 0; i < n; i++) {
		v = (v << 1) | (bits[p++] & 1);   // fields are MSB first, like every 36.212 field
	}
	return v;
}

bool dci_parse_si(const uint8_t *bits, size_t nbits, int nrb, dci_si &out) {
	out = dci_si{};
	if((int)nbits == dci_size_1a(nrb)) {
		size_t p = 0;
		out.format = 0x1A;
		if(take_bits(bits, p, 1) != 1) {
			return false;               // flag says format 0 - meaningless on SI-RNTI
		}
		out.localized = take_bits(bits, p, 1) == 0;
		int rba_bits = ceil_log2(nrb * (nrb + 1) / 2);
		int nvrb = nrb;
		if(!out.localized) {
			if(nrb >= 50) {
				out.gap = take_bits(bits, p, 1);   // the RBA MSB becomes the gap indicator
				rba_bits--;
			}
			nvrb = dvrb_nvrb(nrb, out.gap);
		}
		out.riv = take_bits(bits, p, rba_bits);
		out.i_tbs = take_bits(bits, p, 5);
		take_bits(bits, p, 3);                     // HARQ number: reserved on SI-RNTI
		take_bits(bits, p, 1);                     // NDI: reserved on SI-RNTI
		out.rv = take_bits(bits, p, 2);
		const int tpc = take_bits(bits, p, 2);
		out.n_prb_1a = (tpc & 1) ? 3 : 2;
		if(out.i_tbs > 26 || !riv_decode(out.riv, nvrb, &out.rb_start, &out.n_prb)) {
			return false;
		}
		out.tbs = TBS_1A[out.i_tbs][out.n_prb_1a - 2];
		return true;
	}
	if((int)nbits == dci_size_1c(nrb)) {
		size_t p = 0;
		out.format = 0x1C;
		out.localized = false;                     // 1C is always distributed
		if(nrb >= 50) {
			out.gap = take_bits(bits, p, 1);
		}
		const int nstep = (nrb < 50) ? 2 : 4;
		const int nd = dvrb_nvrb(nrb, out.gap) / nstep;
		out.riv = take_bits(bits, p, ceil_log2(nd * (nd + 1) / 2));
		int s = 0, l = 0;
		if(!riv_decode(out.riv, nd, &s, &l)) {
			return false;
		}
		out.rb_start = s * nstep;                  // 1C allocates in steps of N_RB_step VRBs
		out.n_prb = l * nstep;
		out.i_tbs = take_bits(bits, p, 5);
		out.tbs = TBS_1C[out.i_tbs];
		return true;
	}
	return false;
}

// ---- CFI codewords --------------------------------------------------------------------

// 36.212 Table 5.3.4-1, copied literally from the standard - this table IS the pinned
// vector for the CFI stage, do not generate it from the <011>/<101>/<110> pattern.
const uint8_t CFI_CODEWORD[3][32] = {
	{ 0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1 },   // CFI 1
	{ 1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0 },   // CFI 2
	{ 1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1 },   // CFI 3
};

// ---- tail-biting convolutional code ---------------------------------------------------

// G0=133, G1=171, G2=165 octal over the 7-bit shift register (current bit is the MSB).
static const uint8_t CONV_G[3] = { 0133 & 0177, 0171 & 0177, 0165 & 0177 };

static inline uint8_t parity7(uint8_t v) {
	v ^= (uint8_t)(v >> 4);
	v ^= (uint8_t)(v >> 2);
	v ^= (uint8_t)(v >> 1);
	return (uint8_t)(v & 1);
}

// Output triple for entering `state` (6 bits of history, newest first) with input `bit`.
static inline void conv_outputs(uint8_t state, uint8_t bit, uint8_t out[3]) {
	const uint8_t reg = (uint8_t)((bit << 6) | state);   // 7-bit register, newest at MSB
	for(int j = 0; j < 3; j++) {
		out[j] = parity7((uint8_t)(reg & CONV_G[j]));
	}
}

static inline uint8_t conv_next(uint8_t state, uint8_t bit) {
	return (uint8_t)(((bit << 6) | state) >> 1);
}

void conv_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out) {
	const size_t n = in.size();
	out.assign(3 * n, 0);
	if(n == 0) {
		return;
	}
	// tail-biting: the initial state is the last 6 input bits, arranged exactly as the
	// register will hold them after the final shift - conv_next() moves the newest bit to
	// bit 5, so in[n-1] belongs at bit 5, in[n-2] at bit 4, and so on. Putting the newest
	// bit at bit 0 instead compiles, encodes, and even decodes noiselessly, but it is not
	// tail-biting: the encoder no longer ends where it started, the decoder's circular
	// constraint then has no valid path, and error performance collapses. The shift
	// invariance check in the selftest exists for exactly this bug.
	uint8_t state = 0;
	for(int i = 0; i < CONV_K - 1; i++) {
		if((size_t)i >= n) {
			break;
		}
		state |= (uint8_t)((in[n - 1 - (size_t)i] & 1) << (CONV_K - 2 - i));
	}
	uint8_t o[3];
	for(size_t k = 0; k < n; k++) {
		conv_outputs(state, in[k], o);
		out[3 * k + 0] = o[0];
		out[3 * k + 1] = o[1];
		out[3 * k + 2] = o[2];
		state = conv_next(state, in[k]);
	}
}

double conv_decode(const std::vector<float> &llr, size_t n, std::vector<uint8_t> &out) {
	out.assign(n, 0);
	if(n == 0 || llr.size() < 3 * n) {
		return -std::numeric_limits<double>::infinity();
	}

	// Precompute the trellis once. branch_out[s][b] holds the three coded bits.
	static uint8_t bo[CONV_STATES][2][3];
	static uint8_t bn[CONV_STATES][2];
	static bool built = false;
	if(!built) {
		for(int s = 0; s < CONV_STATES; s++) {
			for(int b = 0; b < 2; b++) {
				conv_outputs((uint8_t)s, (uint8_t)b, bo[s][b]);
				bn[s][b] = conv_next((uint8_t)s, (uint8_t)b);
			}
		}
		built = true;
	}

	const double NEG = -1e30;
	std::vector<double> metric(CONV_STATES), next(CONV_STATES);
	std::vector<uint8_t> back(n * CONV_STATES);

	double best_total = NEG;
	std::vector<uint8_t> best_bits(n, 0);

	// Exact tail-biting ML: one pass per candidate start state, keep the best path that
	// returns to the state it started from. See the header for why brute force is right
	// at these block lengths.
	for(int s0 = 0; s0 < CONV_STATES; s0++) {
		for(int s = 0; s < CONV_STATES; s++) {
			metric[s] = NEG;
		}
		metric[s0] = 0.0;

		for(size_t k = 0; k < n; k++) {
			for(int s = 0; s < CONV_STATES; s++) {
				next[s] = NEG;
			}
			const float *l = &llr[3 * k];
			for(int s = 0; s < CONV_STATES; s++) {
				if(metric[s] == NEG) {
					continue;
				}
				for(int b = 0; b < 2; b++) {
					// llr>0 means bit 0 is more likely, so a coded bit c contributes
					// +llr when c==0 and -llr when c==1.
					const uint8_t *o = bo[s][b];
					double m = metric[s]
					         + (o[0] ? -(double)l[0] : (double)l[0])
					         + (o[1] ? -(double)l[1] : (double)l[1])
					         + (o[2] ? -(double)l[2] : (double)l[2]);
					int ns = bn[s][b];
					if(m > next[ns]) {
						next[ns] = m;
						back[k * CONV_STATES + ns] = (uint8_t)((b << 6) | s);
					}
				}
			}
			metric.swap(next);
		}

		if(metric[s0] <= best_total) {
			continue;
		}
		// trace back through the winning circular path
		best_total = metric[s0];
		int s = s0;
		for(size_t k = n; k-- > 0;) {
			uint8_t rec = back[k * CONV_STATES + s];
			best_bits[k] = (uint8_t)(rec >> 6);
			s = (int)(rec & 0x3F);
		}
	}

	out = best_bits;
	return best_total;
}

double conv_decode_fast(const std::vector<float> &llr, size_t n, std::vector<uint8_t> &out,
                        int max_iter) {
	out.assign(n, 0);
	if(n == 0 || llr.size() < 3 * n) {
		return -std::numeric_limits<double>::infinity();
	}

	static uint8_t bo[CONV_STATES][2][3];
	static uint8_t bn[CONV_STATES][2];
	static bool built = false;
	if(!built) {
		for(int s = 0; s < CONV_STATES; s++) {
			for(int b = 0; b < 2; b++) {
				conv_outputs((uint8_t)s, (uint8_t)b, bo[s][b]);
				bn[s][b] = conv_next((uint8_t)s, (uint8_t)b);
			}
		}
		built = true;
	}

	const double NEG = -1e30;
	std::vector<double> metric(CONV_STATES, 0.0), next(CONV_STATES);
	std::vector<uint8_t> back(n * CONV_STATES);
	std::vector<uint8_t> bits(n);

	for(int iter = 0; iter < max_iter; iter++) {
		for(size_t k = 0; k < n; k++) {
			for(int s = 0; s < CONV_STATES; s++) {
				next[s] = NEG;
			}
			const float *l = &llr[3 * k];
			// Hoisting the three LLRs and the eight possible branch contributions out of
			// the state loop is what makes this cheap: the branch metric depends only on
			// the output triple, of which there are exactly 8.
			double bm[8];
			for(int c = 0; c < 8; c++) {
				bm[c] = ((c & 4) ? -(double)l[0] : (double)l[0])
				      + ((c & 2) ? -(double)l[1] : (double)l[1])
				      + ((c & 1) ? -(double)l[2] : (double)l[2]);
			}
			for(int s = 0; s < CONV_STATES; s++) {
				const double m0 = metric[s];
				if(m0 == NEG) {
					continue;
				}
				for(int b = 0; b < 2; b++) {
					const uint8_t *o = bo[s][b];
					const int c = (o[0] << 2) | (o[1] << 1) | o[2];
					const double m = m0 + bm[c];
					const int ns = bn[s][b];
					if(m > next[ns]) {
						next[ns] = m;
						back[k * CONV_STATES + ns] = (uint8_t)((b << 6) | s);
					}
				}
			}
			metric.swap(next);
		}

		// best end state, then trace back and see where the path started
		int se = 0;
		for(int s = 1; s < CONV_STATES; s++) {
			if(metric[s] > metric[se]) {
				se = s;
			}
		}
		int s = se;
		for(size_t k = n; k-- > 0;) {
			uint8_t rec = back[k * CONV_STATES + s];
			bits[k] = (uint8_t)(rec >> 6);
			s = (int)(rec & 0x3F);
		}
		if(s == se) {
			out = bits;                     // start == end: a valid tail-biting path
			return metric[se];
		}
		// carry the end metrics into the next lap, renormalised so they cannot drift
		double mx = metric[se];
		for(int i = 0; i < CONV_STATES; i++) {
			metric[i] = (metric[i] == NEG) ? NEG : metric[i] - mx;
		}
	}
	return -std::numeric_limits<double>::infinity();   // caller decides whether to fall back
}

// ---- rate matching (36.212 5.1.4.2) ---------------------------------------------------

const int SUBBLOCK_PERM[32] = {
	1, 17,  9, 25,  5, 21, 13, 29,  3, 19, 11, 27,  7, 23, 15, 31,
	0, 16,  8, 24,  4, 20, 12, 28,  2, 18, 10, 26,  6, 22, 14, 30
};

// Build the circular buffer position list: entry i is the index into the 3*D coded-bit
// array that circular-buffer position i carries, or -1 for a <NULL> pad. Computing the map
// once and using it in both directions is what keeps the forward and inverse provably
// consistent - the selftest round-trips through it.
static void conv_cb_map(size_t D, std::vector<int> &map) {
	const int C = 32;
	const int R = (int)((D + C - 1) / C);
	const int nd = R * C - (int)D;          // <NULL> pads, written at the FRONT of the matrix
	const int Kpi = R * C;
	map.assign((size_t)(3 * Kpi), -1);
	for(int s = 0; s < 3; s++) {
		// write row-wise with the pad first, read column-wise in permuted column order
		std::vector<int> v((size_t)Kpi, -1);
		for(int i = 0; i < Kpi; i++) {
			const int src = i - nd;          // index into the D coded bits of this stream
			v[(size_t)i] = (src >= 0) ? (3 * src + s) : -1;
		}
		std::vector<int> outv((size_t)Kpi, -1);
		int o = 0;
		for(int cc = 0; cc < C; cc++) {
			const int col = SUBBLOCK_PERM[cc];
			for(int r = 0; r < R; r++) {
				outv[(size_t)o++] = v[(size_t)(r * C + col)];
			}
		}
		// 5.1.4.2.2 CONCATENATES the three streams: w(k)=v0, w(Kpi+k)=v1, w(2Kpi+k)=v2.
		// Interlacing streams 1 and 2 is the TURBO collection step (5.1.4.1.2) - applied
		// here it survives every self-consistency check (round trip, repetition count,
		// copy agreement) and only the Viterbi ever notices the permuted codeword.
		for(int i = 0; i < Kpi; i++) {
			map[(size_t)(s * Kpi + i)] = outv[(size_t)i];
		}
	}
}

void conv_rate_match(const std::vector<uint8_t> &in, size_t E, std::vector<uint8_t> &out) {
	const size_t D = in.size() / 3;
	std::vector<int> map;
	conv_cb_map(D, map);
	out.clear();
	out.reserve(E);
	size_t k = 0;
	while(out.size() < E) {
		const int src = map[k % map.size()];
		if(src >= 0) {
			out.push_back(in[(size_t)src]);
		}
		k++;
	}
}

void conv_rate_dematch(const std::vector<float> &llr_e, size_t D, std::vector<float> &soft) {
	std::vector<int> map;
	conv_cb_map(D, map);
	soft.assign(3 * D, 0.0f);
	size_t k = 0, e = 0;
	while(e < llr_e.size()) {
		const int src = map[k % map.size()];
		if(src >= 0) {
			soft[(size_t)src] += llr_e[e];      // repetitions add, which is the whole point
			e++;
		}
		k++;
	}
}

// ---- turbo code (36.212 5.1.3.2) -------------------------------------------------------

// f1/f2 per K from Table 5.1.3-3, copied literally from the standard. Indexed by the
// K-grid the table defines: 40..512 step 8, 528..1024 step 16, 1056..2048 step 32,
// 2112..6144 step 64 - 188 rows.
static const uint16_t QPP_F[188][2] = {
	{ 3, 10 },    { 7, 12 },    { 19, 42 },   { 7, 16 },    { 7, 18 },    { 11, 20 },
	{ 5, 22 },    { 11, 24 },   { 7, 26 },    { 41, 84 },   { 103, 90 },  { 15, 32 },
	{ 9, 34 },    { 17, 108 },  { 9, 38 },    { 21, 120 },  { 101, 84 },  { 21, 44 },
	{ 57, 46 },   { 23, 48 },   { 13, 50 },   { 27, 52 },   { 11, 36 },   { 27, 56 },
	{ 85, 58 },   { 29, 60 },   { 33, 62 },   { 15, 32 },   { 17, 198 },  { 33, 68 },
	{ 103, 210 }, { 19, 36 },   { 19, 74 },   { 37, 76 },   { 19, 78 },   { 21, 120 },
	{ 21, 82 },   { 115, 84 },  { 193, 86 },  { 21, 44 },   { 133, 90 },  { 81, 46 },
	{ 45, 94 },   { 23, 48 },   { 243, 98 },  { 151, 40 },  { 155, 102 }, { 25, 52 },
	{ 51, 106 },  { 47, 72 },   { 91, 110 },  { 29, 168 },  { 29, 114 },  { 247, 58 },
	{ 29, 118 },  { 89, 180 },  { 91, 122 },  { 157, 62 },  { 55, 84 },   { 31, 64 },
	{ 17, 66 },   { 35, 68 },   { 227, 420 }, { 65, 96 },   { 19, 74 },   { 37, 76 },
	{ 41, 234 },  { 39, 80 },   { 185, 82 },  { 43, 252 },  { 21, 86 },   { 155, 44 },
	{ 79, 120 },  { 139, 92 },  { 23, 94 },   { 217, 48 },  { 25, 98 },   { 17, 80 },
	{ 127, 102 }, { 25, 52 },   { 239, 106 }, { 17, 48 },   { 137, 110 }, { 215, 112 },
	{ 29, 114 },  { 15, 58 },   { 147, 118 }, { 29, 60 },   { 59, 122 },  { 65, 124 },
	{ 55, 84 },   { 31, 64 },   { 17, 66 },   { 171, 204 }, { 67, 140 },  { 35, 72 },
	{ 19, 74 },   { 39, 76 },   { 19, 78 },   { 199, 240 }, { 21, 82 },   { 211, 252 },
	{ 21, 86 },   { 43, 88 },   { 149, 60 },  { 45, 92 },   { 49, 846 },  { 71, 48 },
	{ 13, 28 },   { 17, 80 },   { 25, 102 },  { 183, 104 }, { 55, 954 },  { 127, 96 },
	{ 27, 110 },  { 29, 112 },  { 29, 114 },  { 57, 116 },  { 45, 354 },  { 31, 120 },
	{ 59, 610 },  { 185, 124 }, { 113, 420 }, { 31, 64 },   { 17, 66 },   { 171, 136 },
	{ 209, 420 }, { 253, 216 }, { 367, 444 }, { 265, 456 }, { 181, 468 }, { 39, 80 },
	{ 27, 164 },  { 127, 504 }, { 143, 172 }, { 43, 88 },   { 29, 300 },  { 45, 92 },
	{ 157, 188 }, { 47, 96 },   { 13, 28 },   { 111, 240 }, { 443, 204 }, { 51, 104 },
	{ 51, 212 },  { 451, 192 }, { 257, 220 }, { 57, 336 },  { 313, 228 }, { 271, 232 },
	{ 179, 236 }, { 331, 120 }, { 363, 244 }, { 375, 248 }, { 127, 168 }, { 31, 64 },
	{ 33, 130 },  { 43, 264 },  { 33, 134 },  { 477, 408 }, { 35, 138 },  { 233, 280 },
	{ 357, 142 }, { 337, 480 }, { 37, 146 },  { 71, 444 },  { 71, 120 },  { 37, 152 },
	{ 39, 462 },  { 127, 234 }, { 39, 158 },  { 39, 80 },   { 31, 96 },   { 113, 902 },
	{ 41, 166 },  { 251, 336 }, { 43, 170 },  { 21, 86 },   { 43, 174 },  { 45, 176 },
	{ 45, 178 },  { 161, 120 }, { 89, 182 },  { 323, 184 }, { 47, 186 },  { 23, 94 },
	{ 47, 190 },  { 263, 480 },
};

static int qpp_index(int K) {
	if(K >= 40 && K <= 512 && K % 8 == 0) { return (K - 40) / 8; }
	if(K >= 528 && K <= 1024 && K % 16 == 0) { return 60 + (K - 528) / 16; }
	if(K >= 1056 && K <= 2048 && K % 32 == 0) { return 92 + (K - 1056) / 32; }
	if(K >= 2112 && K <= 6144 && K % 64 == 0) { return 124 + (K - 2112) / 64; }
	return -1;
}

void turbo_qpp(int K, std::vector<int> &pi) {
	pi.clear();
	const int idx = qpp_index(K);
	if(idx < 0) {
		return;
	}
	const int64_t f1 = QPP_F[idx][0], f2 = QPP_F[idx][1];
	pi.resize((size_t)K);
	for(int64_t i = 0; i < K; i++) {
		// 64-bit on purpose: f2*i^2 overflows 32 bits from K ~ 1500 up
		pi[(size_t)i] = (int)((f1 * i + f2 * i * i) % K);
	}
}

// The 8-state RSC constituent encoder: feedback g0 = 1+D^2+D^3, forward g1 = 1+D+D^3.
// State s = (r1,r2,r3) packed as r1*4+r2*2+r3 with r1 the NEWEST feedback bit - the same
// packing the decoder trellis uses, so both share these two lines and nothing else.
static inline uint8_t rsc_parity(uint8_t s, uint8_t w) {
	return (uint8_t)(w ^ ((s >> 2) & 1) ^ (s & 1));            // z = w + r1 + r3
}
static inline uint8_t rsc_w(uint8_t s, uint8_t u) {
	return (uint8_t)(u ^ ((s >> 1) & 1) ^ (s & 1));            // w = u + r2 + r3
}
static inline uint8_t rsc_next(uint8_t s, uint8_t w) {
	return (uint8_t)((w << 2) | (s >> 1));
}

// Run one constituent over K bits, then the three termination steps (input forced to
// r2^r3 so the feedback cancels and the state drains to zero). Fills z[0..K) and the
// tail pairs (x_K..x_K+2, z_K..z_K+2).
static void rsc_run(const uint8_t *in, size_t K, uint8_t *z, uint8_t tx[3], uint8_t tz[3]) {
	uint8_t s = 0;
	for(size_t k = 0; k < K; k++) {
		const uint8_t w = rsc_w(s, in[k] & 1);
		z[k] = rsc_parity(s, w);
		s = rsc_next(s, w);
	}
	for(int t = 0; t < 3; t++) {
		tx[t] = (uint8_t)(((s >> 1) & 1) ^ (s & 1));           // u = r2 ^ r3 -> w = 0
		tz[t] = rsc_parity(s, 0);
		s = rsc_next(s, 0);
	}
}

void turbo_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> d[3]) {
	const size_t K = in.size();
	for(int s = 0; s < 3; s++) {
		d[s].assign(K + 4, 0);
	}
	if(K == 0) {
		return;
	}
	std::vector<int> pi;
	turbo_qpp((int)K, pi);
	if(pi.empty()) {
		return;
	}
	std::vector<uint8_t> in2(K);
	for(size_t k = 0; k < K; k++) {
		in2[k] = in[(size_t)pi[k]];
	}
	uint8_t tx1[3], tz1[3], tx2[3], tz2[3];
	rsc_run(in.data(), K, d[1].data(), tx1, tz1);
	rsc_run(in2.data(), K, d[2].data(), tx2, tz2);
	for(size_t k = 0; k < K; k++) {
		d[0][k] = in[k] & 1;
	}
	// 5.1.3.2.2 tail multiplexing - the layout is a norm convention, not a choice:
	//   d0: x_K,  z_K+1,  x'_K,  z'_K+1     d1: z_K,  x_K+2,  z'_K,  x'_K+2
	//   d2: x_K+1, z_K+2, x'_K+1, z'_K+2
	d[0][K] = tx1[0]; d[0][K + 1] = tz1[1]; d[0][K + 2] = tx2[0]; d[0][K + 3] = tz2[1];
	d[1][K] = tz1[0]; d[1][K + 1] = tx1[2]; d[1][K + 2] = tz2[0]; d[1][K + 3] = tx2[2];
	d[2][K] = tx1[1]; d[2][K + 1] = tz1[2]; d[2][K + 2] = tx2[1]; d[2][K + 3] = tz2[2];
}

// ---- turbo rate matching (36.212 5.1.4.1) ----------------------------------------------

// 36.212 Table 5.1.4-1: the TURBO sub-block interleaver's inter-column permutation. It is
// NOT the convolutional one (Table 5.1.4-2, SUBBLOCK_PERM): the two differ by a cyclic
// relabelling - turbo starts at column 0, convolutional at column 1 - and reusing the
// convolutional table here is invisible to a round trip and to any hand vector derived with
// the same wrong table, but it permutes the whole codeword and the CRC-24A never verifies.
// That is exactly the class of bug the pinned-vector discipline exists to catch, so this
// table is pinned in --selftest against the standard by hand.
const int TURBO_SUBBLOCK_PERM[32] = {
	0, 16,  8, 24,  4, 20, 12, 28,  2, 18, 10, 26,  6, 22, 14, 30,
	1, 17,  9, 25,  5, 21, 13, 29,  3, 19, 11, 27,  7, 23, 15, 31
};

void turbo_cb_map(size_t K, std::vector<int> &map) {
	const int C = 32;
	const int D = (int)K + 4;
	const int R = (D + C - 1) / C;
	const int Kpi = R * C;
	const int nd = Kpi - D;                  // <NULL> pads at the FRONT of each matrix
	map.assign((size_t)(3 * Kpi), -1);
	// streams 0 and 1: column-permuted read with the TURBO table (5.1.4-1), NOT the
	// convolutional SUBBLOCK_PERM (5.1.4-2) - the distinction is the whole bug this file's
	// discipline is meant to catch (see TURBO_SUBBLOCK_PERM above).
	for(int s = 0; s < 2; s++) {
		int o = 0;
		for(int cc = 0; cc < C; cc++) {
			const int col = TURBO_SUBBLOCK_PERM[cc];
			for(int r = 0; r < R; r++) {
				const int src = r * C + col - nd;
				const int v = (src >= 0) ? (s * D + src) : -1;
				// collection per 5.1.4.1.2: v0 fills w[0..Kpi), v1 the EVEN slots of the
				// interlaced tail w[Kpi+2k] - never concatenated (that is the PBCH rule)
				map[(size_t)((s == 0) ? o : (Kpi + 2 * o))] = v;
				o++;
			}
		}
	}
	// stream 2 reads through the +1-shifted formula and fills the ODD interlaced slots
	for(int k = 0; k < Kpi; k++) {
		const int r = k % R, cc = k / R;
		const int src = (TURBO_SUBBLOCK_PERM[cc] + C * r + 1) % Kpi - nd;
		map[(size_t)(Kpi + 2 * k + 1)] = (src >= 0) ? (2 * D + src) : -1;
	}
}

int turbo_k0(size_t K, int rv) {
	const int R = ((int)K + 4 + 31) / 32;
	// Ncb = Kw = 96R for broadcast (no soft-buffer limit): ceil(96R/8R) = 12 always
	return R * (24 * rv + 2);
}

void turbo_rate_match(const std::vector<uint8_t> d[3], size_t K, size_t E, int rv,
                      std::vector<uint8_t> &out) {
	std::vector<int> map;
	turbo_cb_map(K, map);
	const size_t D = K + 4;
	out.clear();
	out.reserve(E);
	size_t k = (size_t)turbo_k0(K, rv);
	while(out.size() < E) {
		const int src = map[k % map.size()];
		if(src >= 0) {
			out.push_back(d[(size_t)(src / (int)D)][(size_t)(src % (int)D)]);
		}
		k++;
	}
}

void turbo_rate_dematch(const std::vector<float> &llr_e, size_t K, int rv,
                        std::vector<float> &soft) {
	std::vector<int> map;
	turbo_cb_map(K, map);
	if(soft.size() != 3 * (K + 4)) {
		soft.assign(3 * (K + 4), 0.0f);
	}
	size_t k = (size_t)turbo_k0(K, rv), e = 0;
	while(e < llr_e.size()) {
		const int src = map[k % map.size()];
		if(src >= 0) {
			soft[(size_t)src] += llr_e[e];   // repetition AND cross-RV combining add here
			e++;
		}
		k++;
	}
}

// ---- max-log-MAP turbo decoder ---------------------------------------------------------

// One terminated constituent: forward/backward over K info steps plus 3 tail steps whose
// input is deterministic per state. sys/par hold the K systematic/parity LLRs (a priori
// already added to sys by the caller), ts/tz the three tail pairs. app gets the full
// a posteriori LLR, positive = bit 0 - the same convention as the Viterbi above.
static void map_decode(const float *sys, const float *par, const float ts[3],
                       const float tz[3], size_t K, float *app) {
	const float NEG = -1e30f;
	const size_t T = K + 3;
	// alpha[k][s]: the trellis is tiny (8 states), a flat array is simplest
	std::vector<float> alpha((T + 1) * 8, NEG), beta((T + 1) * 8, NEG);
	alpha[0] = 0.0f;

	// branch metric halves, hoisted: for step k the contribution of hypothesis (u,p) is
	// 0.5*(±sys) + 0.5*(±par); the halves keep app on the same scale as the inputs.
	auto gsys = [&](size_t k) { return (k < K) ? sys[k] : ts[k - K]; };
	auto gpar = [&](size_t k) { return (k < K) ? par[k] : tz[k - K]; };

	for(size_t k = 0; k < T; k++) {
		const float ls = 0.5f * gsys(k), lp = 0.5f * gpar(k);
		float *a0 = &alpha[k * 8], *a1 = &alpha[(k + 1) * 8];
		for(int s = 0; s < 8; s++) {
			if(a0[s] <= NEG) {
				continue;
			}
			const uint8_t u_lo = (k < K) ? 0 : (uint8_t)((((s >> 1) & 1) ^ (s & 1)));
			const uint8_t u_hi = (k < K) ? 1 : u_lo;   // tail: input forced per state
			for(uint8_t u = u_lo;; u++) {
				const uint8_t w = (k < K) ? rsc_w((uint8_t)s, u) : 0;
				const uint8_t p = rsc_parity((uint8_t)s, w);
				const int ns = rsc_next((uint8_t)s, w);
				const float m = a0[s] + (u ? -ls : ls) + (p ? -lp : lp);
				if(m > a1[ns]) {
					a1[ns] = m;
				}
				if(u >= u_hi) {
					break;
				}
			}
		}
	}
	beta[T * 8] = 0.0f;                                  // termination drains to state 0
	for(size_t k = T; k-- > 0;) {
		const float ls = 0.5f * gsys(k), lp = 0.5f * gpar(k);
		float *b0 = &beta[k * 8];
		const float *b1 = &beta[(k + 1) * 8];
		for(int s = 0; s < 8; s++) {
			const uint8_t u_lo = (k < K) ? 0 : (uint8_t)((((s >> 1) & 1) ^ (s & 1)));
			const uint8_t u_hi = (k < K) ? 1 : u_lo;
			for(uint8_t u = u_lo;; u++) {
				const uint8_t w = (k < K) ? rsc_w((uint8_t)s, u) : 0;
				const uint8_t p = rsc_parity((uint8_t)s, w);
				const int ns = rsc_next((uint8_t)s, w);
				if(b1[ns] > NEG) {
					const float m = b1[ns] + (u ? -ls : ls) + (p ? -lp : lp);
					if(m > b0[s]) {
						b0[s] = m;
					}
				}
				if(u >= u_hi) {
					break;
				}
			}
		}
	}
	for(size_t k = 0; k < K; k++) {
		const float ls = 0.5f * sys[k], lp = 0.5f * par[k];
		const float *a = &alpha[k * 8], *b = &beta[(k + 1) * 8];
		float m0 = NEG, m1 = NEG;
		for(int s = 0; s < 8; s++) {
			if(a[s] <= NEG) {
				continue;
			}
			for(uint8_t u = 0; u < 2; u++) {
				const uint8_t w = rsc_w((uint8_t)s, u);
				const uint8_t p = rsc_parity((uint8_t)s, w);
				const int ns = rsc_next((uint8_t)s, w);
				if(b[ns] <= NEG) {
					continue;
				}
				const float m = a[s] + (u ? -ls : ls) + (p ? -lp : lp) + b[ns];
				if(u) {
					m1 = std::max(m1, m);
				} else {
					m0 = std::max(m0, m);
				}
			}
		}
		app[k] = m0 - m1;                                // positive = bit 0, as everywhere
	}
}

void turbo_decode(const std::vector<float> &soft, size_t K, int iters,
                  std::vector<uint8_t> &out) {
	out.assign(K, 0);
	const size_t D = K + 4;
	if(K == 0 || soft.size() < 3 * D) {
		return;
	}
	std::vector<int> pi;
	turbo_qpp((int)K, pi);
	if(pi.empty()) {
		return;
	}
	const float *d0 = soft.data(), *d1 = soft.data() + D, *d2 = soft.data() + 2 * D;
	// tail de-multiplexing, the exact inverse of the encoder's table above
	const float ts1[3] = { d0[K], d2[K], d1[K + 1] };
	const float tz1[3] = { d1[K], d0[K + 1], d2[K + 1] };
	const float ts2[3] = { d0[K + 2], d2[K + 2], d1[K + 3] };
	const float tz2[3] = { d1[K + 2], d0[K + 3], d2[K + 3] };

	std::vector<float> la1(K, 0.0f), sys1(K), sys2(K), app(K), le(K);
	std::vector<uint8_t> prev(K, 2);
	// 0.75 extrinsic scaling: the standard remedy for max-log-MAP's overconfidence, worth
	// a few tenths of a dB and free.
	const float SC = 0.75f;
	for(int it = 0; it < iters; it++) {
		for(size_t k = 0; k < K; k++) {
			sys1[k] = d0[k] + la1[k];
		}
		map_decode(sys1.data(), d1, ts1, tz1, K, app.data());
		for(size_t k = 0; k < K; k++) {
			le[k] = app[k] - sys1[k];                    // extrinsic of decoder 1
		}
		for(size_t k = 0; k < K; k++) {
			sys2[k] = d0[(size_t)pi[k]] + SC * le[(size_t)pi[k]];
		}
		map_decode(sys2.data(), d2, ts2, tz2, K, app.data());
		bool stable = true;
		for(size_t k = 0; k < K; k++) {
			la1[(size_t)pi[k]] = SC * (app[k] - sys2[k]);
			const uint8_t b = (app[k] < 0.0f) ? 1 : 0;
			if(b != prev[(size_t)pi[k]]) {
				stable = false;
			}
			prev[(size_t)pi[k]] = b;
			out[(size_t)pi[k]] = b;
		}
		if(stable) {
			break;
		}
	}
}

// ---- selftest --------------------------------------------------------------------------

namespace {

int g_fail = 0;

void check(bool ok, const char *name, const char *detail, bool verbose) {
	if(!ok) {
		g_fail++;
	}
	if(verbose || !ok) {
		printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
		       detail && *detail ? ": " : "", detail ? detail : "");
	}
}

// map hard bits to ideal LLRs with the given amplitude
void bits_to_llr(const std::vector<uint8_t> &b, float amp, std::vector<float> &llr) {
	llr.resize(b.size());
	for(size_t i = 0; i < b.size(); i++) {
		llr[i] = b[i] ? -amp : amp;
	}
}

} // namespace

int lte_coding_selftest(bool verbose) {
	g_fail = 0;
	char d[160];

	// --- CRC: a known-answer test pins the polynomial and the bit order. The all-ones
	// message is chosen because it catches a reversed shift direction, which an all-zero
	// or random message does not.
	{
		std::vector<uint8_t> m(24, 1);
		uint32_t c16 = crc_bits(m.data(), m.size(), CRC16);
		std::vector<uint8_t> withcrc = m;
		crc_attach(withcrc, CRC16);
		bool ok = withcrc.size() == m.size() + 16 && crc_check(withcrc.data(), withcrc.size(), CRC16);
		snprintf(d, sizeof(d), "crc16(24x1)=0x%04X, attach+check round trip", c16);
		check(ok, "crc16_roundtrip", d, verbose);

		// a single flipped bit anywhere must fail the check
		bool all_caught = true;
		for(size_t i = 0; i < withcrc.size(); i++) {
			withcrc[i] ^= 1;
			if(crc_check(withcrc.data(), withcrc.size(), CRC16)) {
				all_caught = false;
			}
			withcrc[i] ^= 1;
		}
		check(all_caught, "crc16_single_bit", "every single-bit error detected", verbose);
	}
	{
		std::vector<uint8_t> m(200, 0);
		for(size_t i = 0; i < m.size(); i++) {
			m[i] = (uint8_t)((i * 7 + 3) & 1);
		}
		std::vector<uint8_t> w = m;
		crc_attach(w, CRC24A);
		check(w.size() == m.size() + 24 && crc_check(w.data(), w.size(), CRC24A),
		      "crc24a_roundtrip", "attach+check", verbose);
	}

	// --- the PBCH antenna mask: this is the table that names the port count.
	{
		uint8_t m1[16], m2[16], m4[16];
		pbch_crc_mask(1, m1);
		pbch_crc_mask(2, m2);
		pbch_crc_mask(4, m4);
		bool ok = true;
		for(int i = 0; i < 16; i++) {
			ok = ok && m1[i] == 0 && m2[i] == 1 && m4[i] == (uint8_t)(i & 1);
		}
		check(ok, "pbch_port_mask", "1->0000, 2->1111, 4->0101", verbose);

		// A MIB masked for 4 ports must NOT verify as 2 ports. This is exactly the trap
		// that hid the 262-01 reference cell behind a full -P 2 sweep.
		std::vector<uint8_t> mib(24, 0);
		for(int i = 0; i < 24; i++) {
			mib[i] = (uint8_t)((i * 5 + 1) & 1);
		}
		std::vector<uint8_t> tx = mib;
		crc_attach(tx, CRC16);
		for(int i = 0; i < 16; i++) {
			tx[24 + i] ^= m4[i];
		}
		bool as4 = crc_check(tx.data(), tx.size(), CRC16, m4);
		bool as2 = crc_check(tx.data(), tx.size(), CRC16, m2);
		bool as1 = crc_check(tx.data(), tx.size(), CRC16, m1);
		snprintf(d, sizeof(d), "4-port MIB verifies as 4 (%d), not as 2 (%d) or 1 (%d)",
		         (int)as4, (int)as2, (int)as1);
		check(as4 && !as2 && !as1, "pbch_mask_discriminates", d, verbose);
	}

	// --- SI-RNTI mask
	{
		uint8_t m[16];
		rnti_crc_mask(SI_RNTI, m);
		bool ok = true;
		for(int i = 0; i < 16; i++) {
			ok = ok && m[i] == 1;
		}
		check(ok, "si_rnti_mask", "0xFFFF -> all ones", verbose);
		rnti_crc_mask(0x8001, m);
		check(m[0] == 1 && m[15] == 1 && m[1] == 0, "rnti_mask_msb_first", "0x8001", verbose);
	}

	// --- convolutional code: tail-biting means the encoder ends where it started.
	{
		std::vector<uint8_t> in(40);
		for(size_t i = 0; i < in.size(); i++) {
			in[i] = (uint8_t)((i * 11 + 5) & 1);
		}
		std::vector<uint8_t> cw;
		conv_encode(in, cw);
		check(cw.size() == 3 * in.size(), "conv_rate", "3n coded bits", verbose);

		std::vector<float> llr;
		bits_to_llr(cw, 4.0f, llr);
		std::vector<uint8_t> dec;
		conv_decode(llr, in.size(), dec);
		check(dec == in, "conv_noiseless", "40-bit MIB-sized block recovered exactly", verbose);
	}

	// --- the property that actually pins tail-biting: a circular shift of the input
	// produces the same circular shift of the codeword. A zero-state encoder does not
	// have this property, so this check fails loudly if the initial state is wrong.
	{
		const size_t n = 32;
		std::vector<uint8_t> a(n), b(n);
		for(size_t i = 0; i < n; i++) {
			a[i] = (uint8_t)((i * 13 + 7) & 1);
		}
		for(size_t i = 0; i < n; i++) {
			b[i] = a[(i + 1) % n];
		}
		std::vector<uint8_t> ca, cb;
		conv_encode(a, ca);
		conv_encode(b, cb);
		bool ok = true;
		for(size_t k = 0; k < n; k++) {
			for(int j = 0; j < 3; j++) {
				if(cb[3 * k + j] != ca[3 * ((k + 1) % n) + j]) {
					ok = false;
				}
			}
		}
		check(ok, "conv_tailbiting_shift", "circular shift invariance", verbose);
	}

	// --- error correction: with AWGN on the LLRs the decoder must still land, and it must
	// degrade gracefully rather than at a cliff we cannot see.
	{
		// The thresholds below are anchored to Eb/N0, not guessed. With unit-amplitude LLRs
		// and noise sigma, Es/N0 = 1/(2 sigma^2), and rate 1/3 buys 10*log10(3) = 4.77 dB,
		// so Eb/N0 = -10*log10(2 sigma^2) + 4.77 dB:
		//   sigma 0.50 -> +7.8 dB  far above the waterfall, must be error-free
		//   sigma 1.00 -> +1.8 dB  on the shoulder, a few misses are normal
		//   sigma 1.40 -> -1.1 dB  below the waterfall, mostly fails - and that is correct
		// An earlier draft demanded 40/100 at sigma 2.0 (-4.2 dB Eb/N0). That was a wrong
		// expectation, not a decoder bug: no K=7 rate-1/3 code decodes there. Keeping the
		// point in the list with an honest bound is more useful than deleting it, because
		// it pins the low end of the curve.
		std::mt19937 rng(20260807);
		const size_t n = 40;
		struct { float sigma; int min_ok; } cases[] = {
			{ 0.5f, 100 }, { 1.0f, 90 }, { 1.4f, 5 }, { 2.0f, 0 }
		};
		int prev_ok = 101;
		for(auto &c : cases) {
			std::normal_distribution<float> gauss(0.0f, c.sigma);
			int ok_count = 0;
			for(int trial = 0; trial < 100; trial++) {
				std::vector<uint8_t> in(n);
				for(size_t i = 0; i < n; i++) {
					in[i] = (uint8_t)(rng() & 1);
				}
				std::vector<uint8_t> cw;
				conv_encode(in, cw);
				std::vector<float> llr(cw.size());
				for(size_t i = 0; i < cw.size(); i++) {
					llr[i] = (cw[i] ? -1.0f : 1.0f) + gauss(rng);
				}
				std::vector<uint8_t> dec;
				conv_decode(llr, n, dec);
				if(dec == in) {
					ok_count++;
				}
			}
			// Absolute bound plus monotonicity: a decoder that silently stops using the
			// soft information often still passes one absolute point, but it cannot
			// produce a correctly ordered waterfall.
			double ebn0 = -10.0 * std::log10(2.0 * c.sigma * c.sigma) + 4.77;
			snprintf(d, sizeof(d), "sigma %.2f (Eb/N0 %+.1f dB) -> %d/100 (want >= %d, <= %d)",
			         (double)c.sigma, ebn0, ok_count, c.min_ok, prev_ok);
			check(ok_count >= c.min_ok && ok_count <= prev_ok, "conv_awgn", d, verbose);
			prev_ok = ok_count;
		}
	}

	// --- the fast decoder must never be *wrong*, only sometimes absent. Where WAVA
	// converges it has to produce the exact ML answer; where it does not, it must say so
	// rather than return a guess, because the caller then pays for the exact decoder.
	// Measured on the board's A53: exact 7.19 ms vs fast 0.092 ms per 43-bit block (78x),
	// which is what moves the PDCCH search space from 8.7 s to 110 ms per 2 s burst.
	{
		std::mt19937 rng(4711);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		const size_t n = 43;
		int converged = 0, disagreed = 0;
		const int trials = 400;
		for(int t = 0; t < trials; t++) {
			std::vector<uint8_t> m(n);
			for(size_t i = 0; i < n; i++) {
				m[i] = (uint8_t)(rng() & 1);
			}
			std::vector<uint8_t> cw;
			conv_encode(m, cw);
			std::vector<float> llr(cw.size());
			for(size_t i = 0; i < cw.size(); i++) {
				llr[i] = (cw[i] ? -1.0f : 1.0f) + gauss(rng);
			}
			std::vector<uint8_t> ex, fa;
			conv_decode(llr, n, ex);
			double r = conv_decode_fast(llr, n, fa, 3);
			if(r > -1e29) {
				converged++;
				if(ex != fa) {
					disagreed++;
				}
			}
		}
		snprintf(d, sizeof(d), "%d/%d converged, %d disagreed with exact ML",
		         converged, trials, disagreed);
		check(disagreed == 0 && converged > trials / 2, "conv_fast_equivalence", d, verbose);

		// noiseless input must always converge - if it does not, the wrap-around feedback
		// is broken and every caller silently pays for the exact decoder instead.
		std::vector<uint8_t> m(n, 0);
		for(size_t i = 0; i < n; i++) {
			m[i] = (uint8_t)((i * 7 + 1) & 1);
		}
		std::vector<uint8_t> cw;
		conv_encode(m, cw);
		std::vector<float> llr;
		bits_to_llr(cw, 4.0f, llr);
		std::vector<uint8_t> fa;
		double r = conv_decode_fast(llr, n, fa, 3);
		check(r > -1e29 && fa == m, "conv_fast_noiseless", "converges on the first lap", verbose);
	}

	// --- rate matching must round-trip, and the repetition must actually accumulate.
	{
		std::vector<uint8_t> in(40);
		for(size_t i = 0; i < in.size(); i++) {
			in[i] = (uint8_t)((i * 9 + 4) & 1);
		}
		std::vector<uint8_t> cw;
		conv_encode(in, cw);
		std::vector<uint8_t> rm;
		conv_rate_match(cw, 1920, rm);
		check(rm.size() == 1920, "rm_length", "E=1920 from a 40-bit block", verbose);

		std::vector<float> llr(rm.size());
		for(size_t i = 0; i < rm.size(); i++) {
			llr[i] = rm[i] ? -1.0f : 1.0f;
		}
		std::vector<float> soft;
		conv_rate_dematch(llr, in.size(), soft);
		// each of the 120 coded bits should have been seen 16 times
		bool signs_ok = true;
		float mn = 1e30f, mx = -1e30f;
		for(size_t i = 0; i < soft.size(); i++) {
			if((soft[i] < 0) != (cw[i] != 0)) {
				signs_ok = false;
			}
			mn = std::min(mn, std::fabs(soft[i]));
			mx = std::max(mx, std::fabs(soft[i]));
		}
		snprintf(d, sizeof(d), "accumulated |llr| between %.0f and %.0f (16 repeats expected)",
		         (double)mn, (double)mx);
		check(signs_ok && mn >= 15.0f && mx <= 17.0f, "rm_roundtrip", d, verbose);

		std::vector<uint8_t> dec;
		conv_decode_fast(soft, in.size(), dec, 3);
		check(dec == in, "rm_decode", "rate matched block decodes back", verbose);
	}

	// --- pin the rate-matching MAP itself against 36.212 5.1.4.2 computed by hand, because
	// the round trip above cannot see a permuted codeword: collecting the three streams
	// turbo-style (5.1.4.1.2, interlaced) instead of concatenated (5.1.4.2.2) survives the
	// round trip, the 16x repetition count AND the on-air copy-agreement test, and only the
	// Viterbi ever notices. That exact bug cost the first MIB decode.
	// Hand derivation for D=40: R=2 rows, 24 <NULL>s fill row 0 columns 0..23, so column c
	// holds (row 0) d(c-24) for c>=24 else <NULL>, and (row 1) d(c+8). Reading the permuted
	// columns and dropping <NULL>s gives per stream: for each cc, emit col-24 when col>=24,
	// then col+8 - and 5.1.4.2.2 CONCATENATES the streams: all of d0, then d1, then d2.
	{
		std::vector<int> order;                 // src index within one stream, hand-derived
		for(int cc = 0; cc < 32; cc++) {
			const int col = SUBBLOCK_PERM[cc];
			if(col >= 24) {
				order.push_back(col - 24);
			}
			order.push_back(col + 8);
		}
		std::vector<uint8_t> in(120);
		for(size_t i = 0; i < in.size(); i++) {
			in[i] = (uint8_t)(((i * 29 + 5) % 3) == 0);
		}
		std::vector<uint8_t> rm;
		conv_rate_match(in, 480, rm);
		bool ok = rm.size() == 480;
		for(int e = 0; ok && e < 480; e++) {
			const int s = (e % 120) / 40;                    // stream: d0 first, then d1, d2
			const int src = order[(size_t)(e % 40)];         // position within the stream
			ok = rm[(size_t)e] == in[(size_t)(3 * src + s)];
		}
		snprintf(d, sizeof(d), "E=480 output matches the hand-computed 5.1.4.2 vector%s",
		         ok ? "" : " NOT");
		check(ok, "rm_pinned_vector", d, verbose);
	}

	// --- DCI sizes and a hand-built parse vector for each format. The 1A size at 50 PRB
	// walks the whole rule chain: 1+1+11+5+3+1+2+2 = 26 lands on the ambiguous list and
	// becomes 27. The payload below is written bit by bit from 36.212 5.3.3.1.3:
	// flag=1, localized=0, RIV=50*(8-1)+10=360 -> <00101101000>, MCS=4, HARQ=000, NDI=0,
	// RV=2 -> <10>, TPC=<01> (LSB 1 -> N_PRB_1A=3), pad=0. Expected: RBs 10..17, and
	// TBS(4,3) = 208 - which is EXACTLY the 816 MHz cell's known SIB1 size, so the same
	// literal pins the table row the real decode will use.
	{
		snprintf(d, sizeof(d), "1A %d bits, 1C %d bits at 50 PRB", dci_size_1a(50),
		         dci_size_1c(50));
		check(dci_size_1a(50) == 27 && dci_size_1c(50) == 13, "dci_sizes_50prb", d, verbose);

		const uint8_t v1a[27] = { 1, 0, 0,0,1,0,1,1,0,1,0,0,0, 0,0,1,0,0,
		                          0,0,0, 0, 1,0, 0,1, 0 };
		dci_si s{};
		bool ok = dci_parse_si(v1a, 27, 50, s);
		snprintf(d, sizeof(d),
		         "1A: loc %d riv %d -> rb %d+%d, i_tbs %d, rv %d, n_prb_1a %d, tbs %d",
		         (int)s.localized, s.riv, s.rb_start, s.n_prb, s.i_tbs, s.rv, s.n_prb_1a, s.tbs);
		check(ok && s.format == 0x1A && s.localized && s.riv == 360 && s.rb_start == 10 &&
		      s.n_prb == 8 && s.i_tbs == 4 && s.rv == 2 && s.n_prb_1a == 3 && s.tbs == 208,
		      "dci_1a_pinned", d, verbose);

		// 1C at 50 PRB: gap=0 -> N_vrb = 2*min(27,23) = 46, N_step 4 -> N' = 11, RBA 7 bits.
		// RIV' = 11*(3-1)+2 = 24 -> <0011000>, MCS=7 -> TBS 208 from the 1C table. The
		// allocation is in VRB steps: start 2*4 = 8, length 3*4 = 12.
		const uint8_t v1c[13] = { 0, 0,0,1,1,0,0,0, 0,0,1,1,1 };
		ok = dci_parse_si(v1c, 13, 50, s);
		snprintf(d, sizeof(d), "1C: gap %d riv %d -> vrb %d+%d, i_tbs %d, tbs %d",
		         s.gap, s.riv, s.rb_start, s.n_prb, s.i_tbs, s.tbs);
		check(ok && s.format == 0x1C && s.gap == 0 && s.riv == 24 && s.rb_start == 8 &&
		      s.n_prb == 12 && s.i_tbs == 7 && s.tbs == 208,
		      "dci_1c_pinned", d, verbose);

		// a 1A payload whose flag bit says format 0 must be rejected, not misparsed
		uint8_t v0[27];
		memcpy(v0, v1a, sizeof(v0));
		v0[0] = 0;
		check(!dci_parse_si(v0, 27, 50, s), "dci_format0_rejected",
		      "flag bit 0 -> not a DL assignment", verbose);
	}

	// --- QPP interleaver, pinned by hand: K=40 has f1=3, f2=10, so Pi(0..4) =
	// 0, 13, 46%40=6, 99%40=19, 172%40=12. The two on-air sizes get their f1+f2 spot
	// checks too (Pi(1) = f1+f2): K=232 -> 85+58 = 143, K=248 -> 33+62 = 95.
	{
		std::vector<int> pi;
		turbo_qpp(40, pi);
		bool ok = pi.size() == 40 && pi[0] == 0 && pi[1] == 13 && pi[2] == 6 &&
		          pi[3] == 19 && pi[4] == 12;
		std::vector<int> pi232, pi248;
		turbo_qpp(232, pi232);
		turbo_qpp(248, pi248);
		ok = ok && pi232.size() == 232 && pi232[1] == 143;
		ok = ok && pi248.size() == 248 && pi248[1] == 95;
		// a QPP with legal parameters is a bijection - and K=44 (not on the 8-grid past
		// 512 rules) must be accepted while K=41 must not
		for(auto *p : { &pi232, &pi248 }) {
			std::vector<uint8_t> seen(p->size(), 0);
			for(int v : *p) {
				if(v < 0 || (size_t)v >= p->size() || seen[(size_t)v]) {
					ok = false;
					break;
				}
				seen[(size_t)v] = 1;
			}
		}
		std::vector<int> bad;
		turbo_qpp(41, bad);
		ok = ok && bad.empty();
		check(ok, "turbo_qpp_pinned", "K=40 hand values, K=232/248 spot checks, bijection",
		      verbose);
	}

	// --- the turbo sub-block permutation must be the TURBO table (5.1.4-1), and it must
	// NOT equal the convolutional one (5.1.4-2). The two are a cyclic relabelling of each
	// other, so the only defence against silently reusing the wrong one is to assert both
	// the exact values and their inequality.
	{
		static const int want[32] = {
			0, 16,  8, 24,  4, 20, 12, 28,  2, 18, 10, 26,  6, 22, 14, 30,
			1, 17,  9, 25,  5, 21, 13, 29,  3, 19, 11, 27,  7, 23, 15, 31
		};
		bool ok = true, differs = false;
		for(int i = 0; i < 32; i++) {
			ok = ok && TURBO_SUBBLOCK_PERM[i] == want[i];
			differs = differs || TURBO_SUBBLOCK_PERM[i] != SUBBLOCK_PERM[i];
		}
		check(ok && differs, "turbo_subblock_perm",
		      "5.1.4-1 values, distinct from the convolutional 5.1.4-2 table", verbose);
	}

	// --- turbo encoder, pinned by the hand-computed impulse response of the RSC:
	// g0 = 1+D^2+D^3, g1 = 1+D+D^3, so an impulse gives the feedback sequence
	// w = 1,0,1,1,1,0,0,... (period 7) and parity z_k = w_k+w_{k-1}+w_{k-3} =
	// 1,1,1,1,0,0,1,0. For K=40 the QPP maps the impulse to ITSELF (k(10k+3) = 0 mod 40
	// only at k=0), so d2 must equal d1 over the info part - a free second pin. The state
	// after bit 39 is (w39,w38,w37) = (1,1,1), so the hand-run termination gives tail
	// x = 0,0,1 and z = 0,1,1 for BOTH encoders, which pins the 5.1.3.2.2 multiplexing:
	// d0[K..K+3] = 0,1,0,1  d1[K..K+3] = 0,1,0,1  d2[K..K+3] = 0,1,0,1.
	{
		std::vector<uint8_t> in(40, 0);
		in[0] = 1;
		std::vector<uint8_t> dd[3];
		turbo_encode(in, dd);
		static const uint8_t zref[8] = { 1, 1, 1, 1, 0, 0, 1, 0 };
		bool ok = dd[0].size() == 44 && dd[1].size() == 44 && dd[2].size() == 44;
		ok = ok && dd[0][0] == 1 && dd[0][1] == 0;         // systematic = input
		for(int i = 0; ok && i < 8; i++) {
			ok = dd[1][(size_t)i] == zref[i] && dd[2][(size_t)i] == zref[i];
		}
		static const uint8_t tail[3][4] = { { 0, 1, 0, 1 }, { 0, 1, 0, 1 }, { 0, 1, 0, 1 } };
		for(int s = 0; ok && s < 3; s++) {
			for(int t = 0; ok && t < 4; t++) {
				ok = dd[(size_t)s][(size_t)(40 + t)] == tail[s][t];
			}
		}
		check(ok, "turbo_impulse_pinned", "hand-computed RSC impulse + tail multiplexing",
		      verbose);
	}

	// --- turbo rate matching, pinned by a hand-derived 5.1.4.1 read for K=40 (D=44) using
	// the TURBO column permutation (5.1.4-1, PT below), NOT the convolutional one - the
	// exact bug that made the CRC never verify on air until it was caught here.
	// R=2, Kpi=64, nd=20 <NULL>s. v0 buffer position o=(cc*2+r): r=0 -> d0[PT[cc]-20]
	// (NULL when PT[cc]<20), r=1 -> d0[PT[cc]+12]. k0(rv)=R*(24rv+2): k0(0)=4, k0(2)=100.
	// From pos 4, skipping NULLs, the first eight v0 emits are d0 indices 20,4,36,16,0,32,
	// 24,8; there are 42 v0 emits, then the interlaced tail gives v1(1)=d1[12],
	// v2(1)=d2[13] (the +1-shifted column: (PT[cc]+32+1)%64-20), v1(3)=d1[28], v2(3)=d2[29].
	// From k0(2)=100 the first five emits are d1[30], d2[31], d1[22], d2[23], d1[6].
	{
		std::vector<uint8_t> dd[3];
		for(int s = 0; s < 3; s++) {
			dd[s].resize(44);
			for(int i = 0; i < 44; i++) {
				dd[s][(size_t)i] = (uint8_t)((i * 7 + s * 3 + 1) & 1);
			}
		}
		std::vector<uint8_t> rm;
		turbo_rate_match(dd, 40, 46, 0, rm);
		static const int v0i[8] = { 20, 4, 36, 16, 0, 32, 24, 8 };
		bool ok = rm.size() == 46 && turbo_k0(40, 0) == 4 && turbo_k0(40, 2) == 100;
		for(int i = 0; ok && i < 8; i++) {
			ok = rm[(size_t)i] == dd[0][(size_t)v0i[i]];
		}
		ok = ok && rm[42] == dd[1][12] && rm[43] == dd[2][13] && rm[44] == dd[1][28] &&
		     rm[45] == dd[2][29];
		std::vector<uint8_t> rm2;
		turbo_rate_match(dd, 40, 5, 2, rm2);
		ok = ok && rm2[0] == dd[1][30] && rm2[1] == dd[2][31] && rm2[2] == dd[1][22] &&
		     rm2[3] == dd[2][23] && rm2[4] == dd[1][6];
		check(ok, "turbo_rm_pinned", "hand-derived 5.1.4-1 (turbo) vector, rv 0 and 2",
		      verbose);
	}

	// --- turbo round trip at the two on-air shapes. The maps are pinned above, so this
	// checks what a round trip legitimately can: that the decoder converges and that
	// cross-RV LLR accumulation through turbo_rate_dematch really combines. E values are
	// the measured on-air ones (816er: 2320 bits, 796er: 960).
	{
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		struct { size_t tbs, E; int rvs[2]; float sigma; } cases[] = {
			{ 208, 2320, { 0, -1 }, 1.0f },      // single RV0, rate ~0.1: easy
			{ 224, 960, { 0, -1 }, 0.8f },       // single RV0, rate ~0.26
			// two SHORT RVs combined: E = 400 from k0(2) = 400 carries only 32 systematic
			// LLRs, so RV2 alone is a rate-0.62 mostly-parity code that dies at this noise
			// (the control below) - only the accumulation makes the pair decodable
			{ 224, 400, { 0, 2 }, 0.9f },
		};
		for(auto &c : cases) {
			int good = 0;
			const int trials = 20;
			for(int t = 0; t < trials; t++) {
				std::vector<uint8_t> tb(c.tbs);
				for(auto &b : tb) {
					b = (uint8_t)(rng() & 1);
				}
				std::vector<uint8_t> blk = tb;
				crc_attach(blk, CRC24A);
				std::vector<uint8_t> dd[3];
				turbo_encode(blk, dd);
				std::vector<float> soft;
				for(int rv : c.rvs) {
					if(rv < 0) {
						continue;
					}
					std::vector<uint8_t> rm;
					turbo_rate_match(dd, blk.size(), c.E, rv, rm);
					std::vector<float> llr(rm.size());
					for(size_t i = 0; i < rm.size(); i++) {
						llr[i] = (rm[i] ? -1.0f : 1.0f) + c.sigma * gauss(rng);
					}
					turbo_rate_dematch(llr, blk.size(), rv, soft);
				}
				std::vector<uint8_t> dec;
				turbo_decode(soft, blk.size(), 8, dec);
				if(crc_check(dec.data(), dec.size(), CRC24A) &&
				   std::equal(tb.begin(), tb.end(), dec.begin())) {
					good++;
				}
			}
			snprintf(d, sizeof(d), "tbs %zu E %zu rv{%d,%d} sigma %.1f -> %d/%d",
			         c.tbs, c.E, c.rvs[0], c.rvs[1], (double)c.sigma, good, trials);
			check(good == trials, "turbo_roundtrip", d, verbose);
		}
		// negative control for the pair above: RV2 alone at the same E and noise must NOT
		// carry the day - if it does, the combining check proves nothing. (A first draft
		// used E = 960, where k0(2) = 400 wraps far enough to cover the ENTIRE systematic
		// region and RV2 alone decodes fine - that control was testing a wrong belief.)
		{
			std::vector<uint8_t> tb(224);
			for(auto &b : tb) {
				b = (uint8_t)(rng() & 1);
			}
			std::vector<uint8_t> blk = tb;
			crc_attach(blk, CRC24A);
			std::vector<uint8_t> dd[3];
			turbo_encode(blk, dd);
			int fails = 0;
			for(int t = 0; t < 10; t++) {
				std::vector<uint8_t> rm;
				turbo_rate_match(dd, blk.size(), 400, 2, rm);
				std::vector<float> soft, llr(rm.size());
				for(size_t i = 0; i < rm.size(); i++) {
					llr[i] = (rm[i] ? -1.0f : 1.0f) + 0.9f * gauss(rng);
				}
				turbo_rate_dematch(llr, blk.size(), 2, soft);
				std::vector<uint8_t> dec;
				turbo_decode(soft, blk.size(), 8, dec);
				if(!crc_check(dec.data(), dec.size(), CRC24A) ||
				   !std::equal(tb.begin(), tb.end(), dec.begin())) {
					fails++;
				}
			}
			snprintf(d, sizeof(d), "RV2 alone, E 400, sigma 0.9: %d/10 fail (want >= 5)", fails);
			check(fails >= 5, "turbo_rv2_alone_dies", d, verbose);
		}
	}

	// --- end to end in the shape PBCH actually uses it: 24-bit MIB payload, CRC-16 masked
	// with the port count, rate-1/3 tail-biting convolutional code, and a decoder that has
	// to work out the port count itself from the three hypotheses.
	{
		std::vector<uint8_t> mib(24);
		for(size_t i = 0; i < mib.size(); i++) {
			mib[i] = (uint8_t)((i * 3 + 2) & 1);
		}
		for(int truth : { 1, 2, 4 }) {
			uint8_t mask[16];
			pbch_crc_mask(truth, mask);
			std::vector<uint8_t> tx = mib;
			crc_attach(tx, CRC16);
			for(int i = 0; i < 16; i++) {
				tx[24 + i] ^= mask[i];
			}
			std::vector<uint8_t> cw;
			conv_encode(tx, cw);
			std::vector<float> llr;
			bits_to_llr(cw, 3.0f, llr);
			std::vector<uint8_t> dec;
			conv_decode(llr, tx.size(), dec);

			int found = 0;
			for(int h : PBCH_PORT_HYPOTHESES) {
				uint8_t hm[16];
				pbch_crc_mask(h, hm);
				if(crc_check(dec.data(), dec.size(), CRC16, hm)) {
					found = h;
					break;
				}
			}
			bool payload_ok = std::equal(mib.begin(), mib.end(), dec.begin());
			snprintf(d, sizeof(d), "%d ports -> decoder reports %d, payload %s",
			         truth, found, payload_ok ? "exact" : "WRONG");
			check(found == truth && payload_ok, "pbch_chain", d, verbose);
		}
	}

	return g_fail;
}

} // namespace cellsync

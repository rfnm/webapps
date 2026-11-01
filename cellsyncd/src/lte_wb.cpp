// cellsync: wideband LTE downlink front end - see lte_wb.h for the doctrine.
#include "lte_wb.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

#include "lte_coding.h"
#include "lte_seq.h"

namespace cellsync {

wb_num wb_num_for(int nrb) {
	wb_num g{};
	g.nrb = nrb;
	switch(nrb) {
	case 6:   g.fft = 128;  break;
	case 15:  g.fft = 256;  break;
	case 25:  g.fft = 512;  break;
	case 50:  g.fft = 1024; break;
	case 75:  g.fft = 1536; break;   // not a power of two: unsupported by fft_plan today
	case 100: g.fft = 2048; break;
	default:  g.fft = 1024; g.nrb = 50; break;
	}
	// Normal CP scales with the FFT: at the 2048 reference it is 160/144, so cp0 = fft*160/2048
	// and cp = fft*144/2048. At 1024 that is 80 and 72.
	g.cp0 = g.fft * 160 / 2048;
	g.cp = g.fft * 144 / 2048;
	g.slot = g.cp0 + g.fft + 6 * (g.cp + g.fft);
	g.subframe = 2 * g.slot;
	g.frame = 10 * g.subframe;
	g.fs = (double)g.fft * SCS;
	return g;
}

bool wb_demod_symbol(const cf *x, size_t n, int64_t slot_start, int l,
                     const wb_num &g, const fft_plan &plan, cf *grid) {
	const int64_t off = slot_start + g.sym_off(l);
	if(off < 0 || (size_t)(off + g.fft) > n) {
		return false;
	}
	std::vector<cf> buf((size_t)g.fft);
	plan.run(x + off, buf.data());
	// bin 0 is DC and carries nothing; the occupied band is +-6*nrb subcarriers around it.
	const int half = 6 * g.nrb;
	for(int i = 0; i < half; i++) {
		grid[i] = buf[(size_t)(g.fft - half + i)];   // negative frequencies, ascending
		grid[half + i] = buf[(size_t)(1 + i)];       // positive frequencies
	}
	return true;
}

int wb_crs_symbols(int port, int l[3]) {
	// 36.211 6.10.1.2: ports 0/1 in symbols 0 and 4 of every slot, ports 2/3 only in
	// symbol 1. Getting this wrong costs the 4-port cells, which is exactly the class our
	// 816 MHz reference belongs to.
	if(port == 0 || port == 1) {
		l[0] = 0;
		l[1] = 4;
		return 2;
	}
	l[0] = 1;
	return 1;
}

// v per 36.211 Table 6.10.1.2-1
static int crs_v(int port, int ns, int l) {
	switch(port) {
	case 0: return (l == 0) ? 0 : 3;
	case 1: return (l == 0) ? 3 : 0;
	case 2: return 3 * (ns & 1);
	case 3: return 3 + 3 * (ns & 1);
	}
	return 0;
}

void wb_crs(int nid_cell, int nrb, int port, int ns, int l, std::vector<crs_ref> &out) {
	out.clear();
	int ls[3];
	const int nl = wb_crs_symbols(port, ls);
	bool carries = false;
	for(int i = 0; i < nl; i++) {
		if(ls[i] == l) {
			carries = true;
		}
	}
	if(!carries) {
		return;
	}
	const int v = crs_v(port, ns, l);
	const int vshift = nid_cell % 6;
	const int count = 2 * nrb;
	// sequence index m' = m + 110 - nrb (the standard makes the labels bandwidth-invariant
	// so a narrowband receiver sees the same values a wideband one does)
	std::vector<cf> r(count);
	crs_seq(nid_cell, ns, l, 110 - nrb, count, r.data());
	out.reserve(count);
	for(int m = 0; m < count; m++) {
		const int k = 6 * m + ((v + vshift) % 6);
		if(k >= 0 && k < 12 * nrb) {
			out.push_back(crs_ref{ k, r[(size_t)m] });
		}
	}
}

void wb_chan_est(const cf *grid, const std::vector<crs_ref> &pilots, int ncarrier, cf *h) {
	if(pilots.empty()) {
		for(int i = 0; i < ncarrier; i++) {
			h[i] = cf(0.0f, 0.0f);
		}
		return;
	}
	// least squares at the pilots: the reference symbols are unit modulus, so dividing by
	// them is a conjugate multiply.
	std::vector<cf> hp(pilots.size());
	for(size_t i = 0; i < pilots.size(); i++) {
		hp[i] = grid[pilots[i].k] * std::conj(pilots[i].ref);
	}
	// linear interpolation between pilots, flat extrapolation outside
	for(int k = 0; k < pilots[0].k; k++) {
		h[k] = hp[0];
	}
	for(size_t i = 0; i + 1 < pilots.size(); i++) {
		const int k0 = pilots[i].k, k1 = pilots[i + 1].k;
		const float span = (float)(k1 - k0);
		for(int k = k0; k < k1; k++) {
			const float t = (float)(k - k0) / span;
			h[k] = hp[i] * (1.0f - t) + hp[i + 1] * t;
		}
	}
	for(int k = pilots.back().k; k < ncarrier; k++) {
		h[k] = hp.back();
	}
}

// ---- PBCH ------------------------------------------------------------------------------

bool wb_pbch_chest(const cf *x, size_t n, int64_t slot1_start, int nid_cell,
                   const wb_num &g, const fft_plan &plan, std::vector<cf> h[4][4]) {
	const int nc = 12 * g.nrb;
	// The four pilot-bearing symbols that bracket PBCH: slot-1 symbols 0, 1 and 4, and
	// symbol 1 of slot 2. Demodulated here rather than taken from the caller so the
	// function owns its own anchors.
	std::vector<cf> g0((size_t)nc), g1((size_t)nc), g4((size_t)nc), g8((size_t)nc);
	if(!wb_demod_symbol(x, n, slot1_start, 0, g, plan, g0.data()) ||
	   !wb_demod_symbol(x, n, slot1_start, 1, g, plan, g1.data())) {
		return false;
	}
	const bool have4 = wb_demod_symbol(x, n, slot1_start, 4, g, plan, g4.data());
	const bool have8 = wb_demod_symbol(x, n, slot1_start + g.slot, 1, g, plan, g8.data());

	for(int p = 0; p < 4; p++) {
		std::vector<cf> ha((size_t)nc), hb((size_t)nc);
		float ta, tb;                       // anchor times in symbol units from slot-1 symbol 0
		std::vector<crs_ref> pa, pb;
		if(p < 2) {
			wb_crs(nid_cell, g.nrb, p, 1, 0, pa);
			wb_chan_est(g0.data(), pa, nc, ha.data());
			ta = 0.0f;
			if(have4) {
				wb_crs(nid_cell, g.nrb, p, 1, 4, pb);
				wb_chan_est(g4.data(), pb, nc, hb.data());
				tb = 4.0f;
			} else {
				hb = ha;
				tb = 1.0f;
			}
		} else {
			wb_crs(nid_cell, g.nrb, p, 1, 1, pa);
			wb_chan_est(g1.data(), pa, nc, ha.data());
			ta = 1.0f;
			if(have8) {
				// slot 2: v moves with the slot parity, so ns matters here
				wb_crs(nid_cell, g.nrb, p, 2, 1, pb);
				wb_chan_est(g8.data(), pb, nc, hb.data());
				tb = 8.0f;
			} else {
				hb = ha;
				tb = 2.0f;
			}
		}
		for(int l = 0; l < 4; l++) {
			h[p][l].resize((size_t)nc);
			const float w = ((float)l - ta) / (tb - ta);
			for(int k = 0; k < nc; k++) {
				h[p][l][(size_t)k] = ha[(size_t)k] * (1.0f - w) + hb[(size_t)k] * w;
			}
		}
	}
	return true;
}

int wb_pbch_extract(const cf *const grid_sym[4], const std::vector<cf> h_sym[4][4],
                    int nrb, int nid_cell, cf *re, cf h_out[4][240]) {
	const int nc = 12 * nrb;
	const int c0 = 6 * nrb - 36;           // first of the 72 centre subcarriers
	int n = 0;
	for(int l = 0; l < 4 && n < 240; l++) {
		// CRS reservation ALWAYS assumes four ports: symbol 0 carries pilots of ports 0/1,
		// symbol 1 of ports 2/3, and both combs are skipped even by a 1-port cell.
		std::vector<uint8_t> skip((size_t)nc, 0);
		if(l < 2) {
			for(int p = 2 * l; p < 2 * l + 2; p++) {
				std::vector<crs_ref> q;
				wb_crs(nid_cell, nrb, p, 1, l, q);
				for(auto &z : q) {
					if(z.k >= 0 && z.k < nc) {
						skip[(size_t)z.k] = 1;
					}
				}
			}
		}
		for(int k = c0; k < c0 + 72 && n < 240; k++) {
			if(skip[(size_t)k]) {
				continue;
			}
			re[n] = grid_sym[l][k];
			for(int p = 0; p < 4; p++) {
				h_out[p][n] = h_sym[p][l].empty() ? cf(0.0f, 0.0f) : h_sym[p][l][(size_t)k];
			}
			n++;
		}
	}
	return n;
}

// ---- transmit-diversity combining -------------------------------------------------------

void wb_sfbc_combine(const cf *re, const cf *const h[4], int n, int nof_ports, cf *out,
                     float *wout) {
	if(nof_ports == 1) {
		for(int i = 0; i < n; i++) {
			const cf hh = h[0][i];
			out[i] = re[i] * std::conj(hh) / (std::norm(hh) + 1e-20f);
			if(wout) {
				wout[i] = std::norm(hh);
			}
		}
		return;
	}
	if(wout) {
		// an odd n leaves the last RE untouched below; it must not read back a stale weight
		for(int i = 0; i < n; i++) {
			wout[i] = 0.0f;
		}
	}
	for(int i = 0; i + 1 < n; i += 2) {
		int pa = 0, pb = 1;
		if(nof_ports == 4) {
			// 6.3.4.3: with four ports SFBC+FSTD works on quadruplets - the first RE pair
			// rides ports 0 and 2, the second ports 1 and 3. Pairing {0,1}/{2,3} looks
			// plausible and cost EVM 272 % on air.
			if((i >> 1) & 1) {
				pa = 1;
				pb = 3;
			} else {
				pa = 0;
				pb = 2;
			}
		}
		// Alamouti assumes the channel constant across the pair, so both taps are read at
		// the first RE.
		const cf h0 = h[pa][i], h1 = h[pb][i];
		const cf y0 = re[i], y1 = re[i + 1];
		const float d = std::norm(h0) + std::norm(h1) + 1e-20f;
		// y0 = h0*x0 - h1*x1', y1 = h0*x1 + h1*x0'. The second output MUST come from y1 and
		// conj(y0); conj(h1)*y0 - h0*conj(y1) yields -conj(x1), which stays on the QPSK grid
		// (EVM-blind) and flips all four copies of a code bit together (agreement-blind).
		// Only the CRC ever sees it - this exact bug cost the first decode on air.
		out[i] = (std::conj(h0) * y0 + h1 * std::conj(y1)) / d;
		out[i + 1] = (std::conj(h0) * y1 - h1 * std::conj(y0)) / d;
		if(wout) {
			wout[i] = wout[i + 1] = d;   // the pair was divided by one gain, so it shares one
		}
	}
}

// ---- PCFICH -----------------------------------------------------------------------------

void wb_pcfich_regs(int nid_cell, int nrb, int k_reg[4]) {
	// 36.211 6.7.4: the cell ID spreads the four quadruplets across the carrier so that
	// neighbouring cells' PCFICHs do not collide; the wrap at the band edge is part of the
	// formula, not an implementation convenience.
	const int kbar = 6 * (nid_cell % (2 * nrb));
	for(int i = 0; i < 4; i++) {
		k_reg[i] = (kbar + 6 * ((i * nrb) / 2)) % (12 * nrb);
	}
}

int wb_pcfich_decode(const cf *x, size_t n, int64_t slot_start, int ns, int nid_cell,
                     int nof_ports, const wb_num &g, const fft_plan &plan, float *margin) {
	const int nc = 12 * g.nrb;
	std::vector<cf> g0((size_t)nc);
	if(!wb_demod_symbol(x, n, slot_start, 0, g, plan, g0.data())) {
		return -1;
	}
	// Channel per port at symbol 0. Ports 0/1 have pilots in this very symbol; ports 2/3
	// only in symbol 1, so their estimate is one symbol stale - acceptable for 16 REs once
	// the CFO has been removed upstream.
	std::vector<cf> h[4];
	std::vector<crs_ref> pil;
	for(int p = 0; p < nof_ports && p < 2; p++) {
		h[p].resize((size_t)nc);
		wb_crs(nid_cell, g.nrb, p, ns, 0, pil);
		wb_chan_est(g0.data(), pil, nc, h[p].data());
	}
	if(nof_ports == 4) {
		std::vector<cf> g1((size_t)nc);
		if(!wb_demod_symbol(x, n, slot_start, 1, g, plan, g1.data())) {
			return -1;
		}
		for(int p = 2; p < 4; p++) {
			h[p].resize((size_t)nc);
			wb_crs(nid_cell, g.nrb, p, ns, 1, pil);
			wb_chan_est(g1.data(), pil, nc, h[p].data());
		}
	}
	for(int p = (nof_ports == 4) ? 4 : nof_ports; p < 4; p++) {
		h[p].assign((size_t)nc, cf(0.0f, 0.0f));
	}

	// Extract the 16 data REs in mapping order: quadruplets i = 0..3, k ascending inside
	// each REG, skipping the two CRS combs of symbol 0 (vshift and vshift+3 - the REG
	// anchors are multiples of 6, so the comb offset inside every REG is the same).
	int kreg[4];
	wb_pcfich_regs(nid_cell, g.nrb, kreg);
	const int vs = nid_cell % 6;
	cf re[16], hb[4][16];
	int m = 0;
	for(int i = 0; i < 4; i++) {
		for(int d = 0; d < 6; d++) {
			if(d == vs || d == (vs + 3) % 6) {
				continue;
			}
			const int k = kreg[i] + d;
			re[m] = g0[(size_t)k];
			for(int p = 0; p < 4; p++) {
				hb[p][m] = h[p][(size_t)k];
			}
			m++;
		}
	}
	const cf *hp[4] = { hb[0], hb[1], hb[2], hb[3] };
	cf s[16];
	wb_sfbc_combine(re, hp, 16, nof_ports, s);

	// QPSK demap (positive LLR = bit 0), then descramble per 36.211 6.7.1. The cinit is
	// re-derived every subframe - PCFICH scrambling changes with the subframe index, unlike
	// PBCH's per-40ms cinit = NID.
	float llr[32];
	for(int i = 0; i < 16; i++) {
		llr[2 * i] = s[i].real();
		llr[2 * i + 1] = s[i].imag();
	}
	uint8_t sc[32];
	const uint32_t cinit =
	    (((uint32_t)(ns / 2 + 1) * (uint32_t)(2 * nid_cell + 1)) << 9) + (uint32_t)nid_cell;
	gold_seq(cinit, 32, sc);
	for(int i = 0; i < 32; i++) {
		if(sc[i]) {
			llr[i] = -llr[i];
		}
	}
	// Soft correlation against the three fixed codewords - that IS the CFI decoder.
	float asum = 1e-20f;
	for(int i = 0; i < 32; i++) {
		asum += std::fabs(llr[i]);
	}
	float best = -1e30f, second = -1e30f;
	int cfi = -1;
	for(int c = 0; c < 3; c++) {
		float corr = 0.0f;
		for(int i = 0; i < 32; i++) {
			corr += CFI_CODEWORD[c][i] ? -llr[i] : llr[i];
		}
		if(corr > best) {
			second = best;
			best = corr;
			cfi = c + 1;
		} else if(corr > second) {
			second = corr;
		}
	}
	if(margin) {
		*margin = (best - second) / asum;
	}
	return cfi;
}

// ---- PHICH ------------------------------------------------------------------------------

int wb_phich_regs(int nid_cell, int nrb, int phich_dur, float phich_res,
                  std::vector<int> &k_reg) {
	k_reg.clear();
	if(phich_dur != 0) {
		return 0;   // extended duration: not built, see header
	}
	// 36.211 6.9: ngroups = ceil(Ng * Nrb/8), Ng in {1/6, 1/2, 1, 2} straight from the MIB
	const int ngroup = (int)std::ceil((double)phich_res * (double)nrb / 8.0);
	// the REGs PHICH may use: symbol 0 minus the four PCFICH anchors, ascending
	int pk[4];
	wb_pcfich_regs(nid_cell, nrb, pk);
	std::vector<int> avail;
	avail.reserve((size_t)(2 * nrb));
	for(int j = 0; j < 2 * nrb; j++) {
		const int k = 6 * j;
		if(k == pk[0] || k == pk[1] || k == pk[2] || k == pk[3]) {
			continue;
		}
		avail.push_back(k);
	}
	const int n0 = (int)avail.size();
	// 6.9.3 with l' = 0 everywhere (normal duration, FDD: m' = m): the NID term rotates
	// the whole PHICH against neighbouring cells, the i term spreads a group's three REGs
	// a third of the carrier apart for frequency diversity.
	for(int m = 0; m < ngroup; m++) {
		for(int i = 0; i < 3; i++) {
			k_reg.push_back(avail[(size_t)((nid_cell + m + i * n0 / 3) % n0)]);
		}
	}
	return ngroup;
}

// ---- PDCCH ------------------------------------------------------------------------------

int wb_pdcch_regs(int nid_cell, int nrb, int cfi, int nof_ports, int phich_dur,
                  float phich_res, std::vector<wb_reg> &regs) {
	regs.clear();
	if(cfi < 1 || cfi > 3) {
		return 0;
	}
	// symbol-0 REGs already owned by PCFICH and PHICH (normal PHICH duration keeps all its
	// REGs in symbol 0; extended duration is not built, wb_phich_regs returns none)
	int pk[4];
	wb_pcfich_regs(nid_cell, nrb, pk);
	std::vector<int> ph;
	wb_phich_regs(nid_cell, nrb, phich_dur, phich_res, ph);
	std::vector<uint8_t> taken((size_t)(12 * nrb), 0);
	for(int i = 0; i < 4; i++) {
		taken[(size_t)pk[i]] = 1;
	}
	for(int k : ph) {
		taken[(size_t)k] = 1;
	}
	const int vs = nid_cell % 6;
	// 6.8.5: subcarrier-major, symbol-minor - k' is the OUTER loop. The REG "at (k', l')"
	// is the one whose lowest subcarrier is k'.
	for(int k = 0; k < 12 * nrb; k++) {
		for(int l = 0; l < cfi; l++) {
			// six-subcarrier REGs wherever the symbol carries pilots: symbol 0 always
			// (ports 0/1 are reserved even by a 1-port cell), symbol 1 only at four ports
			const bool wide = (l == 0) || (l == 1 && nof_ports == 4);
			if(wide ? (k % 6) != 0 : (k % 4) != 0) {
				continue;
			}
			if(l == 0 && taken[(size_t)k]) {
				continue;
			}
			wb_reg r;
			r.l = l;
			int m = 0;
			if(wide) {
				// both CRS combs sit at the same offsets in every REG (anchors are
				// multiples of 6): vshift and vshift+3 - same rule for ports 0/1 and 2/3
				for(int dd = 0; dd < 6; dd++) {
					if(dd == vs || dd == (vs + 3) % 6) {
						continue;
					}
					r.k[m++] = k + dd;
				}
			} else {
				for(int dd = 0; dd < 4; dd++) {
					r.k[m++] = k + dd;
				}
			}
			regs.push_back(r);
		}
	}
	return (int)regs.size();
}

void wb_pdcch_perm(int m, std::vector<int> &P) {
	// 36.212 5.1.4.2.1 on quadruplet indices: R rows of 32 columns, filled row-wise with
	// the <NULL> pad at the FRONT, read column-wise in the permuted column order, <NULL>s
	// dropped. Same matrix the rate matcher uses - but on indices, and never interlaced.
	const int C = 32;
	const int R = (m + C - 1) / C;
	const int nd = R * C - m;
	P.clear();
	P.reserve((size_t)m);
	for(int cc = 0; cc < C; cc++) {
		const int col = SUBBLOCK_PERM[cc];
		for(int r = 0; r < R; r++) {
			const int src = r * C + col - nd;
			if(src >= 0) {
				P.push_back(src);
			}
		}
	}
}

// Channel per port for the control symbols 0..2, interpolated in time exactly like the
// PBCH estimate: ports 0/1 between the slot's pilot symbols 0 and 4, ports 2/3 between
// symbol 1 of this slot and symbol 1 of the next (where v moves with the slot parity).
static bool ctrl_chest(const cf *x, size_t n, int64_t slot_start, int ns, int nid_cell,
                       int nof_ports, const wb_num &g, const fft_plan &plan,
                       std::vector<cf> grid[3], std::vector<cf> h[4][3]) {
	const int nc = 12 * g.nrb;
	for(int l = 0; l < 3; l++) {
		grid[l].resize((size_t)nc);
		if(!wb_demod_symbol(x, n, slot_start, l, g, plan, grid[l].data())) {
			return false;
		}
	}
	std::vector<cf> g4((size_t)nc), g8((size_t)nc);
	const bool have4 = wb_demod_symbol(x, n, slot_start, 4, g, plan, g4.data());
	const bool have8 = (nof_ports == 4) &&
	                   wb_demod_symbol(x, n, slot_start + g.slot, 1, g, plan, g8.data());
	for(int p = 0; p < 4; p++) {
		if(p >= nof_ports) {
			for(int l = 0; l < 3; l++) {
				h[p][l].assign((size_t)nc, cf(0.0f, 0.0f));
			}
			continue;
		}
		std::vector<cf> ha((size_t)nc), hb((size_t)nc);
		float ta, tb;
		std::vector<crs_ref> pa, pb;
		if(p < 2) {
			wb_crs(nid_cell, g.nrb, p, ns, 0, pa);
			wb_chan_est(grid[0].data(), pa, nc, ha.data());
			ta = 0.0f;
			if(have4) {
				wb_crs(nid_cell, g.nrb, p, ns, 4, pb);
				wb_chan_est(g4.data(), pb, nc, hb.data());
				tb = 4.0f;
			} else {
				hb = ha;
				tb = 1.0f;
			}
		} else {
			wb_crs(nid_cell, g.nrb, p, ns, 1, pa);
			wb_chan_est(grid[1].data(), pa, nc, ha.data());
			ta = 1.0f;
			if(have8) {
				wb_crs(nid_cell, g.nrb, p, ns + 1, 1, pb);
				wb_chan_est(g8.data(), pb, nc, hb.data());
				tb = 8.0f;
			} else {
				hb = ha;
				tb = 2.0f;
			}
		}
		for(int l = 0; l < 3; l++) {
			h[p][l].resize((size_t)nc);
			const float w = ((float)l - ta) / (tb - ta);
			for(int k = 0; k < nc; k++) {
				h[p][l][(size_t)k] = ha[(size_t)k] * (1.0f - w) + hb[(size_t)k] * w;
			}
		}
	}
	return true;
}

int wb_pdcch_css_decode(const cf *x, size_t n, int64_t slot_start, int ns, int nid_cell,
                        int nof_ports, int cfi, int phich_dur, float phich_res,
                        const wb_num &g, const fft_plan &plan, std::vector<wb_dci> &out) {
	std::vector<cf> grid[3];
	std::vector<cf> h[4][3];
	if(!ctrl_chest(x, n, slot_start, ns, nid_cell, nof_ports, g, plan, grid, h)) {
		return -1;
	}
	std::vector<wb_reg> regs;
	const int M = wb_pdcch_regs(nid_cell, g.nrb, cfi, nof_ports, phich_dur, phich_res, regs);
	if(M < 9) {
		return -1;
	}
	std::vector<int> P;
	wb_pdcch_perm(M, P);

	// REG m' carries w_bar(m') = w((m'+NID) mod M) = z(P[(m'+NID) mod M]): combine each
	// quadruplet where it stands (the Alamouti pairs live INSIDE a quadruplet, so the
	// permutation never splits them) and write it straight to its z slot.
	std::vector<cf> z((size_t)(4 * M));
	for(int m = 0; m < M; m++) {
		const wb_reg &r = regs[(size_t)m];
		cf re4[4], hb[4][4];
		for(int j = 0; j < 4; j++) {
			re4[j] = grid[r.l][(size_t)r.k[j]];
			for(int p = 0; p < 4; p++) {
				hb[p][j] = h[p][r.l][(size_t)r.k[j]];
			}
		}
		const cf *hp[4] = { hb[0], hb[1], hb[2], hb[3] };
		cf out4[4];
		wb_sfbc_combine(re4, hp, 4, nof_ports, out4);
		const int zi = P[(size_t)((m + nid_cell) % M)];
		for(int j = 0; j < 4; j++) {
			z[(size_t)(4 * zi + j)] = out4[j];
		}
	}

	// QPSK demap and descrambling over the WHOLE multiplexed block - the scrambler runs
	// across all N_REG quadruplets including <NIL> fill, so a candidate's bits sit at
	// their absolute position 72*CCE, not at 0.
	std::vector<float> llr((size_t)(8 * M));
	for(int i = 0; i < 4 * M; i++) {
		llr[(size_t)(2 * i)] = z[(size_t)i].real();
		llr[(size_t)(2 * i + 1)] = z[(size_t)i].imag();
	}
	std::vector<uint8_t> sc((size_t)(8 * M));
	const uint32_t cinit = (((uint32_t)(ns / 2)) << 9) + (uint32_t)nid_cell;
	gold_seq(cinit, 8 * M, sc.data());
	for(int i = 0; i < 8 * M; i++) {
		if(sc[(size_t)i]) {
			llr[(size_t)i] = -llr[(size_t)i];
		}
	}

	const int ncce = M / 9;
	uint8_t simask[16];
	rnti_crc_mask(SI_RNTI, simask);
	const int sizes[2] = { dci_size_1a(g.nrb), dci_size_1c(g.nrb) };
	int found = 0;
	for(int L : { 4, 8 }) {
		for(int c0 = 0; c0 + L <= 16; c0 += L) {
			if(c0 + L > ncce) {
				continue;
			}
			for(int si = 0; si < 2; si++) {
				const int A = sizes[si];
				const size_t K = (size_t)(A + 16);
				const int E = 72 * L;
				std::vector<float> le(llr.begin() + 72 * c0, llr.begin() + 72 * c0 + E);
				std::vector<float> soft;
				conv_rate_dematch(le, K, soft);
				std::vector<uint8_t> dec;
				const double met = conv_decode_fast(soft, K, dec, 4);
				if(!(met > -1e29) || dec.size() != K) {
					continue;
				}
				// the all-zero payload sails through a masked CRC too easily on dead REs
				int nz = 0;
				for(int i = 0; i < A; i++) {
					nz += dec[(size_t)i];
				}
				if(nz == 0 || !crc_check(dec.data(), K, CRC16, simask)) {
					continue;
				}
				wb_dci dci{};
				dci.size = A;
				dci.L = L;
				dci.cce = c0;
				double asum = 1e-20;
				for(float v : soft) {
					asum += std::fabs(v);
				}
				dci.metric = met / asum;
				for(int i = 0; i < A && i < 32; i++) {
					dci.bits[i] = dec[(size_t)i];
				}
				out.push_back(dci);
				found++;
			}
		}
	}
	return found;
}

int wb_pdsch_cw_llr(const cf *x, size_t n, int64_t sf_start, int ns, int nid_cell,
                    int nof_ports, int cfi, int rb_start, int n_prb, uint16_t rnti,
                    const wb_num &g, const fft_plan &plan, std::vector<float> &llr) {
	llr.clear();
	const int nc = 12 * g.nrb;
	if(cfi < 1 || cfi > 3 || rb_start < 0 || n_prb < 1 || rb_start + n_prb > g.nrb ||
	   (nof_ports != 1 && nof_ports != 2 && nof_ports != 4)) {
		return -1;
	}
	// the subframe's 14 symbols are mandatory; the two anchors in the NEXT subframe
	// (symbol 0 for ports 0/1, symbol 1 for 2/3) are optional and only buy margin
	std::vector<cf> grid[16];
	bool have[16];
	for(int t = 0; t < 16; t++) {
		grid[t].assign((size_t)nc, cf(0.0f, 0.0f));
		const int64_t ss = sf_start + (int64_t)(t / 7) * g.slot;
		have[t] = wb_demod_symbol(x, n, ss, t % 7, g, plan, grid[t].data());
		if(t < 14 && !have[t]) {
			return -1;
		}
	}

	// Channel per port, interpolated in TIME between every pilot-bearing symbol of the
	// subframe (ports 0/1: t = 0,4,7,11,14; ports 2/3: t = 1,8,15) - the same discipline
	// as the PBCH estimate, but across a whole subframe instead of four symbols. Frozen
	// estimates cost decode margin on air and PDSCH spans 11 symbols, almost
	// three times the PBCH window.
	struct anchor {
		float t;
		std::vector<cf> h;
	};
	std::vector<anchor> anc[4];
	static const int T01[5] = { 0, 4, 7, 11, 14 };
	static const int T23[3] = { 1, 8, 15 };
	for(int p = 0; p < nof_ports; p++) {
		const int *ts = (p < 2) ? T01 : T23;
		const int nt = (p < 2) ? 5 : 3;
		for(int i = 0; i < nt; i++) {
			const int t = ts[i];
			if(!have[t]) {
				continue;
			}
			std::vector<crs_ref> pil;
			wb_crs(nid_cell, g.nrb, p, ns + t / 7, t % 7, pil);
			anchor a;
			a.t = (float)t;
			a.h.resize((size_t)nc);
			wb_chan_est(grid[t].data(), pil, nc, a.h.data());
			anc[p].push_back(std::move(a));
		}
		if(anc[p].empty()) {
			return -1;
		}
	}

	const int vs = nid_cell % 6;
	const int c0 = 6 * g.nrb - 36;   // first subcarrier of the centre six PRB
	std::vector<cf> hsym[4];
	for(auto &v : hsym) {
		v.resize((size_t)nc);
	}
	std::vector<cf> re, comb;
	std::vector<cf> hre[4];
	for(int t = cfi; t < 14; t++) {
		const int l = t % 7, slot_ns = ns + t / 7;
		// interpolate every active port's channel to this symbol
		for(int p = 0; p < nof_ports; p++) {
			const auto &A = anc[p];
			size_t i = 0;
			while(i + 2 < A.size() && A[i + 1].t <= (float)t) {
				i++;
			}
			const float t0 = A[i].t;
			const float t1 = (i + 1 < A.size()) ? A[i + 1].t : t0;
			const float w = (t1 > t0) ? std::min(std::max(((float)t - t0) / (t1 - t0), 0.0f),
			                                     1.0f)
			                          : 0.0f;
			const cf *ha = A[i].h.data();
			const cf *hb = (i + 1 < A.size()) ? A[i + 1].h.data() : ha;
			for(int k = 0; k < nc; k++) {
				hsym[p][(size_t)k] = ha[k] * (1.0f - w) + hb[k] * w;
			}
		}
		// REs this symbol may not touch: the CRS combs of every CONFIGURED port (36.211
		// 6.3.5 excludes the cell's actual reference signals, unlike PBCH's fixed 4-port
		// reservation), and in subframes 0/5 the centre six PRB of the sync symbols
		// (plus PBCH's four symbols in subframe 0) - the guard REs around PSS/SSS are
		// reserved too, so the exclusion is the full 72 subcarriers, not 62.
		bool resv[6] = { false, false, false, false, false, false };
		for(int p = 0; p < nof_ports; p++) {
			int ls[3];
			const int nl = wb_crs_symbols(p, ls);
			for(int j = 0; j < nl; j++) {
				if(ls[j] == l) {
					resv[(crs_v(p, slot_ns, l) + vs) % 6] = true;
				}
			}
		}
		const bool sync_mid = (ns == 0 || ns == 10) && t / 7 == 0 && (l == 5 || l == 6);
		const bool pbch_mid = (ns == 0) && t >= 7 && t <= 10;
		re.clear();
		for(auto &v : hre) {
			v.clear();
		}
		for(int k = 12 * rb_start; k < 12 * (rb_start + n_prb); k++) {
			if(resv[k % 6]) {
				continue;
			}
			if((sync_mid || pbch_mid) && k >= c0 && k < c0 + 72) {
				continue;
			}
			re.push_back(grid[t][(size_t)k]);
			for(int p = 0; p < nof_ports; p++) {
				hre[p].push_back(hsym[p][(size_t)k]);
			}
		}
		// SFBC pairs and FSTD quadruplets never straddle an OFDM symbol: every per-symbol
		// count here is a multiple of 4 by construction (12 or 12-minus-4-CRS REs per RB).
		// A count that is not is a mapping bug, and decoding on would scramble everything
		// downstream of it - fail loud instead.
		if(nof_ports > 1 && (re.size() % 4) != 0) {
			return -1;
		}
		comb.resize(re.size());
		const cf *hp[4] = { hre[0].data(), hre[1].data(), hre[2].data(), hre[3].data() };
		wb_sfbc_combine(re.data(), hp, (int)re.size(), nof_ports, comb.data());
		for(const cf &v : comb) {
			llr.push_back(v.real());
			llr.push_back(v.imag());
		}
	}

	// descrambling, 36.211 6.3.1: cinit = n_RNTI*2^14 + q*2^13 + floor(ns/2)*2^9 + NID,
	// with q = 0 - the broadcast path never has a second codeword
	const uint32_t cinit = ((uint32_t)rnti << 14) + ((uint32_t)(ns / 2) << 9) +
	                       (uint32_t)nid_cell;
	std::vector<uint8_t> sc(llr.size());
	gold_seq(cinit, llr.size(), sc.data());
	for(size_t i = 0; i < llr.size(); i++) {
		if(sc[i]) {
			llr[i] = -llr[i];
		}
	}
	return (int)llr.size();
}

bool wb_mib_decode_n(const cf *const re[], const cf (*const h[])[240], int nf, int nid_cell,
                     int q0, int *nof_ports, int *nrb, int *sfn, int *phich_dur,
                     float *phich_res, wb_mib_stats *st) {
	// The bound, and it is load-bearing. One 40 ms TTI is ONE payload: the eight SFN MSBs
	// sit in it and step at the boundary. A group running past quarter 3 therefore sums the
	// LLRs of two different MIBs - the agreeing bits reinforce, the disagreeing ones cancel
	// toward zero, the convolutional code fills them back in and CRC-16 passes on a payload
	// that was never sent, with the SFN off by four frames. That SFN then picks the SIB1
	// subframes, so the damage is silent and lands somewhere else entirely. Refuse the group
	// here, where no caller can get it wrong.
	if(nf < 1 || (q0 < 0 ? nf != 1 : (q0 > 3 || q0 + nf > 4))) {
		return false;
	}
	// ONE number per call, folded into `st` on the way out whichever way this leaves: the best
	// of the handful of hypotheses this call itself walks. It is deliberately not maximised
	// into `st` across calls - see wb_mib_metric for what that cost.
	struct metric_fold {
		wb_mib_stats *st;
		double best;
		bool any;
		~metric_fold() {
			if(st && any) {
				st->metric_sum += best;
				st->metric_n++;
			}
		}
	} mf{ st, 0.0, false };
	// PBCH scrambling covers the whole 40 ms block: 1920 bits, cinit = NID, reinitialised
	// when SFN mod 4 == 0. Each frame carries one 480-bit quarter of it and is descrambled
	// with its own segment, which is what makes observations of different frames
	// commensurate. With q0 < 0 the quarter is unknown and becomes a hypothesis dimension.
	std::vector<uint8_t> cseq(1920);
	gold_seq((uint32_t)nid_cell, 1920, cseq.data());

	for(int np : PBCH_PORT_HYPOTHESES) {
		// --- SFBC combining (36.211 6.3.4.2.2 / 6.3.4.3), shared with PCFICH/PDCCH, then
		// --- QPSK demap: positive LLR means bit 0, matching conv_decode's convention
		std::vector<std::vector<float>> l480((size_t)nf);
		for(int f = 0; f < nf; f++) {
			std::vector<cf> sym(240);
			std::vector<float> w(240, 0.0f);
			const cf *hp[4] = { h[f][0], h[f][1], h[f][2], h[f][3] };
			wb_sfbc_combine(re[f], hp, 240, np, sym.data(), w.data());
			std::vector<float> &l = l480[(size_t)f];
			l.reserve(480);
			for(const cf &v : sym) {
				l.push_back(v.real());
				l.push_back(v.imag());
			}
			// --- MRC. This is what makes combining SAFE on a TTI whose four frames are not
			// equally strong. The combiner above zero-forces, so every observation leaves it
			// with the SAME signal amplitude and its OWN noise level; adding those unweighted
			// lets three weak frames bury a strong one, and the group is only ever tried whole
			// (measured before this: one frame alone 150/150, the same frame plus three 20 dB
			// weaker ones 53/150 - combining LOSING cells the single-frame path decodes).
			// Two weights restore the matched filter:
			//   per RE, the effective channel gain the combiner divided out, relative to this
			//   observation's mean so that the mean gain is not counted twice - that is the
			//   frequency-selective half, a deep-faded RE stops shouting;
			//   per observation, 1/noise-variance from the M2M4 moment estimator (m2 = A^2+v,
			//   m4 = A^4+6A^2v+3v^2 => A^2 = sqrt(2 m2^2 - m4)). After zero-forcing A is the
			//   TRANSMIT constellation amplitude, common to every observation whatever the
			//   channel and the receiver gain did, so the variances are directly comparable -
			//   which is the whole reason the estimate can be made per observation and used
			//   across them. It also catches a frame that is weak because the interference
			//   moved, not because the channel faded, which |h|^2 alone cannot see.
			double m2 = 0.0, m4 = 0.0, dsum = 0.0;
			for(float v : l) {
				const double s2 = (double)v * (double)v;
				m2 += s2;
				m4 += s2 * s2;
			}
			m2 /= (double)l.size();
			m4 /= (double)l.size();
			for(float v : w) {
				dsum += (double)v;
			}
			const double a2 = std::sqrt(std::max(2.0 * m2 * m2 - m4, 0.0));
			// the floor keeps a noiseless synthetic observation from taking infinite weight
			const double v_n = std::max(m2 - a2, 1e-3 * m2 + 1e-30);
			const double dbar = dsum > 0.0 ? dsum / (double)w.size() : 1.0;
			for(size_t i = 0; i < l.size(); i++) {
				l[i] = (float)((double)l[i] * ((double)w[i / 2] / dbar) / v_n);
			}
		}

		for(int qi = 0; qi < (q0 >= 0 ? 1 : 4); qi++) {
			const int q = (q0 >= 0) ? q0 : qi;
			// the quarters of a group are disjoint by construction, so this fill never has
			// to sum: the four quarters are four copies of the same 120 mother-code bits
			// and conv_rate_dematch below is what adds them up
			std::vector<float> full(1920, 0.0f);
			for(int f = 0; f < nf; f++) {
				const int q0f = ((q + f) & 3) * 480;
				for(int i = 0; i < 480; i++) {
					const int j = q0f + i;
					full[(size_t)j] = cseq[(size_t)j] ? -l480[(size_t)f][(size_t)i]
					                                  : l480[(size_t)f][(size_t)i];
				}
			}
			std::vector<float> soft;
			conv_rate_dematch(full, 40, soft);
			std::vector<uint8_t> dec;
			const double m = conv_decode_fast(soft, 40, dec, 4);
			if(dec.size() != 40) {
				continue;
			}
			if(st && m > -1e30) {
				// How close this hypothesis came, measured on the word that actually came
				// out: re-encode it and correlate with the soft bits, normalised by
				// sum|soft|, so 1.0 means the surviving path agreed with the sign of every
				// soft bit and the number does not move with the receiver's gain.
				// conv_decode_fast's OWN return value cannot be used for this: a tail-biting
				// path that needs more than one lap is scored from RENORMALISED state
				// metrics, so the same quality of fit reports wildly different numbers
				// (measured: pure-noise means from -0.13 to +0.74 with nothing changed but
				// the noise seed) and the diagnostic swings on the lap count rather than on
				// the signal.
				std::vector<uint8_t> cw;
				conv_encode(dec, cw);
				double agree = 0.0, mag = 0.0;
				for(size_t i = 0; i < soft.size() && i < cw.size(); i++) {
					agree += cw[i] ? -(double)soft[i] : (double)soft[i];
					mag += std::fabs((double)soft[i]);
				}
				if(mag > 0.0 && (!mf.any || agree / mag > mf.best)) {
					mf.best = agree / mag;
					mf.any = true;
				}
			}
			// the all-zero word is a valid codeword and passes an unmasked CRC trivially -
			// reject it and any bandwidth index outside the LTE table before trusting CRC
			int nz = 0;
			for(auto b : dec) {
				nz += b;
			}
			const int bwc = (dec[0] << 2) | (dec[1] << 1) | dec[2];
			if(nz == 0 || bwc > 5) {
				if(st) { st->bw_reject++; }
				continue;
			}
			uint8_t mask[16];
			pbch_crc_mask(np, mask);
			if(!crc_check(dec.data(), 40, CRC16, mask)) {
				if(st) { st->crc_fail++; }
				continue;
			}
			static const int BW[6] = { 6, 15, 25, 50, 75, 100 };
			static const float PHICH_RES[4] = { 1.0f / 6.0f, 0.5f, 1.0f, 2.0f };
			int sfn8 = 0;
			for(int i = 0; i < 8; i++) {
				sfn8 = (sfn8 << 1) | dec[6 + i];
			}
			if(nof_ports) { *nof_ports = np; }
			if(nrb) { *nrb = BW[bwc]; }
			if(sfn) { *sfn = sfn8 * 4 + q; }   // q is observation 0's quarter
			if(phich_dur) { *phich_dur = dec[3]; }
			if(phich_res) { *phich_res = PHICH_RES[(dec[4] << 1) | dec[5]]; }
			return true;
		}
	}
	return false;
}

bool wb_mib_decode(const cf *re, const cf h[4][240], int n_re, int nid_cell,
                   int *nof_ports, int *nrb, int *sfn, int *phich_dur, float *phich_res) {
	return n_re == 240 && wb_mib_decode_n(&re, &h, 1, nid_cell, -1, nof_ports, nrb, sfn,
	                                      phich_dur, phich_res, nullptr);
}

// ---- PSS search -------------------------------------------------------------------------

namespace {
// Unit-energy TIME template of the PSS symbol body at this numerology: the 62 subcarriers
// mapped around DC, transformed back, normalised. IFFT = conj(FFT(conj(.)))/N, and both the
// 1/N and the outer conj fold into the energy normalisation.
//
// ONE definition, used by acquisition and by tracking. They must agree on the reported
// position to the sample - two separately built templates are exactly how they would not,
// and the disagreement would look like a slow tracking drift rather than like a bug.
// `plan` must be of size g.fft.
void wb_pss_tpl(int nid2, const wb_num &g, const fft_plan &plan, std::vector<cf> &t) {
	std::vector<cf> bins((size_t)g.fft, cf(0.0f, 0.0f));
	std::vector<cf> d = pss_freq(nid2);
	for(int i = 0; i < 62; i++) {
		const int k = (i < 31) ? (i - 31) : (i - 30);
		bins[(size_t)((k + g.fft) % g.fft)] = std::conj(d[(size_t)i]);
	}
	std::vector<cf> tt((size_t)g.fft);
	plan.run(bins.data(), tt.data());
	double e = 0.0;
	for(auto &v : tt) {
		e += std::norm(v);
	}
	const float s = (float)(1.0 / std::sqrt(e + 1e-30));
	t.resize((size_t)g.fft);
	for(int i = 0; i < g.fft; i++) {
		t[(size_t)i] = std::conj(tt[(size_t)i]) * s;
	}
}
} // namespace

bool wb_pss_search(const cf *x, size_t n, const wb_num &g, float thresh,
                   int *nid2, int64_t *pos, float *metric) {
	const int L = g.fft;                       // template length = one OFDM symbol
	const unsigned W = 8192;                   // overlap-save window
	if(n < (size_t)L + 1 || W <= (unsigned)L) {
		return false;
	}
	const int valid = (int)W - L + 1;          // lags fully inside the window
	fft_plan pw(W);
	fft_plan ps((unsigned)g.fft);

	// conj(FFT(template)) once per NID2, zero padded to W
	std::vector<std::vector<cf>> tf(3, std::vector<cf>(W));
	for(int m = 0; m < 3; m++) {
		std::vector<cf> t(W, cf(0.0f, 0.0f)), tpl;
		wb_pss_tpl(m, g, ps, tpl);
		std::copy(tpl.begin(), tpl.end(), t.begin());
		std::vector<cf> f(W);
		pw.run(t.data(), f.data());
		for(unsigned i = 0; i < W; i++) {
			tf[(size_t)m][i] = std::conj(f[i]);
		}
	}

	float best = 0.0f;
	int best_m = -1;
	int64_t best_p = 0;
	std::vector<cf> win(W), spec(W), prod(W), corr(W), tmp(W);
	std::vector<double> pfx(W + 1);

	for(size_t off = 0; off + W <= n; off += (size_t)valid) {
		std::copy(x + off, x + off + W, win.begin());
		pw.run(win.data(), spec.data());
		// sliding energy over L samples, from a prefix sum - a window property, computed once
		pfx[0] = 0.0;
		for(unsigned i = 0; i < W; i++) {
			pfx[i + 1] = pfx[i] + (double)std::norm(win[i]);
		}
		for(int m = 0; m < 3; m++) {
			for(unsigned i = 0; i < W; i++) {
				prod[i] = std::conj(spec[i] * tf[(size_t)m][i]);   // conj for the inverse
			}
			pw.run(prod.data(), tmp.data());
			const float inv = 1.0f / (float)W;
			for(int i = 0; i < valid; i++) {
				corr[(size_t)i] = std::conj(tmp[(size_t)i]) * inv;
			}
			for(int i = 0; i < valid; i++) {
				const double p = pfx[(size_t)(i + L)] - pfx[(size_t)i];
				const float mv = (float)(std::norm(corr[(size_t)i]) / (p + 1e-30));
				if(mv > best) {
					best = mv;
					best_m = m;
					best_p = (int64_t)off + i;
				}
			}
		}
	}
	if(best_m < 0 || best < thresh) {
		return false;
	}
	*nid2 = best_m;
	*pos = best_p;
	*metric = best;
	return true;
}

// ---- PSS tracking -----------------------------------------------------------------------

const char *wb_track_str(wb_track_status s) {
	switch(s) {
	case WB_TRACK_OK:      return "ok";
	case WB_TRACK_WEAK:    return "weak";
	case WB_TRACK_EDGE:    return "edge";
	case WB_TRACK_STALE:   return "stale";
	case WB_TRACK_RANGE:   return "range";
	case WB_TRACK_NOSTATE: return "nostate";
	}
	return "?";
}

void wb_track_init(wb_track &st, int nid2, double pos, double t_sec, float metric,
                   double drift_ppm, double unc_ppm) {
	st = wb_track();
	st.valid = nid2 >= 0 && nid2 <= 2;
	st.nid2 = nid2;
	st.pos = pos;
	st.t_sec = t_sec;
	st.drift_ppm = drift_ppm;
	st.unc_ppm = unc_ppm > 0.0 ? unc_ppm : 5.0;
	st.metric = metric;
	st.ref_metric = metric;
}

wb_track_status wb_pss_track(const cf *x, size_t n, double t_sec, const wb_num &g,
                             const fft_plan &plan, const wb_track_cfg &cfg, wb_track &st,
                             int64_t *pos, float *metric) {
	const int L = g.fft;
	if(!st.valid || st.nid2 < 0 || st.nid2 > 2 || (unsigned)L != plan.size() ||
	   n < (size_t)L + 1) {
		return WB_TRACK_NOSTATE;
	}
	const double F = (double)g.frame;
	const double elapsed = t_sec - st.t_sec;
	if(!(elapsed >= 0.0)) {
		return WB_TRACK_NOSTATE;   // time ran backwards: the caller's clock is not ours
	}
	const double E = elapsed * g.fs;          // nominal samples since the state's fix

	// --- the window, derived and not guessed: what we do not know about the drift, times
	// how long we have been away, plus a fixed guard for everything that is not drift.
	// The cap at frame/4 is a hard safety property, not a tuning knob: the PSS repeats
	// every HALF frame, so once the window reaches half of that the identical PSS 5 ms
	// away is inside it and a "successful" fix could be a half-frame off - which decodes
	// subframe 5 as subframe 0 and is exactly the silent wrongness worth refusing.
	int max_half = cfg.max_half;
	const int alias_cap = g.frame / 4;
	if(max_half > alias_cap) {
		max_half = alias_cap;
	}
	if(max_half < 1) {
		max_half = 1;
	}
	const double need = elapsed * st.unc_ppm * 1e-6 * g.fs + (double)cfg.guard;
	if(need > (double)max_half) {
		st.window = max_half;
		return WB_TRACK_STALE;    // a full search is the honest answer from here on
	}
	const int half = (int)std::ceil(need);
	const int nlag = 2 * half + 1;
	st.window = half;

	// --- prediction: the fix, moved back by the elapsed samples, then forward by whole
	// frames - each frame longer than nominal by the drift we have estimated.
	const double d = st.drift_ppm * 1e-6;
	const double fper = F * (1.0 + d);
	int64_t k = (int64_t)std::ceil(((double)half - st.pos + E) / fper);
	if(k < 0) {
		k = 0;
	}
	double pred = st.pos - E + (double)k * fper;
	// step forward until the whole correlation span fits, so a buffer that starts mid-frame
	// tracks the next frame rather than failing (the ceil above lands within one frame, so
	// this is a correction of at most one - the bound only keeps a degenerate fper honest)
	for(int bump = 0; bump < 4 && pred - (double)half < 0.0; bump++) {
		k++;
		pred = st.pos - E + (double)k * fper;
	}
	const int64_t ip = (int64_t)std::llround(pred);
	if(ip - half < 0 || (size_t)(ip + half + L) > n) {
		return WB_TRACK_RANGE;
	}

	// --- sliding correlation, one template. The power is a property of the WINDOW, not of
	// the template, so it comes from a prefix sum once instead of from the inner loop -
	// tracker.cpp can afford to recompute it per lag at 128 taps, at 1024 it would double
	// the cost of the whole function.
	std::vector<cf> tpl;
	wb_pss_tpl(st.nid2, g, plan, tpl);
	const cf *xw = x + (ip - half);
	const size_t span_n = (size_t)(nlag - 1) + (size_t)L;
	std::vector<double> pfx(span_n + 1);
	pfx[0] = 0.0;
	for(size_t i = 0; i < span_n; i++) {
		pfx[i + 1] = pfx[i] + (double)std::norm(xw[i]);
	}

	std::vector<float> met((size_t)nlag);
	float best = 0.0f;
	int besti = 0;
#ifdef __aarch64__
	// conj-template dot, 4-wide - the same NEON shape as cell_tracker::step(), at 1024 taps
	std::vector<float> tr((size_t)L), ti((size_t)L);
	for(int m = 0; m < L; m++) {
		tr[(size_t)m] = tpl[(size_t)m].real();
		ti[(size_t)m] = tpl[(size_t)m].imag();
	}
	for(int i = 0; i < nlag; i++) {
		const float *xp = reinterpret_cast<const float *>(xw + i);
		float32x4_t cr = vdupq_n_f32(0), ci = vdupq_n_f32(0);
		for(int m = 0; m < L; m += 4) {
			float32x4x2_t xv = vld2q_f32(xp + 2 * m);
			float32x4_t trv = vld1q_f32(tr.data() + m), tiv = vld1q_f32(ti.data() + m);
			cr = vfmaq_f32(vfmaq_f32(cr, xv.val[0], trv), xv.val[1], tiv);
			ci = vfmaq_f32(vfmsq_f32(ci, xv.val[0], tiv), xv.val[1], trv);
		}
		const float re = vaddvq_f32(cr), im = vaddvq_f32(ci);
		const double p = pfx[(size_t)(i + L)] - pfx[(size_t)i];
		met[(size_t)i] = (float)((double)(re * re + im * im) / (p + 1e-30));
		if(met[(size_t)i] > best) {
			best = met[(size_t)i];
			besti = i;
		}
	}
#else
	for(int i = 0; i < nlag; i++) {
		const cf *xp = xw + i;
		cf corr(0.0f, 0.0f);
		for(int m = 0; m < L; m++) {
			corr += xp[m] * std::conj(tpl[(size_t)m]);
		}
		const double p = pfx[(size_t)(i + L)] - pfx[(size_t)i];
		met[(size_t)i] = (float)((double)std::norm(corr) / (p + 1e-30));
		if(met[(size_t)i] > best) {
			best = met[(size_t)i];
			besti = i;
		}
	}
#endif

	// peak over the window's own off-peak floor. Reported, not gated on: over a wide window
	// the maximum of pure noise already stands ~10x over its own median, so this number
	// separates nothing there - it is a diagnostic, and the two gates below are the decision.
	{
		const int excl = 8;
		double sum = 0.0;
		int cnt = 0;
		for(int i = 0; i < nlag; i++) {
			if(std::abs(i - besti) > excl) {
				sum += met[(size_t)i];
				cnt++;
			}
		}
		st.margin = cnt >= 16 ? (float)(best / (sum / cnt + 1e-30)) : 0.0f;
	}

	// --- the two honest gates. Absolute: noise in a |corr|^2/power metric of 1024 taps sits
	// at 1/1024, and its maximum over even the widest allowed window reaches ~0.011, so 0.02
	// is below every real cell and above every empty window. Relative: the cell's own
	// smoothed metric - a signal 6 dB under what this cell has been giving us is not this
	// cell, however respectable it looks in absolute terms.
	const float gate = std::max(cfg.thresh, cfg.rel_thresh * st.ref_metric);
	const bool edge = besti == 0 || besti == nlag - 1;
	if(best < gate || edge) {
		st.misses++;
		st.hits = 0;
		st.metric = best;
		// conservative: a miss may be fading, but it may equally be a model that has come
		// loose. Widening costs a few lags; not widening costs the cell.
		st.unc_ppm = std::min(cfg.unc_ceil_ppm, st.unc_ppm * 1.5);
		if(st.misses >= cfg.max_miss) {
			st.valid = false;
		}
		return edge && best >= gate ? WB_TRACK_EDGE : WB_TRACK_WEAK;
	}

	// --- sub-sample peak: the parabola through the three metrics around it, exactly as the
	// sync-path tracker does it
	double frac = 0.0;
	const double den = (double)met[(size_t)(besti - 1)] - 2.0 * (double)met[(size_t)besti] +
	                   (double)met[(size_t)(besti + 1)];
	if(den < 0.0) {
		frac = 0.5 * ((double)met[(size_t)(besti - 1)] - (double)met[(size_t)(besti + 1)]) / den;
	}
	const double p_meas = (double)(ip - half + besti) + frac;
	const double err = p_meas - pred;

	// --- learn the drift, but only from a gap long enough to carry the information. Over a
	// single frame one sample of position noise reads as 6.5 ppm; updating from that would
	// be inventing a drift out of jitter and then widening the window to cover the invention.
	const double sspan = (double)k * F;
	if(sspan >= (double)cfg.min_span_frames * F) {
		st.drift_ppm += cfg.drift_alpha * err / sspan * 1e6;
		// the uncertainty is the evidence we just saw, doubled for headroom, floored at what
		// a single residual cannot see (oscillator wander between bursts) and never allowed
		// to collapse by more than 2x in one step
		double u = 2.0 * std::max(std::fabs(err), cfg.pos_sigma) / sspan * 1e6;
		u = std::max(u, 0.5 * st.unc_ppm);
		st.unc_ppm = std::min(cfg.unc_ceil_ppm, std::max(cfg.unc_floor_ppm, u));
	}

	st.pos = p_meas;
	st.t_sec = t_sec;
	st.frames += k;
	st.metric = best;
	st.ref_metric = st.ref_metric > 0.0f ? 0.7f * st.ref_metric + 0.3f * best : best;
	st.hits++;
	st.misses = 0;
	if(pos) {
		*pos = (int64_t)std::llround(p_meas);
	}
	if(metric) {
		*metric = best;
	}
	return WB_TRACK_OK;
}

// ---- SSS --------------------------------------------------------------------------------

bool wb_sss_decode(const cf *x, size_t n, int64_t pss_pos, int nid2, const wb_num &g,
                   const fft_plan &plan, int *nid1, int *half, float *margin) {
	// SSS is the symbol immediately before the PSS. In a normal-CP slot the PSS is symbol 6
	// and the SSS symbol 5, so stepping back one whole symbol (CP + FFT) lands on it.
	const int64_t sss_pos = pss_pos - (int64_t)(g.cp + g.fft);
	if(sss_pos < 0 || (size_t)(pss_pos + g.fft) > n) {
		return false;
	}
	std::vector<cf> ps((size_t)g.fft), ss((size_t)g.fft);
	plan.run(x + pss_pos, ps.data());
	plan.run(x + sss_pos, ss.data());

	// Both sequences live in the 62 subcarriers around DC, DC itself unused.
	std::vector<cf> pref = pss_freq(nid2);
	std::vector<float> eq(62, 0.0f);
	for(int i = 0; i < 62; i++) {
		const int k = (i < 31) ? (i - 31) : (i - 30);
		const size_t bin = (size_t)((k + g.fft) % g.fft);
		// The PSS is the channel estimate: h = received / transmitted, and the transmitted
		// symbol has unit modulus, so this is one conjugate multiply.
		const cf h = ps[bin] * std::conj(pref[(size_t)i]);
		const float p = std::norm(h);
		if(p < 1e-20f) {
			continue;
		}
		// SSS is BPSK, so only the component along the channel carries information.
		eq[(size_t)i] = (ss[bin] * std::conj(h)).real() / std::sqrt(p);
	}

	float best = -1e30f, second = -1e30f;
	int best_n1 = -1, best_h = 0;
	for(int h2 = 0; h2 < 2; h2++) {
		for(int n1 = 0; n1 < 168; n1++) {
			std::vector<float> ref = sss_seq(n1, nid2, h2);
			float acc = 0.0f;
			for(int i = 0; i < 62; i++) {
				acc += eq[(size_t)i] * ref[(size_t)i];
			}
			acc = std::fabs(acc);
			if(acc > best) {
				second = best;
				best = acc;
				best_n1 = n1;
				best_h = h2;
			} else if(acc > second) {
				second = acc;
			}
		}
	}
	if(best_n1 < 0) {
		return false;
	}
	*nid1 = best_n1;
	*half = best_h;
	*margin = (second > 0.0f) ? best / second : 99.0f;
	return true;
}

// ---- selftest --------------------------------------------------------------------------

namespace {
int g_fail = 0;
void check(bool ok, const char *name, const char *detail, bool verbose) {
	if(!ok) {
		g_fail++;
	}
	if(verbose || !ok) {
		printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail && *detail ? ": " : "",
		       detail ? detail : "");
	}
}

// Build ONE radio frame's PBCH observation: the 24-bit MIB `mib` encoded and rate matched
// to the 1920-bit 40 ms block, of which quarter `q` is transmitted over `ports` antennas
// through the per-port gains `hport`, at `snr_db` per RE. The transmit side is written
// forward from 36.211 6.6 / 6.3.4 exactly as wb_pbch_chain's is - NOT by inverting the
// receiver - so what the combining tests below measure is the receiver's, not a mirror's.
// `re`/`hout` come out as wb_pbch_extract fills them, i.e. ready for wb_mib_decode_n.
void pbch_frame(int nid, int ports, const std::vector<uint8_t> &mib, int q, float snr_db,
                std::mt19937 &rng, const cf hport[4], std::vector<cf> &re, cf hout[4][240]) {
	const int nrb = 50, nc = 12 * nrb, c0 = 6 * nrb - 36;
	const float A = 0.70710678f;
	std::normal_distribution<float> gauss(0.0f, 1.0f);

	std::vector<uint8_t> tx = mib;
	crc_attach(tx, CRC16);
	uint8_t mask[16];
	pbch_crc_mask(ports, mask);
	for(int i = 0; i < 16; i++) {
		tx[24 + i] ^= mask[i];
	}
	std::vector<uint8_t> cw, rm, scr(1920);
	conv_encode(tx, cw);
	conv_rate_match(cw, 1920, rm);
	gold_seq((uint32_t)nid, 1920, scr.data());
	std::vector<cf> s(240);
	for(int i = 0; i < 240; i++) {
		const int j = q * 480 + 2 * i;
		s[(size_t)i] = cf((rm[(size_t)j] ^ scr[(size_t)j]) ? -A : A,
		                  (rm[(size_t)(j + 1)] ^ scr[(size_t)(j + 1)]) ? -A : A);
	}
	// 6.3.4.2.2 for two ports; 6.3.4.3 for four, where the quadruplet puts SFBC(x0,x1) on
	// ports {0,2} in REs 4i/4i+1 and SFBC(x2,x3) on ports {1,3} in REs 4i+2/4i+3
	std::vector<cf> y[4];
	for(auto &v : y) {
		v.assign(240, cf(0.0f, 0.0f));
	}
	if(ports == 1) {
		y[0] = s;
	} else {
		for(int i = 0; i + 1 < 240; i += 2) {
			const int pa = (ports == 4 && ((i >> 1) & 1)) ? 1 : 0;
			const int pb = (ports == 4) ? pa + 2 : 1;
			y[pa][(size_t)i] = s[(size_t)i];
			y[pa][(size_t)(i + 1)] = s[(size_t)(i + 1)];
			y[pb][(size_t)i] = -std::conj(s[(size_t)(i + 1)]);
			y[pb][(size_t)(i + 1)] = std::conj(s[(size_t)i]);
		}
	}
	// centre 72 subcarriers of symbols 0..3, minus the CRS reservation, which always
	// assumes four ports whatever the cell uses
	std::vector<cf> grid[4];
	for(auto &v : grid) {
		v.assign((size_t)nc, cf(0.0f, 0.0f));
	}
	int idx = 0, pos[240][2];
	double pw = 0.0;
	for(int l = 0; l < 4; l++) {
		std::vector<uint8_t> skip((size_t)nc, 0);
		if(l < 2) {
			for(int p = 2 * l; p < 2 * l + 2; p++) {
				std::vector<crs_ref> qq;
				wb_crs(nid, nrb, p, 1, l, qq);
				for(auto &z : qq) {
					skip[(size_t)z.k] = 1;
				}
			}
		}
		for(int k = c0; k < c0 + 72 && idx < 240; k++) {
			if(skip[(size_t)k]) {
				continue;
			}
			cf v(0.0f, 0.0f);
			for(int p = 0; p < ports; p++) {
				v += hport[p] * y[p][(size_t)idx];
			}
			grid[l][(size_t)k] = v;
			pos[idx][0] = l;
			pos[idx][1] = k;
			pw += (double)std::norm(v);
			idx++;
		}
	}
	// noise scaled to the MEASURED mean RE power, so snr_db means the same thing at every
	// port count and channel - the combining gain below is a difference of two such points
	const float sig = (float)std::sqrt(pw / 240.0 * std::pow(10.0, -(double)snr_db / 10.0) / 2.0);
	for(int i = 0; i < 240; i++) {
		grid[pos[i][0]][(size_t)pos[i][1]] += cf(sig * gauss(rng), sig * gauss(rng));
	}
	// the true channel is handed to the receiver; estimation is pinned elsewhere
	std::vector<cf> hs[4][4];
	for(int p = 0; p < 4; p++) {
		for(int l = 0; l < 4; l++) {
			hs[p][l].assign((size_t)nc, hport[p]);
		}
	}
	const cf *gp[4] = { grid[0].data(), grid[1].data(), grid[2].data(), grid[3].data() };
	re.assign(240, cf(0.0f, 0.0f));
	wb_pbch_extract(gp, hs, nrb, nid, re.data(), hout);
}

// The 24-bit MIB of a 50 PRB cell (bw index 3, normal PHICH duration, resource 1) at SFN
// `sfn`: the eight MSBs are the payload, the two LSBs are which quarter carries it.
std::vector<uint8_t> pbch_mib(int sfn) {
	std::vector<uint8_t> mib(24, 0);
	mib[1] = 1;
	mib[2] = 1;
	mib[4] = 1;
	for(int i = 0; i < 8; i++) {
		mib[6 + i] = (uint8_t)(((sfn >> 2) >> (7 - i)) & 1);
	}
	return mib;
}

// One SI grant, as the two reference cells actually carry it: the shape is what the DCI
// names, NOT what the cell's antenna count implies, and the two are independent.
struct sib1_shape {
	int ports, nid, cfi, rb0, n_prb;
	size_t tbs;
};

// Build ONE SIB1 occasion (subframe 5) of that shape over a random 3-tap channel per port
// at `snr_db`, run it through the production receiver, and accumulate its soft bits into
// `soft` exactly as decode_cell() does. Returns false if the RE budget came out wrong.
// The 36.211 mapping itself is pinned by wb_pdsch_chain above; what this builder exists
// for is the MARGIN question - how many occasions a shape needs before the CRC verifies.
bool sib1_occasion(const sib1_shape &sh, int rv, float snr_db, std::mt19937 &rng,
                   const std::vector<uint8_t> &blk, std::vector<float> &soft) {
	wb_num g = wb_num_for(50);
	fft_plan plan((unsigned)g.fft);
	const int nc = 12 * g.nrb, c0 = 6 * g.nrb - 36, ns = 10, vs = sh.nid % 6;
	std::normal_distribution<float> gauss(0.0f, 1.0f);
	std::uniform_real_distribution<float> uni(0.0f, 1.0f);
	const float A = 0.70710678f;

	// per-port frequency response: three taps inside the CP, redrawn every occasion so the
	// accumulation gain measured below is the code's, not one lucky channel's
	std::vector<cf> H[4];
	for(int p = 0; p < 4; p++) {
		float amp[3], del[3], pha[3];
		for(int i = 0; i < 3; i++) {
			amp[i] = (i == 0) ? 1.0f : 0.4f + 0.4f * uni(rng);
			del[i] = uni(rng) * 20.0f;
			pha[i] = 2.0f * (float)M_PI * uni(rng);
		}
		H[p].resize((size_t)nc);
		for(int k = 0; k < nc; k++) {
			const int bin = (k < nc / 2) ? (k - nc / 2) : (k - nc / 2 + 1);
			cf s(0.0f, 0.0f);
			for(int i = 0; i < 3; i++) {
				const float a = pha[i] - 2.0f * (float)M_PI * (float)bin * del[i] /
				                (float)g.fft;
				s += amp[i] * cf(std::cos(a), std::sin(a));
			}
			H[p][(size_t)k] = s * 0.6f;
		}
	}
	std::vector<uint8_t> dd[3];
	turbo_encode(blk, dd);

	// RE budget of this grant, then E = 2 REs (QPSK, one codeword)
	size_t nre = 0;
	for(int pass = 0; pass < 2; pass++) {
		size_t idx = 0;
		std::vector<cf> s, y[4];
		std::vector<uint8_t> rm, sc;
		std::vector<cf> f[14];
		if(pass == 1) {
			const size_t E = 2 * nre;
			turbo_rate_match(dd, blk.size(), E, rv, rm);
			sc.resize(E);
			gold_seq(((uint32_t)0xFFFF << 14) + ((uint32_t)(ns / 2) << 9) + (uint32_t)sh.nid,
			         (int)E, sc.data());
			s.resize(nre);
			for(size_t i = 0; i < nre; i++) {
				s[i] = cf((rm[2 * i] ^ sc[2 * i]) ? -A : A,
				          (rm[2 * i + 1] ^ sc[2 * i + 1]) ? -A : A);
			}
			for(auto &v : y) {
				v.assign(nre, cf(0.0f, 0.0f));
			}
			for(size_t i = 0; i + 1 < nre; i += 2) {   // 6.3.4.2.2 / 6.3.4.3
				const int pa = (sh.ports == 4 && ((i >> 1) & 1)) ? 1 : 0;
				const int pb = (sh.ports == 4) ? pa + 2 : 1;
				y[pa][i] = s[i];
				y[pa][i + 1] = s[i + 1];
				y[pb][i] = -std::conj(s[i + 1]);
				y[pb][i + 1] = std::conj(s[i]);
			}
			for(auto &v : f) {
				v.assign((size_t)nc, cf(0.0f, 0.0f));
			}
		}
		for(int t = sh.cfi; t < 14; t++) {
			const int l = t % 7, sns = ns + t / 7;
			bool resv[6] = { false, false, false, false, false, false };
			for(int p = 0; p < sh.ports; p++) {
				int ls[3];
				const int nl = wb_crs_symbols(p, ls);
				for(int j = 0; j < nl; j++) {
					if(ls[j] == l) {
						resv[(crs_v(p, sns, l) + vs) % 6] = true;
					}
				}
			}
			const bool sync_mid = t / 7 == 0 && (l == 5 || l == 6);
			for(int k = 12 * sh.rb0; k < 12 * (sh.rb0 + sh.n_prb); k++) {
				if(resv[k % 6]) {
					continue;
				}
				if(sync_mid && k >= c0 && k < c0 + 72) {
					continue;
				}
				if(pass == 0) {
					nre++;
					continue;
				}
				cf v(0.0f, 0.0f);
				for(int p = 0; p < sh.ports; p++) {
					v += H[p][(size_t)k] * y[p][idx];
				}
				f[t][(size_t)k] += v;
				idx++;
			}
		}
		if(pass == 0) {
			continue;
		}
		// CRS of every configured port across the whole subframe - the estimator averages
		// them over all 50 PRB, which is where this chain's margin comes from
		for(int p = 0; p < sh.ports; p++) {
			int ls[3];
			const int nl = wb_crs_symbols(p, ls);
			for(int half = 0; half < 2; half++) {
				for(int j = 0; j < nl; j++) {
					const int t = 7 * half + ls[j], sns = ns + half;
					std::vector<cf> r((size_t)(2 * g.nrb));
					crs_seq(sh.nid, sns, ls[j], 110 - g.nrb, 2 * g.nrb, r.data());
					for(int m = 0; m < 2 * g.nrb; m++) {
						const int k = 6 * m + ((crs_v(p, sns, ls[j]) + vs) % 6);
						if(k >= 0 && k < nc) {
							f[t][(size_t)k] += H[p][(size_t)k] * r[(size_t)m];
						}
					}
				}
			}
		}
		std::vector<cf> x((size_t)(2 * g.slot), cf(0.0f, 0.0f));
		const int half = 6 * g.nrb;
		for(int t = 0; t < 14; t++) {
			std::vector<cf> bins((size_t)g.fft, cf(0.0f, 0.0f));
			for(int i = 0; i < half; i++) {
				bins[(size_t)(g.fft - half + i)] = std::conj(f[t][(size_t)i]);
				bins[(size_t)(1 + i)] = std::conj(f[t][(size_t)(half + i)]);
			}
			std::vector<cf> tt((size_t)g.fft);
			plan.run(bins.data(), tt.data());
			for(auto &v : tt) {
				v = std::conj(v) / (float)g.fft;
			}
			const int l = t % 7;
			const int64_t off = (int64_t)(t / 7) * g.slot + g.sym_off(l);
			for(int u = 0; u < g.cp_len(l); u++) {
				x[(size_t)(off - g.cp_len(l) + u)] = tt[(size_t)(g.fft - g.cp_len(l) + u)];
			}
			for(int u = 0; u < g.fft; u++) {
				x[(size_t)(off + u)] = tt[(size_t)u];
			}
		}
		// noise in TIME: the unnormalised g.fft-point receive FFT amplifies it by sqrt(fft),
		// so this sigma makes snr_db the per-RE ratio against a unit-modulus symbol
		const float sig = std::pow(10.0f, -snr_db / 20.0f) / std::sqrt((float)g.fft);
		for(auto &v : x) {
			v += cf(sig * gauss(rng) * A, sig * gauss(rng) * A);
		}
		std::vector<float> llr;
		const int E = wb_pdsch_cw_llr(x.data(), x.size(), 0, ns, sh.nid, sh.ports, sh.cfi,
		                              sh.rb0, sh.n_prb, 0xFFFF, g, plan, llr);
		if(E != (int)(2 * nre)) {
			return false;
		}
		turbo_rate_dematch(llr, blk.size(), rv, soft);
	}
	return true;
}
} // namespace

int lte_wb_selftest(bool verbose) {
	g_fail = 0;
	char d[200];

	// --- numerology has to close exactly, or every symbol offset drifts
	{
		wb_num g = wb_num_for(50);
		snprintf(d, sizeof(d), "fft %d cp0 %d cp %d slot %d frame %d fs %.2f MSps",
		         g.fft, g.cp0, g.cp, g.slot, g.frame, g.fs / 1e6);
		check(g.fft == 1024 && g.cp0 == 80 && g.cp == 72 && g.slot == 7680 &&
		      g.subframe == 15360 && g.frame == 153600 && std::fabs(g.fs - 15.36e6) < 1.0,
		      "wb_num_50rb", d, verbose);
		// the last symbol must end exactly on the slot boundary
		check(g.sym_off(6) + g.fft == g.slot, "wb_slot_closes", "symbol 6 ends on the boundary",
		      verbose);
	}

	// --- CRS geometry. Two properties matter and both are easy to get wrong:
	// ports 0/1 must not collide with each other in the same symbol, and ports 2/3 exist
	// only in symbol 1.
	{
		const int nid = 274, nrb = 50;
		std::vector<crs_ref> p0, p1, p2, p3;
		wb_crs(nid, nrb, 0, 0, 0, p0);
		wb_crs(nid, nrb, 1, 0, 0, p1);
		wb_crs(nid, nrb, 2, 0, 1, p2);
		wb_crs(nid, nrb, 3, 0, 1, p3);
		snprintf(d, sizeof(d), "port0 %zu port1 %zu port2 %zu port3 %zu pilots",
		         p0.size(), p1.size(), p2.size(), p3.size());
		check(p0.size() == (size_t)(2 * nrb) && p1.size() == (size_t)(2 * nrb) &&
		      p2.size() == (size_t)(2 * nrb) && p3.size() == (size_t)(2 * nrb),
		      "wb_crs_count", d, verbose);

		bool collide = false;
		for(auto &a : p0) {
			for(auto &b : p1) {
				if(a.k == b.k) {
					collide = true;
				}
			}
		}
		check(!collide, "wb_crs_p0_p1_disjoint", "ports 0 and 1 never share a subcarrier",
		      verbose);

		std::vector<crs_ref> none;
		wb_crs(nid, nrb, 2, 0, 0, none);
		check(none.empty(), "wb_crs_p2_only_sym1", "port 2 has no pilots in symbol 0", verbose);

		// v_shift must actually shift: two cells 1 apart in NID cannot share a comb.
		std::vector<crs_ref> a, b;
		wb_crs(6, nrb, 0, 0, 0, a);
		wb_crs(7, nrb, 0, 0, 0, b);
		check(!a.empty() && !b.empty() && a[0].k != b[0].k, "wb_crs_vshift",
		      "NID+1 moves the comb", verbose);
	}

	// --- demodulate a synthetic tone: a single subcarrier in, the same subcarrier out.
	// This pins the CP stripping, the FFT direction and the DC-skipping grid layout all at
	// once - if any of the three is wrong the energy lands somewhere else.
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		std::vector<cf> x((size_t)g.slot * 2, cf(0.0f, 0.0f));
		const int kc = 137;                       // grid index to plant
		const int half = 6 * g.nrb;
		const int bin = (kc < half) ? (g.fft - half + kc) : (1 + kc - half);
		// build symbol 3 of slot 0 as a pure tone at that bin, with its cyclic prefix
		std::vector<cf> sym((size_t)g.fft);
		for(int t = 0; t < g.fft; t++) {
			const double ph = 2.0 * M_PI * (double)bin * (double)t / (double)g.fft;
			sym[(size_t)t] = cf((float)std::cos(ph), (float)std::sin(ph));
		}
		const int off = g.sym_off(3);
		for(int t = 0; t < g.cp; t++) {
			x[(size_t)(off - g.cp + t)] = sym[(size_t)(g.fft - g.cp + t)];
		}
		for(int t = 0; t < g.fft; t++) {
			x[(size_t)(off + t)] = sym[(size_t)t];
		}
		std::vector<cf> grid((size_t)(12 * g.nrb));
		bool ok = wb_demod_symbol(x.data(), x.size(), 0, 3, g, plan, grid.data());
		float peak = 0.0f;
		int at = -1;
		for(int i = 0; i < 12 * g.nrb; i++) {
			const float p = std::norm(grid[(size_t)i]);
			if(p > peak) {
				peak = p;
				at = i;
			}
		}
		float rest = 0.0f;
		for(int i = 0; i < 12 * g.nrb; i++) {
			if(i != at) {
				rest += std::norm(grid[(size_t)i]);
			}
		}
		snprintf(d, sizeof(d), "planted %d, recovered %d, leakage %.1f dB", kc, at,
		         10.0 * std::log10((double)(rest + 1e-20f) / (double)(peak + 1e-20f)));
		check(ok && at == kc && rest < 0.001f * peak, "wb_demod_tone", d, verbose);
	}

	// --- channel estimation on a known flat channel: feed pilots through a constant gain
	// and check the estimate reproduces it everywhere, including the extrapolated edges.
	{
		const int nrb = 50, nc = 12 * nrb;
		std::vector<crs_ref> pil;
		wb_crs(274, nrb, 0, 0, 0, pil);
		const cf gain(0.7f, -0.3f);
		std::vector<cf> grid((size_t)nc, cf(0.0f, 0.0f));
		for(auto &p : pil) {
			grid[(size_t)p.k] = p.ref * gain;
		}
		std::vector<cf> h((size_t)nc);
		wb_chan_est(grid.data(), pil, nc, h.data());
		float worst = 0.0f;
		for(int i = 0; i < nc; i++) {
			worst = std::max(worst, std::abs(h[(size_t)i] - gain));
		}
		snprintf(d, sizeof(d), "flat channel reproduced, worst error %.2e", (double)worst);
		check(worst < 1e-5f, "wb_chan_flat", d, verbose);
	}

	// --- PBCH end to end, hardware-free. The transmit side below is written straight from
	// 36.211 6.6 and 6.3.4 (encode, scramble, QPSK, SFBC precode, map), NOT by inverting
	// the receiver, so it pins the two conventions that were wrong on air and invisible to
	// every self-consistency metric: the Alamouti second output (x1, not
	// -conj(x1)) and, through lte_coding, the concatenated bit collection. A flipped
	// combiner or a turbo-style collection makes this decode fail, nothing softer does.
	{
		const int nrb = 50, nc = 12 * nrb, c0 = 6 * nrb - 36, nid = 274;
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		// distinct per-port channels, deliberately far from unit gain and from each other
		const cf hport[4] = { cf(0.9f, 0.3f), cf(-0.5f, 0.8f), cf(0.2f, -0.85f),
		                      cf(-0.7f, -0.4f) };

		for(int truth : { 1, 2, 4 }) {
			for(int noisy = 0; noisy < 2; noisy++) {
				// MIB: 50 PRB (bw index 3), normal PHICH duration, resource 1, SFN 421
				const int sfn_true = 421, q_true = sfn_true & 3, sfn8 = sfn_true >> 2;
				std::vector<uint8_t> mib(24, 0);
				mib[0] = 0; mib[1] = 1; mib[2] = 1;     // dl-Bandwidth = 3 -> 50 PRB
				mib[3] = 0;                             // phich-Duration normal
				mib[4] = 1; mib[5] = 0;                 // phich-Resource '10' -> 1.0
				for(int i = 0; i < 8; i++) {
					mib[6 + i] = (uint8_t)((sfn8 >> (7 - i)) & 1);
				}
				std::vector<uint8_t> tx = mib;
				crc_attach(tx, CRC16);
				uint8_t mask[16];
				pbch_crc_mask(truth, mask);
				for(int i = 0; i < 16; i++) {
					tx[24 + i] ^= mask[i];
				}
				std::vector<uint8_t> cw, rm;
				conv_encode(tx, cw);
				conv_rate_match(cw, 1920, rm);
				std::vector<uint8_t> scr(1920);
				gold_seq((uint32_t)nid, 1920, scr.data());
				// this frame carries quarter q of the 40 ms block
				std::vector<cf> s(240);
				const float A = 0.70710678f;
				for(int i = 0; i < 240; i++) {
					const int j = q_true * 480 + 2 * i;
					const int b0 = rm[(size_t)j] ^ scr[(size_t)j];
					const int b1 = rm[(size_t)(j + 1)] ^ scr[(size_t)(j + 1)];
					s[(size_t)i] = cf(b0 ? -A : A, b1 ? -A : A);   // 36.211 Table 7.1.2-1
				}
				// per-port RE values: 6.3.4.2.2 for two ports; 6.3.4.3 for four, where the
				// quadruplet (x0..x3) puts SFBC(x0,x1) on ports {0,2} in REs 4i/4i+1 and
				// SFBC(x2,x3) on ports {1,3} in REs 4i+2/4i+3
				std::vector<cf> y[4];
				for(auto &v : y) {
					v.assign(240, cf(0.0f, 0.0f));
				}
				if(truth == 1) {
					y[0] = s;
				} else {
					for(int i = 0; i + 1 < 240; i += 2) {
						const int pa = (truth == 4 && ((i >> 1) & 1)) ? 1 : 0;
						const int pb = (truth == 4) ? pa + 2 : 1;
						y[pa][(size_t)i] = s[(size_t)i];
						y[pa][(size_t)(i + 1)] = s[(size_t)(i + 1)];
						y[pb][(size_t)i] = -std::conj(s[(size_t)(i + 1)]);
						y[pb][(size_t)(i + 1)] = std::conj(s[(size_t)i]);
					}
				}
				// map to the grid: same RE rule the standard gives the receiver - centre 72
				// subcarriers of symbols 0..3, minus the 4-port CRS reservation
				std::vector<cf> grid[4];
				for(auto &v : grid) {
					v.assign((size_t)nc, cf(0.0f, 0.0f));
				}
				int idx = 0;
				for(int l = 0; l < 4; l++) {
					std::vector<uint8_t> skip((size_t)nc, 0);
					if(l < 2) {
						for(int p = 2 * l; p < 2 * l + 2; p++) {
							std::vector<crs_ref> qq;
							wb_crs(nid, nrb, p, 1, l, qq);
							for(auto &z : qq) {
								skip[(size_t)z.k] = 1;
							}
						}
					}
					for(int k = c0; k < c0 + 72; k++) {
						if(skip[(size_t)k]) {
							continue;
						}
						cf v(0.0f, 0.0f);
						for(int p = 0; p < truth; p++) {
							v += hport[p] * y[p][(size_t)idx];
						}
						if(noisy) {
							v += cf(0.15f * gauss(rng), 0.15f * gauss(rng));
						}
						grid[l][(size_t)k] = v;
						idx++;
					}
				}
				// receiver, with the true channel handed in (estimation is pinned elsewhere)
				std::vector<cf> hs[4][4];
				for(int p = 0; p < 4; p++) {
					for(int l = 0; l < 4; l++) {
						hs[p][l].assign((size_t)nc, hport[p]);
					}
				}
				const cf *gp[4] = { grid[0].data(), grid[1].data(), grid[2].data(),
				                    grid[3].data() };
				std::vector<cf> re(240);
				static cf hout[4][240];
				const int n_re = wb_pbch_extract(gp, hs, nrb, nid, re.data(), hout);
				int ports = 0, bw = 0, sfn = -1, pd = -1;
				float pr = 0.0f;
				const bool ok = wb_mib_decode(re.data(), hout, n_re, nid, &ports, &bw,
				                              &sfn, &pd, &pr);
				snprintf(d, sizeof(d),
				         "%d ports%s: %d REs -> ports %d, %d PRB, SFN %d, phich %d/%.2f",
				         truth, noisy ? " + noise" : "", n_re, ports, bw, sfn, pd, (double)pr);
				check(ok && n_re == 240 && ports == truth && bw == 50 && sfn == sfn_true &&
				      pd == 0 && std::fabs(pr - 1.0f) < 1e-6f,
				      "wb_pbch_chain", d, verbose);
			}
		}
	}

	// --- PBCH TTI combining. A PBCH TTI is 40 ms: ONE payload, four redundancy versions,
	// one gold sequence segmented into four 480-bit quarters, each frame descrambled with
	// its own segment. For D = 40 every quarter is four copies of the same 120 mother-code
	// bits, so combining the four frames is pure energy accumulation - and it is the whole
	// difference between a cell that stops at pss_only and one that reports a MIB.
	//
	// Three properties are pinned here and only the first is about gain. The second is the
	// BOUND, and it is the one that matters most: the MIB payload changes at the 40 ms
	// boundary (the SFN MSBs are in it), so a group straddling that boundary sums two
	// different payloads - which does NOT fail loudly. It passes CRC on a word that was
	// never sent, with the SFN off by four frames, and a wrong SFN silently kills SIB1
	// (decode_cell picks its windows from it) on a cell whose MIB row looks perfect.
	{
		const int nid = 274, sfn_a = 421;
		const cf hport[4] = { cf(0.9f, 0.3f), cf(-0.5f, 0.8f), cf(0.2f, -0.85f),
		                      cf(-0.7f, -0.4f) };

		// 1. the generalised entry point IS the old one at nf = 1, q0 = -1: same verdict,
		//    same fields, across port counts, quarters and a working-to-hopeless SNR range
		{
			int cmp = 0, agree = 0;
			for(int ports : { 1, 2, 4 }) {
				for(int q = 0; q < 4; q++) {
					for(float snr : { 10.0f, 0.0f, -8.0f }) {
						std::mt19937 rng((unsigned)(20260819 + 17 * q + ports));
						std::vector<cf> re;
						static cf hout[4][240];
						pbch_frame(nid, ports, pbch_mib(sfn_a), q, snr, rng, hport, re,
						           hout);
						int p1 = -1, b1 = -1, s1 = -1, d1 = -1, p2 = -1, b2 = -1, s2 = -1,
						    d2 = -1;
						float r1 = 0.0f, r2 = 0.0f;
						const bool o1 = wb_mib_decode(re.data(), hout, 240, nid, &p1, &b1,
						                              &s1, &d1, &r1);
						const cf *rp = re.data();
						const cf (*hp)[240] = hout;
						const bool o2 = wb_mib_decode_n(&rp, &hp, 1, nid, -1, &p2, &b2,
						                                &s2, &d2, &r2, nullptr);
						cmp++;
						agree += (o1 == o2 && p1 == p2 && b1 == b2 && s1 == s2 &&
						          d1 == d2 && r1 == r2) ? 1 : 0;
					}
				}
			}
			snprintf(d, sizeof(d), "%d/%d single-frame calls identical", agree, cmp);
			check(agree == cmp, "wb_mib_decode_n_wraps", d, verbose);
		}

		// 2. the gain: at an SNR where one frame is hopeless, four combined decode. Both
		//    counts are of a CORRECT MIB - a wrong one would not be a success.
		{
			const int trials = 40;
			const float snr = -10.0f;
			int one = 0, four = 0, wrong = 0;
			for(int t = 0; t < trials; t++) {
				std::mt19937 rng((unsigned)(20260819 + t));
				const int sfn = 4 * (17 + t);   // TTI-aligned, so frame f carries quarter f
				std::vector<cf> re[4];
				static cf hout[4][4][240];
				const cf *rp[4];
				const cf (*hp[4])[240];
				for(int f = 0; f < 4; f++) {
					pbch_frame(nid, 2, pbch_mib(sfn), f, snr, rng, hport, re[f], hout[f]);
					rp[f] = re[f].data();
					hp[f] = hout[f];
				}
				int ports = 0, bw = 0, s = -1, pd = -1;
				float pr = 0.0f;
				if(wb_mib_decode(re[0].data(), hout[0], 240, nid, &ports, &bw, &s, &pd, &pr)) {
					one += (s == sfn && bw == 50 && ports == 2) ? 1 : 0;
					wrong += (s == sfn) ? 0 : 1;
				}
				if(wb_mib_decode_n(rp, hp, 4, nid, 0, &ports, &bw, &s, &pd, &pr, nullptr)) {
					four += (s == sfn && bw == 50 && ports == 2) ? 1 : 0;
					wrong += (s == sfn) ? 0 : 1;
				}
			}
			snprintf(d, sizeof(d), "%.0f dB: 1 frame %d/%d -> 4 combined %d/%d, %d wrong",
			         (double)snr, one, trials, four, trials, wrong);
			check(four >= 2 * one && 2 * four >= trials && wrong == 0, "wb_mib_tti_combine",
			      d, verbose);
		}

		// 3. the bound. Four frames that straddle the boundary - quarters 2,3 of one TTI
		//    and 0,1 of the next, two DIFFERENT payloads - must never produce a MIB, at any
		//    SNR, however clean. Split into the two groups the TTI really has, the same
		//    observations decode to the two correct and different SFNs.
		{
			const int trials = 24;
			int straddle = 0, lo = 0, hi = 0;
			for(int t = 0; t < trials; t++) {
				std::mt19937 rng((unsigned)(20260819 + t));
				const int sfn_lo = 4 * (100 + t) + 2;      // frame 0: quarter 2 of TTI k
				const int sfn_hi = 4 * (101 + t);          // frame 2: quarter 0 of TTI k+1
				std::vector<cf> re[4];
				static cf hout[4][4][240];
				const cf *rp[4];
				const cf (*hp[4])[240];
				for(int f = 0; f < 4; f++) {
					const int sfn = (f < 2) ? sfn_lo : sfn_hi;
					pbch_frame(nid, 2, pbch_mib(sfn), (2 + f) & 3, 12.0f, rng, hport,
					           re[f], hout[f]);
					rp[f] = re[f].data();
					hp[f] = hout[f];
				}
				int ports = 0, bw = 0, s = -1, pd = -1;
				float pr = 0.0f;
				straddle += wb_mib_decode_n(rp, hp, 4, nid, 2, &ports, &bw, &s, &pd, &pr,
				                            nullptr) ? 1 : 0;
				lo += (wb_mib_decode_n(rp, hp, 2, nid, 2, &ports, &bw, &s, &pd, &pr,
				                       nullptr) && s == sfn_lo) ? 1 : 0;
				hi += (wb_mib_decode_n(rp + 2, hp + 2, 2, nid, 0, &ports, &bw, &s, &pd, &pr,
				                       nullptr) && s == sfn_hi) ? 1 : 0;
			}
			snprintf(d, sizeof(d), "straddling groups reporting a MIB: %d/%d; split "
			         "correctly: %d/%d and %d/%d", straddle, trials, lo, trials, hi, trials);
			check(straddle == 0 && lo == trials && hi == trials, "wb_mib_tti_bounded", d,
			      verbose);
		}

		// 4. the SFN a group reports is the FIRST observation's, at every start quarter and
		//    every group length. decode_cell walks back from it to the window's frame 0, so
		//    an off-by-one here lands on the wrong SIB1 subframes and nothing says so.
		{
			int ok = 0, n = 0;
			for(int q0 = 0; q0 < 4; q0++) {
				for(int nf = 1; nf + q0 <= 4; nf++) {
					std::mt19937 rng((unsigned)(20260819 + 31 * q0 + nf));
					const int sfn8 = 77;
					std::vector<cf> re[4];
					static cf hout[4][4][240];
					const cf *rp[4];
					const cf (*hp[4])[240];
					for(int f = 0; f < nf; f++) {
						pbch_frame(nid, 4, pbch_mib(sfn8 * 4), (q0 + f) & 3, 6.0f, rng,
						           hport, re[f], hout[f]);
						rp[f] = re[f].data();
						hp[f] = hout[f];
					}
					int ports = 0, bw = 0, s = -1, pd = -1;
					float pr = 0.0f;
					n++;
					ok += (wb_mib_decode_n(rp, hp, nf, nid, q0, &ports, &bw, &s, &pd, &pr,
					                       nullptr) && s == sfn8 * 4 + q0) ? 1 : 0;
				}
			}
			snprintf(d, sizeof(d), "%d/%d groups reported the first frame's SFN", ok, n);
			check(ok == n, "wb_mib_group_sfn", d, verbose);
		}

		// 5. the TTI is not always four equal frames. wb_sfbc_combine zero-forces, so all
		//    four observations reach the accumulator with the SAME signal amplitude and
		//    their OWN noise level; summed unweighted, three frames far down bury the one
		//    that decodes on its own and the combined path scores WORSE than the scheme it
		//    replaced - measured 53/150 against 150/150 at 20 dB of spread, on cells this
		//    change exists to win. Weighted (MRC) the weak frames stop shouting, so
		//    combining can only add: at every spread, four >= the strong frame alone.
		{
			const int trials = 40;
			const float strong = 4.0f;
			bool never_worse = true, sensitive = false;
			int last_solo = 0, last_four = 0;
			for(float drop : { 6.0f, 12.0f, 20.0f }) {
				int solo = 0, four = 0;
				for(int t = 0; t < trials; t++) {
					std::mt19937 rng((unsigned)(20260820 + t + 101 * (int)drop));
					const int sfn = 4 * (33 + t);   // TTI-aligned: frame f carries quarter f
					std::vector<cf> re[4];
					static cf hout[4][4][240];
					const cf *rp[4];
					const cf (*hp[4])[240];
					for(int f = 0; f < 4; f++) {
						// the strong one is frame 1, i.e. neither first nor last: a
						// weighting that quietly followed position would pass otherwise
						pbch_frame(nid, 2, pbch_mib(sfn), f, f == 1 ? strong : strong - drop,
						           rng, hport, re[f], hout[f]);
						rp[f] = re[f].data();
						hp[f] = hout[f];
					}
					int ports = 0, bw = 0, s = -1, pd = -1;
					float pr = 0.0f;
					solo += (wb_mib_decode(re[1].data(), hout[1], 240, nid, &ports, &bw, &s,
					                       &pd, &pr) && s == sfn + 1) ? 1 : 0;
					four += (wb_mib_decode_n(rp, hp, 4, nid, 0, &ports, &bw, &s, &pd, &pr,
					                         nullptr) && s == sfn) ? 1 : 0;
				}
				never_worse = never_worse && four >= solo;
				sensitive = sensitive || solo >= trials / 2;   // the strong frame IS readable
				last_solo = solo;
				last_four = four;
			}
			snprintf(d, sizeof(d), "at 20 dB spread: strong frame alone %d/%d, four combined "
			         "%d/%d", last_solo, trials, last_four, trials);
			check(never_worse && sensitive, "wb_mib_tti_mrc", d, verbose);
		}

		// 6. the diagnostic number itself. path_metric used to be the MAXIMUM over every
		//    hypothesis of the window, which is an extreme-value statistic: it rose with the
		//    window on PURE NOISE (0.811 at 10 frames to 0.853 at 200) and barely moved with
		//    SNR, so an empty carrier scanned long outranked a real, nearly decodable cell -
		//    the number pointed the operator at empty spectrum. wb_mib_metric AVERAGES over
		//    the groups a hypothesis was tried on instead. Pinned here: the mean is flat in
		//    window length where the max it replaced is not, and a cell at the edge of
		//    decoding beats noise however long the noise was looked at.
		{
			const int groups = 40, few = 10;   // 160 frames against 40, both real windows
			wb_mib_stats sh, lo, cellst;       // the same noise read short and long, and a cell
			double sh_max = -1.0, lo_max = -1.0;
			for(int pass = 0; pass < 2; pass++) {
				// pass 0 = pure noise, a MIB 40 dB under it being no cell at all; pass 1 = a
				// real cell at the edge of decoding. Those are the two rows an operator has
				// to be able to tell apart, and the old number ranked them the wrong way up.
				const float snr = pass ? -12.0f : -40.0f;
				for(int gi = 0; gi < (pass ? few : groups); gi++) {
					std::mt19937 rng((unsigned)(20260820 + 7919 * gi + 13 * pass));
					const cf *rp[4];
					const cf (*hp[4])[240];
					std::vector<cf> rv[4];
					static cf hv[4][4][240];
					for(int f = 0; f < 4; f++) {
						pbch_frame(nid, 2, pbch_mib(4 * (55 + gi)), f, snr, rng, hport,
						           rv[f], hv[f]);
						rp[f] = rv[f].data();
						hp[f] = hv[f];
					}
					int ports = 0, bw = 0, s = -1, pd = -1;
					float pr = 0.0f;
					wb_mib_stats one;
					wb_mib_decode_n(rp, hp, 4, nid, 0, &ports, &bw, &s, &pd, &pr, &one);
					wb_mib_stats &acc = pass ? cellst : lo;
					acc.metric_sum += one.metric_sum;
					acc.metric_n += one.metric_n;
					if(pass == 0) {
						// what the maximum over hypotheses would have reported at each width
						const double m = wb_mib_metric(one);
						lo_max = m > lo_max ? m : lo_max;
						if(gi < few) {
							sh.metric_sum += one.metric_sum;
							sh.metric_n += one.metric_n;
							sh_max = m > sh_max ? m : sh_max;
						}
					}
				}
			}
			const double sh_mean = wb_mib_metric(sh), lo_mean = wb_mib_metric(lo);
			const double cell_mean = wb_mib_metric(cellst);
			snprintf(d, sizeof(d), "noise mean %.3f (%d groups) -> %.3f (%d), max %.3f -> "
			         "%.3f; cell at the edge %.3f", sh_mean, few, lo_mean, groups, sh_max,
			         lo_max, cell_mean);
			check(std::fabs(lo_mean - sh_mean) < 0.02 && lo_max > sh_max &&
			      cell_mean > lo_mean + 0.05, "wb_mib_metric_stable", d, verbose);
		}
	}

	// --- PCFICH REG anchors, pinned by hand against 36.211 6.7.4. Both reference cells:
	// NID 274, 50 PRB: kbar = 6*(274 mod 100) = 444; +150i mod 600 -> 444, 594, 144, 294.
	// NID 377, 50 PRB: kbar = 6*77 = 462 -> 462, 12, 162, 312.
	// Pinned as literals because the synthetic loop below shares wb_pcfich_regs between its
	// transmit and receive side and is therefore structurally blind to a wrong anchor
	// formula.
	{
		int kr[4];
		wb_pcfich_regs(274, 50, kr);
		snprintf(d, sizeof(d), "NID 274: %d %d %d %d", kr[0], kr[1], kr[2], kr[3]);
		check(kr[0] == 444 && kr[1] == 594 && kr[2] == 144 && kr[3] == 294,
		      "wb_pcfich_regs_274", d, verbose);
		wb_pcfich_regs(377, 50, kr);
		snprintf(d, sizeof(d), "NID 377: %d %d %d %d", kr[0], kr[1], kr[2], kr[3]);
		check(kr[0] == 462 && kr[1] == 12 && kr[2] == 162 && kr[3] == 312,
		      "wb_pcfich_regs_377", d, verbose);
	}

	// --- PHICH REG anchors, pinned by hand against 36.211 6.9.3 for BOTH reference cell
	// configs - there is no on-air oracle for PHICH until a PDCCH decodes around it, which
	// is exactly why these literals exist. Worked example, 796 cell (NID 377, 50 PRB,
	// Ng = 1/6, normal duration): ngroups = ceil(50/48) = 2; 96 available REGs after
	// removing PCFICH's j = {2,27,52,77}; group 0 indices (377, 377+32, 377+64) mod 96 =
	// 89/25/57 -> REG j 93/26/60 -> k 558/156/360. 816 cell (NID 274, Ng = 1): 7 groups,
	// group 0 -> (82,18,50) -> j 85/18/52 -> k 510/108/312, group 6 -> k 546/150/348.
	{
		std::vector<int> kr;
		int ng = wb_phich_regs(377, 50, 0, 1.0f / 6.0f, kr);
		snprintf(d, sizeof(d), "NID 377 Ng=1/6: %d groups, g0 %d %d %d, g1 %d %d %d", ng,
		         kr.size() > 5 ? kr[0] : -1, kr.size() > 5 ? kr[1] : -1,
		         kr.size() > 5 ? kr[2] : -1, kr.size() > 5 ? kr[3] : -1,
		         kr.size() > 5 ? kr[4] : -1, kr.size() > 5 ? kr[5] : -1);
		check(ng == 2 && kr.size() == 6 && kr[0] == 558 && kr[1] == 156 && kr[2] == 360 &&
		      kr[3] == 564 && kr[4] == 168 && kr[5] == 366,
		      "wb_phich_regs_377", d, verbose);
		ng = wb_phich_regs(274, 50, 0, 1.0f, kr);
		snprintf(d, sizeof(d), "NID 274 Ng=1: %d groups, g0 %d %d %d, g6 %d %d %d", ng,
		         kr.size() > 20 ? kr[0] : -1, kr.size() > 20 ? kr[1] : -1,
		         kr.size() > 20 ? kr[2] : -1, kr.size() > 20 ? kr[18] : -1,
		         kr.size() > 20 ? kr[19] : -1, kr.size() > 20 ? kr[20] : -1);
		check(ng == 7 && kr.size() == 21 && kr[0] == 510 && kr[1] == 108 && kr[2] == 312 &&
		      kr[18] == 546 && kr[19] == 150 && kr[20] == 348,
		      "wb_phich_regs_274", d, verbose);
		// no PHICH REG may ever collide with a PCFICH REG - structural, any config
		int pk[4];
		wb_pcfich_regs(274, 50, pk);
		bool collide = false;
		for(int k : kr) {
			for(int i = 0; i < 4; i++) {
				if(k == pk[i]) {
					collide = true;
				}
			}
		}
		check(!collide, "wb_phich_no_pcfich_collision", "PHICH avoids all four PCFICH REGs",
		      verbose);
	}

	// --- PCFICH end to end, hardware-free: transmit side written straight from 36.211
	// 6.7.1..6.7.4 (scramble, QPSK, SFBC precode, REG map, CRS for the channel estimator),
	// IFFT into time samples, production receiver. The scrambling cinit on the transmit
	// side is a HAND-COMPUTED literal - (floor(10/2)+1)(2*274+1)*2^9 + 274 = 1686802 - so
	// the receiver's formula is pinned against the norm, not against itself.
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		const int nc = 12 * g.nrb, nid = 274, ns = 10;   // subframe 5, where SIB1 lives
		const uint32_t cinit_pin = 1686802u;
		const cf hport[4] = { cf(0.9f, 0.3f), cf(-0.5f, 0.8f), cf(0.2f, -0.85f),
		                      cf(-0.7f, -0.4f) };
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		const float A = 0.70710678f;

		for(int truth : { 1, 2, 4 }) {
			for(int cfi = 1; cfi <= 3; cfi++) {
				const bool noisy = (cfi == 2);   // one noisy variant per port count
				uint8_t sc[32];
				gold_seq(cinit_pin, 32, sc);
				cf s[16];
				for(int i = 0; i < 16; i++) {
					const int b0 = CFI_CODEWORD[cfi - 1][2 * i] ^ sc[2 * i];
					const int b1 = CFI_CODEWORD[cfi - 1][2 * i + 1] ^ sc[2 * i + 1];
					s[i] = cf(b0 ? -A : A, b1 ? -A : A);
				}
				// per-port precoding, same 6.3.4 equations as the PBCH transmit side above
				cf y[4][16];
				for(auto &row : y) {
					for(auto &v : row) {
						v = cf(0.0f, 0.0f);
					}
				}
				if(truth == 1) {
					for(int i = 0; i < 16; i++) {
						y[0][i] = s[i];
					}
				} else {
					for(int i = 0; i + 1 < 16; i += 2) {
						const int pa = (truth == 4 && ((i >> 1) & 1)) ? 1 : 0;
						const int pb = (truth == 4) ? pa + 2 : 1;
						y[pa][i] = s[i];
						y[pa][i + 1] = s[i + 1];
						y[pb][i] = -std::conj(s[i + 1]);
						y[pb][i + 1] = std::conj(s[i]);
					}
				}
				// frequency grids of symbols 0 and 1: CRS of every active port plus the 16
				// PCFICH REs, all through the per-port channel
				std::vector<cf> f0((size_t)nc, cf(0.0f, 0.0f)), f1((size_t)nc, cf(0.0f, 0.0f));
				for(int p = 0; p < truth; p++) {
					std::vector<crs_ref> q;
					wb_crs(nid, g.nrb, p, ns, (p < 2) ? 0 : 1, q);
					for(auto &z : q) {
						((p < 2) ? f0 : f1)[(size_t)z.k] += hport[p] * z.ref;
					}
				}
				int kreg[4];
				wb_pcfich_regs(nid, g.nrb, kreg);
				const int vs = nid % 6;
				int m = 0;
				for(int i = 0; i < 4; i++) {
					for(int dd = 0; dd < 6; dd++) {
						if(dd == vs || dd == (vs + 3) % 6) {
							continue;
						}
						cf v(0.0f, 0.0f);
						for(int p = 0; p < truth; p++) {
							v += hport[p] * y[p][m];
						}
						f0[(size_t)(kreg[i] + dd)] += v;
						m++;
					}
				}
				// into time samples (IFFT = conj(FFT(conj())/N), CP prepended) so the loop
				// exercises the production demodulator too
				std::vector<cf> x((size_t)g.slot, cf(0.0f, 0.0f));
				const int half = 6 * g.nrb;
				for(int l = 0; l < 2; l++) {
					const std::vector<cf> &f = l ? f1 : f0;
					std::vector<cf> bins((size_t)g.fft, cf(0.0f, 0.0f));
					for(int i = 0; i < half; i++) {
						bins[(size_t)(g.fft - half + i)] = std::conj(f[(size_t)i]);
						bins[(size_t)(1 + i)] = std::conj(f[(size_t)(half + i)]);
					}
					std::vector<cf> t((size_t)g.fft);
					plan.run(bins.data(), t.data());
					for(auto &v : t) {
						v = std::conj(v) / (float)g.fft;
					}
					const int off = g.sym_off(l);
					for(int u = 0; u < g.cp_len(l); u++) {
						x[(size_t)(off - g.cp_len(l) + u)] = t[(size_t)(g.fft - g.cp_len(l) + u)];
					}
					for(int u = 0; u < g.fft; u++) {
						x[(size_t)(off + u)] = t[(size_t)u];
					}
				}
				if(noisy) {
					for(auto &v : x) {
						v += cf(0.02f * gauss(rng), 0.02f * gauss(rng));
					}
				}
				float mg = 0.0f;
				const int got = wb_pcfich_decode(x.data(), x.size(), 0, ns, nid, truth, g,
				                                 plan, &mg);
				snprintf(d, sizeof(d), "%d ports%s: CFI %d -> %d, margin %.3f", truth,
				         noisy ? " + noise" : "", cfi, got, (double)mg);
				check(got == cfi && mg > 0.2f, "wb_pcfich_chain", d, verbose);
			}
		}
	}

	// --- PDCCH REG map, pinned by hand against 36.211 6.2.4 + 6.8.5 for both reference
	// cells. The map is shared between the chain test's transmit side and the receiver, so
	// - lesson 2 - these literals are the only thing that can see a wrong loop order or a
	// wrong REG width. 796 cell (NID 377, 50 PRB, 2 ports, CFI 3, Ng 1/6): symbol 0 has
	// 100 REGs minus 4 PCFICH (462/12/162/312) minus 6 PHICH = 90, symbols 1 and 2 add 150
	// each (four-wide, no pilots at two ports) -> 390 REGs, 43 CCEs. Mapping order is
	// k-outer, l-inner: (0,0),(0,1),(0,2),(4,1),(4,2),(6,0),(8,1),(8,2) - and at k=12 the
	// symbol-0 REG belongs to PCFICH, so only (12,1),(12,2) appear. 816 cell (NID 274,
	// 50 PRB, 4 ports, CFI 2, Ng 1): symbol 1 carries ports 2/3 pilots and is six-wide ->
	// 75 + 100 = 175 REGs, 19 CCEs; the first PHICH anchor 108 leaves only (108,1).
	{
		std::vector<wb_reg> r;
		int m = wb_pdcch_regs(377, 50, 3, 2, 0, 1.0f / 6.0f, r);
		static const int want377[13][2] = {
			{ 0, 0 }, { 0, 1 }, { 0, 2 }, { 4, 1 }, { 4, 2 }, { 6, 0 }, { 8, 1 },
			{ 8, 2 }, { 12, 1 }, { 12, 2 }, { 16, 1 }, { 16, 2 }, { 18, 0 },
		};
		bool ok = m == 390;
		for(int i = 0; ok && i < 13; i++) {
			ok = r[(size_t)i].k[0] == want377[i][0] && r[(size_t)i].l == want377[i][1];
		}
		// data REs of the first symbol-0 REG: vshift = 377 % 6 = 5 skips offsets 5 and 2
		ok = ok && r[0].k[0] == 0 && r[0].k[1] == 1 && r[0].k[2] == 3 && r[0].k[3] == 4;
		ok = ok && r[1].k[0] == 0 && r[1].k[3] == 3;   // symbol 1 at two ports: 4 consecutive
		snprintf(d, sizeof(d), "NID 377 CFI 3: %d REGs (%d CCEs), first 13 anchors%s", m,
		         m / 9, ok ? " ok" : " WRONG");
		check(ok, "wb_pdcch_regs_377", d, verbose);

		m = wb_pdcch_regs(274, 50, 2, 4, 0, 1.0f, r);
		ok = m == 175 && m / 9 == 19;
		// vshift = 274 % 6 = 4 skips offsets 4 and 1 in every six-wide REG
		ok = ok && r[0].l == 0 && r[0].k[0] == 0 && r[0].k[1] == 2 && r[0].k[2] == 3 &&
		     r[0].k[3] == 5;
		ok = ok && r[1].l == 1 && r[1].k[0] == 0;      // symbol 1 at four ports: six-wide
		// 18 subcarrier anchors below 108 give 36 REGs; PHICH holds (108,0) and (114,0)
		ok = ok && r[36].l == 1 && r[36].k[0] == 108 && r[37].l == 1 && r[37].k[0] == 114;
		snprintf(d, sizeof(d), "NID 274 CFI 2: %d REGs (%d CCEs), PHICH holes%s", m, m / 9,
		         ok ? " ok" : " WRONG");
		check(ok, "wb_pdcch_regs_274", d, verbose);
	}

	// --- the quadruplet permutation, pinned by a hand-computed 36.212 5.1.4.2.1 vector.
	// M = 10: R = 1, 22 <NULL>s in front, so only columns >= 22 emit, in permuted column
	// order 25,29,27,23,31,24,28,26,22,30 -> indices 3,7,5,1,9,2,6,4,0,8.
	{
		std::vector<int> P;
		wb_pdcch_perm(10, P);
		static const int want[10] = { 3, 7, 5, 1, 9, 2, 6, 4, 0, 8 };
		bool ok = P.size() == 10;
		for(int i = 0; ok && i < 10; i++) {
			ok = P[(size_t)i] == want[i];
		}
		check(ok, "wb_pdcch_perm_pinned", "M=10 matches the hand-derived read order", verbose);
		// and at a real size it must stay a bijection
		wb_pdcch_perm(390, P);
		std::vector<uint8_t> seen(390, 0);
		ok = P.size() == 390;
		for(int v : P) {
			if(v < 0 || v >= 390 || seen[(size_t)v]) {
				ok = false;
				break;
			}
			seen[(size_t)v] = 1;
		}
		check(ok, "wb_pdcch_perm_bijection", "M=390 is a permutation", verbose);
	}

	// --- PDCCH end to end, hardware-free: transmit side written straight from 36.211
	// 6.8.2..6.8.5 (multiplex with random neighbours, scramble with a HAND-COMPUTED cinit
	// literal, QPSK, SFBC precode per quadruplet, interleave + cyclic shift, REG map)
	// through per-port channels and the production receiver chain including its own CRS
	// channel estimation. Pins the scrambling offset (a candidate's bits sit at 72*CCE of
	// the whole block), the shift direction and the quadruplet-aligned Alamouti pairing.
	// cinit literals: subframe 5, NID 377 -> 5*512+377 = 2937; NID 274 -> 2834;
	// subframe 0, NID 377 -> 377.
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		const int nc = 12 * g.nrb;
		const cf hport[4] = { cf(0.9f, 0.3f), cf(-0.5f, 0.8f), cf(0.2f, -0.85f),
		                      cf(-0.7f, -0.4f) };
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		const float A = 0.70710678f;

		struct pdcch_case {
			int ports, cfi, nid, ns;
			float ng;
			bool is1c;
			int L, cce;
			uint32_t cinit;   // hand literal, NOT the receiver's formula
			bool noisy;
		};
		const pdcch_case cases[] = {
			{ 2, 3, 377, 10, 1.0f / 6.0f, false, 4, 0, 2937u, false },
			{ 4, 2, 274, 10, 1.0f, false, 8, 8, 2834u, true },
			{ 1, 3, 377, 0, 1.0f / 6.0f, true, 8, 0, 377u, false },
			{ 2, 3, 377, 10, 1.0f / 6.0f, true, 4, 12, 2937u, false },
		};
		for(const auto &tc : cases) {
			// the DCI payload: random, but a 1A-sized one must carry the format flag
			const int Abits = tc.is1c ? dci_size_1c(g.nrb) : dci_size_1a(g.nrb);
			std::vector<uint8_t> pay((size_t)Abits);
			for(auto &b : pay) {
				b = (uint8_t)(rng() & 1);
			}
			if(!tc.is1c) {
				pay[0] = 1;
			}
			std::vector<uint8_t> tx = pay;
			crc_attach(tx, CRC16);
			uint8_t simask[16];
			rnti_crc_mask(SI_RNTI, simask);
			for(int i = 0; i < 16; i++) {
				tx[(size_t)(Abits + i)] ^= simask[i];
			}
			std::vector<uint8_t> cw, rm;
			conv_encode(tx, cw);
			conv_rate_match(cw, (size_t)(72 * tc.L), rm);

			std::vector<wb_reg> regs;
			const int M = wb_pdcch_regs(tc.nid, g.nrb, tc.cfi, tc.ports, 0, tc.ng, regs);
			// whole multiplexed block: other CCEs carry random "neighbour" bits, because a
			// receiver that cannot decode beside foreign PDCCHs is useless on air
			std::vector<uint8_t> blk((size_t)(8 * M));
			for(auto &b : blk) {
				b = (uint8_t)(rng() & 1);
			}
			for(int i = 0; i < 72 * tc.L; i++) {
				blk[(size_t)(72 * tc.cce + i)] = rm[(size_t)i];
			}
			std::vector<uint8_t> sc((size_t)(8 * M));
			gold_seq(tc.cinit, 8 * M, sc.data());
			std::vector<cf> zs((size_t)(4 * M));
			for(int i = 0; i < 4 * M; i++) {
				const int b0 = blk[(size_t)(2 * i)] ^ sc[(size_t)(2 * i)];
				const int b1 = blk[(size_t)(2 * i + 1)] ^ sc[(size_t)(2 * i + 1)];
				zs[(size_t)i] = cf(b0 ? -A : A, b1 ? -A : A);
			}
			// interleaver written out literally: fill row-wise with <NULL>s in front, read
			// permuted columns - then the cyclic shift places w((m'+NID) mod M) on REG m'
			std::vector<int> Ptx;
			{
				const int C = 32, R = (M + C - 1) / C, ND = R * C - M;
				for(int cc = 0; cc < C; cc++) {
					const int col = SUBBLOCK_PERM[cc];
					for(int r = 0; r < R; r++) {
						const int src = r * C + col - ND;
						if(src >= 0) {
							Ptx.push_back(src);
						}
					}
				}
			}
			// grids: control symbols 0..2 plus the two pilot anchors the estimator needs
			std::vector<cf> f[3], f4, f8;
			for(auto &v : f) {
				v.assign((size_t)nc, cf(0.0f, 0.0f));
			}
			f4.assign((size_t)nc, cf(0.0f, 0.0f));
			f8.assign((size_t)nc, cf(0.0f, 0.0f));
			for(int m = 0; m < M; m++) {
				const int zi = Ptx[(size_t)((m + tc.nid) % M)];
				const cf *q = &zs[(size_t)(4 * zi)];
				cf y[4][4];
				for(auto &row : y) {
					for(auto &v : row) {
						v = cf(0.0f, 0.0f);
					}
				}
				if(tc.ports == 1) {
					for(int j = 0; j < 4; j++) {
						y[0][j] = q[j];
					}
				} else {
					for(int j = 0; j < 4; j += 2) {
						const int pa = (tc.ports == 4 && ((j >> 1) & 1)) ? 1 : 0;
						const int pb = (tc.ports == 4) ? pa + 2 : 1;
						y[pa][j] = q[j];
						y[pa][j + 1] = q[j + 1];
						y[pb][j] = -std::conj(q[j + 1]);
						y[pb][j + 1] = std::conj(q[j]);
					}
				}
				for(int j = 0; j < 4; j++) {
					cf v(0.0f, 0.0f);
					for(int p = 0; p < tc.ports; p++) {
						v += hport[p] * y[p][j];
					}
					f[regs[(size_t)m].l][(size_t)regs[(size_t)m].k[j]] += v;
				}
			}
			// CRS of every active port through its channel, in all anchor symbols
			for(int p = 0; p < tc.ports; p++) {
				std::vector<crs_ref> q;
				if(p < 2) {
					wb_crs(tc.nid, g.nrb, p, tc.ns, 0, q);
					for(auto &zz : q) {
						f[0][(size_t)zz.k] += hport[p] * zz.ref;
					}
					wb_crs(tc.nid, g.nrb, p, tc.ns, 4, q);
					for(auto &zz : q) {
						f4[(size_t)zz.k] += hport[p] * zz.ref;
					}
				} else {
					wb_crs(tc.nid, g.nrb, p, tc.ns, 1, q);
					for(auto &zz : q) {
						f[1][(size_t)zz.k] += hport[p] * zz.ref;
					}
					wb_crs(tc.nid, g.nrb, p, tc.ns + 1, 1, q);
					for(auto &zz : q) {
						f8[(size_t)zz.k] += hport[p] * zz.ref;
					}
				}
			}
			// two slots of time samples (IFFT = conj(FFT(conj()))/N with CP)
			std::vector<cf> x((size_t)(2 * g.slot), cf(0.0f, 0.0f));
			const int half = 6 * g.nrb;
			auto synth = [&](const std::vector<cf> &fq, int l, int64_t slot_off) {
				std::vector<cf> bins((size_t)g.fft, cf(0.0f, 0.0f));
				for(int i = 0; i < half; i++) {
					bins[(size_t)(g.fft - half + i)] = std::conj(fq[(size_t)i]);
					bins[(size_t)(1 + i)] = std::conj(fq[(size_t)(half + i)]);
				}
				std::vector<cf> t((size_t)g.fft);
				plan.run(bins.data(), t.data());
				for(auto &v : t) {
					v = std::conj(v) / (float)g.fft;
				}
				const int64_t off = slot_off + g.sym_off(l);
				for(int u = 0; u < g.cp_len(l); u++) {
					x[(size_t)(off - g.cp_len(l) + u)] = t[(size_t)(g.fft - g.cp_len(l) + u)];
				}
				for(int u = 0; u < g.fft; u++) {
					x[(size_t)(off + u)] = t[(size_t)u];
				}
			};
			for(int l = 0; l < 3; l++) {
				synth(f[l], l, 0);
			}
			synth(f4, 4, 0);
			if(tc.ports == 4) {
				synth(f8, 1, g.slot);
			}
			if(tc.noisy) {
				for(auto &v : x) {
					v += cf(0.02f * gauss(rng), 0.02f * gauss(rng));
				}
			}
			std::vector<wb_dci> hits;
			wb_pdcch_css_decode(x.data(), x.size(), 0, tc.ns, tc.nid, tc.ports, tc.cfi, 0,
			                    tc.ng, g, plan, hits);
			bool got = false;
			for(const auto &hh : hits) {
				if(hh.size == Abits && hh.L == tc.L && hh.cce == tc.cce) {
					bool same = true;
					for(int i = 0; i < Abits; i++) {
						same = same && hh.bits[i] == pay[(size_t)i];
					}
					got = got || same;
				}
			}
			snprintf(d, sizeof(d), "%d ports CFI %d %s L%d@%d%s: %zu hits, planted %s",
			         tc.ports, tc.cfi, tc.is1c ? "1C" : "1A", tc.L, tc.cce,
			         tc.noisy ? " + noise" : "", hits.size(), got ? "found" : "MISSING");
			check(got, "wb_pdcch_chain", d, verbose);

			// control: one wrong-NID run must find nothing (wrong REGs, shift, scrambling)
			if(&tc == &cases[0]) {
				std::vector<wb_dci> none;
				wb_pdcch_css_decode(x.data(), x.size(), 0, tc.ns, tc.nid + 1, tc.ports,
				                    tc.cfi, 0, tc.ng, g, plan, none);
				snprintf(d, sizeof(d), "NID+1 finds %zu DCIs", none.size());
				check(none.empty(), "wb_pdcch_wrong_nid", d, verbose);
			}
		}
	}

	// --- PDSCH RE budget, pinned by hand against 36.211 6.3.5 for both reference cells'
	// SIB1 grants (subframe 5, CFI 3, symbols 3..13). 796 cell (2 ports, RB 0+4): eight
	// plain symbols x 48 REs + three CRS symbols (t = 4,7,11) x 32 = 480 REs -> E = 960.
	// 816 cell (4 ports, RB 0+10): seven plain x 120 + t = 4,7,11 x 80 + t = 8 (ports 2/3
	// pilots) x 80 = 1160 REs -> E = 2320. A centre allocation (RB 22+6, 1 port, CFI 1)
	// additionally loses ALL of t = 5,6 to PSS/SSS and keeps 10 REs/RB in the port-0
	// pilot symbols: 8 x 72 + 3 x 60 = 756 -> E = 1512. Zero samples suffice - the budget
	// is a pure geometry question and must not depend on the signal.
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		std::vector<cf> x((size_t)g.subframe * 2, cf(0.0f, 0.0f));
		std::vector<float> llr;
		const int e1 = wb_pdsch_cw_llr(x.data(), x.size(), 0, 10, 377, 2, 3, 0, 4, 0xFFFF,
		                               g, plan, llr);
		const int e2 = wb_pdsch_cw_llr(x.data(), x.size(), 0, 10, 274, 4, 3, 0, 10, 0xFFFF,
		                               g, plan, llr);
		const int e3 = wb_pdsch_cw_llr(x.data(), x.size(), 0, 10, 377, 1, 1, 22, 6, 0xFFFF,
		                               g, plan, llr);
		snprintf(d, sizeof(d), "796-shape %d (want 960), 816-shape %d (want 2320), "
		         "centre/1-port %d (want 1512)", e1, e2, e3);
		check(e1 == 960 && e2 == 2320 && e3 == 1512, "wb_pdsch_re_budget", d, verbose);
	}

	// --- PDSCH end to end, hardware-free: TB + CRC-24A + turbo + rate matching, scrambled
	// with a HAND-COMPUTED cinit literal (0xFFFF*2^14 + 5*2^9 + NID: 377 -> 1073728377,
	// 274 -> 1073728274), QPSK, SFBC precode, 6.3.5 mapping, CRS for the estimator, IFFT,
	// then the production receiver and decoder. The two on-air grant shapes plus the
	// centre/1-port case, one noisy; a wrong-NID control must fail the CRC.
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		const int nc = 12 * g.nrb, ns = 10, c0 = 6 * g.nrb - 36;
		const cf hport[4] = { cf(0.9f, 0.3f), cf(-0.5f, 0.8f), cf(0.2f, -0.85f),
		                      cf(-0.7f, -0.4f) };
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		const float A = 0.70710678f;

		struct pdsch_case {
			int ports, cfi, nid, rb0, nrba;
			size_t tbs, E;
			uint32_t cinit;
			bool noisy;
		};
		const pdsch_case cases[] = {
			{ 2, 3, 377, 0, 4, 224, 960, 1073728377u, false },
			{ 4, 3, 274, 0, 10, 208, 2320, 1073728274u, true },
			{ 1, 1, 377, 22, 6, 208, 1512, 1073728377u, false },
		};
		for(const auto &tc : cases) {
			std::vector<uint8_t> tb(tc.tbs);
			for(auto &b : tb) {
				b = (uint8_t)(rng() & 1);
			}
			std::vector<uint8_t> blk = tb;
			crc_attach(blk, CRC24A);
			std::vector<uint8_t> dd[3];
			turbo_encode(blk, dd);
			std::vector<uint8_t> rm;
			turbo_rate_match(dd, blk.size(), tc.E, 0, rm);
			std::vector<uint8_t> sc(tc.E);
			gold_seq(tc.cinit, tc.E, sc.data());
			const size_t nsym = tc.E / 2;
			std::vector<cf> s(nsym);
			for(size_t i = 0; i < nsym; i++) {
				const int b0 = rm[2 * i] ^ sc[2 * i];
				const int b1 = rm[2 * i + 1] ^ sc[2 * i + 1];
				s[i] = cf(b0 ? -A : A, b1 ? -A : A);
			}
			// SFBC(+FSTD) precode, the same 6.3.4 equations as the PBCH/PCFICH TX sides
			std::vector<cf> y[4];
			for(auto &v : y) {
				v.assign(nsym, cf(0.0f, 0.0f));
			}
			if(tc.ports == 1) {
				y[0] = s;
			} else {
				for(size_t i = 0; i + 1 < nsym; i += 2) {
					const int pa = (tc.ports == 4 && ((i >> 1) & 1)) ? 1 : 0;
					const int pb = (tc.ports == 4) ? pa + 2 : 1;
					y[(size_t)pa][i] = s[i];
					y[(size_t)pa][i + 1] = s[i + 1];
					y[(size_t)pb][i] = -std::conj(s[i + 1]);
					y[(size_t)pb][i + 1] = std::conj(s[i]);
				}
			}
			// 6.3.5 mapping walk, written out on the TX side: symbols cfi..13, k ascending,
			// CRS combs of the configured ports and the sync centre excluded
			std::vector<cf> f[14];
			for(auto &v : f) {
				v.assign((size_t)nc, cf(0.0f, 0.0f));
			}
			const int vsh = tc.nid % 6;
			size_t idx = 0;
			for(int t = tc.cfi; t < 14; t++) {
				const int l = t % 7, sns = ns + t / 7;
				bool resv[6] = { false, false, false, false, false, false };
				for(int p = 0; p < tc.ports; p++) {
					int ls[3];
					const int nl = wb_crs_symbols(p, ls);
					for(int j = 0; j < nl; j++) {
						if(ls[j] == l) {
							resv[(crs_v(p, sns, l) + vsh) % 6] = true;
						}
					}
				}
				for(int k = 12 * tc.rb0; k < 12 * (tc.rb0 + tc.nrba); k++) {
					if(resv[k % 6]) {
						continue;
					}
					if(t / 7 == 0 && (l == 5 || l == 6) && k >= c0 && k < c0 + 72) {
						continue;
					}
					cf v(0.0f, 0.0f);
					for(int p = 0; p < tc.ports; p++) {
						v += hport[p] * y[(size_t)p][idx];
					}
					f[t][(size_t)k] += v;
					idx++;
				}
			}
			// CRS of every configured port in all its pilot symbols of this subframe
			for(int p = 0; p < tc.ports; p++) {
				static const int t01[4] = { 0, 4, 7, 11 };
				static const int t23[2] = { 1, 8 };
				const int *ta = (p < 2) ? t01 : t23;
				const int nt = (p < 2) ? 4 : 2;
				for(int i = 0; i < nt; i++) {
					std::vector<crs_ref> q;
					wb_crs(tc.nid, g.nrb, p, ns + ta[i] / 7, ta[i] % 7, q);
					for(auto &z : q) {
						f[ta[i]][(size_t)z.k] += hport[p] * z.ref;
					}
				}
			}
			// one subframe of time samples (IFFT = conj(FFT(conj()))/N with CP)
			std::vector<cf> x((size_t)(2 * g.slot), cf(0.0f, 0.0f));
			const int half = 6 * g.nrb;
			for(int t = 0; t < 14; t++) {
				std::vector<cf> bins((size_t)g.fft, cf(0.0f, 0.0f));
				for(int i = 0; i < half; i++) {
					bins[(size_t)(g.fft - half + i)] = std::conj(f[t][(size_t)i]);
					bins[(size_t)(1 + i)] = std::conj(f[t][(size_t)(half + i)]);
				}
				std::vector<cf> tt((size_t)g.fft);
				plan.run(bins.data(), tt.data());
				for(auto &v : tt) {
					v = std::conj(v) / (float)g.fft;
				}
				const int l = t % 7;
				const int64_t off = (int64_t)(t / 7) * g.slot + g.sym_off(l);
				for(int u = 0; u < g.cp_len(l); u++) {
					x[(size_t)(off - g.cp_len(l) + u)] = tt[(size_t)(g.fft - g.cp_len(l) + u)];
				}
				for(int u = 0; u < g.fft; u++) {
					x[(size_t)(off + u)] = tt[(size_t)u];
				}
			}
			// noise is added in TIME, where the unnormalised 1024-FFT amplifies it by
			// sqrt(N) = 32: amplitude 0.02 puts each RE near 0 dB - the same convention as
			// the PDCCH chain test. 0.05 would be -7 dB, below capacity for this E/K.
			if(tc.noisy) {
				for(auto &v : x) {
					v += cf(0.02f * gauss(rng), 0.02f * gauss(rng));
				}
			}
			std::vector<float> llr;
			const int E = wb_pdsch_cw_llr(x.data(), x.size(), 0, ns, tc.nid, tc.ports,
			                              tc.cfi, tc.rb0, tc.nrba, 0xFFFF, g, plan, llr);
			bool ok = E == (int)tc.E && idx == nsym;
			std::vector<uint8_t> dec;
			if(ok) {
				std::vector<float> soft;
				turbo_rate_dematch(llr, blk.size(), 0, soft);
				turbo_decode(soft, blk.size(), 8, dec);
				ok = crc_check(dec.data(), dec.size(), CRC24A) &&
				     std::equal(tb.begin(), tb.end(), dec.begin());
			}
			snprintf(d, sizeof(d), "%d ports CFI %d RB %d+%d tbs %zu%s: E %d, TB %s",
			         tc.ports, tc.cfi, tc.rb0, tc.nrba, tc.tbs, tc.noisy ? " + noise" : "",
			         E, ok ? "exact" : "WRONG");
			check(ok, "wb_pdsch_chain", d, verbose);

			// control: a wrong NID must fail the CRC (wrong combs, wrong scrambler, wrong
			// channel estimate) - if it ever passes, the receiver is not using the cell ID
			if(&tc == &cases[0]) {
				std::vector<float> llr2, soft2;
				std::vector<uint8_t> dec2;
				const int E2 = wb_pdsch_cw_llr(x.data(), x.size(), 0, ns, tc.nid + 3,
				                               tc.ports, tc.cfi, tc.rb0, tc.nrba, 0xFFFF, g,
				                               plan, llr2);
				bool bad = false;
				if(E2 > 0) {
					turbo_rate_dematch(llr2, blk.size(), 0, soft2);
					turbo_decode(soft2, blk.size(), 8, dec2);
					bad = crc_check(dec2.data(), dec2.size(), CRC24A);
				}
				check(!bad, "wb_pdsch_wrong_nid", "NID+3 fails the transport CRC", verbose);
			}
		}
	}

	// --- SIB1 decode MARGIN, which is a different question from correctness and the one
	// the 796 MHz cell keeps failing on air (57/57 CRC failures at PCI 439, while the
	// 816/2145/2665 MHz cells decode). Everything above pins that the chain is RIGHT; this
	// pins WHAT DECIDES whether one occasion is enough, because the obvious suspect is the
	// wrong one:
	//
	//   the cells that decode have four antenna ports and the cell that does not has two,
	//   so the port count looks like the discriminator - and it is not. What differs is the
	//   GRANT the two eNBs choose. 816 sends RB 0+10 at i_tbs 4 (TBS 208, K 232, E 2320,
	//   rate 0.10); 796 sends RB 0+4 at i_tbs 7 (TBS 224, K 248, E 960, rate 0.26). That is
	//   ~4 dB of coding gain, it belongs to the DCI and not to the antenna array, and the
	//   two are independent - which is why the crossed pair below is part of the test.
	//
	// So the shape is the margin and the accumulation is what buys it back: // already records that the 796 capture only ever decoded on 2+ RVs combined per 80 ms
	// window. Below the crossover a single occasion is hopeless at ANY port count, and four
	// accumulated occasions decode at BOTH.
	{
		const sib1_shape S796{ 2, 439, 3, 0, 4, 224 };    // 796 MHz grant, 2 ports
		const sib1_shape X796{ 4, 439, 3, 0, 4, 224 };    // same grant, 4 ports
		const sib1_shape S816{ 4, 274, 3, 0, 10, 208 };   // 816 MHz grant, 4 ports
		const sib1_shape X816{ 2, 274, 3, 0, 10, 208 };   // same grant, 2 ports
		static const int RVCYC[4] = { 0, 2, 3, 1 };       // 36.321 BCCH cycle over a window
		const int trials = 32;
		const float snr = 0.0f;
		int one[4] = { 0, 0, 0, 0 }, four[4] = { 0, 0, 0, 0 };
		bool budget_ok = true;
		const sib1_shape *shapes[4] = { &S796, &X796, &S816, &X816 };
		for(int si = 0; si < 4; si++) {
			for(int t = 0; t < trials; t++) {
				std::mt19937 rng((unsigned)(20260819 + t));
				std::vector<uint8_t> tb(shapes[si]->tbs);
				for(auto &b : tb) {
					b = (uint8_t)(rng() & 1);
				}
				std::vector<uint8_t> blk = tb;
				crc_attach(blk, CRC24A);
				std::vector<float> soft;
				for(int j = 0; j < 4; j++) {
					if(!sib1_occasion(*shapes[si], RVCYC[j], snr, rng, blk, soft)) {
						budget_ok = false;
						break;
					}
					std::vector<uint8_t> dec;
					turbo_decode(soft, blk.size(), 8, dec);
					const bool ok = crc_check(dec.data(), dec.size(), CRC24A) &&
					                std::equal(tb.begin(), tb.end(), dec.begin());
					if(ok && j == 0) {
						one[si]++;
					}
					if(ok && j == 3) {
						four[si]++;
					}
				}
			}
		}
		check(budget_ok, "sib1_margin_budget", "every synthetic occasion hit its RE budget",
		      verbose);
		// 1. the grant shape decides. Same port count, both grants: 816's low-rate grant
		//    must decode from ONE occasion far more often than 796's.
		snprintf(d, sizeof(d), "1 occasion at %.0f dB: 796-grant %d/%d (rate 0.26), "
		         "816-grant %d/%d (rate 0.10)", (double)snr, one[1], trials, one[2], trials);
		check(one[2] >= 2 * one[0] && one[2] >= 2 * one[1], "sib1_margin_shape", d, verbose);
		// 2. accumulation is what buys the tight grant back - the 796 capture's whole story
		snprintf(d, sizeof(d), "796-grant: 1 occasion %d/%d -> 4 accumulated %d/%d",
		         one[0], trials, four[0], trials);
		check(four[0] >= 2 * one[0] && four[0] * 2 >= trials, "sib1_margin_accum", d,
		      verbose);
		// 3. the port count is NOT the discriminator: at a FIXED grant, 2 and 4 ports land
		//    within a quarter of the trials of each other, in both directions. If this ever
		//    fails, the 2-port transmit-diversity path really has broken and suspects 1/2
		//    of the on-air investigation come back on the table.
		const int dtight = std::abs(one[0] - one[1]), dwide = std::abs(one[2] - one[3]);
		snprintf(d, sizeof(d), "1 occasion, 796-grant 2p %d vs 4p %d; 816-grant 4p %d vs "
		         "2p %d", one[0], one[1], one[2], one[3]);
		check(4 * dtight <= trials && 4 * dwide <= trials, "sib1_margin_ports_irrelevant", d,
		      verbose);
		// 4. and the accumulation does NOT need four distinct RVs: the transport block is
		//    byte-identical in every 80 ms period of a modification period and the RV cycle
		//    repeats with it, so the SAME redundancy version out of four periods is 6 dB of
		//    repetition gain into the same circular buffer - as good as the four distinct
		//    RVs above. This is what lets a cell whose PDCCH only lands once per period
		//    decode at all, and it is why the receiver accumulates over the whole burst
		//    rather than clearing the buffer at the 80 ms boundary.
		{
			int same_rv = 0;
			for(int t = 0; t < trials; t++) {
				std::mt19937 rng((unsigned)(20260819 + t));
				std::vector<uint8_t> tb(S796.tbs);
				for(auto &b : tb) {
					b = (uint8_t)(rng() & 1);
				}
				std::vector<uint8_t> blk = tb;
				crc_attach(blk, CRC24A);
				std::vector<float> soft;
				for(int p = 0; p < 4; p++) {
					sib1_occasion(S796, RVCYC[0], snr, rng, blk, soft);
				}
				std::vector<uint8_t> dec;
				turbo_decode(soft, blk.size(), 8, dec);
				same_rv += (crc_check(dec.data(), dec.size(), CRC24A) &&
				            std::equal(tb.begin(), tb.end(), dec.begin())) ? 1 : 0;
			}
			snprintf(d, sizeof(d), "796-grant RV0 out of 4 periods: %d/%d vs 1 occasion "
			         "%d/%d", same_rv, trials, one[0], trials);
			check(same_rv >= 2 * one[0] && 2 * same_rv >= trials,
			      "sib1_margin_same_rv_across_periods", d, verbose);
		}
	}

	// --- PSS tracking. What has to be proven without hardware is not that a correlator
	// finds a peak - it is that the PREDICTION is right and that the refusal is honest:
	// a tracker which slowly walks off the cell and keeps reporting "found" is worse than
	// no tracker, because everything downstream then decodes the wrong subframe with full
	// confidence. So every check below is either "did it land on the truth" or "did it say
	// no when the truth was not in the window".
	{
		wb_num g = wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		std::mt19937 rng(20260808);
		std::normal_distribution<float> gauss(0.0f, 1.0f);
		const int NID2 = 1;                    // the 816 MHz reference cell's root
		const double F = (double)g.frame;      // 153600 samples = 10 ms
		const double P0 = 12345.678;           // truth: PSS body start, absolute samples

		// The PSS waveform, planted at a REAL offset so a burst can carry a sub-sample
		// position. Built by an explicit inverse DFT out of pss_freq(), deliberately NOT
		// through the receiver's own template helper: which sample the reported position
		// names is exactly the thing under test, and a signal synthesised by the code under
		// test agrees with it by construction. Evaluating the same closed form at negative
		// offsets is also what makes the cyclic prefix exact - no copying required.
		// The per-subcarrier phase is stepped by a recurrence rather than a cos/sin per
		// sample; that is arithmetic, not convention, and it keeps the suite fast.
		std::vector<cf> dfr = pss_freq(NID2);
		auto plant = [&](std::vector<cf> &x, double p, float amp) {
			const int64_t q0 = (int64_t)std::ceil(p) - g.cp, q1 = (int64_t)p + g.fft;
			const int64_t lo = std::max<int64_t>(q0, 0), hi = std::min<int64_t>(q1, (int64_t)x.size());
			if(hi <= lo) {
				return;
			}
			const double sc = (double)amp / std::sqrt(62.0);   // PSS at unit rms per sample
			for(int i = 0; i < 62; i++) {
				const int kk = (i < 31) ? (i - 31) : (i - 30);
				const double w = 2.0 * M_PI * (double)kk / (double)g.fft;
				const double ph0 = w * ((double)lo - p);
				std::complex<double> ph(std::cos(ph0), std::sin(ph0));
				const std::complex<double> st(std::cos(w), std::sin(w));
				const std::complex<double> a = std::complex<double>(dfr[(size_t)i]) * sc;
				for(int64_t q = lo; q < hi; q++) {
					const std::complex<double> v = a * ph;
					x[(size_t)q] += cf((float)v.real(), (float)v.imag());
					ph *= st;
				}
			}
		};

		// Truth model. `j` counts HALF-frames: the cell transmits an identical PSS every
		// 5 ms and only every second one is the frame the state was acquired on, so the
		// synthetic signal carries both - a tracker that slips onto the neighbour has to
		// be able to fail this test. `dppm` is the constant clock difference expressed as
		// the frame period measured in our samples; `wa`/`wp` add a bounded non-linear
		// wander, because a real oscillator is not a straight line.
		auto pss_abs = [&](int64_t j, double dppm, double wa, double wp) {
			double p = P0 + (double)j * 0.5 * F * (1.0 + dppm * 1e-6);
			if(wa > 0.0) {
				p += wa * (1.0 - std::cos(2.0 * M_PI * (double)j / wp));
			}
			return p;
		};
		auto make_burst = [&](int64_t B, size_t n, double dppm, double wa, double wp,
		                      float amp, float nz, bool with_pss, std::vector<cf> &x) {
			x.assign(n, cf(0.0f, 0.0f));
			for(auto &v : x) {
				v = cf(nz * gauss(rng), nz * gauss(rng));
			}
			if(!with_pss) {
				return;
			}
			int64_t j = (int64_t)std::floor(((double)B - P0) / (0.5 * F)) - 2;
			for(;; j++) {
				const double p = pss_abs(j, dppm, wa, wp) - (double)B;
				if(p > (double)n) {
					break;
				}
				if(p + (double)g.fft < 0.0) {
					continue;
				}
				plant(x, p, amp);
			}
		};
		// Distance from an answer to the NEAREST true PSS, and whether that one was the
		// frame the tracker is supposed to be on. Matching against the nearest of ALL
		// instances is deliberate: matching only against the frame-aligned one would score
		// a half-frame slip as a large error instead of naming it.
		auto score = [&](int64_t B, int64_t ans, double dppm, double wa, double wp,
		                 double *err, bool *on_frame) {
			double bestd = 1e18;
			bool ev = false;
			int64_t j0 = (int64_t)std::floor(((double)(B + ans) - P0) / (0.5 * F)) - 2;
			for(int64_t j = j0; j < j0 + 5; j++) {
				const double p = pss_abs(j, dppm, wa, wp) - (double)B;
				if(std::fabs(p - (double)ans) < std::fabs(bestd)) {
					bestd = p - (double)ans;
					ev = (j % 2) == 0;
				}
			}
			*err = -bestd;          // answer minus truth
			*on_frame = ev;
		};

		// noise level: |corr|^2/power lands near 0.12 for these two amplitudes, which is
		// what the two reference captures measure off air (a real cell puts PSS on 62 of
		// 1200 subcarriers, so the metric is a small number even at good SNR)
		const float AMP = 1.0f, NZ = 1.915f;
		const size_t NBUF = (size_t)(g.frame + 2 * 4096 + g.fft + 64);
		const double D_TRUE = 2.5;             // ppm, the drift this bench measures
		wb_track_cfg cfg;

		// --- the handoff: acquire with the full search, then track the same buffer. If
		// these two disagree by a sample, everything downstream of a tracked position is
		// half a symbol out from everything downstream of an acquired one - and nothing
		// else in the suite would notice.
		{
			std::vector<cf> x;
			make_burst(0, (size_t)(2 * F + g.fft), D_TRUE, 0, 1, AMP, NZ, true, x);
			int nid2 = -1;
			int64_t ap = 0;
			float am = 0.0f;
			const bool got = wb_pss_search(x.data(), x.size(), g, 0.02f, &nid2, &ap, &am);
			double aerr = 0.0;
			bool aframe = false;
			score(0, ap, D_TRUE, 0, 1, &aerr, &aframe);
			snprintf(d, sizeof(d), "nid2 %d (want %d), pos %lld, err %+.2f samples, metric %.3f",
			         nid2, NID2, (long long)ap, aerr, (double)am);
			check(got && nid2 == NID2 && std::fabs(aerr) < 1.5, "wb_pss_acquire", d, verbose);

			wb_track st;
			wb_track_init(st, nid2, (double)ap, 0.0, am, 0.0, 5.0);
			int64_t tp = -1;
			float tm = 0.0f;
			const wb_track_status s =
			    wb_pss_track(x.data(), x.size(), 0.0, g, plan, cfg, st, &tp, &tm);
			snprintf(d, sizeof(d), "search %lld, track %lld (%s), delta %lld",
			         (long long)ap, (long long)tp, wb_track_str(s), (long long)(tp - ap));
			check(s == WB_TRACK_OK && std::llabs(tp - ap) <= 1, "wb_track_matches_search", d,
			      verbose);
		}

		// --- one second away, drift entirely unknown. 2.5 ppm moves the frame by 38
		// samples; the 5 ppm uncertainty asks for a +-101 window and finds it there.
		wb_track st;
		wb_track_init(st, NID2, P0, 0.0, 0.12f, 0.0, 5.0);
		{
			const double t = 1.0;
			const int64_t B = (int64_t)std::llround(t * g.fs);
			std::vector<cf> x;
			make_burst(B, NBUF, D_TRUE, 0, 1, AMP, NZ, true, x);
			int64_t p = -1;
			float m = 0.0f;
			const wb_track_status s = wb_pss_track(x.data(), x.size(), t, g, plan, cfg, st, &p, &m);
			double err = 0.0;
			bool onf = false;
			score(B, p, D_TRUE, 0, 1, &err, &onf);
			snprintf(d, sizeof(d), "%s at %lld, err %+.2f samples, metric %.3f, drift now %.2f ppm",
			         wb_track_str(s), (long long)p, err, (double)m, st.drift_ppm);
			check(s == WB_TRACK_OK && onf && std::fabs(err) < 1.0, "wb_track_1s", d, verbose);
		}

		// --- ten seconds away, still with the uncorrected 5 ppm uncertainty: 384 samples
		// of movement inside a +-792 window. This is the frequency-change case - the whole
		// reason the window is computed from the absence instead of being a constant.
		{
			wb_track s10;
			wb_track_init(s10, NID2, P0, 0.0, 0.12f, 0.0, 5.0);
			const double t = 10.0;
			const int64_t B = (int64_t)std::llround(t * g.fs);
			std::vector<cf> x;
			make_burst(B, NBUF, D_TRUE, 0, 1, AMP, NZ, true, x);
			int64_t p = -1;
			float m = 0.0f;
			const wb_track_status s = wb_pss_track(x.data(), x.size(), t, g, plan, cfg, s10, &p, &m);
			double err = 0.0;
			bool onf = false;
			score(B, p, D_TRUE, 0, 1, &err, &onf);
			snprintf(d, sizeof(d), "%s after 10 s (moved %.0f samples), err %+.2f, metric %.3f",
			         wb_track_str(s), D_TRUE * 1e-6 * 10.0 * g.fs, err, (double)m);
			check(s == WB_TRACK_OK && onf && std::fabs(err) < 1.5, "wb_track_10s_gap", d, verbose);
		}

		// --- the drift becomes a measurement. Gaps long enough to carry ppm information
		// pull the estimate onto the truth and the uncertainty - the window - collapses
		// with it. Short gaps must NOT feed it: one sample of jitter over a single frame
		// reads as 6.5 ppm.
		{
			double t = 1.0;
			const double gaps[6] = { 1.0, 1.0, 2.0, 2.0, 4.0, 4.0 };
			int ok = 0;
			for(int i = 0; i < 6; i++) {
				t += gaps[i];
				const int64_t B = (int64_t)std::llround(t * g.fs);
				std::vector<cf> x;
				make_burst(B, NBUF, D_TRUE, 0, 1, AMP, NZ, true, x);
				int64_t p = -1;
				float m = 0.0f;
				if(wb_pss_track(x.data(), x.size(), t, g, plan, cfg, st, &p, &m) == WB_TRACK_OK) {
					ok++;
				}
			}
			snprintf(d, sizeof(d), "%d/6 fixes, drift %.2f ppm (true %.2f), uncertainty %.2f ppm",
			         ok, st.drift_ppm, D_TRUE, st.unc_ppm);
			check(ok == 6 && std::fabs(st.drift_ppm - D_TRUE) < 0.3 && st.unc_ppm <= 1.0,
			      "wb_track_drift_learn", d, verbose);
		}

		// --- honesty 1: nothing there. An empty band must produce a refusal, never a
		// position. This is the check that a peak-picker cannot pass.
		{
			wb_track sn;
			wb_track_init(sn, NID2, P0, 0.0, 0.12f, D_TRUE, 1.0);
			int fixes = 0;
			wb_track_status last = WB_TRACK_OK;
			float worst = 0.0f;
			for(int i = 1; i <= 3; i++) {
				const double t = (double)i;
				const int64_t B = (int64_t)std::llround(t * g.fs);
				std::vector<cf> x;
				make_burst(B, NBUF, D_TRUE, 0, 1, AMP, NZ, false, x);
				int64_t p = -12345;
				float m = 0.0f;
				last = wb_pss_track(x.data(), x.size(), t, g, plan, cfg, sn, &p, &m);
				worst = std::max(worst, sn.metric);
				if(last == WB_TRACK_OK || p != -12345) {
					fixes++;
				}
			}
			snprintf(d, sizeof(d), "3 empty bursts: %s, %d positions reported, best noise "
			         "metric %.4f, state dropped %d", wb_track_str(last), fixes, (double)worst,
			         (int)!sn.valid);
			check(fixes == 0 && last == WB_TRACK_WEAK && !sn.valid, "wb_track_empty_is_lost", d,
			      verbose);
		}

		// --- honesty 2: the truth OUTSIDE the window. A state that believes it knows the
		// drift to 0.5 ppm, ten seconds away from a cell that actually moved 25 ppm, has
		// its cell 3840 samples out of a +-101 window. It must come back empty-handed. The
		// failure mode this forbids is the expensive one: a confident wrong position.
		{
			wb_track sw;
			wb_track_init(sw, NID2, P0, 0.0, 0.12f, 0.0, 0.5);
			const double t = 10.0, dbad = 25.0;
			const int64_t B = (int64_t)std::llround(t * g.fs);
			std::vector<cf> x;
			make_burst(B, NBUF, dbad, 0, 1, AMP, NZ, true, x);
			int64_t p = -12345;
			float m = 0.0f;
			const wb_track_status s = wb_pss_track(x.data(), x.size(), t, g, plan, cfg, sw, &p, &m);
			snprintf(d, sizeof(d), "cell %.0f samples outside a +-%.0f window: %s, metric %.4f",
			         dbad * 1e-6 * t * g.fs, 0.5e-6 * t * g.fs + cfg.guard, wb_track_str(s),
			         (double)sw.metric);
			check(s != WB_TRACK_OK && p == -12345, "wb_track_out_of_window", d, verbose);
		}

		// --- the boundary where tracking stops being the cheap thing. Two ways to reach it,
		// and both have to be named rather than silently paid for: the configured ceiling,
		// and the hard one at frame/4 that keeps the identical PSS 5 ms away out of the
		// window whatever the caller configures.
		{
			wb_track ss;
			wb_track_init(ss, NID2, P0, 0.0, 0.12f, 0.0, 5.0);
			std::vector<cf> x(NBUF, cf(0.0f, 0.0f));
			int64_t p = -12345;
			const wb_track_status s1 =
			    wb_pss_track(x.data(), x.size(), 120.0, g, plan, cfg, ss, &p, nullptr);
			wb_track_cfg wide = cfg;
			wide.max_half = 1 << 20;
			const wb_track_status s2 =
			    wb_pss_track(x.data(), x.size(), 700.0, g, plan, wide, ss, &p, nullptr);
			snprintf(d, sizeof(d), "120 s -> %s (needs %.0f > %d), 700 s uncapped -> %s "
			         "(alias cap %d)", wb_track_str(s1), 5e-6 * 120.0 * g.fs + cfg.guard,
			         cfg.max_half, wb_track_str(s2), g.frame / 4);
			check(s1 == WB_TRACK_STALE && s2 == WB_TRACK_STALE && p == -12345 && ss.valid,
			      "wb_track_stale_says_so", d, verbose);
		}

		// --- and the long run, which is the property that actually matters: over a minute
		// of bursts against an oscillator that wanders, does the tracked position stay ON
		// the acquired frame, or does it walk off a sample at a time while still reporting
		// success? Every accepted fix is scored against the truth, not against the previous
		// fix, so an accumulating error has nowhere to hide.
		{
			wb_track sl;
			wb_track_init(sl, NID2, P0, 0.0, 0.12f, 0.0, 5.0);
			const double WA = 3.0, WP = 2000.0;    // +-3 samples of wander, ~10 s period
			int ok = 0, slip = 0;
			double worst = 0.0, sum2 = 0.0;
			for(int i = 1; i <= 40; i++) {
				const double t = (double)i;
				const int64_t B = (int64_t)std::llround(t * g.fs);
				std::vector<cf> x;
				make_burst(B, NBUF, D_TRUE, WA, WP, AMP, NZ, true, x);
				int64_t p = -1;
				float m = 0.0f;
				if(wb_pss_track(x.data(), x.size(), t, g, plan, cfg, sl, &p, &m) != WB_TRACK_OK) {
					continue;
				}
				ok++;
				double err = 0.0;
				bool onf = false;
				score(B, p, D_TRUE, WA, WP, &err, &onf);
				if(!onf) {
					slip++;
				}
				worst = std::max(worst, std::fabs(err));
				sum2 += err * err;
			}
			snprintf(d, sizeof(d), "%d/40 fixes, %d half-frame slips, worst |err| %.2f, rms "
			         "%.2f samples, drift %.2f ppm, window +-%d", ok, slip, worst,
			         ok ? std::sqrt(sum2 / ok) : 0.0, sl.drift_ppm, sl.window);
			check(ok == 40 && slip == 0 && worst < 2.0, "wb_track_long_run", d, verbose);
		}
	}

	return g_fail;
}

} // namespace cellsync

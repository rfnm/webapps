// cellsync: 36.211 downlink synchronization + reference sequences (Rel-8 FDD, normal CP).
// EVERY convention that could differ on-air lives here (lorad's lora_phy.cpp pattern):
// an interop fix should be a one-liner in lte_seq.cpp, nothing else moves.
#pragma once

#include <cstdint>
#include <vector>

#include "fft.h"

namespace cellsync {

// ---- numerology at the 1.92 MSps sync rate (128-FFT, 15 kHz subcarriers, normal CP)
constexpr int FFT_N = 128;
constexpr int SLOT_SAMP = 960;             // 0.5 ms
constexpr int SUBFRAME_SAMP = 1920;        // 1 ms
constexpr int HALF_FRAME_SAMP = 9600;      // 5 ms (PSS repetition)
constexpr int FRAME_SAMP = 19200;          // 10 ms
constexpr int SYMS_PER_SLOT = 7;
constexpr double FS_SYNC = 1.92e6;
constexpr double SCS = 15e3;

// CP length and data-start offset of symbol l within a slot (normal CP)
int cp_len(int l);
int sym_data_off(int l);                   // offset of the first post-CP sample

// ---- PSS: length-62 Zadoff-Chu, roots {25,29,34} for NID2 {0,1,2} -------------------
// freq-domain d(0..61); mapping to FFT bins: n<31 -> k=n-31, n>=31 -> k=n-30 (DC empty)
std::vector<cf> pss_freq(int nid2);
// 128-sample time template (unit energy) = IFFT of the mapped freq sequence
std::vector<cf> pss_time(int nid2);
// map a length-62 sequence into 128 FFT bins (bin = (k+FFT_N) % FFT_N), rest zero
void map62(const cf *d, cf *bins128);

// ---- SSS: length-62 BPSK, depends on (nid1 0..167, nid2 0..2, half 0|1) --------------
// half=0 -> transmitted in subframe 0, half=1 -> subframe 5
std::vector<float> sss_seq(int nid1, int nid2, int half);
// the (m0, m1) m-sequence cyclic shifts behind nid1 (36.211 Table 6.11.2.1-1):
// hypotheses sharing m0 or m1 with the truth score ~half the true metric, so decision
// margins must be taken against the m-family-DISJOINT runner-up
void sss_m0m1(int nid1, int *m0, int *m1);

// ---- Gold sequence (36.211 7.2), Nc=1600 --------------------------------------------
void gold_seq(uint32_t cinit, int len, uint8_t *out);

// ---- CRS (port 0): QPSK r(m') for slot ns, symbol l ----------------------------------
// cinit = 2^10 (7(ns+1)+l+1)(2 NID+1) + 2 NID + 1   (normal CP)
// fills out[0..count-1] with r(mprime0) .. r(mprime0+count-1)
void crs_seq(int nid_cell, int ns, int l, int mprime0, int count, cf *out);

// CRS port-0 subcarrier layout for a cell of nrb resource blocks:
//   pilots at k = 6m + (v + (nid%6)) % 6,  m = 0..2nrb-1,  v = 0 (l=0) | 3 (l=4)
//   sequence index m' = m + 110 - nrb
// Returns, for FFT bins rel in [-FFT_N/2, FFT_N/2), the pilots that land inside:
// pairs of (fft_bin_rel, mprime). LTE DC hole: freq rel = k-6nrb (k<6nrb) else k-6nrb+1.
struct crs_pos {
	int rel;      // subcarrier offset from DC, never 0
	int mprime;   // CRS sequence index
};
std::vector<crs_pos> crs_positions(int nid_cell, int nrb, int l, int rel_min, int rel_max);

// candidate downlink bandwidths for the N_RB hypothesis test
extern const int NRB_CANDIDATES[6];

} // namespace cellsync

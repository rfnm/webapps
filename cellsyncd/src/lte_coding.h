// cellsync: 36.212 channel coding for the SIB1 receive path.
// EVERY convention that could differ on-air lives here, the same way lte_seq.h owns the
// 36.211 conventions: an interop fix should be a one-liner in lte_coding.cpp, nothing else
// moves. Self-contained on purpose - no external coding library.
//
// Scope is deliberately the SI-RNTI broadcast path and nothing else:
// QPSK only, one code block only, FDD, normal CP. That removes segmentation entirely -
// the 36.212 limit is Z=6144 and the largest SI transport block is 1736 bits - so CRC-24B
// and the segmentation machinery are not built. If that ever changes, it changes here.
#pragma once

#include <cstdint>
#include <vector>

namespace cellsync {

// ---- CRC (36.212 5.1.1) --------------------------------------------------------------
// Polynomials as generator words with the leading 1 included, MSB-first over the message.
enum crc_poly {
	CRC24A = 0x1864CFB,   // D24+D23+D18+D17+D14+D11+D10+D7+D6+D5+D4+D3+D+1  (transport block)
	CRC16  = 0x11021,     // D16+D12+D5+1                                     (PBCH, DCI)
	CRC8   = 0x19B,       // D8+D7+D4+D3+D+1
};
// Compute the parity over bits[0..n) (each 0/1, transmitted order). Returns the CRC in the
// low bits of the word, MSB of the CRC first when appended.
uint32_t crc_bits(const uint8_t *bits, size_t n, crc_poly poly);
// Append the parity to `bits`, growing it by the CRC length.
void crc_attach(std::vector<uint8_t> &bits, crc_poly poly);
// Check a message that carries its CRC in the last crc_len bits. Returns true when the
// remainder is zero after XORing `mask` onto the received parity (MSB-first over the mask
// bits) - that is how both the PBCH antenna-port mask and the DCI RNTI mask are applied.
bool crc_check(const uint8_t *bits, size_t n, crc_poly poly, const uint8_t *mask = nullptr);
int crc_len(crc_poly poly);

// ---- PBCH antenna-port mask (36.212 Table 5.3.1.1-1) ---------------------------------
// The MIB CRC is masked with the transmit-antenna-port count, so a decode does not merely
// check the CRC - it NAMES the port count. Externally this cost a whole measurement
// campaign to discover; here it is one loop over three hypotheses.
// Fills mask[0..15] for ports in {1,2,4}; any other value is a programming error.
void pbch_crc_mask(int nof_ports, uint8_t mask[16]);
// The three hypotheses, in the order worth trying (2 is by far the most common).
extern const int PBCH_PORT_HYPOTHESES[3];

// ---- DCI RNTI mask (36.212 5.3.3.2) --------------------------------------------------
// The 16-bit RNTI is XORed onto the DCI CRC, MSB first. SI-RNTI is 0xFFFF.
constexpr uint16_t SI_RNTI = 0xFFFF;
void rnti_crc_mask(uint16_t rnti, uint8_t mask[16]);

// ---- DCI formats 1A and 1C on the SI-RNTI path (36.212 5.3.3.1, 36.213 7.1.7) --------
// Only the two formats the common search space carries for system information. The sizes
// are a function of the bandwidth alone, and the 1A size includes both padding rules:
// equalisation against format 0 (1A is always the larger of the pair in FDD) and the
// extra bit when the result lands on an ambiguous size (36.212 5.3.3.1.3).
int dci_size_1a(int nrb);   // 27 bits at 50 PRB
int dci_size_1c(int nrb);   // 13 bits at 50 PRB

struct dci_si {
	int format;         // 0x1A or 0x1C
	bool localized;     // 1A only: false = distributed VRB (the usual choice for SIB1)
	int gap;            // distributed, nrb >= 50: 0 = N_gap1, 1 = N_gap2
	int riv;            // the raw resource indication value, for logging
	int rb_start;       // first virtual RB of the allocation
	int n_prb;          // number of allocated virtual RBs
	int i_tbs;          // MCS field: on SI-RNTI this IS the TBS row index
	int rv;             // 1A redundancy-version field (the BCCH RV really follows the
	                    // SFN formula in 36.321, but log what the eNB put there)
	int n_prb_1a;       // 1A: 2 or 3 from the TPC LSB - selects the TBS column
	int tbs;            // transport block size in bits, from 36.213 7.1.7.2.1-1 / .3-1
};
// Parse a CRC-verified payload of dci_size_1a() or dci_size_1c() bits. Returns false when
// the size matches neither format, the 0/1A flag says format 0, or a field is outside the
// tables (an i_tbs above 26 on 1A cannot be SI).
bool dci_parse_si(const uint8_t *bits, size_t nbits, int nrb, dci_si &out);

// ---- CFI codewords (36.212 5.3.4, Table 5.3.4-1) -------------------------------------
// The CFI is not coded, it is one of three fixed 32-bit words - <011>, <101>, <110>
// repeated. Any two of them differ in 21+ of 32 positions, so a soft correlation against
// the table IS the decoder. Index [cfi-1][bit].
extern const uint8_t CFI_CODEWORD[3][32];

// ---- tail-biting convolutional code (36.212 5.1.3.1) ---------------------------------
// K=7, rate 1/3, generators G0=133, G1=171, G2=165 (octal), and the encoder starts in the
// state formed by the LAST 6 input bits - that is what "tail-biting" means here, and it is
// the single most common place to get 36.212 wrong.
constexpr int CONV_K = 7;
constexpr int CONV_STATES = 1 << (CONV_K - 1);   // 64
// Encode n bits into 3n bits, interleaved d0(0),d1(0),d2(0),d0(1),... as the standard
// writes them into three streams; out is resized to 3n.
void conv_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out);

// Exact maximum-likelihood tail-biting Viterbi over soft input.
// llr holds 3n log-likelihood ratios in the same order conv_encode() produces, positive
// meaning "bit 0 is more likely" (the usual convention: llr = log P(0)/P(1)).
// Exactness matters more than speed here: the blocks are tiny (MIB is 40 bits, a DCI is
// at most ~45), so we simply run the trellis once per candidate start state and keep the
// best path that ends in the state it started from. 64 runs over 45 bits is microseconds
// on an A53 and removes a whole class of "wrap-around heuristic" bugs.
// Returns the decoded n bits and the surviving path metric.
double conv_decode(const std::vector<float> &llr, size_t n, std::vector<uint8_t> &out);

// Wrap-around Viterbi (WAVA): one pass with all start states equally likely, then the end
// metrics fed back as the new start metrics, until the traced-back start state matches the
// end state. Converges to the ML tail-biting path in almost every block and costs 2-3
// passes instead of 64.
//
// This is not a micro-optimisation, it is the difference between usable and not: measured
// on the board's Cortex-A53, the exact decoder needs 7.2 ms per block, so the PDCCH common
// search space (12 blind decodes per candidate subframe, ~100 candidate subframes in a 2 s
// burst) would cost 8.7 s - more than the entire decode budget for the whole burst, for
// one stage. Blind decoding is also exactly where exactness is worth least: almost every
// candidate is garbage that must simply fail its CRC, and a rare suboptimal path just adds
// one more failing candidate.
//
// Returns the path metric, or -inf if it did not converge within max_iter passes; callers
// that need a guaranteed answer fall back to conv_decode().
double conv_decode_fast(const std::vector<float> &llr, size_t n, std::vector<uint8_t> &out,
                        int max_iter = 3);

// ---- rate matching for the convolutional code (36.212 5.1.4.2) -----------------------
// Sub-block interleaver (32 columns, the fixed permutation below), bit collection, then a
// circular buffer read of E bits. PBCH uses E = 1920 over a D = 40 bit block, so every
// coded bit is repeated 16 times - that repetition is where PBCH gets its robustness, and
// accumulating it correctly on the soft bits is worth several dB.
extern const int SUBBLOCK_PERM[32];
// in: 3*D coded bits interleaved d0,d1,d2 as conv_encode() produces them. out: E bits.
void conv_rate_match(const std::vector<uint8_t> &in, size_t E, std::vector<uint8_t> &out);
// Inverse on soft values: every position of the circular buffer that was transmitted adds
// its LLR back onto the coded bit it came from. `soft` is resized to 3*D.
void conv_rate_dematch(const std::vector<float> &llr_e, size_t D, std::vector<float> &soft);

// ---- turbo code (36.212 5.1.3.2) ------------------------------------------------------
// The PDSCH leg of the SIB1 chain. Scope stays the broadcast path: one code block
// (B = TBS+24 <= 6144 always holds for SI), so no segmentation, no CRC-24B, no filler
// bits - K equals B exactly because every SI block size + 24 lands on a legal K.
//
// Written here rather than linked: ArmRAL's turbo decoder is NEON-intrinsics code with
// no x86 path, and the host testbench is x86 - a port would cost more than a scalar
// max-log-MAP at K <= 248, which decodes in microseconds anyway.

// QPP interleaver Pi(i) = (f1*i + f2*i^2) mod K per Table 5.1.3-3. Empty result = K is
// not a legal interleaver size (the caller treats that as a corrupt DCI, not a bug).
void turbo_qpp(int K, std::vector<int> &pi);

// 36.212 Table 5.1.4-1: the turbo sub-block interleaver column permutation. Distinct from
// SUBBLOCK_PERM (Table 5.1.4-2, convolutional) - they differ by a cyclic relabelling, and
// crossing them is a silent, round-trip-invariant, CRC-only-visible bug.
extern const int TURBO_SUBBLOCK_PERM[32];

// Encode K bits into three streams d[0..2] of D = K+4 bits each: systematic, parity 1,
// parity 2, with the twelve trellis-termination bits multiplexed per 5.1.3.2.2. Exists
// for the transmit side of the selftest; the receiver only ever decodes.
void turbo_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> d[3]);

// ---- rate matching for the turbo code (36.212 5.1.4.1) --------------------------------
// Circular-buffer position map for one code block: entry i of the Kw = 3*Kpi buffer is
// the source index s*D + j into the three coded streams (stream-major), or -1 for a
// <NULL> pad. Two differences against the convolutional matcher, both load-bearing:
// streams 1 and 2 are INTERLACED after the systematic part (w[Kpi+2k] = v1(k),
// w[Kpi+2k+1] = v2(k)), and stream 2 reads its matrix through the +1-shifted column
// formula pi(k) = (P(floor(k/R)) + 32*(k mod R) + 1) mod Kpi.
void turbo_cb_map(size_t K, std::vector<int> &map);

// Redundancy-version start: k0 = R*(2*ceil(Ncb/(8R))*rv + 2) with Ncb = Kw (broadcast
// has no soft-buffer limit), which collapses to R*(24*rv + 2).
int turbo_k0(size_t K, int rv);

// TX side (selftest only): read E bits from the circular buffer starting at k0(rv).
void turbo_rate_match(const std::vector<uint8_t> d[3], size_t K, size_t E, int rv,
                      std::vector<uint8_t> &out);

// RX: add E soft bits back onto the coded-bit positions they came from. `soft` is
// (re)sized to 3*(K+4) on a size mismatch, otherwise ACCUMULATED into - that is how the
// four SIB1 redundancy versions of one 80 ms window combine, and the combining happens
// here, before the decoder, where it is worth several dB.
void turbo_rate_dematch(const std::vector<float> &llr_e, size_t K, int rv,
                        std::vector<float> &soft);

// Max-log-MAP turbo decoder over the stream-major soft buffer the dematcher fills
// (positive LLR = bit 0, like everything else in this file). Runs `iters` full
// iterations with 0.75 extrinsic scaling, stops early when the hard decision is stable.
// Returns the K decoded bits; the caller checks CRC-24A - the decoder itself has no
// notion of success, which is honest: max-log-MAP always produces bits.
void turbo_decode(const std::vector<float> &soft, size_t K, int iters,
                  std::vector<uint8_t> &out);

// ---- selftest ------------------------------------------------------------------------
// Hardware-free, called from cellsyncd --selftest. Returns the number of failed checks and
// prints one line per check in the daemon's existing [PASS]/[FAIL] style.
int lte_coding_selftest(bool verbose);

} // namespace cellsync

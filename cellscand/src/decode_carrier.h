// cellscan: the per-PCI decode glue. Given a found cell and the (CFO-removed) burst, run the
// exact verified chain - MIB -> PCFICH -> PDCCH(SI-RNTI) -> PDSCH -> turbo (with
// soft-bit accumulation over the SIB1 occasions of one SI modification period) -> SIB1 ASN.1
// head - and fill a cell_result. Every stage is a production function from cellsyncd/src;
// this file is only the sequencing the pdsch.cpp testbench already proved byte-identical to
// the oracle.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "cellfind.h"
#include "lte_coding.h"
#include "scan.h"

namespace cellscan {

// How many radio frames of the burst the MIB search looks at. It used to be 10 - 5% of a 2 s
// dwell - because one frame was assumed to decode on its own; a cell too weak for that got
// nothing out of the other 95%. The bound is 200 frames, i.e. the whole 2 s dwell, and it is
// a bound rather than "all of it" for two reasons. Memory: one cached observation is 240 REs
// plus the four ports' channel at each of them, ~9.6 kB, so a full window is ~1.9 MB per cell
// - freed as soon as the MIB pass ends, and small against the 245 MB burst it is read out of,
// but an 8 s deep dwell at 800 frames would not be. False positives: the blind hypothesis
// count grows linearly with the window (4 quarter hypotheses x one group per 40 ms TTI x 3
// port hypotheses), and at ~1.1e-5 per hypothesis 200 frames is where the measured rate stays
// at ~1/200 bursts. Which is also why widening and combining are ONE change and not two:
// widening alone multiplies the hypotheses without adding energy to any of them, so it makes
// false CRC-16 passes MORE likely (measured 3/200 bursts at this width, no combining), while
// combining folds four frames into one hypothesis and takes the hazard back down ~5x.
constexpr int MIB_MAX_FRAMES = 200;

// How many successive windows of that size the MIB search walks on a DEEP (8 s) burst. The
// bound above is per window and not per capture, and with a single window an escalated
// pss_only cell paid for 8 s of capture and had 6 s of it never looked at - the opposite of
// why the escalation exists ("a second independent window of TTIs", job.cpp). Four windows
// cover the deep dwell exactly; peak memory stays ONE window's worth of observations because
// a window is decoded and dropped before the next is built, and the extra blind hypotheses
// are spent only on cells that already failed the cheap pass. A 2 s burst runs one window
// either way - the second is past the end of the capture and costs nothing.
constexpr int MIB_DEEP_WINDOWS = 4;

// How many windows a burst of this kind is walked in, and which frames (counted from the half
// hypothesis' frame 0) window `wi` covers - consecutive, non-overlapping, and together exactly
// the dwell: 200 frames = the 2 s burst, 4 x 200 = the 8 s one. Exposed for --selftest, which
// is the only affordable way to pin this: the alternative is a 245 MB synthetic burst.
inline int mib_windows(bool deep) { return deep ? MIB_DEEP_WINDOWS : 1; }
inline int mib_win_begin(int wi) { return wi * MIB_MAX_FRAMES; }
inline int mib_win_end(int wi) { return (wi + 1) * MIB_MAX_FRAMES; }

// The PBCH hypothesis grid. `h` is the quarter of the 40 ms TTI carried by the FIRST frame of
// the window, and ONE h fixes both halves of the problem: the quarter of frame f, never
// re-searched per frame, and which frames share a payload. Coupling them is what makes a
// wrong hypothesis die - it descrambles wrongly - instead of quietly summing two payloads.
inline int mib_quarter(int h, int f) { return (h + f) & 3; }
inline int mib_tti(int h, int f) { return (h + f) / 4; }

// SFN of the window's first frame, given the MIB that verified on the group starting at frame
// `f0` (the decoded SFN already carries that frame's quarter). Wrapped into 0..1023 because a
// window is now long enough to start before the SFN the group reports. decode_cell selects
// the SIB1 subframes from this number - 'sf % 10 != 5 || (sfn & 1)' - so an off-by-one here
// silently kills SIB1 on a cell whose MIB row looks perfect.
inline int mib_sfn0(int sfn_group, int f0) { return ((sfn_group - f0) % 1024 + 1024) % 1024; }

// Split the frame indices `fr` (ascending) into maximal groups sharing one 40 ms TTI under
// hypothesis `h`, as half-open [begin,end) ranges into `fr`. Consecutive frames only: a frame
// that failed to demodulate leaves a hole, and the combiner reads observation i as quarter
// q0+i, so a hole would shift every quarter behind it. Exposed for --selftest.
void mib_group_runs(const std::vector<int> &fr, int h,
                    std::vector<std::pair<size_t, size_t>> &runs);

// The soft-bit buffer SIB1 is decoded out of, spanning many 80 ms windows rather than one:
// the transport block is byte-identical throughout an SI modification period, so occasions
// keep combining across period boundaries. Bounded by TWO keys, because the grant alone is
// not enough - see sib1_soft_accum.
struct sib1_soft {
	std::vector<float> soft;                                   // 3*(K+4) circular-buffer LLRs
	size_t K = 0;                                              // TBS + 24, as last accumulated
	int rb_start = -1, n_prb = -1, i_tbs = -1, n_prb_1a = -1;   // the grant this buffer holds
	int mod_period = -1;                                       // sfn/64, the content window
	uint8_t rv_mask = 0;                                       // bit rv per DISTINCT RV folded in
	int n = 0;                                                 // occasions combined so far
};

// Fold one occasion's PDSCH LLRs into `acc`, clearing it first if this occasion cannot be the
// same transport block as the ones already there. Returns the number of occasions now
// combined. Factored out so --selftest can pin the accumulation policy - the thing that
// decides whether a tight SI grant ever closes - without a capture.
int sib1_soft_accum(sib1_soft &acc, const cellsync::dci_si &s, int sfn,
                    const std::vector<float> &llr);

// One attempt: turbo-decode `soft` (K = TBS+24) and, if CRC-24A holds, fill `out` from the
// tbs-bit transport block. Returns 0 on a byte-verified head, 1 if the CRC failed, and the
// negative sib1_head_decode rc if the CRC held but the grammar did not.
int sib1_try_decode(const std::vector<float> &soft, size_t K, size_t tbs, cell_result &out);

// The per-occasion decode ladder: the accumulated buffer first, since it is the stronger
// attempt, then this occasion's own LLRs alone, so one poisoned occasion cannot cost the
// whole burst. Same return codes as sib1_try_decode; `*combined` is set when the decode that
// won was the accumulated one AND it held more than one distinct redundancy version.
// Exposed so --selftest can pin the fallback without a capture.
int sib1_decode_occasion(const sib1_soft &acc, const cellsync::dci_si &s,
                         const std::vector<float> &llr, cell_result &out, bool *combined);

// Decode one cell off the burst into `out`. `deep` says this IS the caller's second, longer
// burst: the MIB search then walks up to MIB_DEEP_WINDOWS windows instead of one,
// so the whole 8 s is examined and not just its first quarter, and more SIB1 windows are
// attempted. Sets out.status to decoded | mib_only | pss_only. `cancel`
// is polled between windows. Returns true if at least the MIB decoded.
bool decode_cell(const std::vector<cf> &x, const found_cell &fc, const wb_num &g,
                 const fft_plan &plan, bool deep, cell_result &out, std::atomic<bool> *cancel);

// Populate the SIB1-derived fields of `out` from a CRC-verified BCCH transport block, exactly
// as decode_cell does after a successful PDSCH decode. Factored out so --selftest can pin the
// emitted row against the two known on-air transport blocks with no hardware.
// Returns the sib1_head_decode status (0 = ok); on success sets out.sib1_ok and derives the
// human fields (operator, eNB/sector, bandwidth).
int fill_cell_from_tb(cell_result &out, const uint8_t *tb, size_t nbytes);

} // namespace cellscan

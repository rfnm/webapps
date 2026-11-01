// cellsync: wideband LTE downlink front end - the layer between the raw 15.36 MSps burst
// and the channel decoders in lte_coding.
//
// The sync layer in tracker.cpp works at 1.92 MSps because that is all PSS/SSS need. SIB1
// needs the whole carrier: the channel estimate averages CRS over all N_RB, and measured
// on air that factor is the entire margin. So this file owns a SECOND
// numerology - 1024-FFT at 15.36 MSps for a 50 PRB cell - alongside, never replacing, the
// 128-FFT sync numerology in lte_seq.h.
//
// Everything here is 36.211: CP lengths, CRS positions for ports 0..3, the PBCH resource
// mapping. Keeping it beside lte_seq.cpp rather than inside it is deliberate - lte_seq owns
// the sync signals, this owns the data-bearing grid.
#pragma once

#include <cstdint>
#include <vector>

#include "fft.h"

namespace cellsync {

// ---- numerology for a data-rate stream (normal CP) ------------------------------------
// One struct per sample rate so the sync path's constants stay untouched. All lengths in
// samples at the stream's own rate.
struct wb_num {
	int nrb;            // resource blocks (50 = 10 MHz)
	int fft;            // FFT size (1024 for 50 RB at 15.36 MSps)
	int cp0;            // CP of symbol 0 in a slot
	int cp;             // CP of symbols 1..6
	int slot;           // samples per 0.5 ms slot
	int subframe;       // samples per 1 ms subframe
	int frame;          // samples per 10 ms radio frame
	double fs;          // sample rate

	// offset of symbol l (0..6) within a slot, pointing at the first post-CP sample
	int sym_off(int l) const {
		return (l == 0) ? cp0 : (cp0 + fft + (l - 1) * (cp + fft) + cp);
	}
	int cp_len(int l) const { return l == 0 ? cp0 : cp; }
};
// The only geometry we need today. 15.36 MSps: cp0=80, cp=72, slot=7680.
wb_num wb_num_for(int nrb);

// ---- OFDM demodulation ----------------------------------------------------------------
// Strip the CP of symbol l in the slot starting at `slot_start` and transform. `grid` is
// filled with nrb*12 subcarriers in ascending frequency order, DC removed: index 0 is the
// lowest occupied subcarrier, index 6*nrb-1 the one just below DC, 6*nrb the one just
// above. Returns false if the window runs past the end of the buffer.
bool wb_demod_symbol(const cf *x, size_t n, int64_t slot_start, int l,
                     const wb_num &g, const fft_plan &plan, cf *grid);

// ---- CRS ------------------------------------------------------------------------------
// Cell-specific reference signals for antenna port p in symbol l of slot ns (36.211 6.10.1).
// Ports 0 and 1 live in l = 0 and l = 4; ports 2 and 3 only in l = 1. Returns the
// subcarrier indices into the grid above together with the reference symbol at each, or an
// empty vector when the port carries no pilots in that symbol.
struct crs_ref {
	int k;        // subcarrier index into the demodulated grid
	cf ref;       // the transmitted reference symbol
};
void wb_crs(int nid_cell, int nrb, int port, int ns, int l, std::vector<crs_ref> &out);
// Which symbols of a slot carry pilots for this port (returns count, fills l[0..2]).
int wb_crs_symbols(int port, int l[3]);

// ---- channel estimation ---------------------------------------------------------------
// Least-squares CRS estimate followed by linear interpolation across frequency, for one
// port and one symbol. `h` is filled for all nrb*12 subcarriers.
// Averaging over the full band is the point: at the ~0 dB SINR of a real cell with
// co-channel neighbours it is worth about a factor of 8 against a centre-6-PRB estimate.
void wb_chan_est(const cf *grid, const std::vector<crs_ref> &pilots, int ncarrier, cf *h);

// ---- transmit-diversity combining -------------------------------------------------------
// SFBC (2 ports) / SFBC+FSTD (4 ports) combining per 36.211 6.3.4.2.2 / 6.3.4.3, shared by
// PBCH, PCFICH and PDCCH - all three ride the same diversity scheme, and the combiner is
// exactly the place where a plausible-looking sign error stays EVM-invisible,
// so it exists once. `re` holds n received REs in mapping order, h[p] the channel of port p
// at each RE; out[i] is the equalised symbol. n must be even for 2 and 4 ports.
//
// The combiner ZERO-FORCES: out carries the transmit constellation at its own amplitude
// whatever the channel did, and the noise on it is scaled by 1/|h_eff|^2. `wout`, when given,
// receives that |h_eff|^2 per output RE (|h|^2 for one port, |h0|^2 + |h1|^2 for the SFBC
// pair, which shares it). Multiplying out by it turns the zero-forcer back into a matched
// filter, which is what any weighted combination of several observations needs - reported
// here rather than recomputed by callers, because recomputing it means duplicating the FSTD
// pairing, and that pairing is exactly the thing this file exists to keep in one place.
void wb_sfbc_combine(const cf *re, const cf *const h[4], int n, int nof_ports, cf *out,
                     float *wout = nullptr);

// ---- PCFICH -----------------------------------------------------------------------------
// The four REG anchors of PCFICH in symbol 0 (36.211 6.7.4): subcarrier
// k = (Nsc/2)(NID mod 2 Nrb) + floor(i Nrb/2)(Nsc/2) mod 12 Nrb, i = 0..3. Each REG spans
// 6 subcarriers of which 4 carry data - symbol 0 always reserves the two CRS combs of
// ports 0/1 regardless of the true port count (36.211 6.2.4). Exported separately because
// the PDCCH mapper must EXCLUDE these REGs from its own interleaver input.
void wb_pcfich_regs(int nid_cell, int nrb, int k_reg[4]);

// Decode the CFI from symbol 0 of the slot starting at `slot_start` (slot number ns, i.e.
// 2*subframe). Demodulates and channel-estimates internally (ports 2/3 from symbol 1, one
// symbol away - frozen, which is fine over 16 REs once the CFO is removed upstream).
// Descrambles with cinit = (floor(ns/2)+1)(2 NID+1) 2^9 + NID and correlates against the
// three fixed codewords. Returns 1..3, or -1 when the symbols run past the buffer.
// `margin` (optional) gets (best-second)/sum|llr|, up to 42/32 = 1.31 on a clean signal
// (any two codewords differ in >=21 of 32 bits): the sharpest oracle-free health probe of
// the whole control-region front end - random input gives values near 0.
int wb_pcfich_decode(const cf *x, size_t n, int64_t slot_start, int ns, int nid_cell,
                     int nof_ports, const wb_num &g, const fft_plan &plan, float *margin);

// ---- PHICH ------------------------------------------------------------------------------
// REG anchors of all PHICH groups (36.211 6.9.3), normal duration only - both reference
// cells run it and extended duration would occupy symbols 1/2, a different mapping we do
// not build until a cell forces us to. We never decode PHICH; the PDCCH interleaver just
// has to know which symbol-0 REGs it must NOT count as its own. ngroups =
// ceil(Ng*Nrb/8) with Ng from the MIB; group m occupies available-REG indices
// (NID + m + floor(i*n0/3)) mod n0, i = 0..2, where the available REGs are the symbol-0
// REGs minus PCFICH's four, in ascending subcarrier order. Fills k_reg with 3*ngroups REG
// start subcarriers (group-major) and returns ngroups; returns 0 for extended duration.
int wb_phich_regs(int nid_cell, int nrb, int phich_dur, float phich_res,
                  std::vector<int> &k_reg);

// ---- PDCCH ------------------------------------------------------------------------------
// One resource-element group of the control region: OFDM symbol l and the four data
// subcarriers k[0..3]. Symbol 0 - and symbol 1 when the cell has four ports - spans six
// subcarriers with the two CRS combs skipped; every other control symbol is four
// consecutive subcarriers (36.211 6.2.4). The width rule is the convention most likely to
// be got wrong: it follows the PILOT pattern of the symbol, not a fixed grid.
struct wb_reg {
	int l;
	int k[4];
};

// All REGs of symbols 0..cfi-1 NOT assigned to PCFICH or PHICH, in the 36.211 6.8.5
// mapping order: the OUTER loop runs over subcarriers k', the INNER over symbols l'. That
// order is a norm convention the interleaver depends on - swapping the loops still yields
// a bijection and decodes nothing. Returns the REG count (N_CCE = count/9).
int wb_pdcch_regs(int nid_cell, int nrb, int cfi, int nof_ports, int phich_dur,
                  float phich_res, std::vector<wb_reg> &regs);

// The 6.8.5 quadruplet permutation: the sub-block interleaver of 36.212 5.1.4.2.1 applied
// to quadruplet INDICES with the <NULL>s dropped. P[i] is the z-index carried by w(i), so
// the receiver reads REG m' into z[P[(m' + NID) mod M]] - the cyclic shift by the cell ID
// sits between the two.
void wb_pdcch_perm(int m, std::vector<int> &P);

// A DCI whose CRC verified against SI-RNTI in the common search space.
struct wb_dci {
	int size;          // payload bits - names the format family (1A-pair vs 1C size)
	int L;             // aggregation level, 4 or 8 (the CSS has no other)
	int cce;           // first CCE of the candidate
	double metric;     // Viterbi path metric over the summed |LLR|, ~1 when clean
	uint8_t bits[32];  // the payload, without CRC
};

// Blind decode of the common search space in one subframe (first slot ns = 2*subframe):
// channel estimation for symbols 0..cfi-1 (ports 0/1 interpolated between the slot's
// pilot symbols 0 and 4, ports 2/3 between symbol 1 of this slot and the next), REG
// extraction, SFBC combining per quadruplet, de-interleaving, descrambling with
// cinit = subframe*2^9 + NID, then 6 candidates x 2 DCI sizes through rate dematch,
// tail-biting Viterbi and CRC-16 masked with SI-RNTI. Y_k = 0 in the CSS, so there is no
// hashing - candidates start at CCE L*i. Appends every verified DCI to `out` and returns
// the number found, or -1 when the needed symbols run past the buffer.
int wb_pdcch_css_decode(const cf *x, size_t n, int64_t slot_start, int ns, int nid_cell,
                        int nof_ports, int cfi, int phich_dur, float phich_res,
                        const wb_num &g, const fft_plan &plan, std::vector<wb_dci> &out);

// ---- PDSCH ------------------------------------------------------------------------------
// Soft bits of the single-codeword PDSCH carrying a localized type-2 allocation (the shape
// every SI grant on the reference cells has): RE extraction over PRBs
// [rb_start, rb_start+n_prb) across symbols cfi..13 per 36.211 6.3.5 (k ascending inside
// each symbol, then l, first slot first), excluding the CRS combs of the CONFIGURED ports,
// and in subframes 0/5 the centre six PRB of the sync symbols (subframe 0 also PBCH's
// four). Channel per port interpolated in time between all pilot symbols of the subframe,
// SFBC(+FSTD) combining per symbol, QPSK demap, descrambling with
// cinit = rnti*2^14 + subframe*2^9 + NID (q = 0, one codeword). Appends nothing on
// failure. Returns E, the number of soft bits (2 per RE), or -1. The result feeds
// turbo_rate_dematch with the RV from the DCI - accumulation across the redundancy
// versions of one 80 ms window happens there, not here.
int wb_pdsch_cw_llr(const cf *x, size_t n, int64_t sf_start, int ns, int nid_cell,
                    int nof_ports, int cfi, int rb_start, int n_prb, uint16_t rnti,
                    const wb_num &g, const fft_plan &plan, std::vector<float> &llr);

// ---- PBCH ------------------------------------------------------------------------------
// Channel estimate per port and per PBCH symbol, interpolated in TIME between the two
// nearest pilot-bearing symbols of each port: ports 0/1 have pilots in slot-1 symbols 0
// and 4, ports 2/3 only in symbol 1 of each slot, so their second anchor is symbol 1 of
// slot 2 (where the comb moves, v depending on slot parity). Freezing the estimate at
// symbol 0 instead carries every residual rotation (CFO, clock error) uncorrected across
// four symbols - measured on air it costs decode margin. h[port][l] is
// resized to 12*nrb. Returns false if the needed symbols run past the buffer.
bool wb_pbch_chest(const cf *x, size_t n, int64_t slot1_start, int nid_cell,
                   const wb_num &g, const fft_plan &plan, std::vector<cf> h[4][4]);

// The 72 centre subcarriers of symbols 0..3 in slot 1, minus the REs reserved for CRS -
// and the reservation always assumes FOUR ports regardless of how many the cell really
// uses, which is the convention most likely to be got wrong. Hypothesis-independent:
// fills `re` with the raw received REs in 36.211 6.6.4 mapping order (k ascending within
// each symbol, symbols 0..3) and h_out[p][i] with the channel of port p at RE i.
// Returns the RE count - always 240 for normal CP.
int wb_pbch_extract(const cf *const grid_sym[4], const std::vector<cf> h_sym[4][4],
                    int nrb, int nid_cell, cf *re, cf h_out[4][240]);

// What a MIB attempt saw when it did not verify, and the one number that says how close it
// came. Accumulated across calls, so a caller can carry one struct through a whole burst.
struct wb_mib_stats {
	uint32_t bw_reject = 0;    // decoded, but all-zero or a dl-Bandwidth outside the table
	uint32_t crc_fail = 0;     // decoded and plausible, CRC-16 said no
	double metric_sum = 0.0;   // sum over CALLS of that call's best normalised path metric
	uint32_t metric_n = 0;     // calls that produced one - read them through wb_mib_metric
};

// How close this run of hypotheses came: the MEAN, over the groups it was tried on, of each
// group's best Viterbi path metric normalised by the total soft magnitude (1.0 = the
// surviving path agreed with the sign of every soft bit). -1 when nothing decoded at all.
//
// A mean and NOT the maximum, which is what this used to report. The maximum over hypotheses
// is an extreme-value statistic: it grows with the number of them and therefore with the
// window, measured 0.811 at 10 frames rising to 0.853 at 200 on PURE NOISE, while barely
// moving with SNR (0.822 at -20 dB to 0.829 at -7 dB on a real cell). Empty spectrum scanned
// long outranked a real, nearly-decodable cell - the exact opposite of the question the
// number exists to answer. A mean converges instead of drifting, so it is comparable across
// window lengths, and it rises with SNR because every group improves, not just the luckiest.
// Take the mean per HYPOTHESIS and the max over the handful of hypotheses, never a mean over
// hypotheses that includes ones known to be wrong.
inline double wb_mib_metric(const wb_mib_stats &s) {
	return s.metric_n ? s.metric_sum / (double)s.metric_n : -1.0;
}

// Full MIB attempt on `nf` CONSECUTIVE radio frames' PBCH observations (240 REs each, as
// wb_pbch_extract fills them): SFBC(+FSTD) combining per observation under each port
// hypothesis, QPSK demap, descrambling, rate dematch, tail-biting Viterbi, CRC-16 against
// that hypothesis' port mask.
//
// One PBCH TTI is 40 ms: ONE payload, four redundancy versions, one gold sequence per TTI
// segmented into four 480-bit quarters. Each frame is descrambled with its OWN segment, so
// LLRs from different frames are already commensurate, and for D=40 every quarter is four
// copies of the same 120 mother-code bits - combining is pure energy accumulation. That is
// worth ~1.9 dB and it is where a weak cell's MIB comes from. The accumulation is MAXIMAL
// RATIO, weighted per RE and per observation (see the body): unweighted, four frames of very
// unequal strength decode WORSE than the best of them alone, which would make the whole
// change a regression on exactly the cells it exists for.
//
// `q0` is the quarter carried by observation 0; observation i carries (q0+i) mod 4. It is
// NOT re-searched per observation: one hypothesis fixes both the quarter and which frames
// share a payload, and a wrong one simply descrambles wrongly and dies. q0 < 0 means "search
// all four", which is only legal with nf == 1 and is exactly today's single-frame behaviour.
// A group that would run past quarter 3 is REFUSED: the eight SFN MSBs live in the payload
// and change at the TTI boundary, so such a group sums two different MIBs (see the body).
// Returns true when a MIB verifies; fills nof_ports, nrb, sfn (full 10-bit, of the FIRST
// observation), phich_dur (0 normal / 1 extended), phich_res (1/6, 1/2, 1 or 2) and, if
// given, `st`.
bool wb_mib_decode_n(const cf *const re[], const cf (*const h[])[240], int nf, int nid_cell,
                     int q0, int *nof_ports, int *nrb, int *sfn, int *phich_dur,
                     float *phich_res, wb_mib_stats *st);

// One frame, quarter unknown: the single-observation case of wb_mib_decode_n.
bool wb_mib_decode(const cf *re, const cf h[4][240], int n_re, int nid_cell,
                   int *nof_ports, int *nrb, int *sfn, int *phich_dur, float *phich_res);

// ---- PSS search ------------------------------------------------------------------------
// Find the strongest PSS in a wideband buffer and return which NID2 it was, where, and how
// strong. Overlap-save FFT correlation, not a sliding dot product: at 15.36 MSps a naive
// search over two frames costs 307200 positions x 1024 taps x 3 templates, which measured
// 11.5 s on the board's A53 and was the whole decode time once SSS removed the PBCH search.
//
// Two things make it cheap, and both come from tracker.cpp's xcorr_win3(): the forward FFT
// of each window and the sliding power sum are properties of the WINDOW, identical for all
// three templates, so they are computed once and reused three times.
// Returns false if nothing crosses `thresh` (normalised |corr|^2, 0.05 is a sane floor).
bool wb_pss_search(const cf *x, size_t n, const wb_num &g, float thresh,
                   int *nid2, int64_t *pos, float *metric);

// ---- PSS tracking ----------------------------------------------------------------------
// Re-finding a cell that is already known, instead of searching for it again.
//
// wb_pss_search above is ACQUISITION: it knows nothing about the cell, so it correlates
// three templates over two whole frames - 307200 candidate positions. Measured on the
// board that is 0.21 s of the 0.22 s a one-second decode costs, i.e. essentially the whole
// bill.
//
// Once a cell has been acquired that ignorance is gone. Its frame boundary is known and it
// runs on predictably: every 10 ms, at 15.36 MSps every 153600 samples. The next burst only
// has to look in a narrow window around the predicted position, and only with the ONE
// template its NID2 names. At that few positions the overlap-save FFT machinery costs more
// than it saves - a sliding dot product is the cheaper form, and it is also the more
// accurate one (no window-boundary lags to drop).
//
// This holds ACROSS frequency changes, which is what makes it worth having: coming back
// after t seconds, the frame has advanced by t*100 frames plus whatever the two clocks
// drifted apart. The window therefore grows linearly with the absence but stays tiny
// against a full search for a long time - and when it stops being tiny, wb_pss_track says
// so (WB_TRACK_STALE) instead of quietly becoming the expensive thing it replaced.
//
// Measured on the board against PCI 274 (816.0 MHz) and PCI 377 (796.0 MHz), one A53 core:
//
//   full acquisition, two frames                    0.204 s
//   tracking, 0.43 s between bursts (window +-28)   0.00032 s   ->  644x
//   tracking, back after 20.6 s on another carrier  0.00144 s   ->  140x
//   tracking, back after 60.6 s on another carrier  0.00371 s   ->   57x
//
// The cost is 0.10 ms of fixed setup plus 3.1 us per lag, so it stays cheaper than a full
// search out to ~66000 lags - which is past the frame/4 safety cap below. Tracking is never
// the expensive option; it only stops being a spectacular saving.
//
// Two things make the short-gap factor larger than the 307200/57 position count alone: the
// three acquisition templates collapse to the one the cell's NID2 names, and the window is
// small enough that the overlap-save FFT is replaced by a plain sliding dot product.
//
// The drift this bench actually shows between the ADC clock and these cells is 0.02..0.07
// ppm, i.e. a frame moves well under a sample per 10 s of absence, and the window is
// dominated by the fixed guard rather than by drift. The defaults below are nevertheless
// sized for the ppm-scale case (and the selftest exercises 2.5 ppm), because the drift is
// a property of whatever reference the board is running on, not of this code.
//
// The estimator itself is borrowed, not reinvented: the |corr|^2/power metric and the
// parabolic sub-sample peak are cell_tracker::step() from tracker.cpp (README's "PSS timing
// follow"), and so is the rule that a peak on the window edge counts as a miss. What is new
// here is that the window is sized from an explicit uncertainty rather than being a fixed
// +-24, because a burst receiver's gaps are seconds, not half-frames.

// What the caller holds between bursts. Small on purpose - it is the whole memory of the
// cell's timing, and the caller (which owns the retunes) is the only thing that knows when
// it should be thrown away.
struct wb_track {
	bool valid = false;      // false: nothing to track from, the caller must acquire
	int nid2 = -1;           // PSS root, from acquisition - the single template we correlate
	double pos = 0.0;        // sub-sample offset of the PSS symbol body, relative to t_sec
	double t_sec = 0.0;      // time base of `pos`, on the caller's own stream clock
	// Signed timing drift, in ppm of the cell's frame period as measured in OUR samples:
	// >0 means a frame takes MORE than g.frame of our samples, i.e. our clock runs fast.
	// Same sign convention as the README's timing slope. Applied to the prediction.
	double drift_ppm = 0.0;
	// What we do NOT know about that drift. This, times the elapsed time, is the window -
	// so it is an uncertainty, never the estimate itself.
	double unc_ppm = 5.0;
	float metric = 0.0f;     // last accepted |corr|^2/power
	float ref_metric = 0.0f; // smoothed metric of accepted fixes - the relative accept gate
	float margin = 0.0f;     // last peak over the window's own off-peak floor (diagnostic)
	int window = 0;          // half-width of the last window, in samples. Derived, not set
	                         // by the caller - it is the number worth logging, because it
	                         // is the whole cost model and it names when to re-acquire
	int hits = 0;            // consecutive successful fixes
	int misses = 0;          // consecutive failures; max_miss of them invalidate the state
	int64_t frames = 0;      // frames advanced since acquisition (diagnostic)
};

enum wb_track_status {
	WB_TRACK_OK = 0,     // fixed; `pos` is the PSS body start in this buffer
	WB_TRACK_WEAK,       // correlated, nothing credible in the window - LOST, re-acquire
	WB_TRACK_EDGE,       // peak sits on the window edge: the truth may be outside it - LOST
	WB_TRACK_STALE,      // the window the elapsed time demands is too wide to be worth it
	WB_TRACK_RANGE,      // no predicted PSS fits in this buffer with its window
	WB_TRACK_NOSTATE,    // state invalid or unusable arguments
};
const char *wb_track_str(wb_track_status s);

struct wb_track_cfg {
	float thresh = 0.02f;      // absolute floor on |corr|^2/power; noise over the widest
	                           // window peaks near 0.011, a real cell sits at 0.05..0.3
	float rel_thresh = 0.25f;  // and at least this fraction of the cell's own smoothed
	                           // metric: -6 dB tolerates fading, not a different signal.
	                           // Measured cost of this gate on PCI 274: 7 refusals in 40
	                           // bursts, every one of them a real fade (the cell was within
	                           // 3 samples of the prediction at 10-13 dB down), and every
	                           // one recovered on the next burst - the miss widens the
	                           // window by itself. Loosening it buys those bursts back at
	                           // the price of accepting positions with no sub-sample worth
	int guard = 24;            // fixed samples on top of time x drift: multipath movement,
	                           // sub-sample residue and retune settling
	int max_half = 4096;       // give up beyond this half-width. 4096 samples at 5 ppm is
	                           // ~53 s of absence; internally also capped at frame/4 so the
	                           // PSS 5 ms away can never be mistaken for ours
	int max_miss = 3;          // consecutive misses before the state is dropped
	int min_span_frames = 20;  // shorter gaps carry no ppm information: one sample of
	                           // position noise over one frame reads as 6.5 ppm, so the
	                           // drift and uncertainty updates are gated on the span
	double pos_sigma = 2.0;    // assumed position noise (samples), the resolution floor of
	                           // a drift observation
	double unc_floor_ppm = 0.5;   // oscillator wander between bursts is not observable from
	double unc_ceil_ppm = 20.0;   // a single residual - never claim to know better
	double drift_alpha = 0.5;  // smoothing of the drift estimate
};

// Seed the state from an acquisition (wb_pss_search + wb_sss_decode). `pos` is that
// search's position, `t_sec` the time base it was measured on, `metric` its metric.
void wb_track_init(wb_track &st, int nid2, double pos, double t_sec, float metric,
                   double drift_ppm, double unc_ppm);

// Follow the cell into a new burst. `x`/`n` is the burst, `t_sec` the time of its FIRST
// sample on the same clock as st.t_sec (the caller's stream sample counter is the exact
// choice; any consistent clock works, because the drift estimated here is the difference
// between THAT clock and the cell's frame period). `plan` must be of size g.fft.
//
// On WB_TRACK_OK the state is advanced and `pos` gets the PSS body start of the first
// tracked frame in the buffer - the same quantity, to the same sample, that wb_pss_search
// reports, so wb_sss_decode and the decoders take it unchanged. On anything else the state
// is NOT advanced (its position and time base stay put, so the window widens by itself on
// the next attempt) and `pos` is left untouched: a tracker that answers when it does not
// know is worse than no tracker.
wb_track_status wb_pss_track(const cf *x, size_t n, double t_sec, const wb_num &g,
                             const fft_plan &plan, const wb_track_cfg &cfg, wb_track &st,
                             int64_t *pos, float *metric);

// ---- SSS -------------------------------------------------------------------------------
// Decode the secondary sync signal in the symbol before the PSS and return NID1 together
// with the half-frame it came from. This replaces two brute-force loops at once: without
// it a receiver has to try all 168 NID1 values AND both half-frame positions, which is
// 336 full PBCH attempts. Measured on the board, that search was 20.7 s of the decode.
//
// pss_pos is the first post-CP sample of the PSS symbol, as the PSS correlator reports it.
// The PSS itself is the channel reference: it sits one symbol later, its sequence is known
// once NID2 is, so dividing the SSS by the PSS-derived channel costs nothing extra.
// Returns true on success and fills nid1 (0..167), half (0 = subframe 0, 1 = subframe 5)
// and margin (best correlation over runner-up; below ~1.3 do not trust it).
bool wb_sss_decode(const cf *x, size_t n, int64_t pss_pos, int nid2, const wb_num &g,
                   const fft_plan &plan, int *nid1, int *half, float *margin);

int lte_wb_selftest(bool verbose);

} // namespace cellsync

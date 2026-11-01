// cellscan: the sweep band plan - the conventions of the automatic survey, isolated in one
// file (house style, like plmn.h): WHICH frequencies the sweep probes, and WHICH probes go on
// to the full decode. A new band is one table row. The plan is not invented: it is the result
// of a real measurement series on this board - B20 has fixed 10 MHz block centres (cells were
// found on all three), B8/B3/B1/B7 have no universal centres and are sampled on a 5 MHz grid
// (the strongest carrier of the reference sweep was in B1, a 1&1 cell in B7).
//
// The sweep is TWO-STAGE because most candidate frequencies are empty and a full decode per
// candidate would waste minutes per empty band:
//   stage 1 (probe): a short capture per candidate, in-band elevation over the out-of-band
//     floor in dB. The DC spike at |f| < 150 kHz is EXCLUDED from the metric - measuring it
//     instead of the carrier gives a very convincing wrong answer.
//   stage 2 (decode): only probes above SWEEP_THRESH_DB, strongest first, capped, through the
//     unchanged full scan path. An empty band costs seconds, not minutes.
// The plan also owns the ANTENNA POLICY (bottom of this file): which RF path receives which
// carrier, because the right antenna is a property of the frequency, not of the job.
#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "scan.h"

namespace cellscan {

// ---- stage-1 economics ---------------------------------------------------------------------
constexpr double SWEEP_PROBE_S = 0.35;        // probe capture length: enough FFT averages
constexpr double SWEEP_SETTLE_S = 0.35;       // measured retune settling - dominates stage 1,
                                              // so it is part of the progress estimate
constexpr double SWEEP_THRESH_DB = 6.0;       // in-band elevation that earns a full decode
constexpr int SWEEP_MAX_DECODES = 16;         // stage-2 cap (strongest first), bounds worst case
constexpr double SWEEP_DECODE_S = 16.0;       // measured offline decode cost of one 2 s burst on
                                              // this board - stage 2 is decode-bound, not
                                              // capture-bound, and the progress estimate has to
                                              // say so (first full board sweep: 47 probes in
                                              // 33 s, 15 decodes in 294 s). Was 13.0, measured
                                              // before the MIB pass widened from 10 frames to a
                                              // 200-frame window with TTI combining. Worst case
                                              // - no cell, both half hypotheses fully walked -
                                              // that pass now costs 0.110 s per cell on the x86
                                              // dev host where it cost 0.007 s, and 0.441 s on a
                                              // deep 8 s burst, at up to 3 cells per carrier.
                                              // Scaled by the A53's margin over that host it is
                                              // a few seconds per escalating carrier, so the
                                              // estimate carries +3 s. Only the progress bar
                                              // reads this: too low stalls the bar, it never
                                              // makes the scan wrong.

// ---- stage-1 metric geometry (Hz, around the tuned centre at 15.36 MSps) -------------------
constexpr double SWEEP_DC_EXCLUDE_HZ = 150e3; // the DC-spike trap: NEVER in-band
constexpr double SWEEP_INBAND_HZ = 2.5e6;     // centre region every 10 MHz carrier covers,
                                              // even one sitting half a grid step off
constexpr double SWEEP_FLOOR_LO_HZ = 6.0e6;   // out-of-band floor window (per side; the metric
constexpr double SWEEP_FLOOR_HI_HZ = 7.0e6;   //  takes the QUIETER side, see cellfind.cpp)

// ---- the band plan: one row per band -------------------------------------------------------
// Candidates run first..last inclusive in steps of `step`. B20 lists the three validated
// block centres (step 10 = one probe per licensed block, every block its own carrier); the
// high bands have a variable raster and are sampled at 5 MHz, so ONE real carrier can light
// SEVERAL adjacent probes - which is why selection suppresses non-maxima on 5 MHz grids only.
struct sweep_band_row {
	int band;            // 3GPP band number (matches plmn.h band_table / band_for_freq_mhz)
	const char *key;     // the name POST /sweep accepts ("b20")
	double first_mhz;    // first candidate centre
	double last_mhz;     // last candidate centre
	double step_mhz;     // candidate spacing
};

inline const std::vector<sweep_band_row> &sweep_band_table() {
	static const std::vector<sweep_band_row> t = {
		{ 20, "b20",  796.0,  816.0, 10.0 },   // validated block centres 796/806/816
		{ 8,  "b8",   930.0,  955.0,  5.0 },   // 925-960, grid clear of the band edges
		{ 3,  "b3",  1810.0, 1875.0,  5.0 },   // 1805-1880, variable raster
		{ 1,  "b1",  2115.0, 2165.0,  5.0 },   // 2110-2170, variable raster
		{ 7,  "b7",  2625.0, 2685.0,  5.0 },   // 2620-2690, variable raster
	};
	return t;
}

inline const sweep_band_row *sweep_band_by_key(const std::string &key) {
	for(const auto &r : sweep_band_table()) {
		if(key == r.key) {
			return &r;
		}
	}
	return nullptr;
}

inline const sweep_band_row *sweep_band_by_id(int band) {
	for(const auto &r : sweep_band_table()) {
		if(band == r.band) {
			return &r;
		}
	}
	return nullptr;
}

// Candidate probes for a set of band ids (empty = the whole plan), in table order, ascending
// within each band. Levels are unmeasured (the job fills them in stage 1).
inline std::vector<sweep_probe> sweep_candidates(const std::vector<int> &bands) {
	std::vector<sweep_probe> out;
	for(const auto &r : sweep_band_table()) {
		bool want = bands.empty();
		for(int b : bands) {
			if(b == r.band) {
				want = true;
			}
		}
		if(!want) {
			continue;
		}
		for(double f = r.first_mhz; f <= r.last_mhz + 1e-6; f += r.step_mhz) {
			sweep_probe p;
			p.freq_mhz = f;
			p.band = r.band;
			out.push_back(p);
		}
	}
	return out;
}

// ---- stage-2 selection ---------------------------------------------------------------------
// Which probes earn a full decode. Rules, in order:
//   1. failed captures never qualify (an unmeasured frequency is not evidence of anything);
//   2. level >= thresh_db;
//   3. on 5 MHz grids only: non-maxima against same-band neighbours are suppressed, because a
//      10 MHz carrier lights 2-3 adjacent probes and only the centre one decodes. B20's 10 MHz
//      spacing means every block centre is its own carrier - NO suppression there (the real
//      measurement found independent cells on all three blocks, none may shadow another);
//   4. strongest first (the interesting carriers come back soonest), capped at max_picks.
// Plateau tie on a grid: the earlier probe survives (>= backward, > forward).
// Returns indices into `probes`, in decode order.
inline std::vector<size_t> sweep_select(const std::vector<sweep_probe> &probes,
                                        double thresh_db, size_t max_picks) {
	std::vector<size_t> sel;
	for(size_t i = 0; i < probes.size(); i++) {
		const sweep_probe &p = probes[i];
		if(!p.ok || p.level_db < thresh_db) {
			continue;
		}
		const sweep_band_row *row = sweep_band_by_id(p.band);
		bool grid = row && row->step_mhz <= 5.0 + 1e-9;
		if(grid) {
			bool prev_same = i > 0 && probes[i - 1].band == p.band;
			bool next_same = i + 1 < probes.size() && probes[i + 1].band == p.band;
			if(prev_same && probes[i - 1].ok && probes[i - 1].level_db >= p.level_db) {
				continue;   // rising edge or plateau tail of a neighbour's carrier
			}
			if(next_same && probes[i + 1].ok && probes[i + 1].level_db > p.level_db) {
				continue;   // falling edge into a stronger neighbour
			}
		}
		sel.push_back(i);
	}
	std::stable_sort(sel.begin(), sel.end(), [&](size_t a, size_t b) {
		return probes[a].level_db > probes[b].level_db;
	});
	if(sel.size() > max_picks) {
		sel.resize(max_picks);
	}
	return sel;
}

// ---- the antenna policy --------------------------------------------------------------------
// One antenna cannot cover 800 MHz to 2.7 GHz: the SMA-A path carries an ~800 MHz high-pass
// (README "Operating discipline", cellsyncd's cellsync.conf) and the embedded antenna is the
// low-band path. MEASURED on one bench board (Hamburg, gain 12, 8 s deep dwell, fresh boot per run):
//   embed: 796 MHz PCI 439 SIB1 DECODED, 816 MHz PCI 274 SIB1 DECODED, 2145 + 2665 no_signal
//   sma_a: 796 MHz mib_only (never decoded), 816 MHz pss_only, 2145 MHz PCI 190 SIB1 DECODED
// - the same filter seen from both sides. So the path is chosen per CARRIER, at a crossover
// that no band straddles: 1000 MHz sits above B8 (925-960) and below B3 (1805-1880).
constexpr double ANTENNA_CROSSOVER_MHZ = 1000.0;
constexpr const char *ANTENNA_LOW = "embed";    // below the crossover: B20/B8
constexpr const char *ANTENNA_HIGH = "sma_a";   // above it: B3/B1/B7

// The policy one job runs under. `pin` non-empty is an explicit operator choice (--antenna, or
// "antenna" in the request body): it holds for the WHOLE job and the crossover is never
// consulted - exactly the behaviour before the policy existed.
struct antenna_policy {
	std::string pin;
	std::string low = ANTENNA_LOW;
	std::string high = ANTENNA_HIGH;
	double crossover_mhz = ANTENNA_CROSSOVER_MHZ;

	const std::string &at(double freq_mhz) const {
		if(!pin.empty()) {
			return pin;
		}
		return freq_mhz < crossover_mhz ? low : high;
	}
};

// Group a decode list by antenna so a job changes the RF path at most ONCE. The change rides
// the same apply as the retune the carrier pays anyway (capture_rfnm.cpp), but it still costs
// the 0.35 s settle discard, so it is worth spending once instead of per carrier. The group of
// the FIRST entry leads, and the order inside each group - sweep_select's strongest-first - is
// preserved. A pinned policy makes this a no-op: one antenna cannot be regrouped.
inline void antenna_group(std::vector<double> &freqs, const antenna_policy &ap) {
	if(freqs.empty()) {
		return;
	}
	const std::string first = ap.at(freqs.front());
	std::stable_partition(freqs.begin(), freqs.end(),
	                      [&](double f) { return ap.at(f) == first; });
}

} // namespace cellscan

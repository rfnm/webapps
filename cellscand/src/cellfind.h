// cellscan: offline cell acquisition on a wideband burst. Runs on the 15.36 MSps CS16 burst
// AFTER capture (never during: no DSP while capturing). The heavy PSS
// search is exactly the sync stage cellsyncd already owns at 1.92 MSps; here it runs at the
// wideband rate because the same burst then feeds the 50-PRB decode without a second capture
//. Uses only production lte_seq /
// lte_wb functions so the DSP conventions stay pinned by cellsyncd's 152 checks.
#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "fft.h"
#include "lte_wb.h"

namespace cellscan {

using cellsync::cf;
using cellsync::fft_plan;
using cellsync::wb_num;

// The detection floors: what a peak has to clear to be REPORTED as a cell at all. They are
// deliberately permissive - a survey that hides a weak cell is worse than one that shows a
// doubtful row - and the price of that is measured: on a noise-only carrier 8 scans produced
// two "cells", a different PCI each time (155/102, earlier 217/20), and on a real carrier a
// phantom PCI appears in most runs (816 MHz gave 248, 288, 422, 2145 MHz gave 191, 284, 420,
// 468, 474). Named here because a threshold nobody can point at is a threshold nobody can
// raise - and because the escalation gate (job.h) is defined RELATIVE to them: what may be
// REPORTED and what may spend 983 MB of RAM on a second look are two different questions.
constexpr double PSS_DETECT_METRIC = 0.05;   // normalised PSS correlation peak, 0..1
constexpr float SSS_DETECT_MARGIN = 1.15f;   // best NID1 correlation over the runner-up

// One cell located by PSS+SSS on the burst.
struct found_cell {
	int pci = -1;
	int nid1 = -1;
	int nid2 = -1;
	int half = 0;            // 0 = PSS in subframe 0, 1 = subframe 5
	int64_t pss_pos = 0;     // first post-CP sample of the PSS symbol
	double pss_metric = 0.0; // normalised PSS correlation peak, 0..1 (>= PSS_DETECT_METRIC)
	double sss_margin = 0.0; // SSS decision margin (>= SSS_DETECT_MARGIN); the second, mostly
	                         // independent quality signal a phantom has to fake as well
	double snr_db = 0.0;     // rough, from the PSS metric (uncalibrated)
};

// Estimate the residual CFO from cyclic-prefix correlation over the first frames anchored on
// `slot0`, derotate `x` in place, and return the measured offset in Hz (the value that WAS
// present - reported per carrier, matching the +14.9 / -805.6 Hz measured).
double remove_cfo(std::vector<cf> &x, const wb_num &g, int64_t slot0);

// Locate the strongest PSS of a given NID2 in [0, search_end): returns the peak sample
// position (PSS symbol start) and writes the normalised metric to *metric. -1 if none.
int64_t pss_search(const std::vector<cf> &x, const wb_num &g, const fft_plan &plan,
                   int nid2, int64_t search_end, double *metric);

// Full acquisition: find a slot anchor, remove the carrier CFO (returned via *cfo_hz), then
// per NID2 locate a PSS above threshold and decode its SSS into a PCI. Returns the cells found
// (deduplicated by PCI). `cancel`, if set mid-search, aborts and returns what was found so far.
std::vector<found_cell> cellfind(std::vector<cf> &x, const wb_num &g, const fft_plan &plan,
                                 double *cfo_hz, std::atomic<bool> *cancel);

// Stage-1 sweep metric: in-band elevation over the out-of-band noise floor in dB, from
// averaged FFTs over the probe capture (fs = sample rate of x). The DC region (|f| < 150 kHz,
// bandplan.h) is EXCLUDED from the in-band mean - the DC spike measured instead of the carrier
// is an expensively-learned trap - and the floor is the QUIETER of the two out-of-band
// side windows, so a carrier half a grid step off centre cannot poison its own reference.
// ~0 dB = empty air; a real carrier reads > 10 dB. Pinned against synthetic spectra in
// --selftest, including the DC trap.
double sweep_probe_level(const std::vector<cf> &x, const fft_plan &plan, double fs);

} // namespace cellscan

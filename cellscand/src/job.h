// cellscan: the scan job - the state machine the task asked for. A scan is a ONE-SHOT auftrag,
// not a daemon mode: POST /scan starts a worker that walks the frequency list (open the burst
// source once, retune per carrier), GET /status reads progress + results, POST /cancel aborts.
// The daemon holds no radio lease at rest; only a running job does. Jobs are strictly
// serialised (one worker at a time). The finished document survives in memory and is persisted
// so appd's routine idle-stop does not lose "scan, walk away, look later".
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "bandplan.h"
#include "burst.h"
#include "scan.h"

namespace cellscan {

struct job_params {
	std::vector<double> freqs_mhz;
	int dwell_s = 2;         // 2 or 8
	bool deep = true;        // allow the single 2 s -> 8 s escalation
	int gain = 12;
	bool agc = false;
	// which RF path each carrier is received on. Default = the frequency-dependent policy
	// (bandplan.h); antenna.pin set from an explicit --antenna / body field holds for the job.
	antenna_policy antenna;
	// sweep mode (POST /sweep): freqs_mhz is ignored - stage 1 probes the band plan
	// (bandplan.h) over `bands` (band ids; empty = the whole plan), then stage 2 decodes the
	// selected probes through the unchanged per-carrier path. Same job type as a scan, so
	// /status and /cancel work untouched.
	bool sweep = false;
	std::vector<int> bands;
};

// ---- the 2 s -> 8 s escalation, and what it is allowed to cost ------------------------------
// The escalation is the single most expensive thing this daemon does: an 8 s capture is ONE
// 983 MB contiguous allocation (burst.h) plus ~8 s of radio plus ~4x the offline decode. It
// exists because a weak cell can still be won by a second, independent window of TTIs
//, and it must stay reachable for those cells. What it must NOT be is automatic:
// the detector reports phantom cells at the detection floors (cellfind.h), a sweep runs up to
// SWEEP_MAX_DECODES = 16 stage-2 carriers, and a gate that opens for every bare-threshold
// detection turns a survey into 16 near-GB allocations on a 3.27 GB board with no swap. Three
// bounds, each pinned by --selftest:
//   (c) a detection has to clear a MARGIN above the floors before it is worth 983 MB
//   (b) a job may escalate DEEP_MAX_PER_JOB times, of which DEEP_MAX_PSS_PER_JOB on cells
//       that never produced a MIB
//   (a) and the allocation has to fit the memory budget at the moment it is made (burst.h)

// (c) The escalation floors for a cell with NO MIB - the only evidence such a cell has is the
// two acquisition numbers, so both have to be CLEAR of their detection floor, not just at it:
//   metric 0.10 = 2x PSS_DETECT_METRIC. In the metric's own terms (m ~ S/(S+N) at the peak)
//   the floor is -12.8 dB and this is -9.5 dB, i.e. ~3.3 dB of real elevation.
//   margin 1.35 against the 1.15 floor: the winning NID1 must beat the runner-up by 35%, not
//   by 15%. A phantom's "winner" is the luckiest of 168 noise draws, and that distribution
//   piles up just above 1 - which is where the measured phantom PCIs (155, 102, 217, 20 on
//   noise; 248, 288, 422, 191, 284, 420, 468, 474 on real carriers) come from.
// Two gates and not one because they are near-independent: a noise peak that happens to look
// like a PSS still has to draw a lucky SSS out of 168 hypotheses.
constexpr double DEEP_PSS_METRIC = 0.10;
constexpr double DEEP_SSS_MARGIN = 1.35;

// (b) How many escalations one job may spend, and how many of those may go to carriers that
// never produced a MIB. 4, because a 16-carrier sweep is already ~5 min of wall time (16 x
// (0.35 s settle + 2 s dwell + ~16 s decode), bandplan.h) and ONE escalation adds 8 s of
// capture plus ~4x the decode, i.e. ~72 s: four of them roughly double the sweep, and past
// that a survey stops being a survey. The sub-cap of 2 is what makes the budget go to the most
// promising carriers instead of to whoever came first: a carrier holding a VERIFIED MIB whose
// SIB1 did not close is evidence of a real cell, a bare PSS/SSS detection is a hypothesis, and
// with phantoms appearing on most carriers the hypotheses would otherwise consume the whole
// budget before the first real cell is reached. Sweeps visit carriers strongest-first
// (bandplan.h), so within a class the budget already goes to the stronger signal.
constexpr int DEEP_MAX_PER_JOB = 4;
constexpr int DEEP_MAX_PSS_PER_JOB = 2;

// How much promise a carrier's unfinished business carries. Ordered: a bigger class is a
// better reason to spend the 983 MB.
enum deep_class {
	DEEP_NONE = 0,   // nothing unfinished, or nothing a longer capture could change
	DEEP_WEAK = 1,   // unfinished, but the only evidence is a bare-threshold detection
	DEEP_PSS = 2,    // a detection clear of both floors, but no MIB yet
	DEEP_MIB = 3,    // a VERIFIED MIB at 50 PRB whose SIB1 did not close - the real cell
};

// The per-cell classification. A 50 PRB cell whose SIB1 never closed, or a cell
// whose MIB never verified at all, can still be finished by a longer capture: pss_only is not
// "no cell here", it is "not enough energy yet", and now that the PBCH combines over a 40 ms
// TTI the longer capture is a second independent window of TTIs. A cell whose MIB says the
// carrier is wider than this numerology can take (nrb != 50) is finished, not stuck, and must
// not spend 8 s of radio on being told so a second time.
//
// The MIB branch is EXACTLY what it was: a cell that verified its MIB escalates on the same
// terms as before this budget existed. Only the no-MIB branch is now conditional, and only on
// the quality of the detection that produced it.
inline deep_class cell_deep_class(const cell_result &c) {
	if(c.sib1_ok) {
		return DEEP_NONE;
	}
	if(c.mib_ok) {
		return c.nrb == 50 ? DEEP_MIB : DEEP_NONE;
	}
	return (c.pss_metric >= DEEP_PSS_METRIC && c.sss_margin >= DEEP_SSS_MARGIN) ? DEEP_PSS
	                                                                            : DEEP_WEAK;
}

// A carrier carries the promise of its most promising cell - so one with NO cells is
// DEEP_NONE, which is what keeps a sweep over empty spectrum cheap.
inline deep_class carrier_deep_class(const carrier_result &cr) {
	deep_class best = DEEP_NONE;
	for(const auto &c : cr.cells) {
		deep_class k = cell_deep_class(c);
		if(k > best) {
			best = k;
		}
	}
	return best;
}

// The predicates the worker used to ask, in the terms it used to ask them. Unchanged for
// MIB-bearing cells; for a cell with no MIB "needs deep" now means "clear of the floors"
// rather than "detected at all".
inline bool cell_needs_deep(const cell_result &c) { return cell_deep_class(c) >= DEEP_PSS; }
inline bool carrier_needs_deep(const carrier_result &cr) {
	return carrier_deep_class(cr) >= DEEP_PSS;
}

// One job's escalation budget. `admit` IS the decision: it books an escalation and returns
// nullptr, or refuses and returns the reason - which is exactly the string that lands in
// carrier_result::deep_skip and in the log line. Pure and header-inline so --selftest can walk
// a simulated 16-carrier sweep through it without a radio.
struct deep_budget {
	int used = 0;       // escalations booked
	int used_pss = 0;   // of those, the ones spent on carriers with no MIB

	// `k` is the carrier's class; `dwell_s` the capture being asked for. A DEEP_NONE carrier
	// is never offered to the budget (it wants nothing), and asking anyway refuses as "weak".
	const char *admit(deep_class k, double dwell_s) {
		if(k <= DEEP_WEAK) {
			return "weak";
		}
		if(used >= DEEP_MAX_PER_JOB) {
			return "job_cap";
		}
		if(k == DEEP_PSS && used_pss >= DEEP_MAX_PSS_PER_JOB) {
			return "job_cap";
		}
		if(!capture_fits(dwell_s, nullptr, nullptr)) {
			return "memory";   // last, and at the moment of the allocation: it is the only
			                   // one of the three that can change under us mid-job
		}
		used++;
		if(k == DEEP_PSS) {
			used_pss++;
		}
		return nullptr;
	}
};

class scanner {
public:
	// `persist_path` empty disables persistence (selftest / --file one-shots may set it).
	explicit scanner(std::string persist_path);
	virtual ~scanner();   // virtual: decode_burst below is the --selftest seam

	// The burst-source factory decouples the state machine from hardware: main injects a
	// board rx_session factory (WITH_RFNM) or a file replay (--file / tests).
	void set_source_factory(std::function<std::unique_ptr<burst_source>()> f) { factory_ = std::move(f); }

	// Load a persisted last scan into the current document, if one exists.
	bool load_persist();

	// Start a scan. Returns the new job id, or 0 if one is already running (caller -> 409) or
	// the params are invalid (caller -> 400, reason in `err`).
	uint64_t start(const job_params &p, std::string &err);

	// Request cancellation of the running job (idempotent; no-op when idle).
	void cancel();

	bool running() const { return running_.load(); }
	uint64_t last_job() const { return job_seq_.load(); }

	// Snapshot the current scan document (thread-safe copy).
	scan_doc snapshot() const;

	bool persist_loaded() const { return persist_loaded_; }
	int64_t persist_saved_at() const { return persist_saved_at_; }

protected:
	// The one seam --selftest needs. Everything the escalation budget decides is decided ON a
	// carrier_result, and the only way to produce a real one is to receive a real cell: a
	// hardware-free test would have to synthesise a 245 MB LTE burst per case. Overriding
	// this instead lets a test supply the carrier_result the decoder WOULD have produced and
	// leaves every line after it - the classification, the budget, the buffer handling, the
	// document - as the shipped code. The production path never overrides it.
	virtual carrier_result decode_burst(std::vector<cf> &x, double freq_mhz, int dwell,
	                                    std::atomic<bool> &cancel);

private:
	void run(job_params p, uint64_t id);
	void set_phase(int index, int total, double freq_mhz, const char *phase, int pct);

	std::string persist_path_;
	std::function<std::unique_ptr<burst_source>()> factory_;

	mutable std::mutex mtx_;
	scan_doc doc_;

	std::thread worker_;
	std::atomic<bool> running_{ false };
	std::atomic<bool> cancel_{ false };
	std::atomic<uint64_t> job_seq_{ 0 };

	bool persist_loaded_ = false;
	int64_t persist_saved_at_ = 0;
};

} // namespace cellscan

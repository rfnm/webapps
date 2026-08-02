// cellsync: acquisition + per-cell tracking on the 1.92 MSps sync stream.
// The engine owns a common derotation NCO (bulk CFO = our reference error, shared by
// every cell); per-cell trackers measure fine CFO from CRS cross-slot phase and an
// independent timing-drift slope from PSS peak positions. The two must agree:
// cfo/f_lo = -timing_slope (both are views of the same reference error).
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "fft.h"
#include "lte_seq.h"

namespace cellsync {

struct cell_meas {
	int pci = -1;
	bool wide = false;          // spectrum occupied beyond the center 6 RB (so >=3 MHz cell);
	                            // N_RB itself is NOT observable from the sync window - 36.211
	                            // makes the CRS m' labels bandwidth-invariant by design
	float snr_db = 0;
	double cfo_hz = 0;          // total apparent CFO (engine coarse + cell fine)
	double offset_ppb = 0;      // = -cfo_hz / f_lo * 1e9  (positive = our clock fast)
	double tim_ppb = 0;         // independent estimate from PSS timing drift
	bool tim_valid = false;
	double weight = 0;          // for the steering median (SNR- and depth-derived)
	double age_s = 0;           // since last good PSS hit
	uint64_t slots = 0;         // CRS slot-pairs accumulated into cfo
};

class cell_tracker;

class sync_engine {
public:
	sync_engine(double f_lo_hz, bool verbose = false);
	~sync_engine();

	void feed(const cf *x, size_t n);            // 1.92 MSps stream (any chunking)

	std::vector<cell_meas> cells();              // thread-safe snapshot
	// consume-and-reset: per-cell measurements accumulated since the last call
	std::vector<cell_meas> take_measurements();
	// timing-drift hygiene: call after any DCO actuation big enough to bend the PSS
	// drift slope. Trackers restart their slope-regression window so tim_ppb never
	// averages across an actuation step - a step mid-window dilutes the measured
	// authority (a reading ~80% post-step reports 0.8x of the true pull).
	void reset_drift() { drift_epoch_.fetch_add(1, std::memory_order_relaxed); }
	// stream discontinuity too large to zero-fill through: every absolute-position
	// anchor (acquire grids, tracker PSS predictions) is false from here on - drop
	// them. A still-present cell re-confirms within seconds. Feed-thread only.
	void flush_acquire();
	// full retune (auto-acquire fallback): flush + empty the ring + new f_lo.
	// Lifetime counters survive so /status stays monotonic. Feed-thread only.
	void reset(double f_lo_hz);
	double coarse_cfo() const { return coarse_cfo_; }
	bool tracking() const { return tracking_; }
	uint64_t samples() const { return total_in_; }
	uint64_t acq_attempts() const { return acq_attempts_; }

	// acquisition tuning (defaults chosen by the selftest + a real B13 bench capture)
	float pss_thr = 0.022f;                      // per-peak normalized |corr|^2 threshold
	float sss_accept = 0.004f;                   // min accumulated-SSS EXCESS over the class noise
	                                             // floor (the median of the 336-hypothesis row)
	float sss_margin = 1.30f;                    // excess ratio vs m-family-disjoint runner-up
	int min_train = 3;                           // PSS occurrences to call it a train
	int min_occ = 12;                            // accumulated occurrences before a weak accept
	int max_cells = 4;

private:
	// weak-signal cell search: correlation peaks are clustered into 5 ms-periodic trains
	// per detection variant, then each (nid2, variant, residue) class accumulates
	// PSS-equalized SSS metrics ACROSS acquire windows until the PCI decision separates -
	// at deep-indoor SNR a single 40 ms window is a coin toss, seconds are not
	struct train {
		int nid2;
		double fg;                                // detection variant frequency (Hz)
		double score;
		std::vector<uint64_t> occ;                // member positions, ascending
	};
	static constexpr int ACQ_NF = 7;              // CFO hypotheses per class: fg + (m-3)*2500 Hz
	static constexpr int ACQ_REF = 480;           // alignment reach around the anchor (samples):
	                                              // detection positions ride ZC ambiguity ridges
	                                              // and can sit hundreds of samples off the cell
	static constexpr int ACQ_NPROF = 2 * ACQ_REF + 1;
	struct acq_class {
		int nid2 = 0;
		double fg = 0;
		uint64_t anchor = 0;                      // first-ever occurrence: residue + parity reference
		uint64_t refpos = 0;                      // grid origin (fixed at anchor)
		uint64_t folded_until = 0;                // grid positions <= this are already folded
		uint64_t last_train = 0;                  // total_in_ at last train sighting (aging)
		int trains_seen = 0;                      // rounds with a matching train (>= 2 to fold)
		bool dead = false;                        // undecidable or duplicate of a tracked cell
		std::vector<float> prof;                  // ACQ_NF x ACQ_NPROF noncoherent |corr|^2 profile
		std::vector<double> comb;                 // ACQ_NF x 336 accumulated SSS metrics
		int fused[ACQ_NF] = { 0 };                // occurrences folded per hypothesis
		double lastp[ACQ_NF] = { 0 };             // newest aligned position per hypothesis
		int64_t lastj[ACQ_NF] = { 0 };            // its half-frame index relative to anchor
	};
	void acquire();
	void augment_scan();                          // add same-carrier cells while tracking
	void process_half_frames();
	void collect_trains(uint64_t w0, size_t wlen, double fg, std::vector<train> &out);
	size_t find_class(const train &t) const;      // matching acq_ index, or acq_.size()
	bool pci_blacklisted(int pci);                // CRS-rejected recently?
	void touch_class(const train &t);             // create/refresh the class for this train
	bool fold_class(size_t ci, bool quiet);       // fold its 5 ms grid, accept when solid
	void select_classes(std::vector<train> &trains);
	void fold_all(bool quiet);
	void fold_fine_into_coarse();

	double f_lo_;
	bool verbose_;
	fft_plan fft512_;
	fft_plan fft128_;
	std::vector<std::vector<cf>> pss_t_;          // 3 time templates
	std::vector<std::vector<cf>> pss_f512_;       // conj FFT-512 of templates

	std::vector<cf> ring_;
	uint64_t base_ = 0;                           // abs index of ring_[0]
	uint64_t total_in_ = 0;
	uint64_t proc_ = 0;                           // abs index: everything before is done
	double coarse_cfo_ = 0;                       // Hz, applied by the feed() NCO
	double nco_phase_ = 0;
	double nco_step_cached_ = 0;                  // feed() phasor state (see feed())
	cf nco_cur_ = cf(1.0f, 0.0f);
	cf nco_stepper_ = cf(1.0f, 0.0f);
	int nco_k_ = 0;
	bool tracking_ = false;
	uint64_t next_acq_ = 0;
	uint64_t next_augment_ = 0;
	uint64_t acq_attempts_ = 0;
	uint64_t last_fold_in_ = 0;
	std::vector<acq_class> acq_;                  // persistent acquisition accumulators
	std::vector<std::pair<int, uint64_t>> pci_bl_;   // CRS-rejected PCIs (5 min expiry)

	std::vector<std::unique_ptr<cell_tracker>> cells_;
	std::mutex meas_mtx_;                         // protects snapshots below
	std::vector<cell_meas> snapshot_;
	std::atomic<uint64_t> drift_epoch_{0};        // bumped by reset_drift(); trackers compare

	// correlation helper: normalized metric of ALL THREE PSS templates over ring
	// window [abs0, abs0+n) -> c[nid2][i] for lag abs0+i. Fused on purpose: the
	// forward FFT and the power prefix are window-properties, identical for every
	// template - the per-nid2 form recomputed both 3x (a third of the old acquire
	// round wall time).
	void xcorr_win3(uint64_t abs0, size_t n, std::vector<float> c[3]);
	const cf *at(uint64_t abs) const { return &ring_[(size_t)(abs - base_)]; }
	uint64_t avail() const { return base_ + ring_.size(); }
	void trim();

	friend class cell_tracker;
};

} // namespace cellsync

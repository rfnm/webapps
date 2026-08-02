// cellsync: Si5510 DCO steering. The actuator is an absolute step register:
// VARIABLE_OFFSET_DCO(MR, steps), one step = 0.100000039438 ppb (plan property, see
// r/si5510/RE/SI5510-HANDOFF.md 3.1), range +-100000 steps (+-10 ppm), 0 = nominal.
// Kernel sysfs (2026-07-07): rfnm_set_dco_steps = raw steps, what this loop writes
// (0.1 ppb resolution); rfnm_set_dco_ppb = human-facing integer ppb (do not use here).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cellsync {

struct dco_ctl {
	double step_ppb = 0.100000039438;
	int32_t range_steps = 20000;     // SERVO AUTHORITY, not the chip limit: the Si5510 DCO
	                                 // reaches +-100000 (+-10 ppm), but factory crystal offsets
	                                 // are 1-2 ppm at worst - a demand beyond +-2 ppm smells like
	                                 // a bad reference/measurement, so rail here instead of
	                                 // chasing it (--dco-range-ppm widens up to 10)
	int sign = 1;                    // +1: positive steps raise the output frequency
	bool mock = true;
	std::string path;
	int32_t steps = 0;               // last written value (driver exposes no readout)
	uint64_t writes = 0;

	// resolve the sysfs attribute: explicit override, else probe rfnm_set_dco_steps
	// (never the ppb knob - it no longer takes steps). Returns false -> stay mock.
	bool open(const std::string &override_path);
	bool write(int32_t s);           // clamps to range, no-ops on unchanged value
	double applied_ppb() const { return steps * step_ppb * sign; }
};

// weighted median of (value, weight) - the cell combiner (robust to one bad cell)
double weighted_median(std::vector<std::pair<double, double>> vw);

class steer_loop {
public:
	enum state_t { WARMUP, SIGNPROBE, TRACK, HOLDOVER };

	struct config {
		double tau_s = 120;          // closed-loop time constant
		double period_s = 10;        // update cadence (informational; caller times it)
		double max_step_ppb = 2000;  // per-update slew limit
		bool sign_probe = true;      // verify actuator sign at first lock (a wrong sign
		                             // diverges; the probe makes the loop self-healing
		                             // against observable-frame inversions)
		int32_t probe_steps = 1000;  // +100 ppb, well above measurement noise
	};

	steer_loop(dco_ctl *dco, config cfg) : dco_(dco), cfg_(cfg) {}

	// one tick: meas_ppb = combined cell offset (positive = our clock fast),
	// valid = false when no usable cells this period
	void update(double meas_ppb, bool valid);

	state_t state() const { return state_; }
	const char *state_name() const;
	double residual_ppb() const { return resid_; }
	double sigma_ppb() const { return sigma_; }
	uint64_t updates() const { return updates_; }
	uint64_t holdovers() const { return holdovers_; }

private:
	dco_ctl *dco_;
	config cfg_;
	state_t state_ = WARMUP;
	int good_ = 0;
	int misses_ = 0;
	uint64_t updates_ = 0;
	uint64_t holdovers_ = 0;
	double resid_ = 0;
	double sigma_ = 0;

	// sign probe bookkeeping
	int probe_phase_ = 0;            // 0 baseline, 1 stepped
	int probe_n_ = 0;
	double probe_acc_ = 0;
	double probe_base_ = 0;
	int32_t probe_saved_ = 0;
};

} // namespace cellsync

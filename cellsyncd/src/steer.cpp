// cellsync: DCO steering loop (single integrator, GPSDO-style) + sysfs actuator.
#include "steer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace cellsync {

bool dco_ctl::open(const std::string &override_path) {
	std::vector<std::string> cand;
	if(!override_path.empty()) {
		cand.push_back(override_path);
	} else {
		// rfnm_set_dco_steps = raw absolute NUM_STEPS (0.1 ppb resolution). Do NOT fall back
		// to rfnm_set_dco_ppb: since 2026-07-07 that knob takes REAL integer ppb, so writing
		// steps there would command a ~10x pull. Kernel + cellsyncd deploy together.
		// Live-verified device path on hardware: i2c-0 0x58.
		cand.push_back("/sys/bus/i2c/devices/0-0058/rfnm_set_dco_steps");
		cand.push_back("/sys/devices/platform/soc@0/30800000.bus/30a20000.i2c/i2c-0/0-0058/rfnm_set_dco_steps");
	}
	for(const auto &p : cand) {
		std::ifstream probe(p);
		std::ofstream wprobe(p, std::ios::app);
		if(probe.good() || wprobe.good()) {
			path = p;
			mock = false;
			printf("cellsync: dco actuator %s\n", p.c_str());
			return true;
		}
	}
	printf("cellsync: no dco sysfs found - running with a mock actuator (measure-only)\n");
	return false;
}

bool dco_ctl::write(int32_t s) {
	s = std::max(-range_steps, std::min(range_steps, s));
	if(s == steps && writes > 0) {
		return true;
	}
	steps = s;
	writes++;
	if(mock) {
		return true;
	}
	std::ofstream f(path);
	if(!f) {
		return false;
	}
	f << s << "\n";
	return f.good();
}

double weighted_median(std::vector<std::pair<double, double>> vw) {
	if(vw.empty()) {
		return 0;
	}
	std::sort(vw.begin(), vw.end());
	double total = 0;
	for(const auto &p : vw) {
		total += p.second;
	}
	double acc = 0;
	for(const auto &p : vw) {
		acc += p.second;
		if(acc >= total / 2) {
			return p.first;
		}
	}
	return vw.back().first;
}

const char *steer_loop::state_name() const {
	switch(state_) {
	case WARMUP: return "warmup";
	case SIGNPROBE: return "signprobe";
	case TRACK: return "track";
	default: return "holdover";
	}
}

void steer_loop::update(double meas_ppb, bool valid) {
	updates_++;
	if(valid) {
		resid_ = 0.8 * resid_ + 0.2 * meas_ppb;
		sigma_ = 0.9 * sigma_ + 0.1 * std::fabs(meas_ppb - resid_);
	}

	switch(state_) {
	case WARMUP:
		if(!valid) {
			good_ = 0;
			return;
		}
		if(++good_ >= 3) {
			state_ = cfg_.sign_probe ? SIGNPROBE : TRACK;
			probe_phase_ = 0;
			probe_n_ = 0;
			probe_acc_ = 0;
		}
		return;

	case SIGNPROBE:
		if(!valid) {
			return;
		}
		probe_acc_ += meas_ppb;
		if(++probe_n_ < 4) {
			return;
		}
		if(probe_phase_ == 0) {
			probe_base_ = probe_acc_ / probe_n_;
			probe_saved_ = dco_->steps;
			dco_->write(dco_->steps + cfg_.probe_steps);
			probe_phase_ = 1;
			probe_n_ = 0;
			probe_acc_ = 0;
			return;
		}
		{
			// stepping the DCO +N raises our clock by N*step*sign_true; measured
			// offset (= our fractional error) must move the same way
			double delta = probe_acc_ / probe_n_ - probe_base_;
			double expect = cfg_.probe_steps * dco_->step_ppb * dco_->sign;
			if(delta * expect < 0) {
				dco_->sign = -dco_->sign;
				printf("cellsync: dco sign probe: response %+.1f ppb for %+.1f expected - sign flipped to %+d\n",
				       delta, expect, dco_->sign);
			} else {
				printf("cellsync: dco sign probe ok (%+.1f ppb for %+.1f expected)\n", delta, expect);
			}
			fflush(stdout);
			dco_->write(probe_saved_);
			state_ = TRACK;
			misses_ = 0;
		}
		return;

	case TRACK:
		if(!valid) {
			if(++misses_ >= 6) {
				state_ = HOLDOVER;
				holdovers_++;
				good_ = 0;
			}
			return;
		}
		misses_ = 0;
		{
			double g = std::min(0.5, cfg_.period_s / cfg_.tau_s);
			double delta_ppb = std::max(-cfg_.max_step_ppb, std::min(cfg_.max_step_ppb, g * meas_ppb));
			int32_t dsteps = (int32_t)llround(delta_ppb / (dco_->step_ppb * dco_->sign));
			dco_->write(dco_->steps - dsteps);
		}
		return;

	default:   // HOLDOVER: freeze the register, wait for the signal to come back
		if(!valid) {
			good_ = 0;
			return;
		}
		if(++good_ >= 3) {
			state_ = TRACK;
			misses_ = 0;
		}
		return;
	}
}

} // namespace cellsync

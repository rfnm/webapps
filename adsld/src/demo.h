// adsld: --demo synthetic traffic (any radio backend, incl. the host mock). Injects a
// small fleet of plausible aircraft straight into the traffic table at ~1 Hz so the whole
// web stack - /traffic JSON, map icons, trails, ageing - is testable without RF hardware.
// Pure logic on the radio clock; nothing touches the receive chain.
#pragma once

#include <cstdint>
#include <vector>

#include "adsl_codec.h"
#include "traffic.h"

namespace adsld {

class demo_traffic {
public:
	// center: where the fleet orbits (the daemon passes --lat/--lon, else a default)
	demo_traffic(double center_lat, double center_lon, uint32_t seed);

	// call from the main loop; updates + injects at most once per second of now_ms.
	void tick(traffic_table &table, uint64_t now_ms);

private:
	struct craft {
		uint8_t amt, cat, fs, emg;
		uint32_t addr;
		double lat, lon;         // deg
		float alt_m;
		float speed_mps;
		float track_deg;
		float turn_dps;          // deg/s (gliders circle, others wander)
		float vrate_mps;
		uint64_t hidden_until;   // ms; > now = silent (disappear/reappear)
	};

	uint32_t rnd();              // xorshift32
	double rndf(double lo, double hi);
	craft make_craft(int i);     // the i-th synthetic aircraft, deterministic per seed

	double clat_, clon_;
	uint32_t rng_;
	uint64_t last_ms_ = 0;
	std::vector<craft> fleet_;
};

} // namespace adsld

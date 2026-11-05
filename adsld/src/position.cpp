// adsld: position sources - see position.h.
#include "position.h"

namespace adsld {

static_source::static_source(double lat_deg, double lon_deg, float alt_m)
	: lat_deg_(lat_deg), lon_deg_(lon_deg), alt_m_(alt_m) {
}

bool static_source::get(uint64_t now_ms, position_fix &out) {
	out = position_fix();
	out.valid = true;
	out.lat_deg = lat_deg_;
	out.lon_deg = lon_deg_;
	out.alt_m = alt_m_;
	out.kinematics_valid = false;
	out.flightstate = 1;  // ground
	out.time_ms = now_ms; // config values are always "fresh"
	// Timestamp field: quarter-seconds mod 60. Without GNSS time the scheduler clock is
	// the best (and, under the mock's fake clock, deterministic) approximation of UTC
	// phase; a real GNSS source overrides this with true UTC quarter-seconds.
	out.ts_qs = (uint8_t)((now_ms / 250) % 60);
	return true;
}

} // namespace adsld

// adsld: own-ship position source. The scheduler pulls a fix on every transmission; the
// <= 500 ms GNSS-to-air latency budget (ADSL_GNSS_MAX_AGE_MS, enforced by the scheduler)
// means sources should return their freshest data and stamp it honestly. static_source
// serves fixed config values (--lat/--lon/--alt) for bench/ground-station use; a GNSS
// source slots in behind the same interface later.
#pragma once

#include <cstdint>

namespace adsld {

struct position_fix {
	bool valid = false;      // false = no fix (encodes as lat/lon 0x800000 on air)
	double lat_deg = 0;      // WGS-84
	double lon_deg = 0;
	float alt_m = 0;         // ellipsoidal altitude, metres
	float speed_mps = 0;     // ground speed
	float vrate_mps = 0;     // vertical rate, + = climbing
	float track_deg = 0;     // course over ground, 0..360
	bool kinematics_valid = false;  // speed/vrate/track usable (static_source: false)
	uint8_t flightstate = 0; // FlightState code: 0=unknown 1=ground 2=airborne
	uint64_t time_ms = 0;    // radio::now_ms() timestamp of the fix
	uint8_t ts_qs = 0;       // Timestamp field: quarter-seconds of the UTC minute mod 60
};

class position_source {
public:
	virtual ~position_source() = default;
	// Fill `out` with the freshest fix; now_ms is the scheduler's clock (the source
	// stamps out.time_ms with the fix age against that clock). Returns false when no
	// data at all is available (out.valid may also be false on a stale/lost fix).
	virtual bool get(uint64_t now_ms, position_fix &out) = 0;
};

// Fixed position from config values; always-fresh, flightstate = ground, no kinematics.
class static_source : public position_source {
public:
	static_source(double lat_deg, double lon_deg, float alt_m);
	bool get(uint64_t now_ms, position_fix &out) override;

private:
	double lat_deg_;
	double lon_deg_;
	float alt_m_;
};

} // namespace adsld

// adsld: --demo synthetic traffic - see demo.h.
#include "demo.h"

#include <cmath>

#include "receiver.h"

namespace adsld {

static constexpr double DEG_PER_M_LAT = 1.0 / 111320.0;

uint32_t demo_traffic::rnd() {
	rng_ ^= rng_ << 13;
	rng_ ^= rng_ >> 17;
	rng_ ^= rng_ << 5;
	return rng_;
}

double demo_traffic::rndf(double lo, double hi) {
	return lo + (hi - lo) * ((double)(rnd() & 0xFFFFFF) / (double)0xFFFFFF);
}

// one of each interesting category (web UI icon coverage), AMT 0 = random/privacy
demo_traffic::craft demo_traffic::make_craft(int i) {
	struct proto {
		uint8_t cat;
		float alt, speed, vrate, turn;
	};
	static const proto protos[] = {
		{ 1, 900, 46, 1.0f, 1.5f },     // light fixed-wing, cruising
		{ 2, 3200, 120, 4.0f, 0.5f },   // heavy fixed-wing, climbing out
		{ 4, 1400, 26, -0.8f, 6.0f },   // glider, circling in (weak) lift
		{ 7, 750, 11, -1.2f, 4.0f },    // paraglider, slow arcs, sinking
		{ 10, 450, 30, 0.0f, 2.0f },    // gyrocopter
		{ 12, 120, 16, 0.0f, 8.0f },    // UAS, tight pattern
		{ 5, 600, 4, 0.5f, 0.2f },      // balloon drifting with the wind
		{ 3, 300, 55, 0.0f, 1.0f },     // helicopter
	};
	const proto &p = protos[i % (int)(sizeof(protos) / sizeof(protos[0]))];
	craft c;
	c.amt = 0;
	c.addr = 0xDE0000u | (uint32_t)(i + 1);   // demo namespace inside AMT 0
	c.cat = p.cat;
	c.fs = 2;                                  // airborne
	c.emg = (i == 1) ? 4 : 0;                  // one aircraft squawks an emergency
	c.lat = clat_ + rndf(-0.05, 0.05);
	c.lon = clon_ + rndf(-0.08, 0.08);
	c.alt_m = p.alt * (float)rndf(0.8, 1.2);
	c.speed_mps = p.speed * (float)rndf(0.85, 1.15);
	c.track_deg = (float)rndf(0, 360);
	c.turn_dps = p.turn * (rnd() & 1 ? 1.0f : -1.0f);
	c.vrate_mps = p.vrate;
	c.hidden_until = 0;
	return c;
}

demo_traffic::demo_traffic(double center_lat, double center_lon, uint32_t seed)
	: clat_(center_lat), clon_(center_lon), rng_(seed ? seed : 0xAD51DE30u) {
	int n = 6 + (int)(rnd() % 3);   // 6..8 aircraft
	for(int i = 0; i < n; i++) {
		fleet_.push_back(make_craft(i));
	}
	if(!fleet_.empty()) {
		fleet_.back().fs = 1;    // one on the ground (muted icon), parked near the center
		fleet_.back().speed_mps = 0;
		fleet_.back().vrate_mps = 0;
		fleet_.back().alt_m = 40;
		fleet_.back().lat = clat_ + 0.004;
		fleet_.back().lon = clon_ - 0.006;
	}
}

void demo_traffic::tick(traffic_table &table, uint64_t now_ms) {
	if(now_ms - last_ms_ < 1000) {
		return;
	}
	double dt = last_ms_ ? (double)(now_ms - last_ms_) / 1000.0 : 1.0;
	if(dt > 5) {
		dt = 5;
	}
	last_ms_ = now_ms;
	int idx = 0;
	for(auto &c : fleet_) {
		idx++;
		// occasionally fall silent for 10..45 s (out of range / shadowed), then reappear
		if(c.hidden_until > now_ms) {
			continue;
		}
		if((rnd() % 200) == 0) {   // ~1 dropout per craft every ~3 min
			c.hidden_until = now_ms + 10000 + rnd() % 35000;
			continue;
		}
		if(c.fs != 1) {
			c.track_deg += c.turn_dps * (float)dt + (float)rndf(-1.5, 1.5);
			while(c.track_deg < 0) { c.track_deg += 360; }
			while(c.track_deg >= 360) { c.track_deg -= 360; }
			c.speed_mps += (float)rndf(-0.4, 0.4);
			if(c.speed_mps < 2) { c.speed_mps = 2; }
			c.vrate_mps += (float)rndf(-0.3, 0.3);
			if(c.vrate_mps > 5) { c.vrate_mps = 5; }
			if(c.vrate_mps < -4) { c.vrate_mps = -4; }
			c.alt_m += c.vrate_mps * (float)dt;
			if(c.alt_m < 60) { c.alt_m = 60; c.vrate_mps = 1; }
			double rad = c.track_deg * M_PI / 180.0;
			double d = (double)c.speed_mps * dt;
			c.lat += d * std::cos(rad) * DEG_PER_M_LAT;
			c.lon += d * std::sin(rad) * DEG_PER_M_LAT / std::cos(c.lat * M_PI / 180.0);
			// soft leash: beyond ~8 km from center, steer the wanderers back
			if(std::fabs(c.lat - clat_) > 0.07 || std::fabs(c.lon - clon_) > 0.11) {
				double home = std::atan2((clon_ - c.lon) * std::cos(c.lat * M_PI / 180.0),
				                         clat_ - c.lat) * 180.0 / M_PI;
				c.track_deg = (float)(home < 0 ? home + 360 : home);
			}
		}
		adsl_traffic t;
		t.amt = c.amt;
		t.addr = c.addr;
		t.category = c.cat;
		t.flightstate = c.fs;
		t.emergency = c.emg;
		t.ts_qs = (uint8_t)((now_ms / 250) % 60);
		t.pos_valid = true;
		t.lat_deg = c.lat;
		t.lon_deg = c.lon;
		t.alt_valid = true;
		t.alt_m = c.alt_m;
		if(c.fs == 1) {
			// ground vehicle reports position only (exercises the absent-kinematics path)
			t.speed_valid = t.vrate_valid = t.track_valid = false;
		} else {
			t.speed_valid = true;
			t.speed_mps = c.speed_mps;
			t.vrate_valid = true;
			t.vrate_mps = c.vrate_mps;
			t.track_valid = true;
			t.track_deg = c.track_deg;
		}
		adsl_rx_meta m;
		m.channel = (int)((now_ms / 1000 + idx) & 1);
		m.freq_hz = ADSL_CH_HZ[m.channel];
		m.phase = (int)(rnd() % 16);
		m.rssi_db = (float)(-25.0 - (idx * 4) + rndf(-3, 3));
		m.corrected = false;
		table.update(t, m, now_ms);
	}
}

} // namespace adsld

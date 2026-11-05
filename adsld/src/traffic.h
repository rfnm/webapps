// adsld: decoded-traffic table for the web GUI. Keyed by (AMT, address); thread-safe
// (the receiver threads update, the httpd thread dumps JSON). JSON emitted in the
// rfnm_json.h style.
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include "adsl_codec.h"
#include "receiver.h"

namespace adsld {

struct traffic_entry {
	adsl_traffic last;        // most recent decoded report
	adsl_rx_meta meta;        // metadata of the most recent packet
	uint64_t first_ms = 0;    // radio::now_ms() of first sighting
	uint64_t last_ms = 0;     // ... and of the latest
	uint64_t wall_ms = 0;     // unix wall-clock ms of the latest (the JSON "ts" field)
	uint64_t count = 0;       // Traffic packets from this sender
	uint64_t corrected = 0;   // ... of which needed the 1-bit CRC correction
};

class traffic_table {
public:
	static constexpr uint64_t DEFAULT_MAX_AGE_MS = 300000;  // 5 min

	// insert/update the sender's entry
	void update(const adsl_traffic &t, const adsl_rx_meta &m, uint64_t now_ms);

	// drop entries not heard for max_age_ms
	void expire(uint64_t now_ms, uint64_t max_age_ms = DEFAULT_MAX_AGE_MS);

	// JSON array of all entries (newest last_ms first is NOT guaranteed; clients sort):
	// [{"amt":..,"addr":"hex6","lat":..,"lon":..,"alt_m":..,"speed_mps":..,
	//   "vrate_mps":..,"track_deg":..,"cat":..,"fs":..,"emg":..,"rssi":..,"ch":..,
	//   "count":..,"age_s":..,"ts":..}, ...]; fields with false validity flags are
	//   omitted (speed/vrate/track absent when the report marked them unavailable);
	//   "ts" is the last-seen unix wall clock in ms.
	std::string to_json(uint64_t now_ms) const;

	size_t size() const;
	uint64_t total() const;   // packets ever accepted (survives expiry)

private:
	static uint64_t key(uint8_t amt, uint32_t addr) {
		return ((uint64_t)amt << 24) | (addr & 0xFFFFFFu);
	}

	mutable std::mutex mtx_;
	std::map<uint64_t, traffic_entry> map_;
	uint64_t total_ = 0;
};

} // namespace adsld

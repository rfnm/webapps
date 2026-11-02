// adsld: traffic table - see traffic.h.
#include "traffic.h"

#include <chrono>
#include <cstdio>

#include "rfnm_json.h"

namespace adsld {

// Shared field emitter for one decoded report (used by the table below and by main.cpp's
// /rx ring): sender + kinematics in natural units, validity-gated fields omitted.
void traffic_fields_json(std::string &j, const adsl_traffic &t, bool &first) {
	char b[48];
	snprintf(b, sizeof(b), "%u", (unsigned)t.amt);
	rfnm::jkv_raw(j, "amt", b, first);
	snprintf(b, sizeof(b), "%06X", t.addr & 0xFFFFFFu);
	rfnm::jkv(j, "addr", b, first);
	if(t.pos_valid) {
		snprintf(b, sizeof(b), "%.6f", t.lat_deg);
		rfnm::jkv_raw(j, "lat", b, first);
		snprintf(b, sizeof(b), "%.6f", t.lon_deg);
		rfnm::jkv_raw(j, "lon", b, first);
	}
	if(t.alt_valid) {
		snprintf(b, sizeof(b), "%.1f", (double)t.alt_m);
		rfnm::jkv_raw(j, "alt_m", b, first);
	}
	if(t.speed_valid) {
		snprintf(b, sizeof(b), "%.2f", (double)t.speed_mps);
		rfnm::jkv_raw(j, "speed_mps", b, first);
	}
	if(t.vrate_valid) {
		snprintf(b, sizeof(b), "%.2f", (double)t.vrate_mps);
		rfnm::jkv_raw(j, "vrate_mps", b, first);
	}
	if(t.track_valid) {
		snprintf(b, sizeof(b), "%.1f", (double)t.track_deg);
		rfnm::jkv_raw(j, "track_deg", b, first);
	}
	snprintf(b, sizeof(b), "%u", (unsigned)t.category);
	rfnm::jkv_raw(j, "cat", b, first);
	snprintf(b, sizeof(b), "%u", (unsigned)t.flightstate);
	rfnm::jkv_raw(j, "fs", b, first);
	snprintf(b, sizeof(b), "%u", (unsigned)t.emergency);
	rfnm::jkv_raw(j, "emg", b, first);
	snprintf(b, sizeof(b), "%u", (unsigned)t.ts_qs);   // Timestamp, quarter seconds (G.1.1)
	rfnm::jkv_raw(j, "ts_qs", b, first);
}

void traffic_table::update(const adsl_traffic &t, const adsl_rx_meta &m, uint64_t now_ms) {
	std::lock_guard<std::mutex> lk(mtx_);
	traffic_entry &e = map_[key(t.amt, t.addr)];
	if(e.count == 0) {
		e.first_ms = now_ms;
	}
	e.last = t;
	e.meta = m;
	e.last_ms = now_ms;
	e.wall_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
	    std::chrono::system_clock::now().time_since_epoch()).count();
	e.count++;
	if(m.corrected) {
		e.corrected++;
	}
	total_++;
}

void traffic_table::expire(uint64_t now_ms, uint64_t max_age_ms) {
	std::lock_guard<std::mutex> lk(mtx_);
	for(auto it = map_.begin(); it != map_.end();) {
		if(now_ms - it->second.last_ms > max_age_ms) {
			it = map_.erase(it);
		} else {
			++it;
		}
	}
}

std::string traffic_table::to_json(uint64_t now_ms) const {
	std::lock_guard<std::mutex> lk(mtx_);
	std::string j = "[";
	bool firste = true;
	for(const auto &kv : map_) {
		const traffic_entry &e = kv.second;
		j += firste ? "{" : ",{";
		firste = false;
		bool f = true;
		traffic_fields_json(j, e.last, f);
		char b[48];
		snprintf(b, sizeof(b), "%.1f", (double)e.meta.rssi_db);
		rfnm::jkv_raw(j, "rssi", b, f);
		snprintf(b, sizeof(b), "%d", e.meta.channel);
		rfnm::jkv_raw(j, "ch", b, f);
		snprintf(b, sizeof(b), "%llu", (unsigned long long)e.count);
		rfnm::jkv_raw(j, "count", b, f);
		double age = now_ms > e.last_ms ? (double)(now_ms - e.last_ms) / 1000.0 : 0.0;
		snprintf(b, sizeof(b), "%.1f", age);
		rfnm::jkv_raw(j, "age_s", b, f);
		snprintf(b, sizeof(b), "%llu", (unsigned long long)e.wall_ms);
		rfnm::jkv_raw(j, "ts", b, f);
		j += "}";
	}
	return j + "]";
}

size_t traffic_table::size() const {
	std::lock_guard<std::mutex> lk(mtx_);
	return map_.size();
}

uint64_t traffic_table::total() const {
	std::lock_guard<std::mutex> lk(mtx_);
	return total_;
}

} // namespace adsld

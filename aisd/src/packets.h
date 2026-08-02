// aisd: decoded-message ring buffer + JSON serialization for the HTTP API. The GUI polls
// /aisapi/messages and expects {total, messages:[{time,ch,type,mmsi,level,info,nmea}]}.
#pragma once

#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>
#include <deque>
#include <cstdint>

namespace aisd {

struct message_entry {
	std::string time;
	char channel = '?';
	int type = 0;
	uint32_t mmsi = 0;
	float level = 0;
	// structured decode (ais_parse): only present fields are emitted as JSON keys, so the
	// GUI reads real numbers/strings instead of regex-parsing `info`.
	bool has_pos = false;
	double lat = 0, lon = 0;
	bool has_sog = false;
	double sog = 0;
	bool has_cog = false;
	double cog = 0;
	bool has_hdg = false;
	int hdg = 0;
	int shiptype = 0;          // 0 = not available
	std::string nav_status;    // navigation-status word, empty if absent
	std::string name;
	std::string callsign;
	std::string destination;
	std::string info;
	std::string nmea;   // the AIVDM sentence(s), newline-joined if multipart
};

class message_ring {
public:
	void push(message_entry e) {
		std::lock_guard<std::mutex> lk(mtx_);
		total_++;
		ring_.push_back(std::move(e));
		if(ring_.size() > 500) {
			ring_.pop_front();
		}
	}

	uint64_t total() const {
		std::lock_guard<std::mutex> lk(mtx_);
		return total_;
	}

	static std::string json_escape(const std::string &s) {
		std::string o;
		for(char c : s) {
			if(c == '"' || c == '\\') {
				o += '\\';
				o += c;
			} else if((unsigned char)c < 0x20) {
				char b[8];
				snprintf(b, sizeof(b), "\\u%04x", c);
				o += b;
			} else {
				o += c;
			}
		}
		return o;
	}

	std::string to_json() const {
		std::lock_guard<std::mutex> lk(mtx_);
		std::string out = "{\"total\":" + std::to_string(total_) + ",\"messages\":[";
		bool first = true;
		for(const auto &m : ring_) {
			char head[160];
			snprintf(head, sizeof(head), "%s{\"time\":\"%s\",\"ch\":\"%c\",\"type\":%d,\"mmsi\":%u,\"level\":%.1f,",
			         first ? "" : ",", m.time.c_str(), m.channel, m.type, m.mmsi, m.level);
			out += head;
			char nb[96];
			if(m.has_pos) {
				snprintf(nb, sizeof(nb), "\"lat\":%.5f,\"lon\":%.5f,", m.lat, m.lon);
				out += nb;
			}
			if(m.has_sog) {
				snprintf(nb, sizeof(nb), "\"sog\":%.1f,", m.sog);
				out += nb;
			}
			if(m.has_cog) {
				snprintf(nb, sizeof(nb), "\"cog\":%.1f,", m.cog);
				out += nb;
			}
			if(m.has_hdg) {
				snprintf(nb, sizeof(nb), "\"hdg\":%d,", m.hdg);
				out += nb;
			}
			if(m.shiptype > 0) {
				snprintf(nb, sizeof(nb), "\"shiptype\":%d,", m.shiptype);
				out += nb;
			}
			if(!m.nav_status.empty()) {
				out += "\"nav_status\":\"" + json_escape(m.nav_status) + "\",";
			}
			if(!m.name.empty()) {
				out += "\"name\":\"" + json_escape(m.name) + "\",";
			}
			if(!m.callsign.empty()) {
				out += "\"callsign\":\"" + json_escape(m.callsign) + "\",";
			}
			if(!m.destination.empty()) {
				out += "\"destination\":\"" + json_escape(m.destination) + "\",";
			}
			out += "\"info\":\"" + json_escape(m.info) + "\",\"nmea\":\"" + json_escape(m.nmea) + "\"}";
			first = false;
		}
		out += "]}";
		return out;
	}

	static std::string now_string() {
		time_t t = time(nullptr);
		struct tm tmv;
		localtime_r(&t, &tmv);
		char b[16];
		strftime(b, sizeof(b), "%H:%M:%S", &tmv);
		return b;
	}

private:
	mutable std::mutex mtx_;
	std::deque<message_entry> ring_;
	uint64_t total_ = 0;
};

} // namespace aisd

// lorad: packet ring buffer + JSON serialization for the HTTP API. The GUI polls
// /lorapi/packets and expects, per packet: {time,epoch_ms,freq,bw,sf,cr,snr,len,
// crc_present,crc_ok,info,hex} and, for Meshtastic frames, a structured "mesh":{from,
// to,id,hop_limit,hop_start,portnum,port,text} object alongside the flat info/hex. MeshCore
// frames carry a "meshcore":{type,route,hops,path,hash,pubkey,name,...} object instead
// (see meshcore_json()).
#pragma once

#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>
#include <deque>
#include <cstdint>

namespace lorad {

struct packet_entry {
	std::string time;
	int64_t epoch_ms = 0;
	double freq = 0;
	double bw = 0;
	int sf = 0;
	int cr = 0;               // 1..4 == LoRa CR 4/5..4/8
	float snr = 0;
	int len = 0;
	bool crc_present = false;
	bool crc_ok = false;
	std::string info;
	std::string hex;

	// meshtastic structured fields (mesh == true once a >=16-byte radio header parsed)
	bool mesh = false;
	std::string m_from, m_to;   // "!xxxxxxxx" / "^all"
	uint32_t m_id = 0;
	int m_hop_limit = 0, m_hop_start = 0;
	bool m_decoded = false;     // Data proto parsed after default-key decrypt
	unsigned m_portnum = 0;
	std::string m_port;         // portnum name, "" if unknown
	bool m_has_text = false;
	std::string m_text;         // decoded TEXT (already UTF-8 validated)
	bool m_has_pos = false;     // decoded POSITION with a plausible lat/lon
	double m_lat = 0, m_lon = 0;
	bool m_has_alt = false;
	int m_alt = 0;

	std::string meshcore_json; // serialized "meshcore" object, empty if none
};

class packet_ring {
public:
	void push(packet_entry e) {
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
		std::string out = "{\"total\":" + std::to_string(total_) + ",\"packets\":[";
		bool first = true;
		for(const auto &p : ring_) {
			char head[256];
			snprintf(head, sizeof(head),
			         "%s{\"time\":\"%s\",\"epoch_ms\":%lld,\"freq\":%.0f,\"bw\":%.0f,\"sf\":%d,\"cr\":%d,\"snr\":%.1f,\"len\":%d,"
			         "\"crc_present\":%s,\"crc_ok\":%s,",
			         first ? "" : ",", p.time.c_str(), (long long)p.epoch_ms, p.freq, p.bw, p.sf, p.cr, p.snr, p.len,
			         p.crc_present ? "true" : "false", p.crc_ok ? "true" : "false");
			out += head;
			out += "\"info\":\"" + json_escape(p.info) + "\",\"hex\":\"" + p.hex + "\"";
			if(p.mesh) {
				char mh[256];
				snprintf(mh, sizeof(mh),
				         ",\"mesh\":{\"from\":\"%s\",\"to\":\"%s\",\"id\":%u,\"hop_limit\":%d,\"hop_start\":%d,\"decoded\":%s,\"portnum\":%u",
				         json_escape(p.m_from).c_str(), json_escape(p.m_to).c_str(), p.m_id, p.m_hop_limit, p.m_hop_start,
				         p.m_decoded ? "true" : "false", p.m_portnum);
				out += mh;
				if(!p.m_port.empty()) {
					out += ",\"port\":\"" + json_escape(p.m_port) + "\"";
				}
				if(p.m_has_text) {
					out += ",\"text\":\"" + json_escape(p.m_text) + "\"";
				}
				if(p.m_has_pos) {
					char pb[96];
					snprintf(pb, sizeof(pb), ",\"lat\":%.7f,\"lon\":%.7f", p.m_lat, p.m_lon);
					out += pb;
					if(p.m_has_alt) {
						out += ",\"alt\":" + std::to_string(p.m_alt);
					}
				}
				out += "}";
			}
			if(!p.meshcore_json.empty()) {
				out += ",\"meshcore\":" + p.meshcore_json;
			}
			out += "}";
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
	std::deque<packet_entry> ring_;
	uint64_t total_ = 0;
};

} // namespace lorad

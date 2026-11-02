#include "parse.h"

#include <cstdio>
#include <cstring>
#include "aes128.h"

namespace lorad {

std::string hex_string(const std::vector<uint8_t> &p) {
	std::string s;
	s.reserve(p.size() * 2);
	char b[4];
	for(uint8_t v : p) {
		snprintf(b, sizeof(b), "%02x", v);
		s += b;
	}
	return s;
}

static uint32_t le32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p) {
	return (uint16_t)(p[0] | (p[1] << 8));
}

// EUIs are little-endian on the wire; display convention is big-endian
static std::string eui_string(const uint8_t *p) {
	char b[24];
	snprintf(b, sizeof(b), "%02x%02x%02x%02x%02x%02x%02x%02x", p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
	return b;
}

// ---- LoRaWAN ----------------------------------------------------------------------

std::string lorawan_info(const std::vector<uint8_t> &p) {
	if(p.empty()) {
		return "";
	}
	static const char *MTYPE[8] = { "JoinReq", "JoinAccept", "UnconfUp", "UnconfDown", "ConfUp", "ConfDown", "RFU", "Proprietary" };
	int mtype = p[0] >> 5;
	char b[192];

	// JoinRequest: MHDR | JoinEUI(8) | DevEUI(8) | DevNonce(2) | MIC(4)
	if(mtype == 0 && p.size() == 23) {
		snprintf(b, sizeof(b), "JoinReq deveui=%s joineui=%s nonce=%04x", eui_string(&p[9]).c_str(), eui_string(&p[1]).c_str(), le16(&p[17]));
		return b;
	}

	// data frames: MHDR | DevAddr(4) | FCtrl | FCnt(2) | FOpts(0..15) | [FPort | FRMPayload] | MIC(4)
	if(mtype >= 2 && mtype <= 5 && p.size() >= 12) {
		uint32_t devaddr = le32(&p[1]);
		uint8_t fctrl = p[5];
		uint16_t fcnt = le16(&p[6]);
		size_t fopts = fctrl & 0x0F;
		size_t idx = 8 + fopts;
		int port = -1;
		size_t frm_len = 0;
		if(idx + 4 < p.size()) {
			port = p[idx];
			frm_len = p.size() - idx - 1 - 4;
		}
		uint32_t mic = p.size() >= 12 + fopts ? le32(&p[p.size() - 4]) : 0;
		int n = snprintf(b, sizeof(b), "%s dev=%08x fcnt=%u", MTYPE[mtype], devaddr, fcnt);
		if(port >= 0) {
			n += snprintf(b + n, sizeof(b) - n, " port=%d len=%zu", port, frm_len);
		}
		if(fctrl & 0x80) {
			n += snprintf(b + n, sizeof(b) - n, " adr");
		}
		if(fctrl & 0x20) {
			n += snprintf(b + n, sizeof(b) - n, " ack");
		}
		snprintf(b + n, sizeof(b) - n, " mic=%08x", mic);
		return b;
	}

	snprintf(b, sizeof(b), "%s len=%zu", MTYPE[mtype], p.size());
	return b;
}

// ---- Meshtastic -------------------------------------------------------------------

// default "AQ==" channel PSK (LongFast); public, ships in the firmware
static const uint8_t DEFAULT_PSK[16] = {
	0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
};

static bool g_chan_set = false;
static uint8_t g_chan_hash = 0, g_chan_psk[16];

void meshtastic_set_channel(uint8_t hash, const uint8_t psk[16]) {
	g_chan_hash = hash;
	memcpy(g_chan_psk, psk, 16);
	g_chan_set = true;
}

static const uint8_t *psk_for(uint8_t chan) {
	return g_chan_set && chan == g_chan_hash ? g_chan_psk : DEFAULT_PSK;
}

static const char *portnum_name(unsigned port) {
	switch(port) {
	case 1: return "TEXT";
	case 3: return "POSITION";
	case 4: return "NODEINFO";
	case 5: return "ROUTING";
	case 6: return "ADMIN";
	case 8: return "WAYPOINT";
	case 10: return "DETECTION";
	case 67: return "TELEMETRY";
	case 70: return "TRACEROUTE";
	case 71: return "NEIGHBORINFO";
	case 73: return "MAPREPORT";
	default: return nullptr;
	}
}

static bool read_varint(const uint8_t *d, size_t len, size_t &i, uint64_t &v) {
	v = 0;
	for(int s = 0; s < 64 && i < len; s += 7) {
		uint8_t byte = d[i++];
		v |= (uint64_t)(byte & 0x7F) << s;
		if(!(byte & 0x80)) {
			return true;
		}
	}
	return false;
}

// minimal scan of the meshtastic.Data protobuf: field 1 = portnum, field 2 = payload.
// A clean parse doubles as the decrypt-success heuristic.
static bool scan_data_proto(const uint8_t *d, size_t len, unsigned &port, std::vector<uint8_t> &payload) {
	size_t i = 0;
	bool have_port = false;
	while(i < len) {
		uint64_t tag;
		if(!read_varint(d, len, i, tag)) {
			return false;
		}
		unsigned field = (unsigned)(tag >> 3), wire = tag & 7;
		if(field == 0 || field > 15) {
			return false;
		}
		if(wire == 0) {
			uint64_t v;
			if(!read_varint(d, len, i, v)) {
				return false;
			}
			if(field == 1) {
				if(v > 511) {
					return false;
				}
				port = (unsigned)v;
				have_port = true;
			}
		} else if(wire == 2) {
			uint64_t l;
			if(!read_varint(d, len, i, l) || i + l > len) {
				return false;
			}
			if(field == 2) {
				payload.assign(d + i, d + i + l);
			}
			i += l;
		} else if(wire == 5) {
			if(i + 4 > len) {
				return false;
			}
			i += 4;
		} else if(wire == 1) {
			if(i + 8 > len) {
				return false;
			}
			i += 8;
		} else {
			return false;
		}
	}
	return have_port;
}

// accept only clean text for inline display: no control chars (except tab/newline/CR)
// and valid UTF-8 - a mis-decrypt that happens to protobuf-parse must not leak raw
// bytes into the /packets JSON (json_escape only handles quotes/backslash/<0x20)
bool clean_text(const std::vector<uint8_t> &v) {
	if(v.empty()) {
		return false;
	}
	size_t i = 0;
	while(i < v.size()) {
		uint8_t c = v[i];
		if(c < 0x20) {
			if(c != 0x09 && c != 0x0A && c != 0x0D) {
				return false;
			}
			i++;
		} else if(c < 0x80) {
			i++;
		} else {
			if(c == 0xC0 || c == 0xC1 || c > 0xF4) {
				return false; // overlong or out-of-range lead byte
			}
			int cont = (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : (c & 0xF8) == 0xF0 ? 3 : -1;
			if(cont < 0 || i + (size_t)cont >= v.size()) {
				return false;
			}
			for(int k = 1; k <= cont; k++) {
				if((v[i + k] & 0xC0) != 0x80) {
					return false;
				}
			}
			i += (size_t)cont + 1;
		}
	}
	return true;
}

// scan a meshtastic.Position protobuf: latitude_i(1)/longitude_i(2) are sfixed32 in
// 1e-7 degrees, altitude(3) is an int32 in metres. Returns true only with a plausible
// lat/lon pair so a mis-decrypt that happens to parse cannot drop a phantom node on the map.
static bool scan_position(const uint8_t *d, size_t len, double &lat, double &lon, bool &has_alt, int &alt) {
	size_t i = 0;
	bool hlat = false, hlon = false;
	while(i < len) {
		uint64_t tag;
		if(!read_varint(d, len, i, tag)) {
			return false;
		}
		unsigned field = (unsigned)(tag >> 3), wire = tag & 7;
		if(field == 0) {
			return false;
		}
		if(wire == 5) {
			if(i + 4 > len) {
				return false;
			}
			if(field == 1) {
				lat = (double)(int32_t)le32(d + i) * 1e-7;
				hlat = true;
			} else if(field == 2) {
				lon = (double)(int32_t)le32(d + i) * 1e-7;
				hlon = true;
			}
			i += 4;
		} else if(wire == 0) {
			uint64_t v;
			if(!read_varint(d, len, i, v)) {
				return false;
			}
			if(field == 3) {
				alt = (int)(int32_t)(uint32_t)v;
				has_alt = true;
			}
		} else if(wire == 2) {
			uint64_t l;
			if(!read_varint(d, len, i, l) || i + l > len) {
				return false;
			}
			i += l;
		} else if(wire == 1) {
			if(i + 8 > len) {
				return false;
			}
			i += 8;
		} else {
			return false;
		}
	}
	return hlat && hlon && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180 && (lat != 0 || lon != 0);
}

std::string mesh_node_string(uint32_t n) {
	if(n == 0xFFFFFFFF) {
		return "^all";
	}
	char b[12];
	snprintf(b, sizeof(b), "!%08x", n);
	return b;
}

std::string meshtastic_info(const std::vector<uint8_t> &p) {
	// radio header: dest(4) sender(4) id(4) flags(1) chanhash(1) next-hop(1) relay(1)
	if(p.size() < 16) {
		return "";
	}
	uint32_t dest = le32(&p[0]), from = le32(&p[4]), id = le32(&p[8]);
	uint8_t flags = p[12], chan = p[13];
	int hop_limit = flags & 7;
	int hop_start = (flags >> 5) & 7;

	char b[128];
	snprintf(b, sizeof(b), "%s>%s id=%08x hop=%d/%d ch=%02x", mesh_node_string(from).c_str(), mesh_node_string(dest).c_str(), id, hop_limit, hop_start, chan);
	std::string info = b;

	if(p.size() > 16) {
		// best-effort default-PSK decrypt; nonce = id as u64 LE || sender LE || 32-bit counter
		std::vector<uint8_t> body(p.begin() + 16, p.end());
		uint8_t nonce[16];
		memset(nonce, 0, sizeof(nonce));
		memcpy(nonce, &p[8], 4);
		memcpy(nonce + 8, &p[4], 4);
		aes128 aes(psk_for(p[13]));
		aes.ctr_crypt(nonce, body.data(), body.size());

		unsigned port = 0;
		std::vector<uint8_t> inner;
		if(scan_data_proto(body.data(), body.size(), port, inner)) {
			if(port == 1) {
				if(clean_text(inner)) {
					info += " \"" + std::string(inner.begin(), inner.end()) + "\"";
					return info;
				}
			}
			const char *name = portnum_name(port);
			if(name) {
				info += std::string(" ") + name;
			} else {
				snprintf(b, sizeof(b), " port=%u", port);
				info += b;
			}
			snprintf(b, sizeof(b), " len=%zu", inner.size());
			info += b;
		} else {
			info += " (encrypted)";
		}
	}
	return info;
}

// structured sibling of meshtastic_info(): same decode, fields instead of a flat string,
// for the mesh visualizer. Shares the radio-header layout and default-PSK decrypt path.
mesh_fields meshtastic_decode(const std::vector<uint8_t> &p) {
	mesh_fields f;
	if(p.size() < 16) {
		return f;
	}
	f.valid = true;
	f.to = le32(&p[0]);
	f.from = le32(&p[4]);
	f.id = le32(&p[8]);
	f.hop_limit = p[12] & 7;
	f.hop_start = (p[12] >> 5) & 7;
	f.chan = p[13];

	if(p.size() > 16) {
		std::vector<uint8_t> body(p.begin() + 16, p.end());
		uint8_t nonce[16];
		memset(nonce, 0, sizeof(nonce));
		memcpy(nonce, &p[8], 4);
		memcpy(nonce + 8, &p[4], 4);
		aes128 aes(psk_for(p[13]));
		aes.ctr_crypt(nonce, body.data(), body.size());

		unsigned port = 0;
		std::vector<uint8_t> inner;
		if(scan_data_proto(body.data(), body.size(), port, inner)) {
			f.decoded = true;
			f.port = port;
			const char *name = portnum_name(port);
			if(name) {
				f.portname = name;
			}
			if(port == 1 && clean_text(inner)) {
				f.is_text = true;
				f.text.assign(inner.begin(), inner.end());
			} else if(port == 3) {
				f.has_pos = scan_position(inner.data(), inner.size(), f.lat, f.lon, f.has_alt, f.alt);
			}
		} else {
			f.encrypted = true;
		}
	}
	return f;
}

} // namespace lorad

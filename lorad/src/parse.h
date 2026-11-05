// lorad: payload interpretation - LoRaWAN MAC header fields and the Meshtastic
// radio header (with best-effort default-key decrypt). No network keys, no MIC
// verification: this is a sniffer, the fields are informational.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lorad {

std::string lorawan_info(const std::vector<uint8_t> &p);
// a private channel: frames with this hash decrypt with this key, all others with the default
void meshtastic_set_channel(uint8_t hash, const uint8_t psk[16]);
std::string meshtastic_info(const std::vector<uint8_t> &p);
std::string hex_string(const std::vector<uint8_t> &p);

// true for non-empty valid UTF-8 without control chars (tab/newline/CR excepted): the
// only text allowed inline in the /packets JSON
bool clean_text(const std::vector<uint8_t> &v);

// format a 32-bit node number as Meshtastic renders it: "!xxxxxxxx" (or "^all" for broadcast)
std::string mesh_node_string(uint32_t n);

// structured Meshtastic decode: radio header always, and the best-effort default-PSK
// decrypt of the Data protobuf when it parses cleanly. Feeds the mesh visualizer JSON.
struct mesh_fields {
	bool valid = false;         // >= 16-byte radio header present
	uint32_t from = 0, to = 0, id = 0;
	uint8_t chan = 0;
	int hop_limit = 0, hop_start = 0;
	bool decoded = false;       // Data proto parsed after decrypt
	unsigned port = 0;          // portnum (raw)
	std::string portname;       // portnum name, empty if unknown
	bool is_text = false;       // TEXT frame with clean UTF-8 text below
	std::string text;
	bool has_pos = false;       // POSITION frame carrying a valid lat/lon
	double lat = 0, lon = 0;    // degrees
	bool has_alt = false;
	int alt = 0;                // metres
	bool encrypted = false;     // body present but did not parse (unknown key / not for us)
};
mesh_fields meshtastic_decode(const std::vector<uint8_t> &p);

} // namespace lorad

// aisd: AIS message interpretation. Decodes the common message types into structured
// fields (ais_parse) and a compact human-readable info string (ais_info, built from those
// same fields). This is a sniffer: fields are informational, no integrity beyond the
// link-layer CRC (checked in the receiver).
#pragma once

#include <cstdint>
#include <string>

#include "ais_phy.h"

namespace aisd {

// Structured decode of the common AIS message types. Only fields the message actually
// carries are populated; "not available" sentinel values leave the has_* flag false (and
// leave name/callsign/destination empty, shiptype 0). This is what the HTTP API emits as
// real JSON keys so the GUI never has to regex-parse the info string.
struct ais_fields {
	int type = 0;
	uint32_t mmsi = 0;
	bool has_pos = false;
	double lat = 0, lon = 0;    // degrees, WGS84
	bool has_sog = false;
	double sog = 0;             // speed over ground, knots
	bool has_cog = false;
	double cog = 0;             // course over ground, degrees
	bool has_hdg = false;
	int hdg = 0;               // true heading, degrees
	bool has_nav = false;
	int nav_status = 0;        // 0..15 navigation-status code (types 1/2/3)
	int shiptype = 0;          // ship/cargo type code (types 5/19/24B); 0 = not available
	std::string name;          // vessel name (types 5/19/24A)
	std::string callsign;      // radio call sign (types 5/24B)
	std::string destination;   // voyage destination (type 5)
};

// Extract the structured fields from a decoded logical message (types 1/2/3/4/5/18/19/24
// + a generic fallback that fills only type/mmsi).
ais_fields ais_parse(const bits &logical);

// One-line human-readable summary of a decoded AIS message, formatted from ais_parse()
// (plus the base-station timestamp for type 4). Kept byte-stable for the GUI's legacy
// info-string fallback parser.
std::string ais_info(const bits &logical);

// Navigation-status word for a 0..15 code (types 1/2/3).
const char *ais_nav_status(int code);

} // namespace aisd

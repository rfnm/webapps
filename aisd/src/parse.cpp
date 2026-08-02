#include "parse.h"

#include <cmath>
#include <cstdio>

namespace aisd {

static const char *NAV_STATUS[16] = {
	"underway-engine", "anchored", "not-under-command", "restricted-manoeuvrability",
	"constrained-by-draught", "moored", "aground", "fishing", "underway-sailing",
	"reserved-hsc", "reserved-wig", "reserved", "reserved", "reserved", "ais-sart", "undefined",
};

const char *ais_nav_status(int code) {
	return NAV_STATUS[code & 15];
}

// position pair shared by class-A (1/2/3), base station (4) and class-B (18/19) reports.
// lon/lat carry a 181 deg / 91 deg "not available" sentinel that leaves has_pos false.
static void parse_pos(ais_fields &f, const bits &b, size_t lon_start, size_t lat_start) {
	int64_t lon = bits_get_int(b, lon_start, 28);
	int64_t lat = bits_get_int(b, lat_start, 27);
	if(lon == 0x6791AC0 || lat == 0x3412140) {   // 181 deg / 91 deg == "not available"
		return;
	}
	f.has_pos = true;
	f.lat = (double)lat / 600000.0;
	f.lon = (double)lon / 600000.0;
}

// speed/course tail shared by class-A (1/2/3) and class-B (18/19) position reports.
// sog 1023, cog 3600 and hdg 511 are the "not available" sentinels.
static void parse_motion(ais_fields &f, const bits &b, size_t sog_start, size_t cog_start, int hdg_start) {
	unsigned sog = (unsigned)bits_get_uint(b, sog_start, 10);
	if(sog != 1023) {
		f.has_sog = true;
		f.sog = sog / 10.0;
	}
	unsigned cog = (unsigned)bits_get_uint(b, cog_start, 12);
	if(cog != 3600) {
		f.has_cog = true;
		f.cog = cog / 10.0;
	}
	if(hdg_start >= 0) {
		unsigned hdg = (unsigned)bits_get_uint(b, (size_t)hdg_start, 9);
		if(hdg != 511) {
			f.has_hdg = true;
			f.hdg = (int)hdg;
		}
	}
}

ais_fields ais_parse(const bits &b) {
	ais_fields f;
	f.type = (int)bits_get_uint(b, 0, 6);
	f.mmsi = (uint32_t)bits_get_uint(b, 8, 30);

	switch(f.type) {
	case 1:
	case 2:
	case 3:
		f.has_nav = true;
		f.nav_status = (int)bits_get_uint(b, 38, 4) & 15;
		parse_pos(f, b, 61, 89);
		parse_motion(f, b, 50, 116, 128);
		break;
	case 4:   // base station report: position (timestamp is info-string only)
		parse_pos(f, b, 79, 107);
		break;
	case 5:   // static and voyage related data
		f.callsign = bits_get_str(b, 70, 7);
		f.name = bits_get_str(b, 112, 20);
		f.shiptype = (int)bits_get_uint(b, 232, 8);
		f.destination = bits_get_str(b, 302, 20);
		break;
	case 18:   // Class B standard position
		parse_pos(f, b, 57, 85);
		parse_motion(f, b, 46, 112, 124);
		break;
	case 19:   // Class B extended position (adds name + ship type)
		parse_pos(f, b, 57, 85);
		parse_motion(f, b, 46, 112, 124);
		f.name = bits_get_str(b, 143, 20);
		f.shiptype = (int)bits_get_uint(b, 263, 8);
		break;
	case 24: {   // static data report, part A (name) or B (type/callsign)
		int part = (int)bits_get_uint(b, 38, 2);
		if(part == 0) {
			f.name = bits_get_str(b, 40, 20);
		} else {
			f.shiptype = (int)bits_get_uint(b, 40, 8);
			f.callsign = bits_get_str(b, 90, 7);
		}
		break;
	}
	default:
		break;
	}
	return f;
}

std::string ais_info(const bits &b) {
	ais_fields f = ais_parse(b);
	char head[48];
	snprintf(head, sizeof(head), "type=%d mmsi=%09u", f.type, f.mmsi);
	std::string s = head;
	char buf[64];

	auto append_pos = [&]() {
		if(f.has_pos) {
			snprintf(buf, sizeof(buf), " pos=%.5f,%.5f", f.lat, f.lon);
			s += buf;
		} else {
			s += " pos=n/a";
		}
	};
	auto append_motion = [&]() {
		if(f.has_sog) {
			snprintf(buf, sizeof(buf), " sog=%.1fkn", f.sog);
			s += buf;
		}
		if(f.has_cog) {
			snprintf(buf, sizeof(buf), " cog=%.1f", f.cog);
			s += buf;
		}
		if(f.has_hdg) {
			snprintf(buf, sizeof(buf), " hdg=%d", f.hdg);
			s += buf;
		}
	};

	switch(f.type) {
	case 1:
	case 2:
	case 3:
		append_pos();
		append_motion();
		s += std::string(" status=") + ais_nav_status(f.nav_status);
		break;
	case 4: {   // base station report: UTC date/time + position
		unsigned year = (unsigned)bits_get_uint(b, 38, 14);
		unsigned mon = (unsigned)bits_get_uint(b, 52, 4);
		unsigned day = (unsigned)bits_get_uint(b, 56, 5);
		unsigned hour = (unsigned)bits_get_uint(b, 61, 5);
		unsigned min = (unsigned)bits_get_uint(b, 66, 6);
		unsigned sec = (unsigned)bits_get_uint(b, 72, 6);
		char t[48];
		snprintf(t, sizeof(t), " base %04u-%02u-%02u %02u:%02u:%02u", year, mon, day, hour, min, sec);
		s += t;
		append_pos();
		break;
	}
	case 5:
		s += " call=\"" + f.callsign + "\"";
		s += " name=\"" + f.name + "\"";
		s += " dest=\"" + f.destination + "\"";
		break;
	case 18:
		append_pos();
		append_motion();
		break;
	case 19:
		append_pos();
		append_motion();
		s += " name=\"" + f.name + "\"";
		break;
	case 24: {
		int part = (int)bits_get_uint(b, 38, 2);
		if(part == 0) {
			s += " name=\"" + f.name + "\"";
		} else {
			s += " call=\"" + f.callsign + "\"";
		}
		break;
	}
	default:
		break;
	}
	return s;
}

} // namespace aisd

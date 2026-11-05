// adsld: conformity helpers - see conform.h
#include "conform.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

#include <sys/random.h>
#include <sys/timex.h>

namespace adsld {

uint32_t adsl_privacy_address(const std::function<uint32_t()> &draw) {
	for(;;) {
		const uint32_t a = draw() & 0xFFFFFFu;
		if(a != 0 && a != 0xFFFFFFu) {
			return a;
		}
	}
}

uint32_t adsl_privacy_address_os() {
	return adsl_privacy_address([]() {
		uint32_t v = 0;
		if(getrandom(&v, sizeof(v), 0) != (ssize_t)sizeof(v)) {
			std::random_device rd;   // no getrandom (very old kernel): the library's source
			v = rd();
		}
		return v;
	});
}

std::vector<conformity_note> conformity_notes(const conformity_input &in) {
	std::vector<conformity_note> n;
	char b[200];
	if(!in.tx) {
		n.push_back({ "tx_off", "B", false, "transmitter off: this station puts nothing on air" });
		return n;
	}
	if(!in.rx) {
		n.push_back({ "no_lbt", "D.3", true,
		              "--rx 0: no receive path, so listen-before-talk never sees a busy channel" });
	}
	if(in.status_interval_s <= 0 || in.status_interval_s > 20) {
		snprintf(b, sizeof(b), "Status every %.1f s%s - at least once per 20 s required",
		         in.status_interval_s, in.status_interval_s <= 0 ? " (off)" : "");
		n.push_back({ "status_interval", "G.2.11", true, b });
	}
	if(std::fabs(in.interval_ground_s - 10.0) > 1e-6) {
		snprintf(b, sizeof(b), "Traffic on ground every %.1f s - 0.1 Hz required",
		         in.interval_ground_s);
		n.push_back({ "interval_ground", "G.1.16", true, b });
	}
	if(in.amt == 0 && !in.addr_random) {
		n.push_back({ "privacy_addr_fixed", "F.2.4", true,
		              "AMT 0 with a configured address - privacy mode draws it at start-up" });
	}
	if(in.addr_changed) {
		n.push_back({ "addr_changed", "F.2.4", true,
		              "AMT or address changed while operating - not admissible" });
	}
	if(!in.utc_timestamp) {
		n.push_back({ "timestamp_not_utc", "G.1.1", true,
		              "Timestamp counts from the radio clock, not from the full UTC hour" });
	}
	if(!in.clock_synced) {
		n.push_back({ "clock_unsynced", "G.1.1", true,
		              "system clock not synchronised - Timestamp cannot be accurate to 10 ms" });
	}
	if(!in.gnss) {
		n.push_back({ "no_gnss", "G.1.1/G.1.2/G.1.17", true,
		              "position from configuration, not GNSS: fix time, flight "
		              "state and quality indicators are not measured" });
	}
	if(!in.cca_calibrated) {
		n.push_back({ "cca_uncalibrated", "D.3", true,
		              "busy threshold is relative (noise floor + 6 dB), not proven <= -79 dBm" });
	}
	if(!in.power_calibrated) {
		n.push_back({ "power_uncalibrated", "C.2", true,
		              "output power not calibrated against 14 dBm ERP" });
	}
	if(in.interval_airborne_s > 1.0) {
		snprintf(b, sizeof(b), "Traffic airborne every %.1f s - 1 Hz recommended",
		         in.interval_airborne_s);
		n.push_back({ "interval_airborne", "G.1.16", false, b });
	}
	return n;
}

std::string conformity_json(const std::vector<conformity_note> &notes) {
	auto esc = [](const std::string &s) {
		std::string o;
		for(char c : s) {
			if(c == '"' || c == '\\') {
				o += '\\';
			}
			o += c;
		}
		return o;
	};
	bool conform = true;
	for(const auto &x : notes) {
		conform = conform && !x.mandatory;
	}
	std::string j = std::string("{\"conform\":") + (conform ? "true" : "false") + ",\"notes\":[";
	for(size_t i = 0; i < notes.size(); i++) {
		const auto &x = notes[i];
		j += i ? ",{" : "{";
		j += "\"key\":\"" + esc(x.key) + "\",\"ref\":\"" + esc(x.ref) + "\",\"mandatory\":" +
		     (x.mandatory ? "true" : "false") + ",\"text\":\"" + esc(x.text) + "\"}";
	}
	return j + "]}";
}

clock_state system_clock_state() {
	clock_state c;
	struct timex tx;
	memset(&tx, 0, sizeof(tx));
	const int st = adjtimex(&tx);
	if(st < 0) {
		return c;
	}
	c.synced = st != TIME_ERROR;
	c.esterror_ms = (double)tx.esterror / 1000.0;
	c.maxerror_ms = (double)tx.maxerror / 1000.0;
	return c;
}

} // namespace adsld

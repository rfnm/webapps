// adsld: the ADS-L obligations that are neither codec nor MAC - kept as pure logic next to
// each other so the selftest can pin them and the daemon only wires them up.
//   - Timestamp (ADS-L.4.SRD860.G.1.1): quarter seconds since the full UTC hour, modulo 60,
//     of the navigation solution - not of the radio's monotonic clock
//   - Privacy mode (F.2.4): AMT 0 draws the whole 24-bit address at start-up, uniformly, and
//     keeps it while the device operates
//   - which settings make a running station NOT conform, with the clause each one breaks,
//     for the start-up log and GET /status
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace adsld {

// G.1.1 Timestamp from UTC milliseconds. 3600 s = 14 400 quarters = 240 x 60, so "since the
// full hour, modulo 60" equals floor(utc_ms / 250) mod 60. Never 60..63.
inline uint8_t adsl_ts_qs_utc(int64_t utc_ms) {
	int64_t q = utc_ms / 250;
	if(utc_ms < 0 && utc_ms % 250 != 0) {
		q--;
	}
	int64_t r = q % 60;
	return (uint8_t)(r < 0 ? r + 60 : r);
}

// radio clock (the scheduler's now_ms) -> UTC milliseconds at that instant
using utc_clock_fn = std::function<int64_t(uint64_t radio_ms)>;

// F.2.4: a uniformly random 24-bit address from `draw` (any 32-bit source; the low 24 bits
// are used). 0x000000 and 0xFFFFFF are drawn again - both read as "no address" to receivers
// and registries - which keeps the remaining 2^24 - 2 values uniform.
uint32_t adsl_privacy_address(const std::function<uint32_t()> &draw);
// the same from the operating system's CSPRNG (getrandom)
uint32_t adsl_privacy_address_os();

// What a running station is configured to do, as far as conformity is concerned.
struct conformity_input {
	bool tx = true;                  // transmitter on
	bool rx = true;                  // receive path on (listen-before-talk needs it)
	double status_interval_s = 15.0;
	double interval_ground_s = 10.0;
	double interval_airborne_s = 1.0;
	uint8_t amt = 0;
	bool addr_random = true;         // AMT 0: address drawn at start-up (false: configured)
	bool addr_changed = false;       // AMT or address changed while running
	bool utc_timestamp = true;       // Timestamp derived from UTC, not from the radio clock
	bool clock_synced = true;        // system clock disciplined (NTP/GNSS)
	bool gnss = false;               // position, time and accuracies from a GNSS receiver
	bool cca_calibrated = false;     // busy threshold proven in dBm (<= -79 dBm, M-band)
	bool power_calibrated = false;   // output power proven <= 14 dBm ERP
};

struct conformity_note {
	const char *key;                 // stable identifier for scripts
	const char *ref;                 // clause, e.g. "G.2.11"
	bool mandatory;                  // false: a "should" of the standard
	std::string text;
};

// Every deviation of `in` from ADS-L Issue 2, mandatory ones first. Empty = nothing known.
// A station with the transmitter off reports only that (nothing it does is on air).
std::vector<conformity_note> conformity_notes(const conformity_input &in);
std::string conformity_json(const std::vector<conformity_note> &notes);

// system clock discipline as the kernel reports it (adjtimex): synced = not TIME_ERROR
struct clock_state {
	bool synced = false;
	double esterror_ms = -1;
	double maxerror_ms = -1;
};
clock_state system_clock_state();

} // namespace adsld

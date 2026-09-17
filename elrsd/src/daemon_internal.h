// elrsd: the daemon's private helpers shared between config.cpp, packets.cpp, status.cpp,
// tracker.cpp, replay.cpp, daemon_selftest.cpp and daemon_main.cpp. Not part of the module
// contract (daemon.h is); nothing here references symbols of A/B/D - only header-inline
// types, so every unit that includes it keeps its own NEEDS line honest.
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include "daemon.h"

namespace elrsd {

// ---- JSON number/string emit (the fleet's flat snprintf style, NaN/inf -> 0) ----------------
std::string jnum(double v, int decimals = 3);
std::string jint(long long v);
std::string jbool(bool b);
std::string jstr(const std::string &s);
std::string hex_bytes(const uint8_t *p, size_t n);   // lowercase, no separators

// ---- enum strings (local so status.cpp does not link against D's tx_policy_str) -------------
const char *lock_state_name(lock_state s);            // "none" | "mod_hop" | "full"
const char *policy_name(tx_policy p);                 // "none" | "embed" | "all"

// the "fhss" object of /status
struct fhss_view {
	link_lock lock;
	int64_t slot = 0;
	uint8_t nonce = 0;
	std::string nonce_conf = "none";
	int fhss_index = -1;
	int channel = -1;
	double freq_hz = 0;
	int64_t next_sync_slot = -1;
	double interval_est_us = 0;
};
// returns  "fhss":{...},"counters":{...}  (no braces around, no leading comma)
std::string fhss_counters_json(const link_stats &st, const fhss_view &v);
// the "rc" object of /status and GET /rc: 16 CRSF values + flags
std::string rc_json(const packet_entry *last_rc);
// the "tx" object of /status
std::string tx_status_json(const tx_status &t);
// the "rx" object of /status
std::string rx_status_json(const daemon_config &cfg, const std::string &source, const source_stats &src,
                           const rx_chain::stats &rx, uint64_t queue_drops, int ch_lo, int ch_hi);

// window of ELRS channels visible in [fc - fs/2, fc + fs/2] (guard 0.6 MHz at both edges)
void channel_window(double fc_hz, double fs_hz, int &ch_lo, int &ch_hi);

// extra tracker hooks (additions to daemon.h's rx_tracker, wired by daemon_main / replay):
// - set_window: the visible channel span for slots_expected / slots_missed bookkeeping
// - set_packet_sink: called (on the DSP thread, outside the tracker mutex) for every entry
//   pushed into the ring -> WebSocket broadcast
struct rx_tracker_hooks {
	static void set_window(rx_tracker &t, int ch_lo, int ch_hi);
	static void set_packet_sink(rx_tracker &t, std::function<void(const packet_entry &)> f);
	static void set_tlm_denom(rx_tracker &t, int tlm_denom);
	static void set_auto_rate(rx_tracker &t, bool on);       // --rate auto: adopt the burst majority
	static void set_wallclock(rx_tracker &t, bool on);       // live: stamp epoch_ms (replay: 0)
	static const packet_entry *last_rc_entry(rx_tracker &t, packet_entry &copy_out);   // nullptr if none
	static int rate_index_now(const rx_tracker &t);
	static fhss_view view(const rx_tracker &t);              // the /status "fhss" fields
	static void set_chain(rx_tracker &t, rx_chain *chain);   // re-point hints after a chain rebuild (fc change)
	static std::vector<int> next_channels(const rx_tracker &t, int n);   // predicted channels slot+1..slot+n ({} without lock)
};

// the UID used by --loopback when no runtime UID is configured (the TEST-UID of
// tools/elrsd_vectors.json - never a real binding)
inline uid_t6 test_uid() { return uid_t6{ 0xd2, 0x4c, 0x3e, 0x2e, 0xfb, 0x4d }; }

} // namespace elrsd

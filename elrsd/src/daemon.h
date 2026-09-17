// elrsd: daemon-side glue - configuration, the RX tracker (rx_burst -> codec -> link model),
// the packet ring, status snapshots and the JSON schemas of the HTTP/WS API.
// Everything here is host-buildable; librfnm only ever appears in
// capture_rfnm.cpp (B) and tx_rfnm.cpp (D). The API wire format is fixed in -
// change it there first.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include "elrs_codec.h"
#include "elrs_link.h"
#include "rx_chain.h"
#include "sample_source.h"
#include "tx_chain.h"

namespace elrsd {

// ---- configuration (precedence: argv > ELRS_* env > defaults; bad values -> warn + default, never exit) ----
struct daemon_config {
	int port = 7694;                     // loopback HTTP
	int ws_port = 0;                     // 0 = no WebSocket server
	std::string rate = "250hz";          // ELRS_RATE: 500hz 250hz 150hz 50hz (333hz/100hz = OTA8 -> refused)
	double fs_hz = 25e6;                 // ELRS_FS
	double fc_hz = 2441.4e6;             // ELRS_FC window centre (default = SYNC channel 41 in the window)
	int gain_db = -24;                   // ELRS_GAIN
	bool agc = false;                    // ELRS_AGC
	std::string antenna = "sma_a";      // ELRS_ANTENNA
	bool conj = false;                   // ELRS_CONJ (RX IQ sign; see sample_source.h)
	bool conj_auto = true;               // ELRS_CONJ_AUTO: probe the other sign while no preamble is found
	                                     // (board-local sign is; replay reports conj_used); --conj 0|1 sets conj_auto=false
	int os = 4;                          // demod oversampling (4 = verified path)
	bool tx_enabled = false;             // --tx 1 (gate 2; gate 1 = board.conf, gate 3 = POST /tx/arm)
	int tx_cap_dbm = -20;                // ELRS_TX_CAP_DBM hard cap for POST /tx/arm power
	double tx_max_duration_s = 120;      // auto-disarm ceiling
	std::string boardconf = "/rfnm/board.conf";
	// replay
	std::string replay_path;             // --replay FILE
	double replay_start_s = 0, replay_stop_s = 0;
	bool replay_realtime = false;
	bool replay_verify = true;           // check against freerun ground truth when a UID is set
	int expect_min_ok = -1;              // --expect-min N: exit 1 if crc_ok < N (replay acceptance)
	std::string packet_log_path;         // --packet-log FILE: replay writes one packet_entry JSON per line (integration check)
	// --selfloop-device SECONDS: hardware self-loop, TX + RX on the ONE device at the
	// same time (the only mode that does), RX on selfloop_rx_path (default "loopback" = the
	// LMS7002M chip-internal TX tap on the Lime card), auto-stop after selfloop_s. 0 = off.
	double selfloop_s = 0;               // -1 = flag given with an invalid value (the mode refuses instead of falling through)
	std::string selfloop_dump;       // --selfloop-dump FILE: write the first selfloop_dump_s seconds of raw RX cs16 (diagnostic)
	double selfloop_dump_s = 0.5;
	int selfloop_tx_dbm = -100;      // requested TX power for --selfloop-device (still clamped to caps)
	// --selfloop-cw: diagnostic variant of --selfloop-device - no bursts, the adapter's idle fill is a
	// continuous tone at LO + selfloop_cw_offset_hz (amplitude 14000) through the SAME feed path, so a
	// raw RX capture (--selfloop-dump) proves whether baseband samples reach the DAC at all
	bool selfloop_cw = false;
	double selfloop_cw_offset_hz = 1.0e6;
	std::string selfloop_rx_path = "loopback";
	int selfloop_rx_channel = 0;         // 0 = this card's rx0; 1 = a second card's rx0 (cable variant)
	// --tx-only SECONDS: the SEND half of the two-board proof. Pure transmit - NO RX
	// path is opened at all (no rfnm_device RX source, no start_rx_locked, no RX apply). The measured
	// failure of 2026-09-14 was that a concurrent RX apply on the same Lime card moves the TX anchor
	// (rfnm::tx_stream latches it once), so every stamped burst is judged "late" and dropped by the
	// kernel (pos_placed 0) while the host still counts bursts_sent. With no RX apply the anchor is
	// stable and the samples reach the DAC (the CW self-loop reached pos_placed 11560 that way). A
	// second board receives separately. 0 = off, -1 = flag given with an invalid value (mode refuses).
	double tx_only_s = 0;
	int tx_only_dbm = -100;              // requested TX power for --tx-only (--tx-only-dbm); clamped UP to
	                                     // power_range.min, never above tx_cap_dbm / the compiled hard cap
	// runtime-only (never from argv: would show in `ps`): binding phrase via ELRS_BINDING_PHRASE env
	// or PUT /config. Kept in RAM only, never logged, never in /status.
	std::string binding_phrase_env;      // filled from getenv at start, cleared after use
	std::string uid_hex_env;             // ELRS_UID (alternative to the phrase)
};
// parse argv + env into cfg; returns false on --help; prints warnings for ignored values
bool parse_config(int argc, char **argv, daemon_config &cfg, std::string &usage_out);

// ---- decoded packet log --------------------------------------------------------------------------
struct packet_entry {
	uint64_t seq = 0;              // monotonic
	double t_s = 0;                // source-clock seconds (replay: file time; live: since start)
	int64_t epoch_ms = 0;          // wall clock (live) / 0 in replay
	int64_t slot = 0;
	int channel = -1;              // estimated
	int channel_predicted = -1;    // from the tracker (-1 = no lock)
	double freq_hz = 0;
	int rate_index = 0, sf = 0;
	float snr_db = 0, rssi_db = 0;
	int ref = 0;
	std::string kind;              // "rc" | "sync" | "msp" | "tlm" | "undecodable"
	bool crc_ok = false;
	int fec_corrected = 0;
	uint8_t nonce = 0;             // FULL lock: exact; else nonce % hop in the low bits, nonce_conf says which
	std::string nonce_conf;        // "none" | "mod_hop" | "full"
	std::array<uint16_t, 16> ch{}; // CRSF values (rc)
	int switch_index = -1; int aux_channel = -1; uint16_t aux_raw = 0;
	bool ch4_high = false;
	sync_packet sync;              // kind == "sync"
	std::string hex;               // 16 hex chars
	bool truth_known = false, truth_match = false;   // replay ground truth
};

class packet_ring {
public:
	explicit packet_ring(size_t cap = 500);
	void push(packet_entry e);
	uint64_t total() const;
	// JSON per: {"total":N,"packets":[...]} newest last; since = only seq > since
	std::string to_json(uint64_t since = 0, size_t max = 500) const;
	std::vector<packet_entry> snapshot(uint64_t since = 0) const;
	static std::string entry_json(const packet_entry &e);
private:
	mutable std::mutex mtx_;
	std::deque<packet_entry> ring_;
	size_t cap_;
	uint64_t total_ = 0;
};

// ---- link statistics / channel occupancy -----------------------------------------------------------
struct link_stats {
	uint64_t bursts = 0, decoded = 0, crc_ok = 0, crc_fail = 0, rc = 0, sync = 0, other = 0;
	uint64_t slots_expected = 0, slots_missed = 0;    // once locked: expected vs seen packets
	double packet_rate_hz = 0;                        // 1 s window
	double lq_pct = 0;                                // link quality over the last 100 expected slots
	float snr_avg_db = 0, rssi_avg_db = 0;
	std::array<uint32_t, FHSS_NUM_CHANNELS> per_channel{};   // bursts per ELRS channel
	std::array<double, FHSS_NUM_CHANNELS> last_seen_s{};
	int channels_in_window = 0, window_ch_lo = -1, window_ch_hi = -1;
	uint64_t lock_acquired = 0, lock_drops = 0;      // addition: lock transitions (fit/sync, drops)
};

// ---- the RX tracker: rx_burst -> decode -> link model -> packet_entry ------------------------------------
// Thread model: on_burst() runs on the DSP thread; every query below is called from the HTTP
// thread (or the replay driver) -> internal mutex, queries return copies. From inside on_burst
// the tracker may call chain->set_channel_hint() (re-entrancy is guaranteed by rx_chain).
class rx_tracker {
public:
	// chain may be nullptr (no channel hints; C's daemon_selftest drives on_burst with synthetic
	// rx_bursts and must not need B's module to link)
	rx_tracker(packet_ring &ring, rx_chain *chain);
	// UID/rate are runtime configuration; without a UID bursts are logged as "undecodable"
	void set_uid(const uid_t6 &uid);
	bool uid_set() const;
	void set_rate(const rate_params &rp);          // forced rate; in auto mode (rx_chain_config.rate_index 0)
	                                               // the tracker adopts b.rate_index of the majority of the last 8 bursts
	const rate_params &rate() const;
	// called on the DSP thread by rx_chain::set_on_burst
	void on_burst(const rx_burst &b);
	// replay verification (C wires it when cfg.replay_verify && uid set). The provider is keyed
	// by the BURST, not by the tracker's slot counter (whose origin is the first detected burst
	// and therefore arbitrary): the freerun captures put slot k at wide-stream sample
	// 62500 + k*100000 (timeline.npz: burst onsets land 0..7500 samples after that, one
	// outlier at 37500, 16 ppm drift -> k = llround((b.sample_start - 62500) / 100000.0) is
	// unambiguous for the 15 s files), tick = k + FREERUN_TICK_OFFSET_250HZ, nonce = tick % 256.
	void set_truth_provider(std::function<bool(const rx_burst &b, std::array<uint8_t, 8> &pkt_out)> f);
	link_stats stats() const;
	link_lock lock() const;
	int64_t current_slot() const;
	uint8_t nonce_now() const;               // per link_model
	int channel_now() const;
	std::string status_json_fragment() const;   // the "fhss":{...},"counters":{...} parts of /status
	std::vector<uint16_t> last_rc() const;      // 16 CRSF values of the last crc_ok RC packet
private:
	struct impl;
	impl *p_;
	friend struct rx_tracker_hooks;             // the daemon's private extras (daemon_internal.h)
};

// ---- status snapshot -> JSON (GET /status) -----------------------------------------------
struct status_snapshot {
	std::string state;            // "idle" | "listening" | "replay" | "tx_armed"
	double uptime_s = 0;
	const rate_params *rate = nullptr;
	bool uid_set = false;
	daemon_config cfg;
	source_stats src;
	rx_chain::stats rx;
	link_stats link;
	link_lock lock;
	int64_t slot = 0; uint8_t nonce = 0; int channel = -1; std::string nonce_conf;
	tx_status tx;
	std::string board;
	std::string last_error;
	uint64_t total_packets = 0;
	// additions (2026-09-14; defaulted, additive - "Ergaenzungen erlaubt"):
	// the remaining /status fields of that live outside link_stats/link_lock
	uint64_t queue_drops = 0;          // capture -> DSP queue overflow (daemon-owned counter)
	int fhss_index = -1;               // fhss ptr at `slot` (-1 without lock)
	int64_t next_sync_slot = -1;       // link_model::next_sync_slot(slot)
	double interval_est_us = 0;        // slot_clock interval estimate
	packet_entry last_rc;              // last crc_ok RC packet (valid when last_rc_valid)
	bool last_rc_valid = false;
	size_t ws_clients = 0;
};
std::string status_to_json(const status_snapshot &s);

// ---- --replay driver (src/replay.cpp): replay source -> rx_chain -> rx_tracker,
//      ground truth via freerun_channels/ota4_build_rc when a UID is set; prints the report as
//      JSON on stdout; returns the process exit code (0, or 1 when cfg.expect_min_ok is not met).
//      With cfg.conj_auto the driver probes the first 2 s of the file in both IQ signs and runs
//      the full file with the sign that found preambles (replay_report.conj_used).
int run_replay(const daemon_config &cfg, replay_report *out = nullptr);

// ---- daemon entry (src/daemon_main.cpp; `// NEEDS:` every module). Called by the
//      permanent bootstrap src/main.cpp for everything except --selftest. Owns: config parse,
//      the ONE rfnm device (rfnm_device_open, WITH_RFNM), the capture/DSP/HTTP/TX threads,
//      signals, and the TX arm state machine: arm = stop RX source -> radio_tx::start;
//      disarm/auto-disarm/SIGTERM = radio_tx::stop -> restart RX source (v1: no duplex).
int daemon_main(int argc, char **argv);

// ---- optional WebSocket push, RFC 6455 text frames, loopback or LAN bind ------------------
class ws_server {
public:
	ws_server(int port, const std::string &bind_addr = "127.0.0.1");
	~ws_server();
	bool ok() const;
	int port() const;                          // bound port (addition: port 0 = ephemeral, selftest)
	void broadcast(const std::string &json);   // non-blocking; slow clients are dropped
	size_t clients() const;
private:
	struct impl;
	impl *p_;
};

} // namespace elrsd

// elrsd: TX chain - ELRS packet -> LI symbols -> chirp burst (cs16 at the device rate) ->
// slot/hop planner -> radio adapter.
// Hardware rules: TX is OFF at start, armed only through POST /tx/arm behind
// three gates (board.conf TX_ALLOWED x --tx 1 x runtime arm with a power cap + auto-disarm).
// In this workflow the radio adapter is only exercised through the MOCK; the loopback
// selftest feeds the synthesized cs16 into the rx_chain (rx_chain.h) and must get the
// payload bytes back. No over-the-air transmission, no deploy.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "elrs_codec.h"
#include "elrs_link.h"
// NOTE: deliberately NO #include "rx_chain.h" - nothing in this header uses B's types. Only
// src/tx_selftest.cpp (the loopback) includes rx_chain.h and declares `// NEEDS: src/rx_chain.cpp`.
// burst_synth generates chirps at the DEVICE rate with its own phase accumulator; it must not
// use css_modem::upchirp() (that lives at os * bw, and would link D against B).

namespace elrsd {

// ---- chirp synthesis ---------------------------------------------------------------------------
struct burst_synth_config {
	double fs_hz = 25e6;            // device sample rate (os = fs / 812.5 kHz = 30.77 - NOT integer)
	double bw_hz = ELRS_BW_HZ;
	int16_t amplitude = 14000;      // fleet amplitude (~0.43 FS); never above 14000
	bool iq_invert = false;         // SetRFLinkRate invertIQ = uid[5] & 1 (mirror the chirps when set)
};

// Synthesize one ELRS burst at the device rate with the carrier at baseband_offset_hz:
//   preamble_len up-chirps + 2 sync-word symbols + 2.25 down-chirps
//   + (SF5/6: 2 fine-sync symbols) + payload symbols. Chirps are generated directly at fs
//   (phase-accumulator, no resampling): symbol s of width N=2^SF is the up-chirp with a
//   cyclic shift of s bins, Tsym = N / bw. Output cs16 interleaved, `samples` complex values.
// sync_word symbols: value pairs derived like lorad (((sync>>4)&0xF)<<3, (sync&0xF)<<3) with
// sync = 0x12 (SX1280 private default, unverified for ELRS - the RX ignores them).
class burst_synth {
public:
	burst_synth(const rate_params &rp, const burst_synth_config &cfg);
	// payload symbols (from li_codec::encode) -> cs16 burst; returns the number of complex samples
	size_t synth(const std::vector<uint16_t> &symbols, double baseband_offset_hz, std::vector<int16_t> &out) const;
	size_t burst_samples() const;      // round(burst_s * fs)
	const rate_params &rate() const { return rp_; }
private:
	rate_params rp_;
	burst_synth_config cfg_;
};

// ---- continuous feed core (host-testable; shared by the rfnm adapter and the mock) -----------------
// Bursts at absolute feed positions -> ONE gapless sample stream: every sample between the
// bursts is written explicitly (zeros, or the idle tone of --selfloop-cw), nothing is seeked over.
// This is the TX shape the bundle has measured on the board (vrtd/lorad/adsld: tx_flags = 0
// packets in 256-multiples, back to back); the sparse write_at/seek/EOB shape of librfnm's
// tx_stream was never seen airing here (2026-09-14 self-loop: host "sent" 2770 bursts, RX heard
// only the LO leak). The feeder pulls next() chunk by chunk at the frontier; queue_burst() is
// called from the scheduler thread with positions >= frontier + (lead the adapter reports).
class tx_feed {
public:
	struct stats {
		uint64_t queued = 0, fed = 0;      // bursts accepted / completely emitted
		uint64_t refused_late = 0;         // pos < frontier (already filled) or overlapping the previous burst
		uint64_t zero_samples = 0, burst_samples = 0;
	};
	explicit tx_feed(uint64_t frontier = 0);
	// the idle fill between bursts: amplitude 0 = zeros; otherwise a tone at cycles_per_sample
	// (offset_hz / fs), phase continuous across chunks
	void set_idle_tone(double cycles_per_sample, int16_t amplitude);
	// burst at an absolute feed position; false + *err when pos < frontier (that span is already
	// written), pos < end of the previously queued burst, or when the queue is full (64)
	bool queue_burst(uint64_t pos, const int16_t *iq, size_t samples, std::string *err = nullptr);
	// emit the next <= max_samples samples of the stream at the frontier into out (cs16
	// interleaved) and advance the frontier; never returns 0 for max_samples > 0
	size_t next(int16_t *out, size_t max_samples);
	uint64_t frontier() const;
	size_t pending() const;                 // bursts queued but not yet fully emitted
	stats get_stats() const;
	void reset(uint64_t frontier);          // drop every pending burst, move the frontier (session restart)
private:
	struct impl;
	impl *p_;
};

// ---- radio adapter (continuous free-run TX over librfnm tx_stream, or the mock) --------------------
enum class tx_policy : uint8_t { NONE = 0, EMBED = 1, ALL = 2 };   // == vsg_tx_policy
const char *tx_policy_str(tx_policy p);
tx_policy tx_policy_from_boardconf(const char *path = "/rfnm/board.conf", std::string *board = nullptr);

struct tx_timing {
	uint64_t tick_hz = 0;      // dcs_clk/2
	uint32_t r_num = 0, r_den = 1;   // ticks per sample
	uint64_t t0_ext = 0;       // TX epoch anchor (extended tick)
	uint32_t epoch = 0;        // TX stream generation the anchor belongs to
	// the lead the SCHEDULER must keep in front of "now" when it queues a burst: on the rfnm
	// adapter = the feeder's pace lead (device contract, >= 2 ms) + 1 ms feeder jitter - NOT the
	// bare device contract (the continuous feeder has already written zeros up to now + pace)
	uint32_t feed_lead_ticks = 0;
	uint64_t now_tick = 0;     // clock_now at query
	// the device's TX anchor (t0/epoch) differs from the one the feed session was started on
	// (a stream re-apply / self-heal re-minted it): every further stamp would be judged on a
	// grid the scheduler does not compute on -> the scheduler restarts the session (rebase)
	bool anchor_moved = false;
};

struct tx_health {
	uint64_t writes = 0, packets = 0, short_flushes = 0, seeks = 0, drops = 0, rebases = 0, stalls = 0;
	uint64_t pos_late = 0, underruns = 0, timed_rejects = 0;
	uint64_t refusals = 0;           // start()/write_at() refused by the ADAPTER's own gate (policy NONE, cap)
	int headroom_peak_abs = 0;
	// continuous feed (tx_feed + device placement, since session start)
	uint64_t bursts_queued = 0, bursts_fed = 0, bursts_late = 0;   // late = refused: behind the feed frontier
	uint64_t zero_samples = 0, burst_samples = 0;                  // what the feeder actually wrote
	uint64_t pos_placed = 0, pos_misaligned = 0;                   // device: POS packets placed / off-grid (delta since session start)
	uint64_t anchor_moves = 0;                                     // re-mints detected by timing()
	int64_t frontier_lead_us = 0;                                  // last measured: feed frontier air time - now
	std::string last_error;
};

struct tx_start_params {
	double fs_hz = 25e6;
	double freq_lo_hz = 2441.4e6;
	int power_dbm = -60;             // requested; adapter clamps to [power_range.min, min(power_range.max, power_cap_dbm)]
	int power_cap_dbm = -20;         // hard cap (daemon_config.tx_cap_dbm) - enforced AGAIN inside the adapter;
	                                 // a board whose power_range.min lies above the cap is refused, never clamped up
	std::string antenna = "sma_a";  // resolved against path_possible; EMBED policy forces embed or refuses
	int lpf_bw_mhz = 25;             // set_tx_channel_rfic_lpf_bw (MANDATORY, else PROBE_FAIL)
	// false = start() brings the RF chain up (freq/power/path/lpf/RF_ON/apply) but does NOT
	// start the feed session; the caller starts it later with session_start(). The self-loop
	// uses this: the session anchor must be latched AFTER the RX apply on the same device (a
	// stream re-apply re-mints the TX anchor; a session latched before it feeds a dead grid).
	bool start_session = true;
};

// Defense in depth (adsld "layer 3"): the adapter is its own last gate. start() and write_at()
// refuse (false, last_error, health.refusals++) when policy() == NONE, when the resolved path
// violates the policy (EMBED without EMBED_ANT in path_possible -> refuse, never fall back),
// or when power would exceed power_cap_dbm - regardless of what the caller checked. The
// caller (elrs_transmitter::arm) checks the same gates first so the API can answer 403/409.
class radio_tx {
public:
	virtual ~radio_tx() {}
	virtual tx_policy policy() const = 0;
	// bring-up: TX ch0 freq/power/path/lpf/status RF_ON+STREAM_ON, apply(tx mask), then (unless
	// p.start_session == false) the feed session: tx_stream start_earliest -> pos0, continuous
	// feeder thread from there. false + last_error on any refusal (including policy NONE).
	// Precondition on the RFNM adapter in the daemon: the RX source is STOPPED;
	// the self-loop is the exception and uses start_session = false + session_start().
	virtual bool start(const tx_start_params &p, uint64_t *pos0) = 0;
	// start the feed session on a radio whose RF chain is up (start() with start_session=false,
	// or after session_stop()): anchor latched NOW, feeder running, *pos0 = first feed position
	virtual bool session_start(uint64_t *pos0) = 0;
	// re-anchor a running session (anchor_moved, or a retune that broke the TX timeline): feeder
	// stopped, pending bursts dropped, a fresh session latched on the device's current anchor
	virtual bool session_restart(uint64_t *pos0) = 0;
	virtual void stop() = 0;            // feeder + stream stop, RF_OFF + STREAM_AUTO, apply
	virtual bool running() const = 0;   // RF up
	virtual bool session_running() const = 0;
	// queue a burst at an absolute feed position (samples since tx epoch); sample-exact. The
	// feeder emits it when the frontier reaches pos. false = refused (behind the frontier =
	// "late", overlapping the previous burst, queue full, policy NONE, no session)
	virtual bool write_at(uint64_t pos, const int16_t *iq, size_t samples) = 0;
	virtual bool flush(bool end_of_burst = true) = 0;   // no-op on the continuous feed (kept for the API)
	virtual uint64_t pos() = 0;                             // feed frontier (samples written so far)
	// idle fill between bursts: a tone at offset_hz from the LO with this amplitude (0 = zeros).
	// --selfloop-cw uses it to prove the feed reaches the DAC with a plain RX capture.
	virtual void set_idle_tone(double offset_hz, int16_t amplitude) = 0;
	// LO retune on the control path (set_tx_channel_freq + apply(tx mask)); returns the
	// measured wall duration in microseconds (or -1 on failure) and sets *timeline_broken
	// from get_apply_timing (bit1) -> caller restarts the stream when set.
	virtual long retune(double freq_lo_hz, bool *timeline_broken) = 0;
	virtual bool timing(tx_timing &out) = 0;
	virtual void health(tx_health &out) = 0;
	virtual int power_applied_dbm() const = 0;
	virtual std::string path_applied() const = 0;          // rf_path_to_string after apply
	virtual std::string describe() const = 0;
	// (addition, self-loop report) the channel's power_range after start(); false = unknown
	virtual bool power_range(int *min_dbm, int *max_dbm) const { (void)min_dbm; (void)max_dbm; return false; }
};

// host/mock adapter (always built): records every write_at/retune, fake clock, settable policy
struct mock_tx_event { uint64_t pos; double freq_lo_hz; std::vector<int16_t> iq; };
class radio_tx_mock : public radio_tx {
public:
	radio_tx_mock();
	void set_policy(tx_policy p);
	void set_paths(const std::vector<std::string> &possible, const std::string &preferred);
	void set_power_range(int min_dbm, int max_dbm);
	void set_retune_us(long us);                // simulated retune duration
	void set_fake_now(uint64_t tick);
	// model a device clock where tick_hz != fs (e.g. R = 4/2: tick_hz = 2*fs). rnum/rden != 0 report
	// that exact (possibly unreduced) ratio through timing(); 0 = reduce tick_hz/fs by gcd.
	void set_clock(uint64_t tick_hz, uint32_t rnum = 0, uint32_t rden = 0);
	// offset the feed frontier (pos0) from the tick axis at the next session start, WITHOUT moving
	// t0_ext - reproduces the observed reclock/RX-apply re-mint (pos0 jumps ~2^31 samples, anchor stays)
	void set_feed_axis_skew(int64_t samples);
	const std::vector<mock_tx_event> &captured() const;
	std::vector<double> retunes() const;
	// the continuous feed as the "DAC" sees it: pull up to max_samples of the stream at the
	// frontier (zeros / tone / burst content, sample-exact) and append them to out
	size_t drain(std::vector<int16_t> &out, size_t max_samples);
	void set_anchor_moved(bool moved);          // simulate a device re-mint (timing() reports it once per session)
	int session_starts() const;                 // start()+session_start()+session_restart() count
	// radio_tx
	tx_policy policy() const override;
	bool start(const tx_start_params &p, uint64_t *pos0) override;
	bool session_start(uint64_t *pos0) override;
	bool session_restart(uint64_t *pos0) override;
	void stop() override;
	bool running() const override;
	bool session_running() const override;
	bool write_at(uint64_t pos, const int16_t *iq, size_t samples) override;
	bool flush(bool end_of_burst) override;
	uint64_t pos() override;
	void set_idle_tone(double offset_hz, int16_t amplitude) override;
	long retune(double freq_lo_hz, bool *timeline_broken) override;
	bool timing(tx_timing &out) override;
	void health(tx_health &out) override;
	int power_applied_dbm() const override;
	std::string path_applied() const override;
	std::string describe() const override;
	bool power_range(int *min_dbm, int *max_dbm) const override;
private:
	struct impl;
	impl *p_;
};

// librfnm adapter (src/tx_rfnm.cpp, WITH_RFNM only). dev_handle is the process's ONE
// rfnm::device* from rfnm_device_open() in sample_source.h (void* so this header stays
// librfnm-free) - the same handle the RX source uses; the adapter never opens a device.
// Returns nullptr in host builds and when policy == NONE (no TX object exists at all then).
radio_tx *make_rfnm_tx(void *dev_handle, tx_policy policy);

// ---- hop planner + burst scheduler ---------------------------------------------------------------
struct tx_plan_config {
	double fs_hz = 25e6;
	double window_hz = 25e6;         // usable = fs; a channel is "digital" if |f_ch - f_lo| <= window_hz/2 - guard
	double guard_hz = 1.5e6;
	double lo_min_offset_hz = 0.5e6; // prefer |offset| >= this (LO leak -12..-15 dBc)
	double gap_min_s = 0.5e-3;       // do not retune if the slot gap is shorter
	int lookahead_slots = 2;         // max bursts written ahead of the feed cursor (within one LO setting)
	double lead_margin_s = 1.0e-3;   // extra lead on top of feed_lead_ticks
	bool wide_window = false;        // "all digital" strategy (>= 80 MHz windows) - never enabled without a measurement
};
// Retune rule: a written burst cannot be unwritten (SCHED_ORDER), so a slot with
// retune_before is written ONLY AFTER radio_tx::retune() returned successfully - the lookahead
// stops at the retune boundary. If the retune finishes too late for the lead rule the slot is
// skipped (skip_reason "retune_late"), never aired on the old LO. wide_window stays false.

struct slot_plan {
	int64_t slot;
	int channel;                  // 0..79
	double f_ch_hz;               // SX1280-exact
	double f_lo_hz;               // LO required for this slot
	double baseband_offset_hz;    // f_ch - f_lo
	bool retune_before;           // LO must move before this slot
	uint64_t feed_pos;            // absolute feed position of the burst start
	bool skip;                    // dropped (lead violated / retune impossible)
	const char *skip_reason;
};

class hop_planner {
public:
	hop_planner(const tx_plan_config &cfg, const rate_params &rp);
	// choose a window centre for the channels of the next `n` dwells starting at ptr
	static double choose_lo(const std::array<uint8_t, FHSS_SEQ_LEN> &seq, int ptr, int n_dwells, const tx_plan_config &cfg);
	// plan one slot given the link model (channel_at) and the current LO
	slot_plan plan_slot(const link_model &lm, int64_t slot, double current_lo_hz, uint64_t slot0_feed_pos, int64_t slot0) const;
	const tx_plan_config &config() const { return cfg_; }
	uint64_t slot_samples() const;   // interval_us * fs / 1e6 (100 000 @ 25 MSps / 4 ms)
private:
	tx_plan_config cfg_;
	rate_params rp_;
};

// ---- the transmitter (runs the slot loop on its own thread) -------------------------------------------
struct tx_arm_request {
	int power_dbm = -60;
	std::string antenna = "sma_a";
	double duration_s = 30;         // auto-disarm (0 = until disarm, capped by max_duration_s)
	uint64_t max_bursts = 0;        // 0 = unlimited within duration
	int rate_index = 6;
	bool sync_packets = true;       // send SYNC on channel 41 visits
	int tlm_denom = 64;
	// --selfloop-cw: no bursts at all; the adapter's idle tone (cw_offset_hz from the LO,
	// cw_amplitude) is the only TX content. Every arming gate applies unchanged.
	bool cw_only = false;
	double cw_offset_hz = 1.0e6;
	int16_t cw_amplitude = 14000;
};

struct tx_status {
	bool enabled = false;           // --tx 1 given
	bool armed = false;
	tx_policy policy = tx_policy::NONE;
	std::string board;
	std::string antenna;            // applied path
	int power_dbm_requested = 0, power_dbm_applied = 0, power_cap_dbm = -60, power_min_dbm = 0, power_max_dbm = 0;
	double freq_lo_hz = 0, window_hz = 0;
	int64_t slot = 0; uint8_t nonce = 0; int fhss_ptr = 0; int channel = -1;
	uint64_t bursts_sent = 0, bursts_dropped = 0, syncs_sent = 0, retunes = 0;
	double retune_ms_p50 = 0, retune_ms_p99 = 0, retune_ms_max = 0;
	double armed_remaining_s = 0;
	bool session = false;           // feed session running (false between arm(..., start_session=false) and start_session())
	bool cw = false;                // cw_only arm
	double cw_offset_hz = 0;
	tx_health health;
	std::string last_error;
};

// Thread model: arm()/disarm()/status()/set_rc_channels() are called from the HTTP thread while
// the scheduler loop runs on the transmitter's own thread -> all public methods are
// thread-safe (internal mutex; status() returns a copy). The daemon never instantiates
// radio_tx_mock outside --selftest/--loopback; on a board build make_rfnm_tx() == nullptr
// (host build, or policy NONE) means POST /tx/arm answers 409 "no tx radio".
class elrs_transmitter {
public:
	// radio: mock or rfnm adapter (owned by the caller). cap_dbm: hard power cap from config.
	elrs_transmitter(radio_tx &radio, const tx_plan_config &plan, int cap_dbm, bool enabled_by_cli);
	~elrs_transmitter();
	void set_uid(const uid_t6 &uid);                  // required before arm
	void set_rc_channels(const std::array<uint16_t, 16> &crsf);   // live RC values (POST /tx/rc)
	// arm: all gates checked here (policy != NONE, enabled, uid set, power <= cap, antenna vs
	// policy). Returns false with a human-readable reason (-> 403/409 in the API).
	// start_session = false: RF up and armed, but the feed session (anchor + feeder + slot grid)
	// is started later by session_start() - the self-loop's "TX RF on, RX apply, THEN anchor".
	bool arm(const tx_arm_request &req, std::string &reason, bool start_session = true);
	// start (or after a rebase: restart) the feed session on the armed radio and lay the slot
	// grid from its first feed position. false + reason when not armed / the radio refuses.
	bool session_start(std::string &reason);
	void disarm();                                    // idempotent; stops the radio
	bool armed() const;
	tx_status status() const;
	// one scheduler iteration for the mock/selftest: plan + write the next slot (no thread)
	bool step_once(std::string *why = nullptr);
	// (addition) false = arm() starts no scheduler thread, the caller drives step_once();
	// default true (the daemon leaves it on). Call before arm().
	void set_threaded(bool on);
private:
	struct impl;
	impl *p_;
};

// ---- device self-loop verification (--selfloop-device; host-testable) ------------------
// The self-loop transmits the normal elrs_transmitter timeline (nonce0 = 0, ptr0 = 0 at slot 0,
// RC packets with a FIXED deterministic channel pattern, SYNC on channel-41 visits) and the
// receiver runs the normal rx_chain + tracker. The checker decides per decoded packet whether
// its 8 bytes are EXACTLY what the transmitter builds: RC = byte-equal to the pattern packet for
// one of the 256 nonces (crcHigh/aux slot vary with the nonce); SYNC = byte-equal to the SYNC
// the transmitter builds for the (fhss_index, nonce) the packet carries AND that pair must lie
// on the transmitter's slot timeline. Counting only, no I/O; the daemon feeds packet_entry
// kind/crc_ok/hex into it and prints the JSON report.
std::array<uint16_t, 16> selfloop_rc_pattern();     // ch[i] = 172 + lowbias32(0x5e1f0000 + i) % 1640
struct selfloop_counts {
	uint64_t packets = 0;          // every tracker entry (incl. undecodable)
	uint64_t undecodable = 0;
	uint64_t crc_ok = 0, crc_fail = 0;
	uint64_t rc = 0, sync = 0, other = 0;        // crc_ok by kind
	uint64_t rc_bitexact = 0, sync_bitexact = 0; // crc_ok AND byte-equal to the transmitted payload
	uint64_t mismatch = 0;                       // crc_ok but NOT what the transmitter sends (wrong uid/pattern/timeline)
	bool ok() const { return crc_ok > 0 && mismatch == 0 && rc_bitexact + sync_bitexact == crc_ok; }
};
std::string selfloop_counts_json(const selfloop_counts &c);   // {"packets":..,"crc_ok":..,...,"ok":bool}
class selfloop_checker {
public:
	selfloop_checker(const uid_t6 &uid, const rate_params &rp, const std::array<uint16_t, 16> &rc, int tlm_denom);
	// kind: "rc" | "sync" | "msp" | "tlm" | "undecodable" (packet_entry.kind); pkt8 = the decoded
	// 8 bytes (nullptr when undecodable)
	void on_packet(const std::string &kind, bool crc_ok, const uint8_t *pkt8);
	bool rc_expected(const uint8_t pkt[8]) const;
	bool sync_expected(const uint8_t pkt[8]) const;
	selfloop_counts counts() const { return counts_; }
	static bool hex_to_bytes(const std::string &hex16, uint8_t out[8]);
private:
	uid_t6 uid_;
	rate_params rp_;
	int tlm_denom_;
	std::vector<std::array<uint8_t, 8>> rc_candidates_;      // 256 nonce variants (deduplicated)
	std::vector<uint8_t> sync_pair_ok_;                       // [ptr * 256 + nonce] = 1 when on the tx timeline
	selfloop_counts counts_;
};

// selftest: synth -> rx_chain loopback for every verified rate (payload byte-equal,
// with and without iq_invert, with a baseband offset of +3.7 MHz and 0.6 MHz), planner
// tests on the mock (positions on the slot grid, lead rule, retune-in-gap, skip on late,
// lookahead cap, wide_window off), arming matrix (policy x enabled x uid x power cap x antenna),
// auto-disarm, the retune p50/p99 bookkeeping, and the self-loop checker (pattern, RC/SYNC
// byte-equality, timeline consistency, counting, JSON).
int run_selftest_tx();

} // namespace elrsd

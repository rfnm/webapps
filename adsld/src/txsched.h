// adsld: pure-logic TX scheduler + MAC (no radio, no threads, no wall clock - everything
// arrives through tick(), so the selftest drives it with a fake clock). Implements the
// ADS-L M-band channel access rules:
//   - Traffic broadcast >= 1 Hz airborne, 0.1 Hz on ground (flightstate-driven).
//   - Status (payload type 3) at least every status_interval_s, 15 s by default - G.2.11
//     asks for at least one every 20 s, and listen-before-talk delays every frame a little.
//   - Traffic and Status each alternate 868.2 / 868.4 MHz on every transmission of their
//     own kind (ADS-L.4.SRD860.C.2.5).
//   - LBT 1-persistent CSMA per frame kind: when a frame's channel is busy it backs off a
//     uniform random LBT_BACKOFF_MIN_MS..LBT_BACKOFF_MAX_MS and re-listens. After
//     LBT_FORCE_TX_MS measured from THAT frame's first busy attempt on ITS channel, the frame
//     is forced (D.3 permits it for any packet), then nothing is sent for LBT_QUIET_MS.
//   - While one frame waits for its busy channel, the other kind may use the other, free
//     channel. A Status frame overdue by STATUS_OVERDUE_MS outranks Traffic, so a
//     permanently busy channel cannot starve Status.
//   - No frame of any kind on a channel within TOFF_MIN_MS of the previous frame on that
//     channel (EN 300 220-2 V3.3.1 Table 18, minimum Toff at the same operating frequency) -
//     including a manual beacon (claim_manual_traffic) and across reconfigure().
//   - Position data older than GNSS_MAX_AGE_MS is not sent (skip the slot).
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "adsl_codec.h"
#include "adsl_phy.h"
#include "conform.h"
#include "position.h"

namespace adsld {

// The beacon template's defaults where the daemon's differ from the codec's: Emergency
// Status 1 "No emergency" (ADS-L.4.SRD860.G.1.4) - 0 "Undefined" says less than a station
// that knows it has no emergency. The codec's own default stays 0 (reference vectors).
inline adsl_traffic default_beacon_template() {
	adsl_traffic t;
	t.emergency = 1;
	return t;
}

struct txsched_config {
	double interval_airborne_s = 1.0;   // beacon interval when airborne (>= 1 Hz)
	double interval_ground_s = 10.0;    // beacon interval on ground (0.1 Hz)
	position_source *pos = nullptr;     // NOT owned; must outlive the scheduler
	// sender + static fields: amt/addr/category/emergency/quality are taken from this
	// template; timestamp, flightstate and kinematics are overwritten from the fix.
	adsl_traffic tmpl = default_beacon_template();
	uint32_t seed = 1;                  // backoff PRNG seed (deterministic tests)
	// Radio clock -> UTC. Set: every Traffic Timestamp is the UTC of its fix, quarter seconds
	// since the full hour mod 60 (ADS-L.4.SRD860.G.1.1). Empty: the fix's own ts_qs is sent
	// unchanged (the position sources count from the radio clock - tests only).
	utc_clock_fn utc_ms_at;
	// Status payload period. ADS-L.4.SRD860.G.2.11: "The minimum nominal transmission rate
	// is once per 20 seconds", and the Declaration of Conformity names Status next to
	// Traffic - so it is on by default, with margin: LBT delays each frame by up to
	// LBT_BACKOFF_MAX_MS, so a 20 s schedule would overrun. 0 turns Status off: only for tests
	// that pin the Traffic cadence, never for a station that claims conformance.
	// 3.28 ms per 15 s = 0.022 %.
	double status_interval_s = 15.0;
	// the capability fields of every Status frame; amt/addr/relay are taken from tmpl at
	// transmit time, so a POST /config address change reaches Status and Traffic together
	adsl_status status_tmpl;
};

// What a tx_request carries - for airtime accounting.
enum class tx_kind {
	traffic,     // ADS-L iConspicuity Traffic (payload type 2)
	status,      // ADS-L Status (payload type 3)
};

struct tx_request {
	int channel = 0;                    // index into ADSL_CH_HZ
	// the on-air packet: ADSL_PKT_LEN bytes for Traffic, ADSL_STATUS_PKT_LEN for Status
	std::vector<uint8_t> packet;
	tx_kind kind = tx_kind::traffic;
};

class txsched {
public:
	// ---- MAC constants (milliseconds) ------------------------------------------------
	static constexpr int LBT_BACKOFF_MIN_MS = 15;
	static constexpr int LBT_BACKOFF_MAX_MS = 250;
	static constexpr int LBT_FORCE_TX_MS = 3000;    // force after this much busy (own channel)
	static constexpr int LBT_QUIET_MS = 2000;       // min quiet time after a forced TX
	static constexpr int GNSS_MAX_AGE_MS = 500;     // GNSS-to-air latency budget
	// channel-busy decision: rssi above noise floor by this margin = occupied
	static constexpr float LBT_BUSY_MARGIN_DB = 6.0f;
	// polite spectrum access: minimum off time after a transmission on the same operating
	// frequency (EN 300 220-2 V3.3.1 Table 18).
	static constexpr int TOFF_MIN_MS = 100;
	// a Status frame this late outranks Traffic - one blocked episode (force + quiet) past
	// its due time - so a channel that stays busy cannot keep Status off the air
	static constexpr int STATUS_OVERDUE_MS = LBT_FORCE_TX_MS + LBT_QUIET_MS;

	// scheduler state, exposed for the selftest and GET /status
	enum class state {
		idle,      // nothing waiting for a busy channel
		backoff,   // at least one due frame is backing off from its busy channel
		quiet,     // post-forced-TX quiet period
	};

	void init(const txsched_config &cfg);

	// Take a changed configuration (POST /config: address, category, intervals, ...) WITHOUT
	// resetting the MAC: channel alternation, Toff per channel, the next Traffic slot and the
	// Status schedule carry on.
	void reconfigure(const txsched_config &cfg);

	// Advance to now_ms. rssi_db / noise_floor_db point to ADSL_NUM_CHANNELS entries of
	// current per-channel measurements (receiver.rssi_db()/noise_floor_db()). Returns a
	// burst to hand to the modulator + radio right now, or nullopt. Call frequently
	// (every RX block, ~ms granularity); never blocks.
	std::optional<tx_request> tick(uint64_t now_ms, const float *rssi_db,
	                               const float *noise_floor_db);

	// A Traffic frame sent outside tick() (manual POST /tx) on channel `ch`. Returns false
	// while the channel access rules hold it back - quiet period after a forced frame,
	// TOFF_MIN_MS on `ch`, or `ch` busy - and the caller retries later. true = the caller
	// transmits now; the frame is recorded for Toff and the Traffic alternation continues on
	// the other channel.
	bool claim_manual_traffic(int ch, uint64_t now_ms, const float *rssi_db,
	                          const float *noise_floor_db);

	state current_state() const { return state_; }
	int next_channel() const { return traffic_channel_; }        // the next Traffic frame's
	int next_status_channel() const { return status_channel_; }
	uint64_t tx_count() const { return tx_count_; }    // ALL frames handed out
	uint64_t forced_count() const { return forced_count_; }    // forced Traffic + Status
	uint64_t skipped_count() const { return skipped_count_; }  // slots dropped (stale fix)
	uint64_t status_frames() const { return status_frames_; }  // Status frames handed out
	uint64_t status_forced() const { return status_forced_; }  // ... of which forced

private:
	// the frame kinds the MAC arbitrates; declaration order = normal priority
	enum class pending {
		traffic,
		status,
	};

	// listen-before-talk bookkeeping of one frame kind: D.3 counts the forcing deadline from
	// the frame's initial attempt, so every kind keeps its own clock
	struct lbt_wait {
		bool waiting = false;    // this kind's due frame found its channel busy
		uint64_t since = 0;      // first busy attempt
		uint64_t retry_at = 0;   // end of the current random backoff
	};

	bool toff_clear(int ch, uint64_t now_ms) const;

	txsched_config cfg_;
	state state_ = state::idle;
	int traffic_channel_ = 0;       // alternates on every Traffic frame (C.2.5)
	int status_channel_ = 1;        // alternates on every Status frame (C.2.5)
	uint64_t next_slot_ms_ = 0;     // when the next beacon is due
	uint64_t next_status_ms_ = 0;   // when the next Status frame is due
	uint64_t quiet_until_ms_ = 0;
	uint64_t last_tx_ms_[ADSL_NUM_CHANNELS] = {0};   // last frame per channel (TOFF_MIN_MS)
	bool have_tx_[ADSL_NUM_CHANNELS] = {false};
	lbt_wait wait_[2];              // indexed by pending
	uint64_t tx_count_ = 0;
	uint64_t forced_count_ = 0;
	uint64_t skipped_count_ = 0;
	uint64_t status_frames_ = 0;
	uint64_t status_forced_ = 0;
	uint32_t rng_ = 1;              // xorshift backoff PRNG state
};

} // namespace adsld

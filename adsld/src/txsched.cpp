// adsld: TX scheduler / MAC implementation - see txsched.h for the channel-access rules.
// Pure logic: the only inputs are tick()'s clock and the per-channel rssi/noise arrays,
// the only outputs are ready-to-modulate packets (25 bytes Traffic, 17 bytes Status) -
// fully deterministic under the selftest's fake clock and fixed seed.
#include "txsched.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace adsld {

namespace {

// dynamic Traffic fields come from the fix; sender/quality stay from the template
void fill_from_fix(adsl_traffic &t, const position_fix &f) {
	t.ts_qs = f.ts_qs;
	t.flightstate = f.flightstate;
	t.pos_valid = f.valid;
	t.lat_deg = f.lat_deg;
	t.lon_deg = f.lon_deg;
	t.alt_valid = f.valid;
	t.alt_m = f.alt_m;
	t.speed_valid = f.kinematics_valid;
	t.speed_mps = f.speed_mps;
	t.vrate_valid = f.kinematics_valid;
	t.vrate_mps = f.vrate_mps;
	t.track_valid = f.kinematics_valid;
	t.track_deg = f.track_deg;
}

bool channel_busy(const float *rssi_db, const float *noise_floor_db, int ch) {
	return rssi_db[ch] > noise_floor_db[ch] + txsched::LBT_BUSY_MARGIN_DB;
}

} // namespace

void txsched::init(const txsched_config &cfg) {
	cfg_ = cfg;
	rng_ = cfg.seed ? cfg.seed : 1;
	state_ = state::idle;
	traffic_channel_ = 0;
	status_channel_ = 1 % ADSL_NUM_CHANNELS;   // the first Status on the other channel
	next_slot_ms_ = 0;   // first beacon slot is due immediately
	next_status_ms_ = 0; // ... and the first Status right behind it
	quiet_until_ms_ = 0;
	for(int c = 0; c < ADSL_NUM_CHANNELS; c++) {
		last_tx_ms_[c] = 0;
		have_tx_[c] = false;
	}
	for(auto &w : wait_) {
		w = lbt_wait{};
	}
	tx_count_ = 0;
	forced_count_ = 0;
	skipped_count_ = 0;
	status_frames_ = 0;
	status_forced_ = 0;
}

void txsched::reconfigure(const txsched_config &cfg) {
	cfg_ = cfg;
}

bool txsched::toff_clear(int ch, uint64_t now_ms) const {
	return !have_tx_[ch] || now_ms >= last_tx_ms_[ch] + (uint64_t)TOFF_MIN_MS;
}

bool txsched::claim_manual_traffic(int ch, uint64_t now_ms, const float *rssi_db,
                                   const float *noise_floor_db) {
	if(ch < 0 || ch >= ADSL_NUM_CHANNELS || now_ms < quiet_until_ms_ || !toff_clear(ch, now_ms) ||
	   channel_busy(rssi_db, noise_floor_db, ch)) {
		return false;
	}
	have_tx_[ch] = true;
	last_tx_ms_[ch] = now_ms;
	traffic_channel_ = (ch + 1) % ADSL_NUM_CHANNELS;
	return true;
}

std::optional<tx_request> txsched::tick(uint64_t now_ms, const float *rssi_db,
                                        const float *noise_floor_db) {
	auto busy_on = [&](int ch) { return channel_busy(rssi_db, noise_floor_db, ch); };
	// xorshift32 backoff draw, uniform LBT_BACKOFF_MIN_MS..LBT_BACKOFF_MAX_MS
	auto backoff_ms = [&]() {
		rng_ ^= rng_ << 13;
		rng_ ^= rng_ >> 17;
		rng_ ^= rng_ << 5;
		return (uint64_t)(LBT_BACKOFF_MIN_MS +
		                  rng_ % (uint32_t)(LBT_BACKOFF_MAX_MS - LBT_BACKOFF_MIN_MS + 1));
	};
	auto toff_ok = [&](int ch) { return toff_clear(ch, now_ms); };
	auto mark_tx = [&](int ch) {
		have_tx_[ch] = true;
		last_tx_ms_[ch] = now_ms;
	};
	auto want_traffic = [&]() { return now_ms >= next_slot_ms_; };
	auto want_status = [&]() {
		return cfg_.status_interval_s > 0 && now_ms >= next_status_ms_;
	};
	auto status_overdue = [&]() {
		return want_status() && now_ms >= next_status_ms_ + (uint64_t)STATUS_OVERDUE_MS;
	};
	// a Traffic frame will want its channel within TOFF_MIN_MS: Status keeps off it, so the
	// Toff guard can never make the primary duty wait for a secondary frame
	auto traffic_imminent = [&]() { return next_slot_ms_ <= now_ms + (uint64_t)TOFF_MIN_MS; };

	// The channel a frame of kind p would use right now, or -1 if it has to wait without
	// listening: Toff on its channel, or the channel an imminent Traffic frame needs.
	auto channel_for = [&](pending p) -> int {
		if(p == pending::traffic) {
			return toff_ok(traffic_channel_) ? traffic_channel_ : -1;
		}
		if(!toff_ok(status_channel_)) {
			return -1;
		}
		if(status_channel_ == traffic_channel_ && traffic_imminent() && !status_overdue()) {
			return -1;
		}
		return status_channel_;
	};

	// transmit a Status frame: sender from the Traffic template, capabilities from
	// status_tmpl, its own channel alternation
	auto transmit_status = [&](int ch, bool forced) -> std::optional<tx_request> {
		wait_[(int)pending::status] = lbt_wait{};
		if(forced) {
			forced_count_++;
			status_forced_++;
			quiet_until_ms_ = now_ms + LBT_QUIET_MS;
		}
		adsl_status s = cfg_.status_tmpl;
		s.amt = cfg_.tmpl.amt;
		s.addr = cfg_.tmpl.addr;
		s.relay = false;
		tx_request req;
		req.channel = ch;
		req.kind = tx_kind::status;
		req.packet.resize(ADSL_PKT_LEN);
		req.packet.resize(adsl_status_encode(s, req.packet.data()));
		status_channel_ = (ch + 1) % ADSL_NUM_CHANNELS;
		const double interval_s = std::max(cfg_.status_interval_s, 1.0);
		next_status_ms_ = now_ms + (uint64_t)std::llround(interval_s * 1000.0);
		mark_tx(ch);
		tx_count_++;
		status_frames_++;
		return req;
	};
	// transmit Traffic right now: pull the freshest fix (the <= GNSS_MAX_AGE_MS check
	// happens at the moment of transmission, so backoff time never eats the GNSS-to-air
	// budget unnoticed), encode, alternate the channel, schedule the next slot
	auto transmit_traffic = [&](int ch, bool forced) -> std::optional<tx_request> {
		wait_[(int)pending::traffic] = lbt_wait{};
		position_fix fix;
		bool have = cfg_.pos && cfg_.pos->get(now_ms, fix);
		double interval_s = (have && fix.flightstate == 1) ? cfg_.interval_ground_s
		                                                   : cfg_.interval_airborne_s;
		next_slot_ms_ = now_ms + (uint64_t)std::llround(interval_s * 1000.0);
		if(!have || fix.time_ms > now_ms || now_ms - fix.time_ms > GNSS_MAX_AGE_MS) {
			skipped_count_++;   // no or stale position: drop the slot, never send old data
			return std::nullopt;
		}
		if(forced) {
			// forced through a busy channel: mandatory quiet period afterwards
			forced_count_++;
			quiet_until_ms_ = now_ms + LBT_QUIET_MS;
		}
		adsl_traffic t = cfg_.tmpl;
		fill_from_fix(t, fix);
		if(cfg_.utc_ms_at) {
			// G.1.1: the time of the navigation solution, from the full UTC hour
			t.ts_qs = adsl_ts_qs_utc(cfg_.utc_ms_at(fix.time_ms));
		}
		tx_request req;
		req.channel = ch;
		req.packet.resize(ADSL_PKT_LEN);
		adsl_encode(t, req.packet.data());
		traffic_channel_ = (ch + 1) % ADSL_NUM_CHANNELS;
		mark_tx(ch);
		tx_count_++;
		return req;
	};

	auto run = [&]() -> std::optional<tx_request> {
		if(now_ms < quiet_until_ms_) {
			return std::nullopt;         // after a forced frame nothing goes out (D.3)
		}
		static constexpr pending NORMAL[2] = { pending::traffic, pending::status };
		static constexpr pending OVERDUE[2] = { pending::status, pending::traffic };
		const pending *order = status_overdue() ? OVERDUE : NORMAL;
		for(int i = 0; i < 2; i++) {
			const pending p = order[i];
			const bool wanted = p == pending::traffic ? want_traffic() : want_status();
			if(!wanted) {
				continue;
			}
			lbt_wait &w = wait_[(int)p];
			if(w.waiting && now_ms < w.retry_at) {
				continue;                // backing off; a lower kind may use the other channel
			}
			const int ch = channel_for(p);
			if(ch < 0) {
				continue;
			}
			if(!busy_on(ch)) {
				std::optional<tx_request> r = p == pending::traffic ? transmit_traffic(ch, false)
				                                                    : transmit_status(ch, false);
				if(r) {
					return r;
				}
				continue;                // a Traffic slot without a fresh fix was dropped
			}
			if(!w.waiting) {
				w.waiting = true;        // D.3: the deadline counts from the initial attempt
				w.since = now_ms;
			}
			if(now_ms - w.since >= (uint64_t)LBT_FORCE_TX_MS) {
				std::optional<tx_request> r = p == pending::traffic ? transmit_traffic(ch, true)
				                                                    : transmit_status(ch, true);
				if(r) {
					return r;
				}
				continue;
			}
			w.retry_at = now_ms + backoff_ms();   // 1-persistent CSMA: random backoff
		}
		return std::nullopt;
	};

	std::optional<tx_request> r = run();
	if(now_ms < quiet_until_ms_) {
		state_ = state::quiet;
	} else {
		bool waiting = false;
		for(const auto &w : wait_) {
			waiting = waiting || w.waiting;
		}
		state_ = waiting ? state::backoff : state::idle;
	}
	return r;
}

} // namespace adsld

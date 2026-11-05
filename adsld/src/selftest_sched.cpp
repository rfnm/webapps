// adsld selftest: scheduler/MAC cases driven by a fake clock - beacon intervals
// (airborne/ground), channel alternation, LBT backoff bounds, forced TX after 3000 ms
// busy + 2000 ms quiet, stale-fix slot skipping; mock-radio TX gating.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "radio.h"
#include "selftest.h"
#include "txsched.h"

namespace adsld {

namespace {

// controllable position source: freshness (age_ms), flightstate, fix validity, data loss
class fake_pos : public position_source {
public:
	bool have_data = true;   // get() return value
	bool valid = true;       // fix validity (false = broadcast "no fix")
	uint8_t fs = 1;          // 1 = ground, 2 = airborne
	uint64_t age_ms = 0;     // how stale the stamped fix is
	double lat = 48.35, lon = 11.79;
	float alt = 480;

	bool get(uint64_t now_ms, position_fix &out) override {
		if(!have_data) {
			return false;
		}
		out = position_fix();
		out.valid = valid;
		out.lat_deg = lat;
		out.lon_deg = lon;
		out.alt_m = alt;
		out.kinematics_valid = false;
		out.flightstate = fs;
		out.time_ms = now_ms >= age_ms ? now_ms - age_ms : 0;
		out.ts_qs = (uint8_t)((now_ms / 250) % 60);
		return true;
	}
};

txsched_config base_cfg(fake_pos *pos, uint32_t seed) {
	txsched_config c;
	c.interval_airborne_s = 1.0;
	c.interval_ground_s = 10.0;
	c.pos = pos;
	c.tmpl.amt = 2;
	c.tmpl.addr = 0x3D5C01;
	c.tmpl.category = 9;
	c.seed = seed;
	// These cases pin the Traffic cadence and count every frame; Status (on by default) has
	// its own cases in selftest_status.cpp.
	c.status_interval_s = 0;
	return c;
}

constexpr float QUIET_RSSI = -95.0f, QUIET_NOISE = -95.0f;   // channel free
constexpr float BUSY_RSSI = -75.0f;                          // 20 dB over the floor

} // namespace

int selftest_sched() {
	int fails = 0;
	char detail[256];

	// ---- beacon intervals + channel alternation + packet content (ground, 0.1 Hz) ------
	{
		fake_pos pos;
		txsched sched;
		sched.init(base_cfg(&pos, 7));
		float rssi[ADSL_NUM_CHANNELS] = { QUIET_RSSI, QUIET_RSSI };
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		std::vector<uint64_t> times;
		std::vector<int> chans;
		std::vector<std::vector<uint8_t>> pkts;
		for(uint64_t t = 0; t <= 120000; t += 10) {
			auto req = sched.tick(t, rssi, noise);
			if(req) {
				times.push_back(t);
				chans.push_back(req->channel);
				pkts.push_back(req->packet);
			}
		}
		// slots at 0, 10 s, ..., 120 s = 13 transmissions, exactly 10 s apart
		bool spacing = times.size() == 13;
		for(size_t i = 1; i < times.size() && spacing; i++) {
			spacing = times[i] - times[i - 1] == 10000;
		}
		snprintf(detail, sizeof(detail), "%zu tx in 120 s", times.size());
		check(fails, spacing, "sched ground interval 0.1 Hz", detail);

		bool alt = !chans.empty();
		for(size_t i = 0; i < chans.size() && alt; i++) {
			alt = chans[i] == (int)(i % 2);
		}
		check(fails, alt, "sched channel alternation 0,1,0,1");

		bool content = !pkts.empty();
		for(const auto &p : pkts) {
			adsl_traffic t;
			if(p.size() != ADSL_PKT_LEN ||
			   adsl_decode(p.data(), t) == adsl_decode_result::crc_fail || t.amt != 2 ||
			   t.addr != 0x3D5C01 || t.category != 9 || t.flightstate != 1 || !t.pos_valid ||
			   std::abs(t.lat_deg - 48.35) > 1e-4 || std::abs(t.lon_deg - 11.79) > 1e-4) {
				content = false;
			}
		}
		snprintf(detail, sizeof(detail), "%zu packets decoded", pkts.size());
		check(fails, content, "sched packet content round-trip", detail);
		check(fails, sched.tx_count() == times.size() && sched.forced_count() == 0 &&
		             sched.skipped_count() == 0, "sched counters (ground run)");
	}

	// ---- airborne interval (1 Hz) ------------------------------------------------------
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 8));
		float rssi[ADSL_NUM_CHANNELS] = { QUIET_RSSI, QUIET_RSSI };
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		int n = 0;
		for(uint64_t t = 0; t <= 30000; t += 10) {
			if(sched.tick(t, rssi, noise)) {
				n++;
			}
		}
		snprintf(detail, sizeof(detail), "%d tx in 30 s", n);
		check(fails, n == 31, "sched airborne interval 1 Hz", detail);
	}

	// ---- LBT backoff: bounds + randomness ----------------------------------------------
	// Channel busy exactly when a slot comes due, then immediately free again: the
	// transmission is delayed by exactly one backoff draw, which must lie in
	// [LBT_BACKOFF_MIN_MS, LBT_BACKOFF_MAX_MS].
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 0xB0FF));
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		uint64_t lo = 1 << 30, hi = 0;
		int episodes = 0, txs = 0;
		uint64_t slot_due = 0, t = 0;
		while(episodes < 40 && t < 600000) {
			bool busy_now = t == slot_due;   // busy only at the exact slot tick
			float r = busy_now ? BUSY_RSSI : QUIET_RSSI;
			float rssi[ADSL_NUM_CHANNELS] = { r, r };
			auto req = sched.tick(t, rssi, noise);
			if(busy_now && sched.current_state() == txsched::state::backoff) {
				episodes++;
			}
			if(req) {
				txs++;
				uint64_t delay = t - slot_due;
				if(delay > 0) {
					lo = std::min(lo, delay);
					hi = std::max(hi, delay);
				}
				slot_due = t + 1000;   // airborne interval
				t = slot_due - 1;      // jump to just before the next slot
			}
			t++;
		}
		bool ok = episodes == 40 && lo >= txsched::LBT_BACKOFF_MIN_MS &&
		          hi <= txsched::LBT_BACKOFF_MAX_MS && hi > lo;
		snprintf(detail, sizeof(detail), "%d episodes, backoff %llu..%llu ms", episodes,
		         (unsigned long long)lo, (unsigned long long)hi);
		check(fails, ok, "sched LBT backoff bounds 15..250 ms", detail);
	}

	// ---- LBT backoff uniformity (chi-square smoke) -------------------------------------
	// Same one-draw-per-episode pattern as above, 400 samples binned 8 ways over the
	// 15..250 ms range; a uniform draw keeps the chi-square statistic well under the
	// df=7, p=0.001 critical value (24.3). Deterministic under the fixed seed.
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 0xC41));
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		constexpr int SAMPLES = 400, BINS = 8;
		constexpr int SPAN = txsched::LBT_BACKOFF_MAX_MS - txsched::LBT_BACKOFF_MIN_MS + 1;
		int hist[BINS] = { 0 };
		int got = 0;
		uint64_t slot_due = 0, t = 0, mn = 1 << 30, mx = 0;
		while(got < SAMPLES && t < 3000000) {
			bool busy_now = t == slot_due;   // busy only at the exact slot tick
			float r = busy_now ? BUSY_RSSI : QUIET_RSSI;
			float rssi[ADSL_NUM_CHANNELS] = { r, r };
			auto req = sched.tick(t, rssi, noise);
			if(req) {
				uint64_t delay = t - slot_due;   // == the single backoff draw
				if(delay > 0) {
					mn = std::min(mn, delay);
					mx = std::max(mx, delay);
					int idx = (int)((delay - txsched::LBT_BACKOFF_MIN_MS) * BINS / SPAN);
					hist[std::min(idx, BINS - 1)]++;
					got++;
				}
				slot_due = t + 1000;
				t = slot_due - 1;
			}
			t++;
		}
		double chi2 = 0;
		double exp_n = (double)got / BINS;
		for(int i = 0; i < BINS; i++) {
			double d = hist[i] - exp_n;
			chi2 += d * d / exp_n;
		}
		bool ok = got == SAMPLES && mn >= txsched::LBT_BACKOFF_MIN_MS &&
		          mx <= txsched::LBT_BACKOFF_MAX_MS && chi2 < 24.3;
		snprintf(detail, sizeof(detail), "%d samples in %llu..%llu ms, chi2=%.1f", got,
		         (unsigned long long)mn, (unsigned long long)mx, chi2);
		check(fails, ok, "sched backoff uniformity (chi-square smoke)", detail);
	}

	// ---- forced TX after 3000 ms busy, then 2000 ms quiet ------------------------------
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 3));
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		float busy_r[ADSL_NUM_CHANNELS] = { BUSY_RSSI, BUSY_RSSI };
		float free_r[ADSL_NUM_CHANNELS] = { QUIET_RSSI, QUIET_RSSI };
		// jammed channel from the first slot (t = 0) on
		uint64_t forced_at = 0;
		for(uint64_t t = 0; t <= 10000 && !forced_at; t++) {
			if(sched.tick(t, busy_r, noise)) {
				forced_at = t;
			}
		}
		bool ok = forced_at >= (uint64_t)txsched::LBT_FORCE_TX_MS &&
		          forced_at <= (uint64_t)(txsched::LBT_FORCE_TX_MS +
		                                  txsched::LBT_BACKOFF_MAX_MS + 1) &&
		          sched.forced_count() == 1 &&
		          sched.current_state() == txsched::state::quiet;
		snprintf(detail, sizeof(detail), "forced at %llu ms, forced_count=%llu",
		         (unsigned long long)forced_at, (unsigned long long)sched.forced_count());
		check(fails, ok, "sched forced TX after 3000 ms busy", detail);

		// channel now free, but the post-force quiet period must hold >= 2000 ms: the 1 Hz
		// slot due during it must not transmit before quiet ends
		uint64_t next_tx = 0;
		for(uint64_t t = forced_at + 1; t <= forced_at + 5000 && !next_tx; t++) {
			if(sched.tick(t, free_r, noise)) {
				next_tx = t;
			}
		}
		ok = next_tx >= forced_at + txsched::LBT_QUIET_MS && next_tx != 0 &&
		     sched.forced_count() == 1;
		snprintf(detail, sizeof(detail), "next tx %llu ms after forced tx",
		         (unsigned long long)(next_tx - forced_at));
		check(fails, ok, "sched 2000 ms quiet after forced TX", detail);
	}

	// ---- continuous carrier: only forced TX ever happens -------------------------------
	// Channel jammed for the whole run: every transmission must be a forced one, spaced
	// by at least the quiet period plus a fresh 3000 ms busy episode.
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 9));
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		float busy_r[ADSL_NUM_CHANNELS] = { BUSY_RSSI, BUSY_RSSI };
		std::vector<uint64_t> tx_times;
		for(uint64_t t = 0; t <= 30000; t++) {
			if(sched.tick(t, busy_r, noise)) {
				tx_times.push_back(t);
			}
		}
		bool ok = sched.tx_count() == sched.forced_count() && sched.forced_count() >= 5 &&
		          sched.forced_count() <= 7;
		for(size_t i = 1; i < tx_times.size() && ok; i++) {
			ok = tx_times[i] - tx_times[i - 1] >=
			     (uint64_t)(txsched::LBT_QUIET_MS + txsched::LBT_FORCE_TX_MS);
		}
		snprintf(detail, sizeof(detail), "%llu tx, %llu forced in 30 s",
		         (unsigned long long)sched.tx_count(),
		         (unsigned long long)sched.forced_count());
		check(fails, ok, "sched continuous carrier: forced TX only", detail);
	}

	// ---- GNSS-to-air latency: delayed TX encodes the fix at the TX instant -------------
	// Busy at the slot tick delays the transmission by one backoff; the packet on air
	// must carry the timestamp of the actual TX moment (fresh get() in transmit()), not
	// of the missed slot. 'crossed' proves at least one episode straddled a quarter-
	// second boundary so slot-time encoding would have been caught.
	{
		fake_pos pos;
		pos.fs = 2;
		txsched sched;
		sched.init(base_cfg(&pos, 11));
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		bool fresh = true, crossed = false;
		int done = 0;
		uint64_t slot_due = 0, t = 0;
		while(done < 20 && t < 100000) {
			bool busy_now = t == slot_due;
			float r = busy_now ? BUSY_RSSI : QUIET_RSSI;
			float rssi[ADSL_NUM_CHANNELS] = { r, r };
			auto req = sched.tick(t, rssi, noise);
			if(req) {
				adsl_traffic tr;
				if(adsl_decode(req->packet.data(), tr) == adsl_decode_result::crc_fail ||
				   tr.ts_qs != (uint8_t)((t / 250) % 60)) {
					fresh = false;
				}
				if((t / 250) % 60 != (slot_due / 250) % 60) {
					crossed = true;
				}
				done++;
				slot_due = t + 1000;
				t = slot_due - 1;
			}
			t++;
		}
		snprintf(detail, sizeof(detail), "%d delayed tx, boundary crossed: %s", done,
		         crossed ? "yes" : "no");
		check(fails, done == 20 && fresh && crossed,
		      "sched GNSS-to-air freshness (timestamp at TX instant)", detail);
	}

	// ---- stale / missing fix: slot skipped, never sent ---------------------------------
	{
		fake_pos pos;
		pos.fs = 2;
		pos.age_ms = txsched::GNSS_MAX_AGE_MS + 100;   // stale beyond the 500 ms budget
		txsched sched;
		sched.init(base_cfg(&pos, 4));
		float rssi[ADSL_NUM_CHANNELS] = { QUIET_RSSI, QUIET_RSSI };
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		int n = 0;
		for(uint64_t t = 0; t <= 3000; t += 10) {
			if(sched.tick(t, rssi, noise)) {
				n++;
			}
		}
		uint64_t skipped_stale = sched.skipped_count();
		pos.age_ms = 0;   // fix fresh again: beaconing must resume
		for(uint64_t t = 3010; t <= 6000; t += 10) {
			if(sched.tick(t, rssi, noise)) {
				n++;
			}
		}
		bool ok = skipped_stale >= 3 && n >= 2 && sched.tx_count() == (uint64_t)n;
		snprintf(detail, sizeof(detail), "%llu skipped while stale, %d tx after recovery",
		         (unsigned long long)skipped_stale, n);
		check(fails, ok, "sched stale fix skips the slot", detail);

		// source with no data at all: also a skip, not a crash or a stale broadcast
		fake_pos dead;
		dead.have_data = false;
		txsched s2;
		s2.init(base_cfg(&dead, 5));
		bool any = false;
		for(uint64_t t = 0; t <= 3000; t += 10) {
			any = any || s2.tick(t, rssi, noise).has_value();
		}
		check(fails, !any && s2.skipped_count() >= 3, "sched missing source skips");
	}

	// ---- no-fix (valid=false) still beacons, with the 0x800000 no-fix encoding ---------
	{
		fake_pos pos;
		pos.fs = 2;
		pos.valid = false;
		txsched sched;
		sched.init(base_cfg(&pos, 6));
		float rssi[ADSL_NUM_CHANNELS] = { QUIET_RSSI, QUIET_RSSI };
		float noise[ADSL_NUM_CHANNELS] = { QUIET_NOISE, QUIET_NOISE };
		auto req = sched.tick(0, rssi, noise);
		adsl_traffic t;
		bool ok = req.has_value() &&
		          adsl_decode(req->packet.data(), t) != adsl_decode_result::crc_fail &&
		          !t.pos_valid;
		check(fails, ok, "sched no-fix beacon (pos_valid=false on air)");
	}

	// ---- mock-radio TX gating ----------------------------------------------------------
	{
		std::unique_ptr<mock_radio> mock(make_mock_radio());
		mock->set_tx_policy(tx_allowed::none);
		int16_t iq[8] = { 0 };
		bool refused = !mock->start_tx(2e6, 867.9e6, 0, "") && !mock->submit_tx(iq, 4);
		check(fails, refused, "mock radio refuses TX under policy none");

		mock->set_tx_policy(tx_allowed::all);
		bool ok = mock->start_tx(2e6, 867.9e6, 0, "") && mock->submit_tx(iq, 4) &&
		          mock->tx_captured().size() == 8;
		check(fails, ok, "mock radio TX capture under policy all");
	}

	return fails;
}

} // namespace adsld

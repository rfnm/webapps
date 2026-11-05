// adsld selftest: the obligations of conform.h and how the scheduler applies them
//   - Timestamp (G.1.1) from UTC: known instants incl. the minute and hour wrap, never 60..63
//   - the scheduler stamps the Timestamp from UTC at the time of the FIX, not of the
//     transmission and not from the radio clock; without a UTC clock it keeps the fix's value
//   - Emergency Status defaults to 1 "No emergency" (G.1.4)
//   - Privacy mode (F.2.4): reserved values redrawn, OS draws spread over all 24 bits
//   - the conformity notes: a conform station has none, every deviation names its clause
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <set>
#include <string>
#include <vector>

#include "adsl_codec.h"
#include "conform.h"
#include "selftest.h"
#include "txsched.h"

namespace adsld {

namespace {

// a fix `age_ms` old that carries a deliberately wrong Timestamp of its own
class aged_pos : public position_source {
public:
	uint64_t age_ms = 120;
	uint8_t ts = 42;

	bool get(uint64_t now_ms, position_fix &out) override {
		out = position_fix();
		out.valid = true;
		out.lat_deg = 53.55;
		out.lon_deg = 9.99;
		out.alt_m = 50;
		out.flightstate = 1;
		out.time_ms = now_ms >= age_ms ? now_ms - age_ms : 0;
		out.ts_qs = ts;
		return true;
	}
};

struct traffic_tx {
	uint64_t t;
	adsl_traffic tr;
};

std::vector<traffic_tx> run_ground(const txsched_config &cfg, uint64_t until_ms) {
	txsched s;
	s.init(cfg);
	const float noise[ADSL_NUM_CHANNELS] = { -95.0f, -95.0f };
	const float rssi[ADSL_NUM_CHANNELS] = { -95.0f, -95.0f };
	std::vector<traffic_tx> out;
	for(uint64_t t = 1000; t <= until_ms; t += 10) {
		auto r = s.tick(t, rssi, noise);
		if(r && r->kind == tx_kind::traffic && r->packet.size() == ADSL_PKT_LEN) {
			traffic_tx x;
			x.t = t;
			if(adsl_decode(r->packet.data(), x.tr) == adsl_decode_result::ok) {
				out.push_back(x);
			}
		}
	}
	return out;
}

bool has_key(const std::vector<conformity_note> &n, const char *key) {
	for(const auto &x : n) {
		if(strcmp(x.key, key) == 0) {
			return true;
		}
	}
	return false;
}

std::string keys(const std::vector<conformity_note> &n) {
	std::string s;
	for(const auto &x : n) {
		s += (s.empty() ? "" : ",") + std::string(x.key);
	}
	return s;
}

} // namespace

int selftest_conform() {
	int fails = 0;
	char detail[300];

	// ---- G.1.1 Timestamp from UTC ------------------------------------------------------
	{
		struct {
			int64_t ms;
			uint8_t qs;
		} kat[] = { { 0, 0 }, { 249, 0 }, { 250, 1 }, { 14999, 59 }, { 15000, 0 },
		            { 3599999, 59 }, { 3600000, 0 }, { 3600250, 1 } };
		bool ok = true;
		for(const auto &k : kat) {
			ok = ok && adsl_ts_qs_utc(k.ms) == k.qs;
		}
		// 2026-09-13 11:34:17.260 UTC: 34 min 17.26 s after the hour = 8229 quarters -> 9
		struct tm tm_ = {};
		tm_.tm_year = 126;
		tm_.tm_mon = 8;
		tm_.tm_mday = 13;
		tm_.tm_hour = 11;
		tm_.tm_min = 34;
		tm_.tm_sec = 17;
		const int64_t real = (int64_t)timegm(&tm_) * 1000 + 260;
		ok = ok && adsl_ts_qs_utc(real) == 9;
		bool range = true;
		for(int64_t ms = real; ms < real + 3600000; ms += 7) {
			range = range && adsl_ts_qs_utc(ms) < 60;
		}
		snprintf(detail, sizeof(detail), "11:34:17.260 -> %u", adsl_ts_qs_utc(real));
		check(fails, ok, "conform: G.1.1 Timestamp = quarter seconds since the full UTC hour mod 60",
		      detail);
		check(fails, range, "conform: G.1.1 Timestamp never 60..63 over a full hour (7 ms steps)");
	}

	// ---- the scheduler stamps UTC at the time of the fix ---------------------------------
	{
		aged_pos pos;
		txsched_config cfg;
		cfg.pos = &pos;
		cfg.tmpl.addr = 0x3C5A11;
		cfg.status_interval_s = 0;
		check(fails, cfg.tmpl.emergency == 1,
		      "conform: G.1.4 Emergency Status defaults to 1 (No emergency)");
		// UTC = radio clock + an offset that sits 10 ms past a quarter: every fix taken 120 ms
		// before its transmission belongs to the quarter BEFORE the transmission's
		const int64_t base = 1789299257010;   // 2026-09-13 11:34:17.010 UTC at radio 0
		cfg.utc_ms_at = [base](uint64_t radio_ms) { return base + (int64_t)radio_ms; };
		const auto log = run_ground(cfg, 61000);
		bool ok = log.size() >= 6, fix_ref = true, differs = true, emerg = true;
		for(const auto &x : log) {
			const uint8_t want = adsl_ts_qs_utc(base + (int64_t)(x.t - pos.age_ms));
			fix_ref = fix_ref && x.tr.ts_qs == want;
			differs = differs && adsl_ts_qs_utc(base + (int64_t)x.t) != want;
			emerg = emerg && x.tr.emergency == 1;
		}
		snprintf(detail, sizeof(detail), "%zu frames, first ts %u", log.size(),
		         log.empty() ? 0 : log[0].tr.ts_qs);
		check(fails, ok && fix_ref && differs,
		      "conform: Traffic Timestamp = UTC of the fix (not of the transmission, not the fix "
		      "source's radio-clock value)", detail);
		check(fails, ok && emerg, "conform: every Traffic frame carries Emergency Status 1");

		cfg.utc_ms_at = nullptr;
		const auto legacy = run_ground(cfg, 31000);
		bool keep = !legacy.empty();
		for(const auto &x : legacy) {
			keep = keep && x.tr.ts_qs == pos.ts;
		}
		check(fails, keep, "conform: without a UTC clock the fix's own Timestamp is kept");
	}

	// ---- F.2.4 privacy address ------------------------------------------------------------
	{
		const uint32_t seq[] = { 0x5F000000u, 0x00FFFFFFu, 0xAB123456u };
		size_t i = 0;
		const uint32_t a = adsl_privacy_address([&]() { return seq[i++]; });
		check(fails, a == 0x123456 && i == 3,
		      "conform: F.2.4 address 000000 and FFFFFF are drawn again, upper byte ignored");

		std::set<uint32_t> seen;
		int ones[24] = { 0 };
		bool valid = true;
		const int N = 4096;
		for(int k = 0; k < N; k++) {
			const uint32_t r = adsl_privacy_address_os();
			valid = valid && r != 0 && r <= 0xFFFFFEu;
			seen.insert(r);
			for(int b = 0; b < 24; b++) {
				ones[b] += (r >> b) & 1;
			}
		}
		double worst = 0.5;
		for(int b = 0; b < 24; b++) {
			const double f = ones[b] / (double)N;
			worst = std::fabs(f - 0.5) > std::fabs(worst - 0.5) ? f : worst;
		}
		snprintf(detail, sizeof(detail), "%zu distinct of %d, worst bit share %.3f", seen.size(),
		         N, worst);
		// 4096 draws from 2^24 collide ~0.5 times on average; more than 8 happens with p < 1e-9
		check(fails, valid && seen.size() >= (size_t)N - 8 && std::fabs(worst - 0.5) < 0.05,
		      "conform: F.2.4 OS-drawn addresses are valid and spread over all 24 bits", detail);
	}

	// ---- conformity notes ---------------------------------------------------------------
	{
		conformity_input good;
		good.gnss = true;
		good.cca_calibrated = true;
		good.power_calibrated = true;
		const auto none = conformity_notes(good);
		check(fails, none.empty() && conformity_json(none) == "{\"conform\":true,\"notes\":[]}",
		      "conform: a conform configuration has no notes", keys(none));

		struct {
			const char *key;
			void (*apply)(conformity_input &);
		} one[] = {
			{ "no_lbt", [](conformity_input &c) { c.rx = false; } },
			{ "status_interval", [](conformity_input &c) { c.status_interval_s = 0; } },
			{ "status_interval", [](conformity_input &c) { c.status_interval_s = 20.5; } },
			{ "interval_ground", [](conformity_input &c) { c.interval_ground_s = 2; } },
			{ "privacy_addr_fixed", [](conformity_input &c) { c.addr_random = false; } },
			{ "addr_changed", [](conformity_input &c) { c.addr_changed = true; } },
			{ "timestamp_not_utc", [](conformity_input &c) { c.utc_timestamp = false; } },
			{ "clock_unsynced", [](conformity_input &c) { c.clock_synced = false; } },
			{ "no_gnss", [](conformity_input &c) { c.gnss = false; } },
			{ "cca_uncalibrated", [](conformity_input &c) { c.cca_calibrated = false; } },
			{ "power_uncalibrated", [](conformity_input &c) { c.power_calibrated = false; } },
			{ "interval_airborne", [](conformity_input &c) { c.interval_airborne_s = 2; } },
		};
		bool ok = true;
		std::string bad;
		for(const auto &o : one) {
			conformity_input c = good;
			o.apply(c);
			const auto n = conformity_notes(c);
			if(n.size() != 1 || strcmp(n[0].key, o.key) != 0) {
				ok = false;
				bad += std::string(o.key) + "->" + keys(n) + " ";
			}
		}
		check(fails, ok, "conform: each single deviation yields exactly its own note", bad);

		conformity_input addr_set = good;
		addr_set.amt = 6;
		addr_set.addr_random = false;
		check(fails, conformity_notes(addr_set).empty(),
		      "conform: a configured address outside AMT 0 is no privacy-mode deviation");

		conformity_input should = good;
		should.interval_airborne_s = 3;
		check(fails, conformity_json(conformity_notes(should)).rfind("{\"conform\":true,", 0) == 0,
		      "conform: a 'should' alone keeps the station conform");

		// a typical test configuration
		conformity_input bench;
		bench.interval_ground_s = 2;
		bench.addr_random = false;
		const auto bn = conformity_notes(bench);
		bool all = has_key(bn, "interval_ground") && has_key(bn, "privacy_addr_fixed") &&
		           has_key(bn, "no_gnss") && has_key(bn, "cca_uncalibrated") &&
		           has_key(bn, "power_uncalibrated") && bn.size() == 5;
		check(fails, all, "conform: a test configuration lists its five deviations", keys(bn));

		conformity_input off = bench;
		off.tx = false;
		const auto on = conformity_notes(off);
		check(fails, on.size() == 1 && strcmp(on[0].key, "tx_off") == 0 && !on[0].mandatory,
		      "conform: transmitter off reports only that");
	}

	return fails;
}

} // namespace adsld

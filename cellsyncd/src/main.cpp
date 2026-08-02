// cellsyncd: LTE-disciplined oscillator daemon for the RFNM (see README.md).
//   --freq MHz         cell center frequency (omit to auto-acquire: survey the
//                      built-in US/EU candidate set + persisted last-good, run on
//                      the strongest confirmed cell - see src/acquire.h; a persisted
//                      last-good that re-confirms commits without the full pass)
//   --candidates f1,f2,..  replace the built-in survey set (MHz; site lists from --scan)
//   --offset HZ        LO offset above the cell center, default +607.5 kHz (parks the
//                      zero-IF DC spike outside the +-540 kHz sync band, mid-subcarrier)
//   --rate HZ          capture rate, default 3.84 MSps (rational-resampled to 1.92M)
//   --tau S            steering time constant (default 120), --period S update tick (10)
//   --dco-path P       sysfs override; --dco-sign +1|-1; --sign-probe; --mock-dco
//   --iq FILE          process a cf32 capture instead of live RX (host analysis)
//   --scan f1,f2,..    survey listed MHz freqs for cells, then exit (needs hardware)
//   --selftest / --sim acceptance gate / closed-loop steering simulation
// HTTP on 127.0.0.1:7686 behind nginx /cellapi/: /status.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "acquire.h"
#include "feed_chain.h"
#include "rfnm_httpd.h"
#include "resampler.h"
#include "steer.h"
#include "tracker.h"

#ifdef WITH_RFNM
#include "capture.h"
#endif

// auto-acquire fallback: with no confirmed cell for this much STREAM time on the
// committed frequency (never wall time - a starved stream must not shorten the
// window), drop back to the survey instead of camping on a dead band forever
static constexpr double ACQ_GIVEUP_S = 90.0;

namespace cellsync {
int run_selftest();
int run_sim(bool quiet);
}

using namespace cellsync;

static std::atomic<bool> g_stop{false};

static void on_signal(int) {
	g_stop = true;
}

struct app_config {
	double freq_hz = 0;
	// LO tune-away: parks the zero-IF DC spike OUTSIDE the +-540 kHz LTE sync band
	// after the digital remix (the old 300 kHz default landed it INSIDE the band,
	// exactly on subcarrier +20 - the worst spot). 607.5 kHz clears the band and
	// sits mid-subcarrier (607.5/15 = 40.5) so the spur never occupies one bin.
	double offset_hz = 607.5e3;
	// 3.84 MSps (= 61.44M/16) keeps the offset carrier's sync-band edge at 60% of
	// Nyquist (at the old 2.4 MSps a >540 kHz offset pushed it to 96% - into the
	// device DDC filter roll-off) and makes the 1.92M ratio a clean 1/2.
	double rate_hz = 3.84e6;
	int gain_db = 30;
	bool agc = false;            // --agc 1: device AGC on, gain_db is the starting point
	std::string antenna;         // RF path by name; empty = preferred, "embed" = internal antenna
	int port = 7686;
	double tau_s = 120;
	double period_s = 10;
	std::string dco_path;
	int dco_sign = 1;
	double dco_range_ppm = 2.0;      // servo authority (steer.h range_steps); chip max 10
	bool sign_probe = false;     // OFF by default: the MR DCO sign is deterministic on this
	                             // platform (+steps raise every Si5510 output; offset_ppb and
	                             // tim_ppb both read +0.1 ppb/step, hardware-proven).
	                             // The probe's only outcomes are "confirm +1" or "misfire to
	                             // -1 on noise -> rail", so it is pure downside here. Opt in
	                             // with --sign-probe only when bringing up an unknown board.
	bool mock_dco = false;
	std::string iq_file;
	std::string dump_file;       // write raw cf32 capture (pre-NCO) and exit
	double dump_secs = 4;
	bool steer_timing = false;   // steer on the PSS timing-drift view instead of CFO. Both
	                             // views are DCO-coupled (MR pulls the whole Si5510); CFO is
	                             // the default - faster, no drift re-warm. Timing is a
	                             // diagnostic / fallback (proven to converge, but its slope
	                             // window smooths and lags the loop).
	std::vector<double> scan_mhz;
	std::vector<double> cand_mhz;   // --candidates override of k_acquire_mhz
};

// ---- DCO cal persistence (EEPROM via the kernel rfnm_settings framework) -----------------
// The bootconfig driver owns the EEPROM block; we only touch two validated sysfs files.
// Values are stored biased +100000 (the framework is unsigned); the si5510 driver applies
// the cal at probe on every boot, so this loop must SEED its absolute step cache from the
// same setting at startup or its first write would jump the DCO off the boot-applied value.
static const int32_t DCO_CAL_BIAS = 100000;

static std::string settings_dir() {
	const char *cands[] = {
		"/sys/bus/i2c/devices/0-0050/rfnm_settings",
		"/sys/devices/platform/soc@0/30800000.bus/30a20000.i2c/i2c-0/0-0050/rfnm_settings",
	};
	for(const char *c : cands) {
		FILE *f = fopen((std::string(c) + "/dco_cal_steps").c_str(), "r");
		if(f) {
			fclose(f);
			return c;
		}
	}
	return "";
}

static bool setting_read(const std::string &dir, const char *name, long &out) {
	FILE *f = fopen((dir + "/" + name).c_str(), "r");
	if(!f) {
		return false;
	}
	char buf[32] = { 0 };
	bool ok = fgets(buf, sizeof(buf), f) != nullptr;
	fclose(f);
	if(ok) {
		out = strtol(buf, nullptr, 10);
	}
	return ok;
}

static bool setting_write(const std::string &dir, const char *name, long val) {
	FILE *f = fopen((dir + "/" + name).c_str(), "w");
	if(!f) {
		return false;
	}
	bool ok = fprintf(f, "%ld\n", val) > 0;
	fclose(f);
	return ok;
}

// measurement gating for the steering combiner
static bool usable(const cell_meas &m) {
	// deep-fringe cells are legitimate steering sources: CRS confirmation has already
	// vetted the PCI and the weighted median handles quality - gate only the absurd
	return m.slots >= 200 && m.weight > 0 && m.snr_db > -16.0f;
}

int main(int argc, char **argv) {
	app_config cfg;
	bool selftest = false, sim = false;
	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
		if(a == "--selftest") {
			selftest = true;
		} else if(a == "--sim") {
			sim = true;
		} else if(a == "--freq") {
			cfg.freq_hz = atof(next()) * 1e6;
		} else if(a == "--offset") {
			cfg.offset_hz = atof(next());
		} else if(a == "--rate") {
			cfg.rate_hz = atof(next());
		} else if(a == "--gain") {
			cfg.gain_db = atoi(next());
		} else if(a == "--agc") {
			cfg.agc = atoi(next()) != 0;
		} else if(a == "--antenna") {
			cfg.antenna = next();
		} else if(a == "--embed-ant") {
			cfg.antenna = "embed";
		} else if(a == "--observable") {
			cfg.steer_timing = std::string(next()) == "timing";
		} else if(a == "--port") {
			cfg.port = atoi(next());
		} else if(a == "--tau") {
			double v = atof(next());
			if(v > 0) {
				cfg.tau_s = v;   // empty/zero (unset ${CELLSYNC_TAU_S}) keeps the default
			}
		} else if(a == "--period") {
			cfg.period_s = atof(next());
		} else if(a == "--dco-path") {
			cfg.dco_path = next();
		} else if(a == "--dco-sign") {
			cfg.dco_sign = atoi(next()) < 0 ? -1 : 1;
		} else if(a == "--dco-range-ppm") {
			cfg.dco_range_ppm = atof(next());
		} else if(a == "--sign-probe") {
			cfg.sign_probe = true;
		} else if(a == "--no-sign-probe") {
			cfg.sign_probe = false;
		} else if(a == "--mock-dco") {
			cfg.mock_dco = true;
		} else if(a == "--iq") {
			cfg.iq_file = next();
		} else if(a == "--dump") {
			cfg.dump_file = next();
		} else if(a == "--dump-secs") {
			cfg.dump_secs = atof(next());
		} else if(a == "--scan" || a == "--candidates") {
			std::vector<double> &dst = a == "--scan" ? cfg.scan_mhz : cfg.cand_mhz;
			std::string s = next();
			size_t p = 0;
			while(p <= s.size()) {
				size_t c = s.find(',', p);
				std::string tok = s.substr(p, c == std::string::npos ? std::string::npos : c - p);
				if(!tok.empty()) {
					dst.push_back(atof(tok.c_str()));
				}
				if(c == std::string::npos) {
					break;
				}
				p = c + 1;
			}
		} else {
			fprintf(stderr, "cellsyncd: unknown arg %s (see README)\n", a.c_str());
			return 2;
		}
	}

	// The timing observable is a drift-slope over a multi-second window, so it lags the
	// sign-probe's few-tick response window and can latch the wrong sign (historical rail).
	// The sign is known anyway - never probe when steering on timing.
	if(cfg.steer_timing) {
		cfg.sign_probe = false;
	}

	if(selftest) {
		return run_selftest();
	}
	if(sim) {
		return run_sim(false);
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	// rational resampler ratio capture-rate -> 1.92M
	long ig = std::gcd((long)1920000, (long)cfg.rate_hz);
	int L = (int)(1920000 / ig), M = (int)((long)cfg.rate_hz / ig);
	if(L > 64 || M > 64) {
		fprintf(stderr, "cellsyncd: rate %.0f has no small rational ratio to 1.92M\n", cfg.rate_hz);
		return 2;
	}

	bool auto_freq = false;
	(void)auto_freq;
#ifdef WITH_RFNM
	// One survey pass over the candidate set; returns the strongest confirmed cell's
	// MHz, or 0. Best of the set, never first-to-confirm - EVERY candidate is visited
	// EXCEPT on the startup fast path: a persisted last-good (always ordered first)
	// that re-confirms near its remembered strength commits immediately, so a settled
	// site pays one visit, not the table. The whole pass rides ONE rx session with
	// per-candidate retunes (a per-candidate device open/close cost ~5 s each).
	auto survey_pick = [&]() -> double {
		const double *tbl = cfg.cand_mhz.empty() ? k_acquire_mhz : cfg.cand_mhz.data();
		size_t tbl_n = cfg.cand_mhz.empty() ? k_acquire_n : cfg.cand_mhz.size();
		last_good lg = last_good_load(k_last_good_path);
		auto cand = acquire_order(lg.mhz, tbl, tbl_n);
		{
			std::string cs;
			char cb[32];
			for(double m : cand) {
				snprintf(cb, sizeof(cb), " %.1f", m);
				cs += cb;
			}
			printf("cellsyncd: surveying:%s MHz\n", cs.c_str());
			fflush(stdout);
		}
		auto t0 = std::chrono::steady_clock::now();
		double best_mhz = 0, best_snr = -1e9;
		bool early = false;
		rx_session ses;
		bool ses_open = false;
		for(double mhz : cand) {
			if(g_stop || early) {
				break;
			}
			double lo = mhz * 1e6 + cfg.offset_hz;
			if(!ses_open) {
				if(!ses.open(lo, cfg.rate_hz, cfg.gain_db, cfg.agc, cfg.antenna)) {
					return 0;
				}
				ses_open = true;
			} else if(!ses.retune(lo)) {
				break;   // keep any confirms so far; the outer loop retries a fresh pass
			}
			sync_engine acq_eng(lo, false);
			feed_chain chain(cfg.offset_hz, cfg.rate_hz, L, M, acq_eng);
			std::atomic<uint64_t> samp{0};
			std::atomic<bool> stop_one{false};
			auto acb = [&](const cf *x, size_t nn, uint64_t gap) {
				chain(x, nn, gap);
				if(samp > (uint64_t)(1.2 * cfg.rate_hz)) {
					stop_one = true;
				}
			};
			if(!ses.run(stop_one, samp, acb)) {
				break;   // stream death mid-pass
			}
			for(const auto &m : acq_eng.cells()) {
				printf("cellsyncd: acquire %.1f MHz: PCI %d snr %.1f dB\n", mhz, m.pci, m.snr_db);
				if(m.snr_db > best_snr) {
					best_snr = m.snr_db;
					best_mhz = mhz;
				}
				if(survey_commit_early(lg, mhz, m.snr_db)) {
					if(lg.have_snr) {
						printf("cellsyncd: last-good %.1f MHz re-confirmed (snr %.1f dB vs stored %.1f) - committing early\n", mhz, m.snr_db, lg.snr_db);
					} else {
						printf("cellsyncd: last-good %.1f MHz re-confirmed (snr %.1f dB) - committing early\n", mhz, m.snr_db);
					}
					early = true;
				}
			}
		}
		if(best_mhz > 0) {
			double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
			printf("cellsyncd: acquired %.1f MHz (best cell snr %.1f dB, survey %.1f s%s)\n",
			       best_mhz, best_snr, secs, early ? ", early-commit" : "");
			fflush(stdout);
		}
		return best_mhz;
	};
#endif
	if(cfg.iq_file.empty() && cfg.scan_mhz.empty() && cfg.freq_hz <= 0) {
#ifdef WITH_RFNM
		// auto-acquire: no region-safe single default exists (acquire.h). Survey the
		// candidate set until a cell confirms; the live loop below drops back here
		// whenever the committed frequency stays dark for ACQ_GIVEUP_S.
		auto_freq = true;
		while(!g_stop && cfg.freq_hz <= 0) {
			double got = survey_pick();
			if(got > 0) {
				cfg.freq_hz = got * 1e6;
			} else if(!g_stop) {
				fprintf(stderr, "cellsyncd: auto-acquire: no cells on any candidate; rescanning (--freq/--scan for other bands)\n");
			}
		}
		if(cfg.freq_hz <= 0) {
			return 0;   // stopped before a cell was found
		}
#else
		fprintf(stderr, "cellsyncd: --freq is required (or --iq / --scan / --selftest / --sim)\n");
		return 2;
#endif
	}

	double f_lo = cfg.freq_hz + cfg.offset_hz;
	sync_engine engine(f_lo > 0 ? f_lo : 751.3e6, true);

	dco_ctl dco;
	dco.sign = cfg.dco_sign;
	double rp = cfg.dco_range_ppm < 0.5 ? 0.5 : (cfg.dco_range_ppm > 10.0 ? 10.0 : cfg.dco_range_ppm);
	dco.range_steps = (int32_t)(rp * 10000.0);
	std::string caldir;
	int32_t cal_stored = 0;          // last value known to be in EEPROM (biased-domain removed)
	bool cal_have = false;
	if(!cfg.mock_dco && cfg.iq_file.empty()) {
		dco.open(cfg.dco_path);
		caldir = settings_dir();
		long v = 0, biased = 0;
		if(!dco.mock && !caldir.empty() && setting_read(caldir, "dco_cal_valid", v) && v == 1 &&
		   setting_read(caldir, "dco_cal_steps", biased)) {
			int32_t seed = (int32_t)(biased - DCO_CAL_BIAS);
			if(seed >= -dco.range_steps && seed <= dco.range_steps) {
				// cache-only seed: the si5510 driver already applied this at boot; writing
				// again is harmless but pointless, and the servo must start from the value
				// the chip is actually running (the actuator is absolute + write-only)
				dco.steps = seed;
				cal_stored = seed;
				cal_have = true;
				printf("cellsyncd: seeded DCO from EEPROM cal: %d steps (%.1f ppb)\n", seed, seed * dco.step_ppb);
			}
		}
	}
	steer_loop::config scfg;
	scfg.tau_s = cfg.tau_s;
	scfg.period_s = cfg.period_s;
	scfg.sign_probe = cfg.sign_probe && !dco.mock;
	steer_loop steer(&dco, scfg);

	std::mutex st_mtx;
	std::vector<cell_meas> st_cells;
	double st_median = 0;
	std::atomic<uint64_t> discont{0}, gap_max_us{0};   // stream-gap /status counters
	std::atomic<bool> freq_saved{false};   // last-good persisted once per acquired freq
	auto t_start = std::chrono::steady_clock::now();

	// ---- steering thread: consume engine measurements every period ----------------
	std::thread steer_thr([&] {
		double acc = 0;
		int32_t last_steps = dco.steps;
		int stable_ticks = 0;
		while(!g_stop) {
			for(int i = 0; i < 10 && !g_stop; i++) {
				std::this_thread::sleep_for(std::chrono::milliseconds((int)(cfg.period_s * 100)));
			}
			if(g_stop) {
				break;
			}
			std::vector<cell_meas> all = engine.take_measurements();
			std::vector<std::pair<double, double>> vw;
			for(const auto &m : all) {
				if(!usable(m)) {
					continue;
				}
				if(cfg.steer_timing) {
					if(m.tim_valid) {
						vw.emplace_back(m.tim_ppb, m.weight);
					}
				} else {
					vw.emplace_back(m.offset_ppb, m.weight);
				}
			}
			double med = weighted_median(vw);
			steer.update(med, !vw.empty());
			// first usable measurement = this frequency carries a tracked cell here:
			// persist it (with the best cell's snr - the next survey's commit-early
			// reference) as the next start's first auto-acquire candidate
			if(!freq_saved && !vw.empty() && cfg.freq_hz > 0 && cfg.iq_file.empty()) {
				double best = -1e9;
				for(const auto &m : all) {
					if(usable(m) && m.snr_db > best) {
						best = m.snr_db;
					}
				}
				if(last_good_store(k_last_good_path, cfg.freq_hz / 1e6, best)) {
					printf("cellsyncd: stored last-good freq %.1f MHz (snr %.1f dB)\n", cfg.freq_hz / 1e6, best);
				} else {
					fprintf(stderr, "cellsyncd: last-good freq store failed (%s)\n", k_last_good_path);
				}
				freq_saved = true;
			}
			// timing-drift hygiene: an actuation >= 1000 steps (100 ppb) bends the PSS
			// slope enough to poison in-flight regressions (sign probe, convergence
			// slews); restart them. Sub-100-ppb tracking dither biases a full window
			// by <1% and is not worth the 30 s tim_valid re-warmup.
			if(std::llabs((long long)dco.steps - (long long)last_steps) >= 1000) {
				engine.reset_drift();
			}
			last_steps = dco.steps;
			(void)acc;
			// persist the converged cal: sustained clean TRACK (3 min), real actuator, and
			// a move of >2 ppb vs what EEPROM holds (hysteresis - this is a warm-start seed,
			// not a live mirror; the block is EEPROM, don't churn it)
			if(!dco.mock && !caldir.empty() && steer.state() == steer_loop::TRACK &&
			   std::abs(steer.residual_ppb()) < 3.0 && steer.sigma_ppb() < 5.0) {
				stable_ticks++;
			} else {
				stable_ticks = 0;
			}
			if(stable_ticks >= 18 && (!cal_have || std::abs(dco.steps - cal_stored) > 20)) {
				if(setting_write(caldir, "dco_cal_steps", (long)dco.steps + DCO_CAL_BIAS) &&
				   setting_write(caldir, "dco_cal_valid", 1)) {
					cal_stored = dco.steps;
					cal_have = true;
					printf("cellsyncd: stored DCO cal to EEPROM: %d steps (%.1f ppb)\n",
					       dco.steps, dco.steps * dco.step_ppb);
				} else {
					fprintf(stderr, "cellsyncd: EEPROM cal store FAILED (%s)\n", caldir.c_str());
				}
				stable_ticks = 0;   // full stable window again before any re-store
			}
			std::lock_guard<std::mutex> lk(st_mtx);
			st_cells = all;
			st_median = med;
		}
	});

	// ---- status endpoint (shared fleet httpd; GET /api self-description comes free) ---
	rfnm::httpd server(cfg.port, "clocksync", 1, { { "GET", "/status", "LTE sync state + tracked cells" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method != "GET" || (req.path != "/status" && req.path != "/" && !req.path.empty())) {
			return {};
		}
		std::lock_guard<std::mutex> lk(st_mtx);
		double up = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
		char b[640];
		snprintf(b, sizeof(b),
		         "{\"mode\":\"%s\",\"freq\":%.0f,\"offset\":%.0f,\"rate\":%.0f,\"uptime\":%.1f,\"samples\":%llu,"
		         "\"duty\":%.3f,\"discont\":%llu,\"gap_max_us\":%llu,"
		         "\"coarse_cfo\":%.1f,\"acquisitions\":%llu,\"dco_steps\":%d,\"dco_range_steps\":%d,\"dco_ppb\":%.2f,\"dco_writes\":%llu,"
		         "\"median_ppb\":%.3f,\"residual_ppb\":%.3f,\"sigma_ppb\":%.3f,\"updates\":%llu,\"cells\":[",
		         engine.tracking() ? steer.state_name() : "acquire",
		         cfg.freq_hz, cfg.offset_hz, cfg.rate_hz, up, (unsigned long long)engine.samples(),
		         up > 1.0 ? (double)engine.samples() / (up * 1920000.0) : 0.0,
		         (unsigned long long)discont.load(), (unsigned long long)gap_max_us.load(),
		         engine.coarse_cfo(), (unsigned long long)engine.acq_attempts(),
		         dco.steps, dco.range_steps, dco.applied_ppb(), (unsigned long long)dco.writes,
		         st_median, steer.residual_ppb(), steer.sigma_ppb(), (unsigned long long)steer.updates());
		std::string j = b;
		bool first = true;
		for(const auto &m : st_cells) {
			char c[256];
			snprintf(c, sizeof(c),
			         "%s{\"pci\":%d,\"wide\":%s,\"snr_db\":%.1f,\"cfo_hz\":%.2f,\"offset_ppb\":%.3f,"
			         "\"tim_ppb\":%.1f,\"tim_valid\":%s,\"weight\":%.0f,\"slots\":%llu,\"age\":%.2f}",
			         first ? "" : ",", m.pci, m.wide ? "true" : "false", m.snr_db, m.cfo_hz, m.offset_ppb,
			         m.tim_ppb, m.tim_valid ? "true" : "false", m.weight, (unsigned long long)m.slots, m.age_s);
			j += c;
			first = false;
		}
		return { 200, j + "]}" };
	});
	if(!server.ok()) {
		fprintf(stderr, "cellsyncd: http bind failed on 127.0.0.1:%d\n", cfg.port);
		g_stop = true;
		steer_thr.join();
		return 1;
	}

	// ---- sample pipeline: feed_chain = NCO(-offset) -> L/M -> engine + gap policy -----
	feed_chain pipeline(cfg.offset_hz, cfg.rate_hz, L, M, engine, &discont, &gap_max_us);

	int rc = 0;
	if(!cfg.iq_file.empty()) {
		// offline analysis of a cf32 capture (already centered like live RX)
		FILE *f = fopen(cfg.iq_file.c_str(), "rb");
		if(!f) {
			fprintf(stderr, "cellsyncd: cannot open %s\n", cfg.iq_file.c_str());
			rc = 1;
		} else {
			printf("cellsyncd: processing %s at %.3f MSps (f_lo %.3f MHz)\n", cfg.iq_file.c_str(), cfg.rate_hz / 1e6, f_lo / 1e6);
			std::vector<cf> buf(1 << 16);
			size_t got;
			uint64_t total = 0;
			while(!g_stop && (got = fread(buf.data(), sizeof(cf), buf.size(), f)) > 0) {
				pipeline(buf.data(), got, 0);
				total += got;
				if(total % (1 << 22) < (1 << 16)) {
					for(const auto &m : engine.cells()) {
						printf("  PCI %3d %s snr %5.1f dB cfo %+9.2f Hz offset %+8.3f ppb (slots %llu)\n",
						       m.pci, m.wide ? "wide" : "1.4M", m.snr_db, m.cfo_hz, m.offset_ppb, (unsigned long long)m.slots);
					}
				}
			}
			fclose(f);
			printf("cellsyncd: done, %llu samples; final:\n", (unsigned long long)total);
			for(const auto &m : engine.cells()) {
				printf("  PCI %3d %s snr %5.1f dB cfo %+9.2f Hz offset %+8.3f ppb tim %+7.1f ppb%s\n",
				       m.pci, m.wide ? "wide" : "1.4M", m.snr_db, m.cfo_hz, m.offset_ppb, m.tim_ppb, m.tim_valid ? "" : " (tim n/a)");
			}
		}
		g_stop = true;
	}
#ifdef WITH_RFNM
	else if(!cfg.dump_file.empty()) {
		// raw cf32 capture at the LO (pre-NCO), for host-side analysis / --iq replay
		FILE *f = fopen(cfg.dump_file.c_str(), "wb");
		if(!f) {
			fprintf(stderr, "cellsyncd: cannot write %s\n", cfg.dump_file.c_str());
			rc = 1;
		} else {
			uint64_t want = (uint64_t)(cfg.dump_secs * cfg.rate_hz);
			std::atomic<uint64_t> samp{0};
			std::atomic<bool> stop_one{false};
			bool cap_ok = run_capture(f_lo, cfg.rate_hz, cfg.gain_db, cfg.agc, cfg.antenna, stop_one, samp, [&](const cf *x, size_t n, uint64_t) {
				fwrite(x, sizeof(cf), n, f);
				if(samp >= want) {
					stop_one = true;
				}
			});
			fclose(f);
			if(cap_ok && samp >= want) {
				printf("cellsyncd: dumped %.1fs cf32 at %.3f MHz (LO %.3f MHz, rate %.3f MSps) -> %s\n",
				       cfg.dump_secs, cfg.freq_hz / 1e6, f_lo / 1e6, cfg.rate_hz / 1e6, cfg.dump_file.c_str());
			} else {
				fprintf(stderr, "cellsyncd: dump FAILED (%llu of %llu samples)\n",
				        (unsigned long long)samp.load(), (unsigned long long)want);
				rc = 1;
			}
		}
		g_stop = true;
	} else if(!cfg.scan_mhz.empty()) {
		// cell survey: one rx session, retune + ~1.2 s dwell per listed frequency, exit
		rx_session scan_ses;
		bool scan_open = false;
		for(double mhz : cfg.scan_mhz) {
			if(g_stop) {
				break;
			}
			double lo = mhz * 1e6 + cfg.offset_hz;
			if(!scan_open) {
				if(!scan_ses.open(lo, cfg.rate_hz, cfg.gain_db, cfg.agc, cfg.antenna)) {
					rc = 1;
					break;
				}
				scan_open = true;
			} else if(!scan_ses.retune(lo)) {
				rc = 1;
				break;
			}
			sync_engine scan_eng(lo, false);
			feed_chain schain(cfg.offset_hz, cfg.rate_hz, L, M, scan_eng);
			std::atomic<uint64_t> samp{0};
			std::atomic<bool> stop_one{false};
			auto cb = [&](const cf *x, size_t n, uint64_t gap) {
				schain(x, n, gap);
				if(samp > (uint64_t)(1.2 * cfg.rate_hz)) {
					stop_one = true;
				}
			};
			if(!scan_ses.run(stop_one, samp, cb)) {
				rc = 1;
				break;
			}
			auto cells = scan_eng.cells();
			printf("scan %.1f MHz: %zu cell(s)", mhz, cells.size());
			for(const auto &m : cells) {
				printf("  [PCI %d snr %.1f dB cfo %+.0f Hz]", m.pci, m.snr_db, scan_eng.coarse_cfo());
			}
			printf("\n");
			fflush(stdout);
		}
		g_stop = true;
	} else {
		std::atomic<uint64_t> samp{0};
		bool ok = true;
		while(!g_stop) {
			double f_lo_run = cfg.freq_hz + cfg.offset_hz;
			printf("cellsyncd: freq %.3f MHz (LO %+.0f Hz offset) rate %.3f MSps -> 1.92M (%d/%d) tau %.0fs dco %s\n",
			       cfg.freq_hz / 1e6, cfg.offset_hz, cfg.rate_hz / 1e6, L, M, cfg.tau_s,
			       dco.mock ? "MOCK" : dco.path.c_str());
			fflush(stdout);
			std::atomic<bool> stop_run{false};
			uint64_t track_mark = engine.samples();
			ok = run_capture(f_lo_run, cfg.rate_hz, cfg.gain_db, cfg.agc, cfg.antenna, stop_run, samp,
			                 [&](const cf *x, size_t n, uint64_t gap) {
				pipeline(x, n, gap);
				if(g_stop) {
					stop_run = true;
				} else if(engine.tracking()) {
					track_mark = engine.samples();
				} else if(auto_freq && engine.samples() - track_mark > (uint64_t)(ACQ_GIVEUP_S * 1920000.0)) {
					stop_run = true;   // committed band stayed dark: back to the survey
				}
			});
			if(g_stop || !auto_freq || !ok) {
				break;
			}
			fprintf(stderr, "cellsyncd: no cell confirmed at %.3f MHz for %ds of stream - resurveying\n",
			        cfg.freq_hz / 1e6, (int)ACQ_GIVEUP_S);
			cfg.freq_hz = 0;
			freq_saved = false;   // the re-pick earns its own last-good store
			while(!g_stop && cfg.freq_hz <= 0) {
				double got = survey_pick();
				if(got > 0) {
					cfg.freq_hz = got * 1e6;
				}
			}
			if(cfg.freq_hz > 0) {
				engine.reset(cfg.freq_hz + cfg.offset_hz);
			}
		}
		rc = ok ? 0 : 1;
		g_stop = true;
	}
#else
	else {
		fprintf(stderr, "cellsyncd: built without librfnm (WITH_RFNM=0); use --selftest/--sim/--iq\n");
		g_stop = true;
		rc = 2;
	}
#endif

	steer_thr.join();
	printf("cellsyncd: exiting, dco_steps=%d (%.2f ppb), %llu updates\n",
	       dco.steps, dco.applied_ppb(), (unsigned long long)steer.updates());
	return rc;
}

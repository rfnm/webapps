// adsld: ADS-L transceiver daemon for the RFNM web GUI (EASA ADS-L 4 SRD-860 Issue 2,
// M-band iConspicuity Traffic and Status on 868.2 / 868.4 MHz, 2-GFSK 100 kchip/s Manchester).
//   One 2 MSps capture at 867.9 MHz feeds both per-channel DDC (+300 kHz / +500 kHz)
//   + receiver chains; the optional beacon TX renders GFSK bursts into the same tuning.
// TX gating: /rfnm/board.conf TX_ALLOWED (default none = refuse), the --tx flag (default 0),
// and per-burst listen-before-talk in the scheduler.
// Host builds (WITH_RFNM=0) run against the deterministic mock radio - the full HTTP
// surface works without hardware, no samples ever arrive and the beacon clock is the
// mock's fake clock advanced by the main loop. --demo (any backend) injects synthetic
// traffic so the map UI is testable without RF.
// HTTP on 127.0.0.1:7693 behind nginx /api/app/adsl/: /status /traffic /rx /config /tx, plus
// the map web UI (GET / and static files from --www / ADSLD_WWW / the www dir next to the
// binary; websrv.h is the static-file extension of the common httpd).
// Config: ADSLD_* environment variables provide defaults, argv overrides; bad values are
// clamped/ignored with a warning - the daemon never crash-loops over configuration.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <climits>
#include <unistd.h>

// Compile-time TX power ceiling. ADS-L M-band allows 14 dBm ERP. Power is not calibrated.
#ifndef ADSLD_TX_HARD_CAP_DBM
#define ADSLD_TX_HARD_CAP_DBM 14
#endif

#include "adsl_codec.h"
#include "adsl_phy.h"
#include "channelizer.h"
#include "conform.h"
#include "demo.h"
#include "gfsk_mod.h"
#include "position.h"
#include "radio.h"
#include "receiver.h"
#include "rfnm_json.h"
#include "traffic.h"
#include "txsched.h"
#include "websrv.h"

#ifdef WITH_RFNM
#include "rfnm_boardconf.h"
#endif

namespace adsld {
int run_selftest();
// shared decoded-report field emitter (traffic.cpp)
void traffic_fields_json(std::string &j, const adsl_traffic &t, bool &first);
}

using namespace adsld;

static std::atomic<bool> g_stop{false};

static void on_signal(int) {
	g_stop = true;
}

// ---- configuration ----------------------------------------------------------------------

static constexpr double DDC_CUTOFF_HZ = 130e3;   // per-channel lowpass at the 400 kHz output
static constexpr int16_t TX_AMPLITUDE = 14000;   // CS16 DAC peak for the RFNM TX path

struct app_config {
	double center = 867.9e6;    // LO; channels at +300 kHz (868.2) and +500 kHz (868.4)
	double rate = 2e6;          // capture/TX sample rate (CS16)
	double out_rate = 400e3;    // derived: rate / decim (4 samples/chip nominal)
	int decim = 5;
	int gain = 12;
	bool agc = false;
	std::string antenna;
	int port = 7693;
	bool tx = false;            // beacon TX master enable (still gated by boardconf + LBT)
	// Receive master enable. Default on: adsld is a transceiver. --rx 0 makes it a pure
	// TRANSMITTER, which the RFNM needs: the driver refuses a TX plan while an RX stream is open
	// at this rate ("concurrent duplex supports shared-DCS rate x2 in [50e6..100e6] only").
	// Opening RX first would therefore cost the transmitter its radio.
	bool rx = true;
	double power_dbm = 10;
	bool pos_set = false;       // --lat/--lon given; false = broadcast "no fix" (0x800000)
	double lat = 0, lon = 0, alt = 0;
	uint32_t addr = 0;          // 24-bit sender address
	// --addr / ADSLD_ADDR given. Without it, AMT 0 is privacy mode: a random address drawn
	// at start-up (ADS-L.4.SRD860.F.2.4); with it, AMT 0 keeps the configured address and is
	// reported as not conform.
	bool addr_set = false;
	int amt = 0;                // address-mapping type (6 bits)
	int emergency = 1;          // Emergency Status (G.1.4): 1 = No emergency
	int category = 0;           // aircraft category (5 bits)
	double interval_air = 1.0, interval_ground = 10.0;
	// Status payload period in seconds (ADS-L.4.SRD860.G.2.11: at least once per 20 s). 15 s
	// leaves margin for listen-before-talk. 0 switches Status off; that and anything above
	// 20 s only warn, below 5 s is raised to 5 s.
	double status_interval = 15.0;
	bool selftest = false, bench = false;
	double bench_secs = 5;
	bool demo = false;          // inject synthetic traffic (~1 Hz) for the web UI
	std::string www;            // wwwroot override (default: "www" next to the binary)
	std::string bind = "127.0.0.1";  // http bind; 0.0.0.0 for LAN on hosts without nginx
};

static void usage() {
	fprintf(stderr,
	        "usage: adsld [--center HZ] [--rate HZ] [--gain dB] [--agc 0|1] [--antenna name]\n"
	        "             [--port n] [--tx 0|1] [--rx 0|1] [--power dBm] [--lat deg] [--lon deg]\n"
	        "             [--alt m]\n"
	        "             [--addr hex24] [--amt n] [--category n] [--emergency n]\n"
	        "             [--interval-air s]\n"
	        "             [--interval-ground s] [--status-interval s] [--www dir]\n"
	        "             [--bind addr] [--demo] [--bench [secs]] [--selftest]\n"
	        "  --status-interval s    ADS-L Status frame period, default 15. The standard asks\n"
	        "                         for at least one every 20 s (G.2.11); 0 turns Status off,\n"
	        "                         which - like a longer period - is reported as not conform.\n"
	        "  --amt 0                privacy mode (default): without --addr the address is drawn\n"
	        "                         at random at every start (ADS-L F.2.4). --addr with AMT 0\n"
	        "                         keeps that address and is reported as not conform.\n"
	        "  --emergency n          Emergency Status (G.1.4), default 1 = no emergency.\n"
	        "Deviations from ADS-L are logged at start-up and listed in GET /status (conformity).\n"
	        "environment defaults (overridden by flags): ADSLD_CENTER ADSLD_RATE ADSLD_GAIN\n"
	        "  ADSLD_AGC ADSLD_ANTENNA ADSLD_PORT ADSLD_TX ADSLD_RX ADSLD_POWER ADSLD_LAT ADSLD_LON\n"
	        "  ADSLD_ALT ADSLD_ADDR ADSLD_AMT ADSLD_CATEGORY ADSLD_EMERGENCY ADSLD_INTERVAL_AIR\n"
	        "  ADSLD_INTERVAL_GROUND ADSLD_STATUS_INTERVAL ADSLD_WWW ADSLD_DEMO ADSLD_BIND\n");
}

static bool env_num(const char *name, double &out) {
	const char *v = getenv(name);
	if(!v || !*v) {
		return false;
	}
	char *end = nullptr;
	double x = strtod(v, &end);
	if(!end || *end != '\0') {
		fprintf(stderr, "adsld: ignoring malformed %s='%s'\n", name, v);
		return false;
	}
	out = x;
	return true;
}

// environment variables seed the defaults, argv still wins
static void env_defaults(app_config &c) {
	double v;
	if(env_num("ADSLD_CENTER", v)) { c.center = v; }
	if(env_num("ADSLD_RATE", v)) { c.rate = v; }
	if(env_num("ADSLD_GAIN", v)) { c.gain = (int)v; }
	if(env_num("ADSLD_AGC", v)) { c.agc = v != 0; }
	if(env_num("ADSLD_PORT", v)) { c.port = (int)v; }
	if(env_num("ADSLD_TX", v)) { c.tx = v != 0; }
	if(env_num("ADSLD_RX", v)) { c.rx = v != 0; }
	if(env_num("ADSLD_POWER", v)) { c.power_dbm = v; }
	if(env_num("ADSLD_LAT", v)) { c.lat = v; c.pos_set = true; }
	if(env_num("ADSLD_LON", v)) { c.lon = v; c.pos_set = true; }
	if(env_num("ADSLD_ALT", v)) { c.alt = v; }
	if(env_num("ADSLD_AMT", v)) { c.amt = (int)v; }
	if(env_num("ADSLD_CATEGORY", v)) { c.category = (int)v; }
	if(env_num("ADSLD_EMERGENCY", v)) { c.emergency = (int)v; }
	if(env_num("ADSLD_INTERVAL_AIR", v)) { c.interval_air = v; }
	if(env_num("ADSLD_INTERVAL_GROUND", v)) { c.interval_ground = v; }
	if(env_num("ADSLD_STATUS_INTERVAL", v)) { c.status_interval = v; }
	if(env_num("ADSLD_DEMO", v)) { c.demo = v != 0; }
	const char *s = getenv("ADSLD_ANTENNA");
	if(s && *s) {
		c.antenna = s;
	}
	s = getenv("ADSLD_ADDR");
	if(s && *s) {
		c.addr = (uint32_t)strtoul(s, nullptr, 16) & 0xFFFFFFu;
		c.addr_set = true;
	}
	s = getenv("ADSLD_WWW");
	if(s && *s) {
		c.www = s;
	}
	s = getenv("ADSLD_BIND");
	if(s && *s) {
		c.bind = s;
	}
}

static bool parse_args(int argc, char **argv, app_config &c) {
	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if(a == "--selftest") {
			c.selftest = true;
		} else if(a == "--demo") {
			c.demo = true;
		} else if(a == "--www" && i + 1 < argc) {
			c.www = argv[++i];
		} else if(a == "--bind" && i + 1 < argc) {
			c.bind = argv[++i];
		} else if(a == "--bench") {
			c.bench = true;
			if(i + 1 < argc) {
				char *end = nullptr;
				double v = strtod(argv[i + 1], &end);
				if(end && *end == '\0' && v > 0) {
					c.bench_secs = v;
					i++;
				}
			}
		} else if(a == "--center" && i + 1 < argc) {
			c.center = atof(argv[++i]);
		} else if(a == "--rate" && i + 1 < argc) {
			c.rate = atof(argv[++i]);
		} else if(a == "--gain" && i + 1 < argc) {
			c.gain = atoi(argv[++i]);
		} else if(a == "--agc" && i + 1 < argc) {
			c.agc = atoi(argv[++i]) != 0;
		} else if(a == "--antenna" && i + 1 < argc) {
			c.antenna = argv[++i];
		} else if(a == "--port" && i + 1 < argc) {
			c.port = atoi(argv[++i]);
		} else if(a == "--tx" && i + 1 < argc) {
			c.tx = atoi(argv[++i]) != 0;
		} else if(a == "--rx" && i + 1 < argc) {
			c.rx = atoi(argv[++i]) != 0;
		} else if(a == "--power" && i + 1 < argc) {
			c.power_dbm = atof(argv[++i]);
		} else if(a == "--lat" && i + 1 < argc) {
			c.lat = atof(argv[++i]);
			c.pos_set = true;
		} else if(a == "--lon" && i + 1 < argc) {
			c.lon = atof(argv[++i]);
			c.pos_set = true;
		} else if(a == "--alt" && i + 1 < argc) {
			c.alt = atof(argv[++i]);
		} else if(a == "--addr" && i + 1 < argc) {
			c.addr = (uint32_t)strtoul(argv[++i], nullptr, 16) & 0xFFFFFFu;
			c.addr_set = true;
		} else if(a == "--amt" && i + 1 < argc) {
			c.amt = atoi(argv[++i]);
		} else if(a == "--category" && i + 1 < argc) {
			c.category = atoi(argv[++i]);
		} else if(a == "--emergency" && i + 1 < argc) {
			c.emergency = atoi(argv[++i]);
		} else if(a == "--interval-air" && i + 1 < argc) {
			c.interval_air = atof(argv[++i]);
		} else if(a == "--interval-ground" && i + 1 < argc) {
			c.interval_ground = atof(argv[++i]);
		} else if(a == "--status-interval" && i + 1 < argc) {
			c.status_interval = atof(argv[++i]);
		} else {
			usage();
			return false;
		}
	}
	return true;
}

static double clampd(double v, double lo, double hi, const char *name) {
	if(v < lo || v > hi) {
		double c = v < lo ? lo : hi;
		fprintf(stderr, "adsld: %s %.3f out of range [%.3f..%.3f], clamped to %.3f\n",
		        name, v, lo, hi, c);
		return c;
	}
	return v;
}

// never crash-loop over a config value - clamp/warn/fallback
static void resolve_config(app_config &c) {
	if(!(c.rate >= 800e3) || !(c.rate <= 10e6)) {
		fprintf(stderr, "adsld: rate %.0f unusable, falling back to 2 MSps\n", c.rate);
		c.rate = 2e6;
	}
	c.decim = (int)std::llround(c.rate / 400000.0);
	if(c.decim < 1) {
		c.decim = 1;
	}
	c.out_rate = c.rate / c.decim;
	// the receiver needs an integer number of samples per chip at the DDC output
	double spc = c.out_rate / ADSL_CHIP_RATE_HZ;
	if(std::fabs(spc - std::llround(spc)) > 1e-6 || spc < 2) {
		fprintf(stderr, "adsld: rate %.0f gives %.3f samples/chip (need integer >= 2), "
		        "falling back to 2 MSps\n", c.rate, spc);
		c.rate = 2e6;
		c.decim = 5;
		c.out_rate = 400e3;
	}
	if(!(c.center > 100e6) || !(c.center < 7.2e9)) {
		fprintf(stderr, "adsld: center %.0f out of range, falling back to 867.9 MHz\n", c.center);
		c.center = 867.9e6;
	}
	c.gain = (int)clampd(c.gain, -30, 70, "gain");
	if(c.port < 1 || c.port > 65535) {
		fprintf(stderr, "adsld: port %d invalid, falling back to 7693\n", c.port);
		c.port = 7693;
	}
	c.power_dbm = clampd(c.power_dbm, -40, ADSLD_TX_HARD_CAP_DBM, "power");
	c.lat = clampd(c.lat, -90, 90, "lat");
	c.lon = clampd(c.lon, -180, 180, "lon");
	c.alt = clampd(c.alt, -1000, 20000, "alt");
	c.amt &= 0x3F;
	c.category &= 0x1F;
	if(c.emergency < 0 || c.emergency > 6) {
		fprintf(stderr, "adsld: emergency %d undefined in G.1.4, falling back to 1 (no emergency)\n",
		        c.emergency);
		c.emergency = 1;
	}
	c.interval_air = clampd(c.interval_air, 0.1, 60, "interval-air");
	c.interval_ground = clampd(c.interval_ground, 0.5, 600, "interval-ground");
	c.status_interval = clampd(c.status_interval, 0, 600, "status-interval");
	if(c.status_interval > 0 && c.status_interval < 5) {
		// G.2.11 only bounds the period from above; faster costs airtime on a 1 % band
		fprintf(stderr, "adsld: status-interval %.2f s raised to 5 s\n", c.status_interval);
		c.status_interval = 5;
	}
	// Conformance is judged once the whole station is set up (clock, address) by
	// conformity_notes() in conform.cpp - logged at start-up and listed in GET /status.
}

// ---- own-ship position (runtime-updatable via POST /config) -----------------------------

class live_source : public position_source {
public:
	live_source(bool have, double lat, double lon, float alt)
		: have_(have), lat_(lat), lon_(lon), alt_(alt) {
	}

	bool get(uint64_t now_ms, position_fix &out) override {
		std::lock_guard<std::mutex> lk(mtx_);
		out = position_fix();
		out.valid = have_;
		out.lat_deg = lat_;
		out.lon_deg = lon_;
		out.alt_m = alt_;
		out.kinematics_valid = false;
		out.flightstate = flightstate_;
		out.time_ms = now_ms;   // config values are always "fresh" (<= 500 ms rule)
		out.ts_qs = (uint8_t)((now_ms / 250) % 60);
		return true;
	}

	void set_position(double lat, double lon, float alt) {
		std::lock_guard<std::mutex> lk(mtx_);
		have_ = true;
		lat_ = lat;
		lon_ = lon;
		alt_ = alt;
	}

	void set_flightstate(uint8_t fs) {
		std::lock_guard<std::mutex> lk(mtx_);
		flightstate_ = fs;
	}

	void snapshot(bool &have, double &lat, double &lon, float &alt, uint8_t &fs) {
		std::lock_guard<std::mutex> lk(mtx_);
		have = have_;
		lat = lat_;
		lon = lon_;
		alt = alt_;
		fs = flightstate_;
	}

private:
	std::mutex mtx_;
	bool have_;
	double lat_, lon_;
	float alt_;
	uint8_t flightstate_ = 1;   // ground
};

// ---- recent raw-decode ring for GET /rx -------------------------------------------------

struct rx_entry {
	std::string time;        // UTC wall-clock HH:MM:SS
	uint64_t t_ms = 0;       // radio clock at decode
	int channel = 0;
	double freq = 0;
	float rssi = 0;
	bool corrected = false;
	std::string hex;         // the on-air bytes, 25 or 17 for Status (still scrambled)
	bool decoded = false;    // adsl_decode succeeded
	bool status = false;     // a Status frame (payload type 3), decoded
	int payload_type = -1;   // >= 0: CRC-valid frame of a payload type this build does not parse
	int64_t utc_ms = 0;      // wall clock (UTC ms) at decode
	adsl_traffic t;
};

class rx_ring {
public:
	void push(rx_entry e) {
		std::lock_guard<std::mutex> lk(mtx_);
		total_++;
		ring_.push_back(std::move(e));
		if(ring_.size() > 200) {
			ring_.pop_front();
		}
	}

	uint64_t total() const {
		std::lock_guard<std::mutex> lk(mtx_);
		return total_;
	}

	std::string to_json() const {
		std::lock_guard<std::mutex> lk(mtx_);
		std::string j = "{\"total\":" + std::to_string(total_) + ",\"packets\":[";
		bool firste = true;
		for(const auto &e : ring_) {
			j += firste ? "{" : ",{";
			firste = false;
			bool f = true;
			char b[48];
			rfnm::jkv(j, "time", e.time, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)e.t_ms);
			rfnm::jkv_raw(j, "t_ms", b, f);
			snprintf(b, sizeof(b), "%d", e.channel);
			rfnm::jkv_raw(j, "ch", b, f);
			snprintf(b, sizeof(b), "%.0f", e.freq);
			rfnm::jkv_raw(j, "freq", b, f);
			snprintf(b, sizeof(b), "%.1f", (double)e.rssi);
			rfnm::jkv_raw(j, "rssi", b, f);
			rfnm::jkv_raw(j, "corrected", e.corrected ? "true" : "false", f);
			rfnm::jkv(j, "hex", e.hex, f);
			if(e.status) {
				rfnm::jkv_raw(j, "status", "true", f);
			}
			snprintf(b, sizeof(b), "%lld", (long long)e.utc_ms);
			rfnm::jkv_raw(j, "utc_ms", b, f);
			if(e.payload_type >= 0) {
				rfnm::jkv_raw(j, "unknown", "true", f);
				snprintf(b, sizeof(b), "%d", e.payload_type);
				rfnm::jkv_raw(j, "payload_type", b, f);
			}
			if(e.decoded) {
				traffic_fields_json(j, e.t, f);
			}
			j += "}";
		}
		return j + "]}";
	}

	static std::string now_string() {
		time_t t = time(nullptr);
		struct tm tm_;
		gmtime_r(&t, &tm_);
		char b[16];
		strftime(b, sizeof(b), "%H:%M:%S", &tm_);
		return b;
	}

private:
	mutable std::mutex mtx_;
	std::deque<rx_entry> ring_;
	uint64_t total_ = 0;
};

// UTC milliseconds from the system clock
static int64_t utc_now_ms() {
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static std::string hex_bytes(const uint8_t *p, size_t n) {
	static const char *dig = "0123456789ABCDEF";
	std::string s;
	s.reserve(n * 2);
	for(size_t i = 0; i < n; i++) {
		s += dig[p[i] >> 4];
		s += dig[p[i] & 15];
	}
	return s;
}

// directory next to the running binary (argv[0]-relative; /proc/self/exe preferred so
// PATH-found invocations resolve too) - the default for the wwwroot
static std::string next_to_binary(const char *argv0, const char *name) {
	char buf[PATH_MAX];
	ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
	std::string p;
	if(n > 0) {
		buf[n] = '\0';
		p = buf;
	} else {
		p = argv0;
	}
	size_t s = p.rfind('/');
	return (s == std::string::npos ? std::string(".") : p.substr(0, s)) + "/" + name;
}

// fill a Traffic report's dynamic fields from a position fix (the sender/quality fields
// come from the template)
static void fill_from_fix(adsl_traffic &t, const position_fix &f) {
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

// ---- bench ------------------------------------------------------------------------------

static double cpu_seconds() {
	struct timespec ts;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// --bench: no hardware; synthesize complex Gaussian noise at the capture rate and push it
// through both DDC + receiver chains single-threaded in 4096-sample blocks, exactly as
// the capture path would. Prints CPU seconds and the real-time factor (sim/CPU).
static int run_bench(const app_config &c, double seconds) {
	printf("adsld bench: channels=%d rate=%.3f MSps out=%.1f kHz (decim %d) sim=%.1fs\n",
	       ADSL_NUM_CHANNELS, c.rate / 1e6, c.out_rate / 1e3, c.decim, seconds);
	fflush(stdout);

	std::vector<std::unique_ptr<ddc>> dcs;
	std::vector<std::unique_ptr<adsl_receiver>> rxs;
	std::atomic<uint64_t> detections{0};
	for(int j = 0; j < ADSL_NUM_CHANNELS; j++) {
		dcs.emplace_back(new ddc(c.rate, c.out_rate, ADSL_CH_HZ[j] - c.center, DDC_CUTOFF_HZ));
		rxs.emplace_back(new adsl_receiver(j, ADSL_CH_HZ[j], c.out_rate,
		                                   [&](const uint8_t *, const adsl_rx_meta &) { detections++; }));
	}

	const size_t B = 4096;
	uint64_t nblocks = (uint64_t)(seconds * c.rate) / B;
	if(nblocks == 0) {
		nblocks = 1;
	}
	std::mt19937 rng(0xAD51CAFE);
	std::normal_distribution<float> nd(0.0f, 0.05f);
	std::vector<cf> block(B);
	std::vector<cf> base;
	double cpu = 0;
	uint64_t fed = 0;
	for(uint64_t bi = 0; bi < nblocks; bi++) {
		for(size_t i = 0; i < B; i++) {
			block[i] = cf(nd(rng), nd(rng));
		}
		double t0 = cpu_seconds();
		for(size_t j = 0; j < dcs.size(); j++) {
			base.clear();
			dcs[j]->process(block.data(), B, base);
			rxs[j]->feed(base.data(), base.size());
		}
		cpu += cpu_seconds() - t0;
		fed += B;
	}
	double sim = (double)fed / c.rate;
	double rtf = cpu > 0 ? sim / cpu : 0;
	printf("adsld bench: cpu=%.3fs realtime_factor=%.3fx (sim/cpu) sim=%.3fs samples=%llu detections=%llu\n",
	       cpu, rtf, sim, (unsigned long long)fed, (unsigned long long)detections.load());
	fflush(stdout);
	return 0;
}

// ---- per-channel worker (aisd pattern: capture thread fans blocks out to bounded queues,
// each worker runs its own DDC + receiver so the channels spread across cores) ------------

struct channel_worker {
	ddc dc;
	adsl_receiver rx;
	std::mutex mtx;
	std::condition_variable cv;
	std::deque<std::shared_ptr<std::vector<cf>>> q;
	uint64_t drops = 0;
	std::atomic<uint64_t> rx_ok{0};
	std::thread thread;

	channel_worker(const app_config &c, int idx, adsl_receiver::packet_cb on_packet)
		: dc(c.rate, c.out_rate, ADSL_CH_HZ[idx] - c.center, DDC_CUTOFF_HZ),
		  rx(idx, ADSL_CH_HZ[idx], c.out_rate, std::move(on_packet)) {
		thread = std::thread([this] { loop(); });
	}

	void push(const std::shared_ptr<std::vector<cf>> &block) {
		std::lock_guard<std::mutex> lk(mtx);
		if(q.size() >= 64) {
			q.pop_front();
			drops++;
		}
		q.push_back(block);
		cv.notify_one();
	}

	void loop() {
		std::vector<cf> base;
		while(true) {
			std::shared_ptr<std::vector<cf>> block;
			{
				std::unique_lock<std::mutex> lk(mtx);
				cv.wait(lk, [this] { return !q.empty() || g_stop.load(); });
				if(q.empty()) {
					return;
				}
				block = q.front();
				q.pop_front();
			}
			base.clear();
			dc.process(block->data(), block->size(), base);
			rx.feed(base.data(), base.size());
		}
	}

	void join() {
		cv.notify_one();
		if(thread.joinable()) {
			thread.join();
		}
	}
};

// ---- daemon -----------------------------------------------------------------------------

int main(int argc, char **argv) {
	app_config cfg;
	env_defaults(cfg);
	if(!parse_args(argc, argv, cfg)) {
		return 2;
	}
	if(cfg.selftest) {
		return run_selftest();
	}

	resolve_config(cfg);
	// ADS-L.4.SRD860.F.2.4 privacy mode: AMT 0 without a configured address draws the whole
	// 24-bit address at every start, uniformly, from the kernel's CSPRNG, and keeps it until
	// the process ends. A configured address with AMT 0 stays and is listed as a deviation.
	std::atomic<bool> addr_random{ false };
	if(cfg.amt == 0 && !cfg.addr_set) {
		cfg.addr = adsl_privacy_address_os();
		addr_random = true;
		printf("adsld: privacy mode (AMT 0): address %06X drawn at random for this run "
		       "(ADS-L.4.SRD860.F.2.4)\n", cfg.addr);
	}
	if(cfg.bench) {
		return run_bench(cfg, cfg.bench_secs);
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	// static TX gate: board policy x --tx flag (LBT gates each individual burst later)
	tx_allowed pol = tx_allowed::none;
#ifdef WITH_RFNM
	char board[64];
	enum vsg_tx_policy bpol = vsg_boardconf_parse("/rfnm/board.conf", board, sizeof(board));
	if(cfg.tx) {
		pol = bpol == VSG_TXPOL_ALL ? tx_allowed::all
		    : bpol == VSG_TXPOL_EMBED ? tx_allowed::embed
		                              : tx_allowed::none;
	}
	printf("adsld: board=%s tx_allowed=%s (boardconf=%s, --tx %d)\n", board, tx_allowed_str(pol),
	       vsg_tx_policy_str(bpol), cfg.tx ? 1 : 0);
#else
	pol = cfg.tx ? tx_allowed::all : tx_allowed::none;
	printf("adsld: host build - mock radio, tx_allowed=%s (--tx %d)\n",
	       tx_allowed_str(pol), cfg.tx ? 1 : 0);
#endif

	mock_radio *mock = nullptr;
	radio *r = nullptr;
#ifdef WITH_RFNM
	r = make_rfnm_radio(pol, cfg.agc);
	if(!r) {
		fprintf(stderr, "adsld: rfnm radio init failed\n");
		return 1;
	}
#else
	mock = make_mock_radio();
	mock->set_tx_policy(pol);
	r = mock;
#endif

	auto t_start = std::chrono::steady_clock::now();
	traffic_table table;
	rx_ring ring;

	std::atomic<uint64_t> samples{0};
	std::atomic<uint64_t> rx_ok{0}, crc_fail{0}, corrected{0}, unknown_rx{0}, status_rx{0};
	std::atomic<uint64_t> tx_count{0}, manual_tx_count{0}, lbt_backoffs{0};
	std::atomic<bool> tx_enabled{cfg.tx};
	std::atomic<bool> tx_up{false};
	std::atomic<int> tx_toggle_req{-1};   // -1 none, 0 off, 1 on (applied by the main loop)
	std::atomic<int> manual_tx_req{-1};   // -1 none, else channel index (or 2 = "next")

	// decoded-packet sink: table + ring + counters + console line
	auto on_packet = [&](const uint8_t *pkt, const adsl_rx_meta &m) {
		uint64_t now = r->now_ms();
		rx_entry e;
		e.time = rx_ring::now_string();
		e.t_ms = now;
		e.channel = m.channel;
		e.freq = m.freq_hz;
		e.rssi = m.rssi_db;
		e.corrected = m.corrected;
		const size_t onair = adsl_onair_len(pkt[0]);
		e.hex = hex_bytes(pkt, onair ? onair : ADSL_PKT_LEN);
		e.utc_ms = utc_now_ms();
		// A CRC-valid frame whose payload type this build does not parse: counted and listed in
		// GET /rx as unknown, never mistaken for a CRC failure
		const auto on_unknown = [&]() {
			e.payload_type = adsl_payload_type(pkt);
			unknown_rx++;
			printf("ch%d unknown payload type 0x%02X rssi=%.1f\n", m.channel,
			       (unsigned)(e.payload_type & 0xFF), (double)m.rssi_db);
			fflush(stdout);
		};
		// Status frames are the only 17-byte packets the receiver delivers
		if(pkt[0] == ADSL_LEN_STATUS) {
			adsl_status s;
			if(adsl_status_decode(pkt, s) == adsl_decode_result::crc_fail) {
				on_unknown();
			} else {
				e.status = true;
				status_rx++;
				if(m.corrected) {
					corrected++;
				}
				printf("ch%d %02X:%06X status maxver=%u rx-m=%u rssi=%.1f%s\n",
				       m.channel, s.amt, s.addr, (unsigned)s.max_proto_ver,
				       (unsigned)s.rx_mband, (double)m.rssi_db,
				       m.corrected ? " corrected" : "");
				fflush(stdout);
			}
			ring.push(std::move(e));
			return;
		}
		adsl_traffic t;
		if(adsl_decode(pkt, t) == adsl_decode_result::crc_fail) {
			// the receiver has already checked the CRC, so this is another payload type - or a
			// codec/receiver mismatch, which the selftest would catch
			if(adsl_payload_type(pkt) >= 0 && adsl_payload_type(pkt) != ADSL_PAYLOAD_TRAFFIC) {
				on_unknown();
			} else {
				crc_fail++;
			}
		} else {
			e.decoded = true;
			e.t = t;
			rx_ok++;
			if(m.corrected) {
				corrected++;
			}
			table.update(t, m, now);
			printf("ch%d %02X:%06X rssi=%.1f%s%s\n", m.channel, t.amt, t.addr,
			       (double)m.rssi_db, m.corrected ? " corrected" : "",
			       t.pos_valid ? "" : " no-fix");
			fflush(stdout);
		}
		ring.push(std::move(e));
	};

	std::vector<std::unique_ptr<channel_worker>> workers;
	for(int i = 0; i < ADSL_NUM_CHANNELS; i++) {
		auto per_chan = [&workers, i, &on_packet](const uint8_t *pkt, const adsl_rx_meta &m) {
			workers[i]->rx_ok++;
			on_packet(pkt, m);
		};
		workers.emplace_back(new channel_worker(cfg, i, per_chan));
	}

	// beacon scheduler: frame template + own-ship source, ticked by the main loop
	live_source pos(cfg.pos_set, cfg.lat, cfg.lon, (float)cfg.alt);
	std::mutex sched_mtx;   // guards sched + scfg (main loop tick vs httpd config writes)
	txsched sched;
	txsched_config scfg;
	scfg.interval_airborne_s = cfg.interval_air;
	scfg.interval_ground_s = cfg.interval_ground;
	scfg.pos = &pos;
	scfg.tmpl.amt = (uint8_t)cfg.amt;
	scfg.tmpl.addr = cfg.addr;
	scfg.tmpl.category = (uint8_t)cfg.category;
	scfg.tmpl.emergency = (uint8_t)cfg.emergency;
	// G.1.1: the Timestamp counts quarter seconds from the full UTC hour. The radio clock is
	// monotonic from start-up; it is mapped through the system clock at the moment of use, so
	// a clock correction reaches the very next frame.
	const utc_clock_fn radio_to_utc = [&r](uint64_t radio_ms) {
		return utc_now_ms() - ((int64_t)r->now_ms() - (int64_t)radio_ms);
	};
	scfg.utc_ms_at = radio_to_utc;
	// Status (G.2): what this station tells its peers it can do. Both M-band channels are
	// demodulated concurrently from one capture, which is "full" (> 80 % dwell, G.2.2); a
	// transmit-only station (--rx 0) receives nothing and says so. Max. Protocol Version 1
	// promises to parse versions 0 and 1 (G.2.1).
	scfg.status_interval_s = cfg.status_interval;
	scfg.status_tmpl.max_proto_ver = 1;
	scfg.status_tmpl.rx_mband = cfg.rx ? 3 : 0;
	scfg.seed = (uint32_t)std::random_device{}();
	if(scfg.seed == 0) {
		scfg.seed = 1;
	}

	sched.init(scfg);
	// ---- ADS-L conformity -------------------------------------------------------------------
	// Notes, not refusals (a test run may want other values), but whoever starts or polls the
	// daemon sees what makes it no ADS-L-conform station. Evaluated from the live
	// configuration, so POST /config changes show up in GET /status too.
	std::atomic<bool> addr_changed_rt{ false };
	auto conformity = [&](bool tx_on) {
		conformity_input ci;
		{
			std::lock_guard<std::mutex> lk(sched_mtx);
			ci.status_interval_s = scfg.status_interval_s;
			ci.interval_ground_s = scfg.interval_ground_s;
			ci.interval_airborne_s = scfg.interval_airborne_s;
			ci.amt = scfg.tmpl.amt;
			ci.utc_timestamp = (bool)scfg.utc_ms_at;
		}
		ci.tx = tx_on;
		ci.rx = cfg.rx;
		ci.addr_random = addr_random.load();
		ci.addr_changed = addr_changed_rt.load();
		ci.clock_synced = system_clock_state().synced;
		ci.gnss = false;   // configuration position, no GNSS receiver
		return conformity_notes(ci);
	};
	for(const auto &n : conformity(cfg.tx)) {
		fprintf(stderr, "adsld: %s (ADS-L.4.SRD860.%s): %s\n",
		        n.mandatory ? "NOT ADS-L conform" : "ADS-L recommends otherwise", n.ref,
		        n.text.c_str());
	}

	// current mutable config as JSON (GET /config and the POST /config response)
	auto config_json = [&]() {
		bool have;
		double plat, plon;
		float palt;
		uint8_t pfs;
		pos.snapshot(have, plat, plon, palt, pfs);
		std::string j = "{";
		bool f = true;
		char b[64];
		snprintf(b, sizeof(b), "%.0f", cfg.center);
		rfnm::jkv_raw(j, "center", b, f);
		snprintf(b, sizeof(b), "%.0f", cfg.rate);
		rfnm::jkv_raw(j, "rate", b, f);
		snprintf(b, sizeof(b), "%d", cfg.gain);
		rfnm::jkv_raw(j, "gain", b, f);
		rfnm::jkv_raw(j, "agc", cfg.agc ? "true" : "false", f);
		rfnm::jkv(j, "antenna", cfg.antenna, f);
		rfnm::jkv_raw(j, "tx", tx_enabled.load() ? "true" : "false", f);
		rfnm::jkv(j, "tx_allowed", tx_allowed_str(pol), f);
		snprintf(b, sizeof(b), "%.1f", cfg.power_dbm);
		rfnm::jkv_raw(j, "power_dbm", b, f);
		rfnm::jkv_raw(j, "pos_valid", have ? "true" : "false", f);
		snprintf(b, sizeof(b), "%.6f", plat);
		rfnm::jkv_raw(j, "lat", b, f);
		snprintf(b, sizeof(b), "%.6f", plon);
		rfnm::jkv_raw(j, "lon", b, f);
		snprintf(b, sizeof(b), "%.1f", (double)palt);
		rfnm::jkv_raw(j, "alt", b, f);
		snprintf(b, sizeof(b), "%u", (unsigned)pfs);
		rfnm::jkv_raw(j, "flightstate", b, f);
		std::lock_guard<std::mutex> lk(sched_mtx);
		snprintf(b, sizeof(b), "%06X", scfg.tmpl.addr & 0xFFFFFFu);
		rfnm::jkv(j, "addr", b, f);
		snprintf(b, sizeof(b), "%u", (unsigned)scfg.tmpl.amt);
		rfnm::jkv_raw(j, "amt", b, f);
		snprintf(b, sizeof(b), "%u", (unsigned)scfg.tmpl.category);
		rfnm::jkv_raw(j, "category", b, f);
		snprintf(b, sizeof(b), "%u", (unsigned)scfg.tmpl.emergency);
		rfnm::jkv_raw(j, "emergency", b, f);
		snprintf(b, sizeof(b), "%.1f", scfg.interval_airborne_s);
		rfnm::jkv_raw(j, "interval_airborne_s", b, f);
		snprintf(b, sizeof(b), "%.1f", scfg.interval_ground_s);
		rfnm::jkv_raw(j, "interval_ground_s", b, f);
		return j + "}";
	};

	// runtime config update; unknown keys ignored, bad values clamped - never fatal
	auto apply_config = [&](const rfnm::jv &v) {
		if(v.get("lat") || v.get("lon") || v.get("alt")) {
			bool have;
			double plat, plon;
			float palt;
			uint8_t pfs;
			pos.snapshot(have, plat, plon, palt, pfs);
			plat = clampd(v.num("lat", plat), -90, 90, "lat");
			plon = clampd(v.num("lon", plon), -180, 180, "lon");
			palt = (float)clampd(v.num("alt", palt), -1000, 20000, "alt");
			pos.set_position(plat, plon, palt);
		}
		if(v.get("flightstate")) {
			int fs = (int)v.num("flightstate", 1);
			pos.set_flightstate((uint8_t)(fs >= 0 && fs <= 2 ? fs : 1));
		}
		bool dirty = false;
		uint8_t new_amt = 0;
		uint32_t new_addr = 0;
		uint8_t old_amt = 0;
		uint32_t old_addr = 0;
		{
			std::lock_guard<std::mutex> lk(sched_mtx);
			old_amt = scfg.tmpl.amt;
			old_addr = scfg.tmpl.addr;
			const rfnm::jv *a = v.get("addr");
			if(a) {
				uint32_t addr = a->t == rfnm::jv::STR
				              ? (uint32_t)strtoul(a->s.c_str(), nullptr, 16)
				              : (uint32_t)a->n;
				scfg.tmpl.addr = addr & 0xFFFFFFu;
				dirty = true;
			}
			if(v.get("amt")) {
				scfg.tmpl.amt = (uint8_t)((int)v.num("amt", scfg.tmpl.amt) & 0x3F);
				dirty = true;
			}
			if(v.get("category")) {
				scfg.tmpl.category = (uint8_t)((int)v.num("category", scfg.tmpl.category) & 0x1F);
				dirty = true;
			}
			if(v.get("emergency")) {
				scfg.tmpl.emergency = (uint8_t)((int)v.num("emergency", scfg.tmpl.emergency) & 0x7);
				dirty = true;
			}
			if(v.get("interval_airborne_s")) {
				scfg.interval_airborne_s =
				    clampd(v.num("interval_airborne_s", scfg.interval_airborne_s), 0.1, 60, "interval_airborne_s");
				dirty = true;
			}
			if(v.get("interval_ground_s")) {
				scfg.interval_ground_s =
				    clampd(v.num("interval_ground_s", scfg.interval_ground_s), 0.5, 600, "interval_ground_s");
				dirty = true;
			}
			if(dirty) {
				// reconfigure, not init: channel alternation, Toff per channel and the Status
				// schedule must survive a config change
				sched.reconfigure(scfg);
			}
			new_amt = scfg.tmpl.amt;
			new_addr = scfg.tmpl.addr;
		}
		// F.2.4: the address must not change while the device operates; a POST that repeats
		// the current values changes nothing and is no deviation
		if(new_amt != old_amt || new_addr != old_addr) {
			addr_changed_rt = true;
			if(new_addr != old_addr) {
				addr_random = false;   // no longer the start-up draw
			}
		}
		if(v.get("tx")) {
			tx_toggle_req = v.boolean("tx", tx_enabled.load()) ? 1 : 0;
		}
	};

	std::string wwwroot = cfg.www.empty() ? next_to_binary(argv[0], "www") : cfg.www;
	adsld::websrv server(cfg.port, "adsl", 1,
	                   { { "GET", "/status", "radio + receiver + scheduler state" },
	                     { "GET", "/traffic", "decoded traffic table" },
	                     { "GET", "/rx", "recent raw packet ring" },
	                     { "GET", "/config", "current config" },
	                     { "POST", "/config", "update runtime config" },
	                     { "POST", "/tx", "queue one manual beacon" },
	                     { "GET", "/", "map web UI (static wwwroot)" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method == "GET" && req.path == "/traffic") {
			return { 200, table.to_json(r->now_ms()) };
		}
		if(req.method == "GET" && req.path == "/rx") {
			return { 200, ring.to_json() };
		}
		if(req.method == "GET" && req.path == "/config") {
			return { 200, config_json() };
		}
		if(req.method == "POST" && req.path == "/config") {
			rfnm::jv v;
			if(!rfnm::jparse(req.body, v) || v.t != rfnm::jv::OBJ) {
				return { 400, "{\"error\":\"body must be a JSON object\"}" };
			}
			apply_config(v);
			return { 200, config_json() };
		}
		if(req.method == "POST" && req.path == "/tx") {
			if(pol == tx_allowed::none) {
				return { 409, "{\"error\":\"tx not allowed (boardconf/--tx)\"}" };
			}
			if(!tx_enabled.load()) {
				return { 409, "{\"error\":\"tx disabled\"}" };
			}
			int ch = ADSL_NUM_CHANNELS;   // "next" sentinel: main loop uses the MAC's channel
			rfnm::jv v;
			if(!req.body.empty() && rfnm::jparse(req.body, v) && v.t == rfnm::jv::OBJ) {
				int want = (int)v.num("channel", -1);
				if(want >= 0 && want < ADSL_NUM_CHANNELS) {
					ch = want;
				}
			}
			manual_tx_req = ch;
			return { 200, "{\"queued\":true}" };
		}

		// every other GET falls through to the websrv static wwwroot -> index.html, the map UI
		if(req.method == "GET" && req.path == "/status") {
			double up = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
			std::string j = "{";
			bool f = true;
			char b[64];
			rfnm::jkv(j, "radio", r->describe(), f);
			rfnm::jkv(j, "tx_allowed", tx_allowed_str(pol), f);
			rfnm::jkv_raw(j, "tx_enabled", tx_enabled.load() ? "true" : "false", f);
			rfnm::jkv_raw(j, "tx_up", tx_up.load() ? "true" : "false", f);
			snprintf(b, sizeof(b), "%.0f", cfg.center);
			rfnm::jkv_raw(j, "center", b, f);
			snprintf(b, sizeof(b), "%.0f", cfg.rate);
			rfnm::jkv_raw(j, "rate", b, f);
			snprintf(b, sizeof(b), "%.0f", cfg.out_rate);
			rfnm::jkv_raw(j, "out_rate", b, f);
			snprintf(b, sizeof(b), "%d", cfg.gain);
			rfnm::jkv_raw(j, "gain", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)samples.load());
			rfnm::jkv_raw(j, "samples", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)rx_ok.load());
			rfnm::jkv_raw(j, "rx_ok", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)status_rx.load());
			rfnm::jkv_raw(j, "status_rx", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)unknown_rx.load());
			rfnm::jkv_raw(j, "unknown_rx", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)crc_fail.load());
			rfnm::jkv_raw(j, "crc_fail", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)corrected.load());
			rfnm::jkv_raw(j, "corrected", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)tx_count.load());
			rfnm::jkv_raw(j, "tx_count", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)manual_tx_count.load());
			rfnm::jkv_raw(j, "manual_tx", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)lbt_backoffs.load());
			rfnm::jkv_raw(j, "lbt_backoffs", b, f);
			{
				std::lock_guard<std::mutex> lk(sched_mtx);
				snprintf(b, sizeof(b), "%llu", (unsigned long long)sched.forced_count());
				rfnm::jkv_raw(j, "forced_tx", b, f);
				snprintf(b, sizeof(b), "%llu", (unsigned long long)sched.skipped_count());
				rfnm::jkv_raw(j, "skipped_slots", b, f);
				snprintf(b, sizeof(b), "%llu", (unsigned long long)sched.status_frames());
				rfnm::jkv_raw(j, "status_tx", b, f);
				snprintf(b, sizeof(b), "%llu", (unsigned long long)sched.status_forced());
				rfnm::jkv_raw(j, "status_forced", b, f);
				txsched::state st = sched.current_state();
				rfnm::jkv(j, "sched",
				          st == txsched::state::idle ? "idle"
				        : st == txsched::state::backoff ? "backoff" : "quiet", f);
				snprintf(b, sizeof(b), "%d", sched.next_channel());
				rfnm::jkv_raw(j, "next_channel", b, f);
			}
			// ADS-L conformity of what is configured right now, and the clock discipline behind
			// the Timestamp obligation
			rfnm::jkv_raw(j, "conformity", conformity_json(conformity(tx_enabled.load())), f);
			{
				const clock_state cs = system_clock_state();
				char cb[160];
				snprintf(cb, sizeof(cb), "{\"synced\":%s,\"esterror_ms\":%.3f,\"maxerror_ms\":%.3f}",
				         cs.synced ? "true" : "false", cs.esterror_ms, cs.maxerror_ms);
				rfnm::jkv_raw(j, "clock", cb, f);
			}
			snprintf(b, sizeof(b), "%zu", table.size());
			rfnm::jkv_raw(j, "traffic", b, f);
			snprintf(b, sizeof(b), "%llu", (unsigned long long)table.total());
			rfnm::jkv_raw(j, "traffic_total", b, f);
			snprintf(b, sizeof(b), "%.1f", up);
			rfnm::jkv_raw(j, "uptime", b, f);
			std::string chans = "[";
			for(int i = 0; i < ADSL_NUM_CHANNELS; i++) {
				channel_worker &w = *workers[i];
				uint64_t wdrops, wq;
				{
					std::lock_guard<std::mutex> lk(w.mtx);
					wdrops = w.drops;
					wq = w.q.size();
				}
				char cb[192];
				snprintf(cb, sizeof(cb),
				         "%s{\"freq\":%.0f,\"rssi\":%.1f,\"noise\":%.1f,\"rx_ok\":%llu,"
				         "\"drops\":%llu,\"queue\":%llu}",
				         i ? "," : "", ADSL_CH_HZ[i], (double)w.rx.rssi_db(),
				         (double)w.rx.noise_floor_db(), (unsigned long long)w.rx_ok.load(),
				         (unsigned long long)wdrops, (unsigned long long)wq);
				chans += cb;
			}
			chans += "]";
			rfnm::jkv_raw(j, "channels", chans, f);
			return { 200, j + "}" };
		}
		return {};
	}, wwwroot, 65536, cfg.bind);
	if(!server.ok()) {
		fprintf(stderr, "adsld: http bind failed on 127.0.0.1:%d\n", cfg.port);
		g_stop = true;
		for(auto &w : workers) {
			w->join();
		}
		delete r;
		return 1;
	}

	// capture -> both channel workers (CS16 -> cf once, shared block)
	auto capture_cb = [&](const int16_t *iq, size_t n) {
		samples += n;
		auto block = std::make_shared<std::vector<cf>>(n);
		const float s = 1.0f / 32768.0f;
		for(size_t i = 0; i < n; i++) {
			(*block)[i] = cf((float)iq[2 * i] * s, (float)iq[2 * i + 1] * s);
		}
		for(auto &w : workers) {
			w->push(block);
		}
	};
	// --rx 0: never open the receive stream. The demodulator workers stay up but are simply
	// never fed, so every counter reads zero and the map stays empty - which is the truth for a
	// transmit-only station and better than pretending otherwise.
	bool rx_up = cfg.rx ? r->start_rx(cfg.rate, cfg.center, cfg.gain, cfg.antenna, capture_cb)
	                    : false;
	if(!cfg.rx) {
		printf("adsld: receive disabled (--rx 0) - transmit-only station\n");
	} else if(!rx_up) {
		fprintf(stderr, "adsld: start_rx failed\n");
		// --demo keeps the daemon (and the map web UI) alive without a working radio -
		// the synthetic traffic is the whole point of the mode; real RX is optional there
		if(!cfg.demo) {
			g_stop = true;
			for(auto &w : workers) {
				w->join();
			}
			delete r;
			return 1;
		}
		fprintf(stderr, "adsld: continuing without radio (--demo)\n");
	}

	if(cfg.tx && pol != tx_allowed::none) {
		tx_up = r->start_tx(cfg.rate, cfg.center, cfg.power_dbm, cfg.antenna);
		if(!tx_up.load()) {
			fprintf(stderr, "adsld: start_tx refused - running RX-only\n");
			tx_enabled = false;
		}
	}

	printf("adsld: %s center=%.3f MHz rate=%.3f MSps out=%.1f kHz gain=%d antenna=%s http=:%d\n",
	       r->describe().c_str(), cfg.center / 1e6, cfg.rate / 1e6, cfg.out_rate / 1e3, cfg.gain,
	       cfg.antenna.empty() ? "preferred" : cfg.antenna.c_str(), cfg.port);
	printf("adsld: channels: 0=%.3f 1=%.3f MHz; beacon %s (%02X:%06X)\n",
	       ADSL_CH_HZ[0] / 1e6, ADSL_CH_HZ[1] / 1e6,
	       tx_up.load() ? "on" : "off", (unsigned)scfg.tmpl.amt, scfg.tmpl.addr);
	{
		std::string idx;
		std::string ct;
		bool have = websrv::serve_file(wwwroot, "/index.html", idx, ct);
		printf("adsld: www=%s%s\n", wwwroot.c_str(), have ? "" : " (no index.html - web UI off)");
	}

	// --demo: synthetic traffic around --lat/--lon at ~1 Hz
	std::unique_ptr<demo_traffic> demo;
	if(cfg.demo) {
		double dlat = cfg.pos_set ? cfg.lat : 47.26;
		double dlon = cfg.pos_set ? cfg.lon : 11.35;
		demo.reset(new demo_traffic(dlat, dlon, (uint32_t)std::random_device{}()));
		printf("adsld: demo traffic on around %.4f/%.4f\n", dlat, dlon);
	}
	fflush(stdout);

	// render one burst and hand it to the radio
	auto transmit = [&](int channel, const uint8_t *pkt, size_t len) {
		bits chips = frame_build(pkt, len);
		std::vector<int16_t> iq =
		    gfsk_render_burst_cs16(chips, cfg.rate, ADSL_CH_HZ[channel] - cfg.center, TX_AMPLITUDE);
		return r->submit_tx(iq.data(), iq.size() / 2);
	};

	uint64_t last_expire = 0;
	txsched::state prev_state = txsched::state::idle;
	while(!g_stop.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		if(mock) {
			mock->advance_ms(10);       // host build: the fake clock is ours to drive
			(void)mock->tx_captured();  // drain so long runs don't accumulate IQ
		}
		uint64_t now = r->now_ms();

		// TX enable/disable requested via POST /config
		int tog = tx_toggle_req.exchange(-1);
		if(tog == 1 && !tx_up.load()) {
			if(pol != tx_allowed::none) {
				tx_up = r->start_tx(cfg.rate, cfg.center, cfg.power_dbm, cfg.antenna);
				tx_enabled = tx_up.load();
				if(!tx_up.load()) {
					fprintf(stderr, "adsld: start_tx refused\n");
				}
			} else {
				fprintf(stderr, "adsld: tx enable ignored (tx_allowed=none)\n");
			}
		} else if(tog == 0 && tx_up.load()) {
			r->stop_tx();
			tx_up = false;
			tx_enabled = false;
		}

		float rssi[ADSL_NUM_CHANNELS], noise[ADSL_NUM_CHANNELS];
		for(int i = 0; i < ADSL_NUM_CHANNELS; i++) {
			rssi[i] = workers[i]->rx.rssi_db();
			noise[i] = workers[i]->rx.noise_floor_db();
		}

		// beacon MAC: only ticked while the TX path is actually up, so no slots are
		// consumed (or channels alternated) that we could not transmit
		if(tx_up.load()) {
			std::optional<tx_request> req;
			{
				std::lock_guard<std::mutex> lk(sched_mtx);
				req = sched.tick(now, rssi, noise);
				txsched::state st = sched.current_state();
				if(st == txsched::state::backoff && prev_state != txsched::state::backoff) {
					lbt_backoffs++;
				}
				prev_state = st;
			}
			if(req && transmit(req->channel, req->packet.data(), req->packet.size())) {
				tx_count++;
			}
		}

		// manual beacon requested via POST /tx: the same channel-access rules as a scheduled
		// one - nothing in the quiet period after a forced frame, >= TOFF_MIN_MS on the
		// channel, listen before talk (ADS-L.4.SRD860.D.3) - so while any of them holds it
		// back the request stays pending and is retried on the next pass (10 ms)
		int mch = manual_tx_req.exchange(-1);
		if(mch >= 0 && tx_up.load()) {
			position_fix fix;
			if(pos.get(now, fix)) {
				adsl_traffic t;
				int channel;
				bool clear;
				{
					std::lock_guard<std::mutex> lk(sched_mtx);
					t = scfg.tmpl;
					channel = mch < ADSL_NUM_CHANNELS ? mch : sched.next_channel();
					clear = sched.claim_manual_traffic(channel, now, rssi, noise);
				}
				if(!clear) {
					int none = -1;
					manual_tx_req.compare_exchange_strong(none, mch);   // unless a newer request came
				} else {
					fill_from_fix(t, fix);
					t.ts_qs = adsl_ts_qs_utc(radio_to_utc(fix.time_ms));   // as the scheduler does
					uint8_t pkt[ADSL_PKT_LEN];
					adsl_encode(t, pkt);
					if(transmit(channel, pkt, ADSL_PKT_LEN)) {
						manual_tx_count++;
						tx_count++;
					}
				}
			}
		}

		if(demo) {
			demo->tick(table, now);   // self-limits to ~1 Hz
		}

		if(now - last_expire > 5000) {
			table.expire(now);
			last_expire = now;
		}
	}

	g_stop = true;
	r->stop_rx();
	if(tx_up.load()) {
		r->stop_tx();
	}
	for(auto &w : workers) {
		w->join();
	}
	printf("adsld: exiting, %llu samples, %llu packets (%llu senders)\n",
	       (unsigned long long)samples.load(), (unsigned long long)rx_ok.load(),
	       (unsigned long long)table.size());
	delete r;
	return 0;
}

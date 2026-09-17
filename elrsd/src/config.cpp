// elrsd: configuration - argv > ELRS_* env > defaults. Invalid values warn and
// fall back to the default; the daemon never exits because of a bad knob (vrtd rule: no
// crash loop under systemd). The binding phrase / UID are read from the environment ONLY
// (never a flag: flags show in `ps`) and the variables are removed from the process
// environment right after reading. No NEEDS: only header-inline symbols are used.
// 
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "daemon.h"
#include "daemon_internal.h"

namespace elrsd {

static bool parse_int(const std::string &s, int &out) {
	if(s.empty()) return false;
	char *end = nullptr;
	long v = strtol(s.c_str(), &end, 10);
	if(!end || *end) return false;
	out = (int)v;
	return true;
}

static bool parse_double(const std::string &s, double &out) {
	if(s.empty()) return false;
	char *end = nullptr;
	double v = strtod(s.c_str(), &end);
	if(!end || *end) return false;
	out = v;
	return true;
}

static bool parse_bool(const std::string &s, bool &out) {
	if(s == "1" || s == "true" || s == "on" || s == "yes") { out = true; return true; }
	if(s == "0" || s == "false" || s == "off" || s == "no") { out = false; return true; }
	return false;
}

static void warn(const char *what, const std::string &val, const char *why) {
	fprintf(stderr, "elrsd: config: ignoring %s=\"%s\" (%s), keeping the default\n", what, val.c_str(), why);
}

// one knob: name, how to apply, shared by env and argv (argv wins because it runs last)
static void apply_knob(daemon_config &cfg, const std::string &key, const std::string &val) {
	int iv; double dv; bool bv;
	if(key == "port") {
		if(parse_int(val, iv) && iv > 0 && iv < 65536) cfg.port = iv; else warn("port", val, "1..65535");
	} else if(key == "ws-port") {
		if(parse_int(val, iv) && iv >= 0 && iv < 65536) cfg.ws_port = iv; else warn("ws-port", val, "0..65535");
	} else if(key == "rate") {
		const rate_params *rp = rate_by_name(val);
		if(val == "auto") cfg.rate = val;   // replay/scan: rx_chain rate_index 0 + tracker majority vote
		else if(!rp) warn("rate", val, "unknown rate (500hz 250hz 150hz 50hz auto)");
		else if(!rp->verified) warn("rate", val, "OTA8 rate unsupported (unverified)");
		else cfg.rate = val;
	} else if(key == "fc") {
		if(parse_double(val, dv) && dv >= 2.3e9 && dv <= 2.6e9) cfg.fc_hz = dv; else warn("fc", val, "2.3e9..2.6e9 Hz");
	} else if(key == "fs") {
		if(parse_double(val, dv) && dv >= 1e6 && dv <= 200e6) cfg.fs_hz = dv; else warn("fs", val, "1e6..200e6 Hz");
	} else if(key == "gain") {
		if(parse_int(val, iv) && iv >= -100 && iv <= 100) cfg.gain_db = iv; else warn("gain", val, "-100..100 dB");
	} else if(key == "agc") {
		if(parse_bool(val, bv)) cfg.agc = bv; else warn("agc", val, "0|1");
	} else if(key == "antenna") {
		if(!val.empty() && val.size() < 32) cfg.antenna = val; else warn("antenna", val, "name expected");
	} else if(key == "conj") {
		if(parse_bool(val, bv)) { cfg.conj = bv; cfg.conj_auto = false; } else warn("conj", val, "0|1");
	} else if(key == "conj-auto") {
		if(parse_bool(val, bv)) cfg.conj_auto = bv; else warn("conj-auto", val, "0|1");
	} else if(key == "os") {
		if(parse_int(val, iv) && (iv == 1 || iv == 4)) cfg.os = iv; else warn("os", val, "1|4");
	} else if(key == "tx") {
		if(parse_bool(val, bv)) cfg.tx_enabled = bv; else warn("tx", val, "0|1");
	} else if(key == "tx-cap-dbm") {
		if(parse_int(val, iv) && iv >= -100 && iv <= 30) cfg.tx_cap_dbm = iv; else warn("tx-cap-dbm", val, "-100..30");
	} else if(key == "tx-max-duration") {
		if(parse_double(val, dv) && dv > 0 && dv <= 3600) cfg.tx_max_duration_s = dv; else warn("tx-max-duration", val, "0..3600 s");
	} else if(key == "boardconf") {
#if defined(WITH_RFNM)
		warn("boardconf", val, "ignored in the board build: the TX policy always comes from /rfnm/board.conf");
#else
		if(!val.empty()) cfg.boardconf = val; else warn("boardconf", val, "path expected");
#endif
	} else if(key == "replay") {
		cfg.replay_path = val;
	} else if(key == "replay-start") {
		if(parse_double(val, dv) && dv >= 0) cfg.replay_start_s = dv; else warn("replay-start", val, ">= 0 s");
	} else if(key == "replay-stop") {
		if(parse_double(val, dv) && dv >= 0) cfg.replay_stop_s = dv; else warn("replay-stop", val, ">= 0 s");
	} else if(key == "expect-min") {
		if(parse_int(val, iv) && iv >= 0) cfg.expect_min_ok = iv; else warn("expect-min", val, ">= 0");
	} else if(key == "packet-log") {
		cfg.packet_log_path = val;
	} else if(key == "selfloop-device") {
		// an invalid value must NOT fall through to the live daemon: -1 = requested but invalid -> refused
		if(parse_double(val, dv) && dv > 0 && dv <= 3600) cfg.selfloop_s = dv; else { warn("selfloop-device", val, "seconds 0..3600"); cfg.selfloop_s = -1; }
	} else if(key == "selfloop-dump") {
		if(!val.empty()) cfg.selfloop_dump = val; else warn("selfloop-dump", val, "file path expected");
	} else if(key == "selfloop-cw") {
		if(val.empty() || val == "1") cfg.selfloop_cw = true;
		else if(val == "0") cfg.selfloop_cw = false;
		else if(parse_double(val, dv) && std::fabs(dv) >= 1e3 && std::fabs(dv) <= 40e6) { cfg.selfloop_cw = true; cfg.selfloop_cw_offset_hz = dv; }
		else warn("selfloop-cw", val, "0|1|OFFSET_HZ (1e3..40e6)");
	} else if(key == "selfloop-tx-dbm") {
		if(parse_int(val, iv) && iv >= -100 && iv <= 30) cfg.selfloop_tx_dbm = iv; else warn("selfloop-tx-dbm", val, "-100..30");
	} else if(key == "selfloop-rx-path") {
		if(!val.empty() && val.size() < 32) cfg.selfloop_rx_path = val; else warn("selfloop-rx-path", val, "path name expected");
	} else if(key == "selfloop-rx-channel") {
		if(parse_int(val, iv) && iv >= 0 && iv < 8) cfg.selfloop_rx_channel = iv; else warn("selfloop-rx-channel", val, "0..7");
	} else if(key == "tx-only") {
		// pure transmit, no RX path. An invalid value must NOT fall through to the live
		// daemon: -1 = requested but invalid -> the mode refuses instead.
		if(parse_double(val, dv) && dv > 0 && dv <= 3600) cfg.tx_only_s = dv; else { warn("tx-only", val, "seconds 0..3600"); cfg.tx_only_s = -1; }
	} else if(key == "tx-only-dbm") {
		if(parse_int(val, iv) && iv >= -100 && iv <= 30) cfg.tx_only_dbm = iv; else warn("tx-only-dbm", val, "-100..30");
	}
}

static const char *k_usage =
	"usage: elrsd [--port N] [--rate 500hz|250hz|150hz|50hz|auto] [--fc HZ] [--fs HZ] [--gain dB]\n"
	"             [--agc 0|1] [--antenna NAME] [--conj 0|1] [--os 1|4] [--tx 0|1] [--tx-cap-dbm N]\n"
	"             [--ws-port N] [--boardconf PATH]\n"
	"       elrsd --selftest                          hardware-free module selftests\n"
	"       elrsd --loopback                          TX synth -> own RX chain -> decode (host, no radio)\n"
	"       elrsd --selfloop-device S --tx 1          board: TX ch0 -> chip loopback -> RX (same device, S seconds,\n"
	"             [--selfloop-tx-dbm N] [--selfloop-rx-path loopback|embed_ant|sma_a] [--selfloop-rx-channel N]  all TX gates apply, min power)\n"
	"             [--selfloop-dump FILE]  raw RX cs16 of the first 0.5 s;  [--selfloop-cw [OFFSET_HZ]]  no bursts, a tone at LO+1 MHz\n"
	"       elrsd --tx-only S --tx 1                  board: PURE TX, no RX path (stable anchor) - the send half of\n"
	"             [--tx-only-dbm N] [--rate R] [--fc HZ] [--fs HZ]   the two-board proof; JSON report shows pos_placed (samples at the DAC)\n"
	"       elrsd --bench [s]                         RX chain CPU proxy on a synthetic 25 MSps stream\n"
	"       elrsd --replay FILE [--fs HZ] [--fc HZ] [--rate R] [--conj 0|1] [--replay-start S]\n"
	"             [--replay-stop S] [--realtime] [--no-verify] [--expect-min N] [--packet-log FILE]\n"
	"env:   ELRS_RATE ELRS_FC ELRS_FS ELRS_GAIN ELRS_AGC ELRS_ANTENNA ELRS_CONJ ELRS_CONJ_AUTO\n"
	"       ELRS_TX_CAP_DBM ELRS_WS_PORT (argv wins over env, env over defaults)\n"
	"       ELRS_BINDING_PHRASE | ELRS_UID  runtime binding (never a flag; removed from the\n"
	"       environment after reading; also settable via PUT /config)\n";

bool parse_config(int argc, char **argv, daemon_config &cfg, std::string &usage_out) {
	usage_out = k_usage;
	// 1. environment
	static const struct { const char *env; const char *key; } env_map[] = {
		{ "ELRS_RATE", "rate" }, { "ELRS_FC", "fc" }, { "ELRS_FS", "fs" }, { "ELRS_GAIN", "gain" },
		{ "ELRS_AGC", "agc" }, { "ELRS_ANTENNA", "antenna" }, { "ELRS_CONJ", "conj" },
		{ "ELRS_CONJ_AUTO", "conj-auto" }, { "ELRS_TX_CAP_DBM", "tx-cap-dbm" }, { "ELRS_WS_PORT", "ws-port" },
		{ "ELRS_OS", "os" },
	};
	for(const auto &m : env_map) {
		const char *v = getenv(m.env);
		if(v && *v) apply_knob(cfg, m.key, v);
	}
	// secrets: read, then scrub from the environment
	if(const char *p = getenv("ELRS_BINDING_PHRASE"); p && *p) {
		cfg.binding_phrase_env = p;
	}
	if(const char *u = getenv("ELRS_UID"); u && *u) {
		cfg.uid_hex_env = u;
	}
	unsetenv("ELRS_BINDING_PHRASE");
	unsetenv("ELRS_UID");

	// 2. argv (wins)
	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> std::string { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
		if(a == "--help" || a == "-h") return false;
		if(a == "--selftest" || a == "--loopback") continue;      // handled by main / daemon_main
		if(a == "--bench") {
			if(i + 1 < argc && argv[i + 1][0] != '-') i++;          // optional seconds
			continue;
		}
		if(a == "--realtime") { cfg.replay_realtime = true; continue; }
		if(a == "--no-verify") { cfg.replay_verify = false; continue; }
		if(a.rfind("--", 0) == 0) {
			std::string key = a.substr(2), val;
			size_t eq = key.find('=');
			if(eq != std::string::npos) { val = key.substr(eq + 1); key = key.substr(0, eq); }
			else val = next();
			if(key == "binding-phrase" || key == "uid" || key == "phrase") {
				fprintf(stderr, "elrsd: config: --%s is refused (visible in ps); use ELRS_BINDING_PHRASE / ELRS_UID or PUT /config\n", key.c_str());
				continue;
			}
			static const char *known[] = { "port", "ws-port", "rate", "fc", "fs", "gain", "agc", "antenna", "conj",
				"conj-auto", "os", "tx", "tx-cap-dbm", "tx-max-duration", "boardconf", "replay", "replay-start",
				"replay-stop", "expect-min", "packet-log", "selfloop-device", "selfloop-dump", "selfloop-cw", "selfloop-tx-dbm", "selfloop-rx-path", "selfloop-rx-channel", "tx-only", "tx-only-dbm", nullptr };
			bool ok = false;
			for(int k = 0; known[k]; k++) if(key == known[k]) ok = true;
			if(!ok) { fprintf(stderr, "elrsd: config: unknown flag --%s ignored\n", key.c_str()); continue; }
			apply_knob(cfg, key, val);
		} else {
			fprintf(stderr, "elrsd: config: stray argument \"%s\" ignored\n", a.c_str());
		}
	}
	if(cfg.replay_stop_s > 0 && cfg.replay_stop_s <= cfg.replay_start_s) {
		fprintf(stderr, "elrsd: config: replay-stop <= replay-start, using the whole file\n");
		cfg.replay_start_s = 0;
		cfg.replay_stop_s = 0;
	}
	return true;
}

} // namespace elrsd

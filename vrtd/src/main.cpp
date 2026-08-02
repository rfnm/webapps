// vrtd: VITA 49.2 Ethernet bridge for the RFNM.
//   RX export:  radio RX -> VRT signal-data + context packets -> UDP destination
//   TX ingest:  VRT signal-data packets (stream id --sid-tx) -> radio TX
//   Control:    VITA 49.2 command packets (execute/dry-run, ack per CAM) -> tuner ops
// See README.md for the wire profile and v1 limits.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core.h"
#include "radio.h"
#include "rfnm_boardconf.h"   // apps/common: TX policy from /rfnm/board.conf (fail-safe none)
#include "rfnm_httpd.h"       // apps/common: the fleet HTTP server (GET /api self-description)

using namespace vrtd;

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) {
	g_stop = 1;
}

int main(int argc, char **argv) {
	config cfg;
	bool selftest = false;
	bool mock = false;
	std::string antenna;   // fleet --antenna convention (sma_a|embed|a..h); empty = preferred
	bool agc = false;      // fleet parks *_AGC=0 (kernel peak-detector defect) but the flag is wired like every RX daemon
	double rx_freq = 751e6, rate = 2.4e6, rx_gain = 0;
	int status_port = 7689;   // 7687 is rtsa's recorder - vrt owns 7689 (manifest api.port)

	auto next = [&](int &i) -> const char * {
		if(i + 1 >= argc) {
			fprintf(stderr, "vrtd: %s needs a value\n", argv[i]);
			exit(2);
		}
		return argv[++i];
	};
	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if(a == "--selftest") {
			selftest = true;
		} else if(a == "--mock") {
			mock = true;
		} else if(a == "--antenna") {
			antenna = next(i);
		} else if(a == "--agc") {
			agc = atoi(next(i)) != 0;
		} else if(a == "--dest") {
			std::string d = next(i);
			size_t c = d.find(':');
			cfg.dest_host = d.substr(0, c);
			if(c != std::string::npos) {
				cfg.dest_port = (uint16_t)atoi(d.c_str() + c + 1);
			}
		} else if(a == "--listen") {
			cfg.listen_port = (uint16_t)atoi(next(i));
		} else if(a == "--sid-rx") {
			cfg.sid_rx = (uint32_t)strtoul(next(i), nullptr, 0);
		} else if(a == "--sid-tx") {
			cfg.sid_tx = (uint32_t)strtoul(next(i), nullptr, 0);
		} else if(a == "--cid") {
			cfg.controllee_id = (uint32_t)strtoul(next(i), nullptr, 0);
		} else if(a == "--oui") {
			cfg.oui = (uint32_t)strtoul(next(i), nullptr, 0);
		} else if(a == "--samples-per-pkt") {
			cfg.samples_per_pkt = (size_t)atoi(next(i));
		} else if(a == "--freq") {
			rx_freq = atof(next(i)) * 1e6;
		} else if(a == "--rate") {
			rate = atof(next(i));
		} else if(a == "--gain") {
			rx_gain = atof(next(i));
		} else if(a == "--port") {
			status_port = atoi(next(i));
		} else {
			fprintf(stderr, "vrtd: unknown option %s\n", a.c_str());
			return 2;
		}
	}

	if(selftest) {
		extern int run_selftest();
		return run_selftest();
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	// TX SAFETY: policy from /rfnm/board.conf; a missing file is the fail-safe (no TX).
	// Three layers enforce it: engine data drop + command 0x3 acks (core), and the radio
	// backend's hard stop / embed-antenna override (rfnm_radio).
	char board[64];
	enum vsg_tx_policy pol = vsg_boardconf_parse("/rfnm/board.conf", board, sizeof(board));
	cfg.tx_allowed = pol != VSG_TXPOL_NONE;

	radio *rad = nullptr;
	if(mock) {
		rad = make_mock_radio();
	} else {
#ifdef WITH_RFNM
		rad = make_rfnm_radio(antenna, (int)pol, agc);
#else
		(void)antenna;
		(void)agc;
		fprintf(stderr, "vrtd: built without WITH_RFNM - use --mock or --selftest\n");
		return 2;
#endif
	}
	if(!rad) {
		return 1;
	}
	rad->set_samp_rate(rate);
	rad->set_rx_freq(rx_freq);
	rad->set_rx_gain(rx_gain);

	core eng(cfg, rad);
	if(!eng.start()) {
		fprintf(stderr, "vrtd: start failed\n");
		return 1;
	}
	printf("vrtd: %s | board %s tx_policy %s | export %s:%u sid %u | listen %u (tx sid %u, control) | %.3f MHz %.3f MSps gain %.0f\n",
	       rad->describe().c_str(), board, vsg_tx_policy_str(pol),
	       cfg.dest_host.empty() ? "(off)" : cfg.dest_host.c_str(), cfg.dest_port,
	       cfg.sid_rx, cfg.listen_port, cfg.sid_tx, rx_freq / 1e6, rate / 1e6, rx_gain);
	fflush(stdout);

	rfnm::httpd server(status_port, "vrt", 1,
	                   { { "GET", "/status", "radio + VRT engine counters (incl. tx_forbidden)" } },
	                   [&](const rfnm::http_req &r) -> rfnm::http_resp {
		if(r.method != "GET" || (r.path != "/status" && r.path != "/" && !r.path.empty())) {
			return {};
		}
		radio_state s = rad->state();
		char b[900];
		snprintf(b, sizeof(b),
		         "{\"radio\":\"%s\",\"board\":\"%s\",\"tx_policy\":\"%s\",\"rx_freq\":%.0f,\"tx_freq\":%.0f,\"rate\":%.0f,\"rx_gain\":%.1f,\"tx_power\":%.1f,"
		         "\"rx_on\":%s,\"tx_on\":%s,\"data_out\":%llu,\"ctx_out\":%llu,\"samples_out\":%llu,"
		         "\"data_in\":%llu,\"samples_in\":%llu,\"seq_gaps_in\":%llu,\"tx_drops\":%llu,\"tx_forbidden\":%llu,"
		         "\"cmds_in\":%llu,\"cmds_rejected\":%llu,\"acks_out\":%llu,\"malformed_in\":%llu}",
		         rad->describe().c_str(), board, vsg_tx_policy_str(pol),
		         s.rx_freq_hz, s.tx_freq_hz, s.samp_rate_hz, s.rx_gain_db, s.tx_power_dbm,
		         s.rx_on ? "true" : "false", s.tx_on ? "true" : "false",
		         (unsigned long long)eng.st.data_pkts_out.load(), (unsigned long long)eng.st.ctx_pkts_out.load(),
		         (unsigned long long)eng.st.samples_out.load(), (unsigned long long)eng.st.data_pkts_in.load(),
		         (unsigned long long)eng.st.samples_in.load(), (unsigned long long)eng.st.seq_gaps_in.load(),
		         (unsigned long long)eng.st.tx_drops.load(), (unsigned long long)eng.st.tx_forbidden.load(),
		         (unsigned long long)eng.st.cmds_in.load(), (unsigned long long)eng.st.cmds_rejected.load(),
		         (unsigned long long)eng.st.acks_out.load(), (unsigned long long)eng.st.malformed_in.load());
		return { 200, std::string(b) };
	});
	if(!server.ok()) {
		fprintf(stderr, "vrtd: http bind failed on 127.0.0.1:%d\n", status_port);
	}

	while(!g_stop) {
		usleep(200000);
	}
	eng.stop();
	delete rad;
	printf("vrtd: exiting\n");
	return 0;
}

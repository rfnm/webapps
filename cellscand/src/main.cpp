// cellscand: LTE cell-survey daemon for the RFNM (see README.md). A scan is a one-shot
// auftrag: the daemon starts idle and opens no radio; POST /scan captures a 15.36 MSps burst
// per listed carrier, decodes MIB->PCFICH->PDCCH->PDSCH->SIB1 offline (the verified cellsyncd
// chain, linked unchanged from ../cellsyncd/src) and reports every cell's PCI, operator (from
// the PLMN), TAC, cellIdentity + eNB/sector, bandwidth, ports, frequency and receive power.
//   --selftest              hardware-free acceptance gate (pins the two SIB1 blocks)
//   --file <cf32> [--freq MHz] [--no-conj]   offline decode of a capture, then exit
//   --port N --gain dB --antenna auto|NAME --agc 0|1   radio/serve knobs (antenna: auto = the
//                           per-carrier band policy in bandplan.h, a name pins the whole job)
//   --preset NAME           default frequency list when a POST /scan omits freqs_mhz
// HTTP on 127.0.0.1:7692, published by nginx as /api/app/cellscan/.
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bandplan.h"
#include "burst.h"
#include "job.h"
#include "plmn.h"
#include "report.h"
#include "rfnm_httpd.h"
#include "rfnm_json.h"

namespace cellscan {
int run_selftest(bool verbose);
}
using namespace cellscan;

static std::atomic<bool> g_stop{ false };
static void on_signal(int) { g_stop = true; }

static const char *k_persist_path = "/rfnm/apps/cellscan/last_scan.json";

// ---- preset frequency lists (the manifest config key selects one) -------------------------
// Only choices[] survive appd (no free text) - so the PRESET is a fixed menu, and the real,
// arbitrary list arrives in the POST /scan body, which is not choice-constrained.
static std::vector<double> preset_freqs(const std::string &name, const scan_doc &last) {
	if(name == "b20") {
		return { 796.0, 806.0, 816.0 };
	}
	if(name == "b20+b8") {
		return { 796.0, 806.0, 816.0, 942.0, 951.0 };
	}
	if(name == "eu-macro") {
		return { 796.0, 806.0, 816.0, 942.0, 951.0, 1815.0, 1835.0, 2120.0, 2140.0, 2650.0 };
	}
	if(name == "last" && !last.freqs_mhz.empty()) {
		return last.freqs_mhz;
	}
	return { 796.0, 806.0, 816.0 };   // safe default = the reference B20 block
}

// An explicit antenna name PINS the RF path for a whole job, exactly as before the policy
// existed. "auto" - and the empty string systemd substitutes for an unset config key - means
// the per-carrier band policy (bandplan.h) instead.
static std::string antenna_pin(const std::string &name) {
	return name == "auto" ? std::string() : name;
}

static rfnm::http_resp err(int code, const std::string &msg) {
	return { code, std::string("{\"err\":\"") + rfnm::jesc(msg) + "\"}" };
}

// ---- offline one-shot: decode a cf32 file and print the cells (acceptance path) -----------
static int run_file(const std::string &path, double freq_mhz, bool conj, int dwell) {
	scanner sc("");   // no persistence for a one-shot
	sc.set_source_factory([path, conj]() -> std::unique_ptr<burst_source> {
		return std::make_unique<file_burst_source>(path, conj);
	});
	job_params p;
	p.freqs_mhz = { freq_mhz };
	p.dwell_s = dwell;
	p.deep = false;   // the file IS the whole burst; there is no longer capture to escalate to
	std::string e;
	if(!sc.start(p, e)) {
		fprintf(stderr, "cellscand: %s\n", e.c_str());
		return 2;
	}
	while(sc.running()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	scan_doc d = sc.snapshot();
	printf("cellscand: %s @ %.3f MHz (%s) -> %s\n", path.c_str(), freq_mhz,
	       conj ? "conjugated" : "as-is", d.state.c_str());
	for(const auto &cr : d.carriers) {
		printf("  carrier %.3f MHz  band %d  EARFCN %d  power %.1f dB  status %s\n",
		       cr.freq_mhz, cr.band, cr.earfcn, cr.power_db, cr.status.c_str());
		for(const auto &c : cr.cells) {
			printf("    PCI %d (NID1 %d NID2 %d) %s\n", c.pci, c.nid1, c.nid2, c.status.c_str());
			if(c.mib_ok) {
				printf("      %d ports, %d PRB (%.1f MHz), SFN %d, %s\n",
				       c.ports, c.nrb, c.bw_mhz, c.sfn, c.duplex.c_str());
			}
			if(c.sib1_ok) {
				printf("      operator %s  PLMN", c.op.c_str());
				for(const auto &pl : c.plmns) {
					printf(" %s", pl.plmn.c_str());
				}
				printf("\n      TAC 0x%04X (%d)  ECI 0x%07X  eNB %u  sector %d  band_ind %d\n",
				       (unsigned)c.tac, c.tac, c.eci, c.enb, c.sector, c.band_ind);
				printf("      q-RxLevMin %d dBm  siWindow %d ms  SIB16 %s  barred %s  SIBs [",
				       c.q_rxlevmin_dbm, c.si_window_ms, c.sib16 ? "yes" : "no",
				       c.cell_barred ? "yes" : "no");
				for(size_t i = 0; i < c.sibs.size(); i++) {
					printf("%s%d", i ? " " : "", c.sibs[i]);
				}
				printf("]\n");
			}
		}
	}
	printf("%s\n", emit_scan(d).c_str());
	return 0;
}

int main(int argc, char **argv) {
	bool selftest = false;
	int port = 7692;
	int gain = 12;
	bool agc = false;
	std::string antenna;   // empty = the per-carrier policy; a name pins it
	std::string preset = "b20";
	std::string file;
	double file_freq = 816.0;
	bool conj = true;
	int dwell = 2;

	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
		if(a == "--selftest") {
			selftest = true;
		} else if(a == "--port") {
			port = atoi(next());
		} else if(a == "--gain") {
			gain = atoi(next());
		} else if(a == "--agc") {
			agc = atoi(next()) != 0;
		} else if(a == "--antenna") {
			antenna = antenna_pin(next());
		} else if(a == "--preset") {
			preset = next();
		} else if(a == "--file") {
			file = next();
		} else if(a == "--freq") {
			file_freq = atof(next());
		} else if(a == "--no-conj") {
			conj = false;
		} else if(a == "--dwell") {
			dwell = atoi(next());
		} else {
			fprintf(stderr, "cellscand: unknown arg %s (see README)\n", a.c_str());
			return 2;
		}
	}

	if(selftest) {
		return run_selftest(true);
	}
	if(!file.empty()) {
		return run_file(file, file_freq, conj, dwell);
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	scanner sc(k_persist_path);
	sc.load_persist();
#ifdef WITH_RFNM
	sc.set_source_factory([gain, agc, antenna]() -> std::unique_ptr<burst_source> {
		(void)gain; (void)agc; (void)antenna;
		return std::make_unique<rfnm_burst_source>();
	});
#else
	// host build: no radio. The daemon still serves /status (and rejects /scan cleanly) so the
	// panel and API can be exercised without hardware.
	sc.set_source_factory(nullptr);
#endif

	auto t_start = std::chrono::steady_clock::now();

	rfnm::httpd server(port, "cellscan", 1,
	                   { { "GET", "/status", "scan state, progress and results" },
	                     { "POST", "/scan", "start a survey over a frequency list" },
	                     { "POST", "/sweep", "automatic band sweep: probe the band plan, decode carriers above threshold" },
	                     { "POST", "/cancel", "abort the running survey" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method == "GET" && (req.path == "/status" || req.path == "/" || req.path.empty())) {
			double up = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
			scan_doc d = sc.snapshot();
			std::string j = "{\"state\":\"";
			j += sc.running() ? "scanning" : "idle";
			j += "\",";
			char b[96];
			snprintf(b, sizeof(b), "\"uptime\":%.1f,", up);
			j += b;
			snprintf(b, sizeof(b), "\"persist\":{\"loaded\":%s,\"saved_at\":%lld},",
			         sc.persist_loaded() ? "true" : "false", (long long)sc.persist_saved_at());
			j += b;
			j += "\"scan\":" + emit_scan(d) + "}";
			return { 200, j };
		}
		if(req.method == "POST" && req.path == "/scan") {
			job_params p;
			p.gain = gain;
			p.agc = agc;
			p.antenna.pin = antenna;
			p.dwell_s = 2;
			p.deep = true;
			rfnm::jv body;
			bool have_body = !req.body.empty() && rfnm::jparse(req.body, body) && body.t == rfnm::jv::OBJ;
			if(have_body) {
				if(body.get("antenna")) {
					p.antenna.pin = antenna_pin(body.str("antenna"));
				}
				if(const rfnm::jv *fr = body.get("freqs_mhz"); fr && fr->t == rfnm::jv::ARR) {
					for(const auto &fv : fr->a) {
						if(fv.t == rfnm::jv::NUM) {
							p.freqs_mhz.push_back(fv.n);
						}
					}
				}
				if(body.get("dwell_s")) {
					p.dwell_s = (int)body.num("dwell_s", 2);
				}
				if(body.get("deep")) {
					p.deep = body.boolean("deep", true);
				}
			}
			if(p.freqs_mhz.empty()) {
				p.freqs_mhz = preset_freqs(preset, sc.snapshot());
			}
			std::string e;
			uint64_t id = sc.start(p, e);
			if(id == 0) {
				return err(sc.running() ? 409 : 400, e);
			}
			char b[64];
			snprintf(b, sizeof(b), "{\"job\":%llu,\"state\":\"running\"}", (unsigned long long)id);
			return { 200, b };
		}
		if(req.method == "POST" && req.path == "/sweep") {
			// the one-button survey: no body = the whole band plan; {"bands":["b20","b3"]}
			// restricts it. Everything else (threshold, candidates, selection) is bandplan.h's
			// business - the caller picks bands, never frequencies.
			job_params p;
			p.gain = gain;
			p.agc = agc;
			p.antenna.pin = antenna;
			p.dwell_s = 2;
			p.deep = true;
			p.sweep = true;
			rfnm::jv body;
			if(!req.body.empty() && rfnm::jparse(req.body, body) && body.t == rfnm::jv::OBJ) {
				if(body.get("antenna")) {
					p.antenna.pin = antenna_pin(body.str("antenna"));
				}
				if(const rfnm::jv *bs = body.get("bands"); bs && bs->t == rfnm::jv::ARR) {
					for(const auto &bv : bs->a) {
						if(bv.t != rfnm::jv::STR) {
							return err(400, "bands: Liste von Bandnamen erwartet");
						}
						std::string key = bv.s;
						for(auto &ch : key) {
							ch = (char)tolower((unsigned char)ch);
						}
						const sweep_band_row *row = sweep_band_by_key(key);
						if(!row) {
							return err(400, "unbekanntes Band: " + bv.s);
						}
						p.bands.push_back(row->band);
					}
				}
			}
			size_t ncand = sweep_candidates(p.bands).size();
			std::string e;
			uint64_t id = sc.start(p, e);
			if(id == 0) {
				return err(sc.running() ? 409 : 400, e);
			}
			char b[112];
			snprintf(b, sizeof(b), "{\"job\":%llu,\"state\":\"running\",\"mode\":\"sweep\",\"candidates\":%zu}",
			         (unsigned long long)id, ncand);
			return { 200, b };
		}
		if(req.method == "POST" && req.path == "/cancel") {
			bool was = sc.running();
			sc.cancel();
			char b[80];
			if(was) {
				snprintf(b, sizeof(b), "{\"job\":%llu,\"state\":\"cancelled\"}",
				         (unsigned long long)sc.last_job());
			} else {
				snprintf(b, sizeof(b), "{\"state\":\"idle\"}");
			}
			return { 200, b };
		}
		return {};
	});
	if(!server.ok()) {
		fprintf(stderr, "cellscand: http bind failed on 127.0.0.1:%d\n", port);
		return 1;
	}
	printf("cellscand: serving on 127.0.0.1:%d (preset %s, gain %d, antenna %s)%s\n",
	       port, preset.c_str(), gain, antenna.empty() ? "auto" : antenna.c_str(),
#ifdef WITH_RFNM
	       "");
#else
	       " [host build: no radio]");
#endif
	fflush(stdout);

	while(!g_stop.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	sc.cancel();
	printf("cellscand: exiting\n");
	return 0;
}

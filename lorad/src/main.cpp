// lorad: RX-only LoRa sniffer daemon for the RFNM web GUI.
//   --mode lorawan     - LoRaWAN uplink channels for the selected --region (default)
//   --mode meshtastic  - Meshtastic LongFast preset for the selected --region
//   --region eu868|us915 selects the band plan (default eu868).
// Manual overrides (any region/plan): --center HZ, --freqs f1,f2,.. (MHz),
//   --sync 0xNN, and for LoRaWAN us915 --subband N (1..8).
// HTTP on 127.0.0.1:7684 behind nginx /lorapi/: /status, /packets.
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

#include "channelizer.h"
#include "rfnm_httpd.h"
#include "packets.h"
#include "parse.h"
#include "receiver.h"

#ifdef WITH_RFNM
#include "capture.h"
#endif

namespace lorad {
int run_selftest();
int run_selfloop_offline(double snr_db, double cfo_hz);
}

using namespace lorad;

static std::atomic<bool> g_stop{false};

static void on_signal(int) {
	g_stop = true;
}

// ---- radio configuration ---------------------------------------------------------
// A single resolved plan drives both the live capture path and --bench, so the
// benchmark exercises exactly the channel set the daemon would run.
struct radio_config {
	std::string mode = "lorawan";   // "lorawan" | "meshtastic"
	std::string region = "eu868";   // "eu868" | "us915"
	double center = 0;
	double rate = 2000000;
	double out_rate = 0;            // 125000 lorawan / 250000 meshtastic
	std::vector<double> freqs;
	uint8_t sync = 0x34;
	unsigned sf_mask = 0;
	bool bw250 = false;
	int subband = 1;               // us915 lorawan sub-band 1..8
};

// --freqs is a comma list in MHz (e.g. 902.3,902.5,902.7); values become Hz.
static bool parse_freq_list(const std::string &s, std::vector<double> &out) {
	out.clear();
	size_t i = 0;
	while(i <= s.size()) {
		size_t c = s.find(',', i);
		std::string tok = s.substr(i, c == std::string::npos ? std::string::npos : c - i);
		if(!tok.empty()) {
			out.push_back(atof(tok.c_str()) * 1e6);
		}
		if(c == std::string::npos) {
			break;
		}
		i = c + 1;
	}
	return !out.empty();
}

// Fill the preset for (mode, region); manual overrides are applied by the caller.
static bool build_preset(radio_config &c) {
	if(c.mode == "lorawan") {
		c.out_rate = 125000;
		c.bw250 = false;
		c.sync = 0x34;
		if(c.region == "eu868") {
			c.center = 867.8e6;
			c.freqs = { 868.1e6, 868.3e6, 868.5e6, 867.1e6, 867.3e6, 867.5e6, 867.7e6, 867.9e6 };
			c.sf_mask = 0x1F80; // SF7..SF12
		} else if(c.region == "us915") {
			if(c.subband < 1 || c.subband > 8) {
				fprintf(stderr, "lorad: --subband must be 1..8\n");
				return false;
			}
			// sub-band k: 8 x 125 kHz channels from 902.3 MHz + (k-1)*1.6 MHz, 200 kHz apart
			double start = 902.3e6 + (c.subband - 1) * 1.6e6;
			c.freqs.clear();
			for(int j = 0; j < 8; j++) {
				c.freqs.push_back(start + j * 0.2e6);
			}
			c.center = start + 3.5 * 0.2e6; // midpoint of the 8
			c.sf_mask = 0x0780; // SF7..SF10 only: US915 125 kHz uplinks are DR0..DR3 (no SF11/12)
		} else {
			return false;
		}
	} else if(c.mode == "meshtastic") {
		c.out_rate = 250000;
		c.bw250 = true;
		c.sync = 0x2B;
		c.sf_mask = 1u << 11; // LongFast: SF11
		if(c.region == "eu868") {
			c.center = 869.525e6;
		} else if(c.region == "us915") {
			c.center = 906.875e6; // US LongFast default
		} else {
			return false;
		}
		c.freqs = { c.center };
	} else {
		return false;
	}
	return true;
}

static double cpu_seconds() {
	struct timespec ts;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// --bench: no hardware. Synthesize complex Gaussian noise at the configured rate and
// push it through every DDC + channel_receiver single-threaded in 4096-sample blocks,
// exactly as the capture thread would. Prints CPU seconds and the real-time factor
// (sim seconds / CPU seconds) - a portable proxy for the on-board steady-state cost.
static int run_bench(const radio_config &cfg, double seconds) {
	printf("lorad bench: mode=%s region=%s channels=%zu rate=%.3f MSps out=%.0f kHz sim=%.1fs\n",
	       cfg.mode.c_str(), cfg.region.c_str(), cfg.freqs.size(), cfg.rate / 1e6, cfg.out_rate / 1e3, seconds);
	fflush(stdout);

	std::vector<std::unique_ptr<ddc>> dcs;
	std::vector<std::unique_ptr<channel_receiver>> rxs;
	std::atomic<uint64_t> detections{0};
	for(double f : cfg.freqs) {
		dcs.emplace_back(new ddc(cfg.rate, cfg.out_rate, f - cfg.center));
		rxs.emplace_back(new channel_receiver(f, cfg.sync, cfg.sf_mask, cfg.bw250,
		                                      [&](const rx_packet &) { detections++; }));
	}

	const size_t B = 4096;
	uint64_t nblocks = (uint64_t)(seconds * cfg.rate) / B;
	if(nblocks == 0) {
		nblocks = 1;
	}
	std::mt19937 rng(0xB0A7CAFE);
	std::normal_distribution<float> nd(0.0f, 0.2f);
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
	double sim = (double)fed / cfg.rate;
	double rtf = cpu > 0 ? sim / cpu : 0;
	printf("lorad bench: cpu=%.3fs realtime_factor=%.3fx (sim/cpu) sim=%.3fs samples=%llu blocks=%llu detections=%llu\n",
	       cpu, rtf, sim, (unsigned long long)fed, (unsigned long long)nblocks, (unsigned long long)detections.load());
	fflush(stdout);
	return 0;
}

// one worker per channel: the capture thread fans raw blocks out to bounded queues,
// each worker runs its own DDC + receiver so the channels spread across cores
struct channel_worker {
	ddc dc;
	channel_receiver rx;
	std::mutex mtx;
	std::condition_variable cv;
	std::deque<std::shared_ptr<std::vector<cf>>> q;
	uint64_t drops = 0;
	std::atomic<uint64_t> packets{0};       // decoded frames on this channel (set by the rx callback)
	std::atomic<float> last_snr{0};
	std::thread thread;

	channel_worker(double in_rate, double out_rate, double offset_hz, double freq_hz, uint8_t sync,
	               unsigned sf_mask, bool bw250, std::function<void(const rx_packet &)> on_packet)
		: dc(in_rate, out_rate, offset_hz), rx(freq_hz, sync, sf_mask, bw250, std::move(on_packet)) {
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

static void usage() {
	fprintf(stderr,
	        "usage: lorad [--mode lorawan|meshtastic] [--region eu868|us915] [--subband N]\n"
	        "             [--center HZ] [--freqs f1,f2,.. (MHz)] [--sync 0xNN]\n"
	        "             [--gain dB] [--agc 0|1] [--antenna name] [--rate hz] [--port n] [--bench [secs]] [--selftest]\n");
}

int main(int argc, char **argv) {
	std::string mode = "lorawan";
	std::string region = "eu868";
	int gain = 30;
	bool agc = false;
	std::string antenna;
	double rate = 2000000;
	int port = 7684;
	int subband = 1;
	bool selftest = false;
	bool bench = false;
	double bench_secs = 5;
	bool have_center = false, have_freqs = false, have_sync = false;
	double ov_center = 0;
	uint8_t ov_sync = 0;
	std::vector<double> ov_freqs;

	bool selfloop = false;
	bool selfloop_offline = false;
	double sl_snr = 10.0, sl_cfo = 2500.0;
	int tx_power = 0;
	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if(a == "--selftest") {
			selftest = true;
		} else if(a == "--selfloop-offline") {
			selfloop_offline = true;
		} else if(a == "--sl-snr" && i + 1 < argc) {
			sl_snr = atof(argv[++i]);
		} else if(a == "--sl-cfo" && i + 1 < argc) {
			sl_cfo = atof(argv[++i]);
		} else if(a == "--selfloop") {
			selfloop = true;
		} else if(a == "--tx-power" && i + 1 < argc) {
			tx_power = atoi(argv[++i]);
		} else if(a == "--bench") {
			bench = true;
			if(i + 1 < argc) {
				char *end = nullptr;
				double v = strtod(argv[i + 1], &end);
				if(end && *end == '\0' && v > 0) {
					bench_secs = v;
					i++;
				}
			}
		} else if(a == "--mode" && i + 1 < argc) {
			mode = argv[++i];
		} else if(a == "--region" && i + 1 < argc) {
			region = argv[++i];
		} else if(a == "--subband" && i + 1 < argc) {
			subband = atoi(argv[++i]);
		} else if(a == "--center" && i + 1 < argc) {
			ov_center = atof(argv[++i]);
			have_center = true;
		} else if(a == "--freqs" && i + 1 < argc) {
			if(!parse_freq_list(argv[++i], ov_freqs)) {
				fprintf(stderr, "lorad: --freqs needs a comma list of MHz values\n");
				return 2;
			}
			have_freqs = true;
		} else if(a == "--sync" && i + 1 < argc) {
			ov_sync = (uint8_t)strtol(argv[++i], nullptr, 0);
			have_sync = true;
		} else if(a == "--gain" && i + 1 < argc) {
			gain = atoi(argv[++i]);
		} else if(a == "--agc" && i + 1 < argc) {
			agc = atoi(argv[++i]) != 0;
		} else if(a == "--antenna" && i + 1 < argc) {
			antenna = argv[++i];
		} else if(a == "--rate" && i + 1 < argc) {
			rate = atof(argv[++i]);
		} else if(a == "--port" && i + 1 < argc) {
			port = atoi(argv[++i]);
		} else {
			usage();
			return 2;
		}
	}

	if(selftest) {
		return run_selftest();
	}

	if(selfloop_offline) {
		return run_selfloop_offline(sl_snr, sl_cfo);
	}

	if(selfloop) {
#ifdef WITH_RFNM
		// LoRa carrier at LO+400 kHz (off the DC spike); --center overrides the LO, --gain / --tx-power tune levels.
		double lo = have_center ? ov_center : 906.475e6;
		int n = run_selfloop(lo, 400e3, rate, 250e3, gain, tx_power, 30);
		return n > 0 ? 0 : 1;
#else
		fprintf(stderr, "lorad: --selfloop needs the WITH_RFNM build\n");
		return 2;
#endif
	}

	radio_config cfg;
	cfg.mode = mode;
	cfg.region = region;
	cfg.rate = rate;
	cfg.subband = subband;
	if(!build_preset(cfg)) {
		fprintf(stderr, "lorad: unknown mode/region '%s/%s'\n", mode.c_str(), region.c_str());
		usage();
		return 2;
	}
	// manual overrides on top of the preset
	if(have_sync) {
		cfg.sync = ov_sync;
	}
	if(have_freqs) {
		cfg.freqs = ov_freqs;
		if(!have_center) {
			double lo = ov_freqs[0], hi = ov_freqs[0];
			for(double f : ov_freqs) {
				lo = std::min(lo, f);
				hi = std::max(hi, f);
			}
			cfg.center = (lo + hi) / 2; // default center = midpoint of the given channels
		}
	}
	if(have_center) {
		cfg.center = ov_center;
	}
	if(fmod(cfg.rate, cfg.out_rate) != 0) {
		fprintf(stderr, "lorad: rate %.0f is not an integer multiple of the channel rate %.0f\n", cfg.rate, cfg.out_rate);
		return 2;
	}

	if(bench) {
		return run_bench(cfg, bench_secs);
	}

#ifndef WITH_RFNM
	(void)gain;
	(void)agc;
	(void)port;
	(void)on_signal;
	fprintf(stderr, "lorad: built without librfnm (WITH_RFNM=0); only --selftest and --bench work in this build\n");
	return 2;
#else
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	packet_ring ring;
	auto t_start = std::chrono::steady_clock::now();
	std::atomic<uint64_t> samples{0};

	auto on_packet = [&](const rx_packet &pkt) {
		packet_entry e;
		e.time = packet_ring::now_string();
		e.epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		                 std::chrono::system_clock::now().time_since_epoch()).count();
		e.freq = pkt.freq_hz;
		e.bw = pkt.bw_hz;
		e.sf = pkt.sf;
		e.cr = pkt.cr;
		e.snr = pkt.snr_db;
		e.len = (int)pkt.payload.size();
		e.crc_present = pkt.crc_present;
		e.crc_ok = pkt.crc_ok;
		e.hex = hex_string(pkt.payload);
		if(cfg.mode == "meshtastic") {
			e.info = meshtastic_info(pkt.payload);
			mesh_fields mf = meshtastic_decode(pkt.payload);
			if(mf.valid) {
				e.mesh = true;
				e.m_from = mesh_node_string(mf.from);
				e.m_to = mesh_node_string(mf.to);
				e.m_id = mf.id;
				e.m_hop_limit = mf.hop_limit;
				e.m_hop_start = mf.hop_start;
				e.m_decoded = mf.decoded;
				e.m_portnum = mf.port;
				e.m_port = mf.portname;
				e.m_has_text = mf.is_text;
				e.m_text = mf.text;
				e.m_has_pos = mf.has_pos;
				e.m_lat = mf.lat;
				e.m_lon = mf.lon;
				e.m_has_alt = mf.has_alt;
				e.m_alt = mf.alt;
			}
		} else {
			e.info = lorawan_info(pkt.payload);
		}
		if(pkt.crc_present && !pkt.crc_ok) {
			e.info = "[crc bad] " + (e.info.empty() ? e.hex : e.info);
		}
		printf("pkt %.3f MHz SF%d snr=%.1f len=%d %s\n", pkt.freq_hz / 1e6, pkt.sf, pkt.snr_db, e.len, e.info.c_str());
		fflush(stdout);
		ring.push(std::move(e));
	};

	std::vector<std::unique_ptr<channel_worker>> workers;
	for(size_t i = 0; i < cfg.freqs.size(); i++) {
		double f = cfg.freqs[i];
		// wrap the shared handler so each channel keeps its own packet counter + last SNR
		// (invoked from the worker's own thread, after the workers vector is fully built)
		auto per_chan = [&workers, i, &on_packet](const rx_packet &pkt) {
			workers[i]->packets++;
			workers[i]->last_snr = pkt.snr_db;
			on_packet(pkt);
		};
		workers.emplace_back(new channel_worker(cfg.rate, cfg.out_rate, f - cfg.center, f, cfg.sync, cfg.sf_mask, cfg.bw250, per_chan));
	}

	rfnm::httpd server(port, "lora", 1,
	                   { { "GET", "/status", "scanner + per-channel decode state" }, { "GET", "/packets", "decoded packet ring" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method != "GET") {
			return {};
		}
		const std::string &path = req.path;
		if(path == "/packets") {
			return { 200, ring.to_json() };
		}
		if(path == "/status" || path == "/" || path.empty()) {
			double up = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
			uint64_t drops = 0;
			std::string chans = "[";
			for(size_t i = 0; i < workers.size(); i++) {
				auto &w = workers[i];
				uint64_t wdrops, wq;
				{
					std::lock_guard<std::mutex> lk(w->mtx);
					wdrops = w->drops;
					wq = w->q.size();
				}
				drops += wdrops;
				uint64_t pk = w->packets.load();
				char cb[160];
				snprintf(cb, sizeof(cb), "%s{\"freq\":%.0f,\"packets\":%llu,\"drops\":%llu,\"queue\":%llu",
				         i ? "," : "", cfg.freqs[i], (unsigned long long)pk, (unsigned long long)wdrops, (unsigned long long)wq);
				chans += cb;
				if(pk > 0) {
					snprintf(cb, sizeof(cb), ",\"last_snr\":%.1f", (double)w->last_snr.load());
					chans += cb;
				}
				chans += "}";
			}
			chans += "]";
			char b[320];
			snprintf(b, sizeof(b),
			         "{\"mode\":\"%s\",\"region\":\"%s\",\"channels\":%zu,\"rate\":%.0f,\"samples\":%llu,\"uptime\":%.1f,\"queue_drops\":%llu,\"total\":%llu,\"chans\":",
			         cfg.mode.c_str(), cfg.region.c_str(), cfg.freqs.size(), cfg.rate, (unsigned long long)samples.load(), up,
			         (unsigned long long)drops, (unsigned long long)ring.total());
			return { 200, std::string(b) + chans + "}" };
		}
		return {};
	});
	if(!server.ok()) {
		fprintf(stderr, "lorad: http bind failed on 127.0.0.1:%d\n", port);
		g_stop = true;
		for(auto &w : workers) {
			w->join();
		}
		return 1;
	}

	std::string sb = (cfg.mode == "lorawan" && cfg.region == "us915") ? (" subband=" + std::to_string(cfg.subband)) : "";
	printf("lorad: mode=%s region=%s%s center=%.3f MHz rate=%.3f MSps out=%.0f kHz channels=%zu sync=0x%02x gain=%d antenna=%s http=:%d\n",
	       cfg.mode.c_str(), cfg.region.c_str(), sb.c_str(),
	       cfg.center / 1e6, cfg.rate / 1e6, cfg.out_rate / 1e3, cfg.freqs.size(), cfg.sync, gain,
	       antenna.empty() ? "preferred" : antenna.c_str(), port);
	printf("lorad: channels:");
	for(double f : cfg.freqs) {
		printf(" %.3f", f / 1e6);
	}
	printf(" MHz\n");
	fflush(stdout);

	auto cb = [&](const cf *buf, size_t n) {
		auto block = std::make_shared<std::vector<cf>>(buf, buf + n);
		for(auto &w : workers) {
			w->push(block);
		}
	};
	bool ok = run_capture(cfg.center, cfg.rate, gain, agc, antenna, g_stop, samples, cb);

	g_stop = true;
	for(auto &w : workers) {
		w->join();
	}
	printf("lorad: exiting (%s), %llu samples, %llu packets\n", ok ? "clean" : "capture failed",
	       (unsigned long long)samples.load(), (unsigned long long)ring.total());
	return ok ? 0 : 1;
#endif
}

// aisd: RX-only AIS receiver daemon for the RFNM web GUI. Decodes marine AIS (GMSK
// 9600 bps on the two 25 kHz VHF channels) into NMEA AIVDM sentences.
//   channel A = 161.975 MHz (87B), channel B = 162.025 MHz (88B); one wideband capture
//   centred at 162.0 MHz feeds both per-channel DDC + receiver chains.
// HTTP on 127.0.0.1:7686 behind nginx /aisapi/: /status, /messages.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "ais_phy.h"
#include "channelizer.h"
#include "rfnm_httpd.h"
#include "packets.h"
#include "parse.h"
#include "receiver.h"

#ifdef WITH_RFNM
#include "capture.h"
#endif

namespace aisd {
int run_selftest();
}

using namespace aisd;

static std::atomic<bool> g_stop{false};

static void on_signal(int) {
	g_stop = true;
}

// One resolved config drives both the live capture path and --bench so the benchmark
// exercises exactly the channel set the daemon would run.
struct radio_config {
	double center = 162.0e6;
	double rate = 2000000;
	double out_rate = 0;        // derived: rate / decim, ~100 kHz
	int decim = 0;
	std::vector<double> freqs;  // channel centre frequencies
	std::vector<char> labels;   // 'A' / 'B'
};

static void resolve_rates(radio_config &c) {
	c.decim = (int)std::llround(c.rate / 100000.0);
	if(c.decim < 1) {
		c.decim = 1;
	}
	c.out_rate = c.rate / c.decim;
}

static double cpu_seconds() {
	struct timespec ts;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// --bench: no hardware. Synthesize complex Gaussian noise at the capture rate and push it
// through every DDC + receiver single-threaded in 4096-sample blocks, exactly as the
// capture thread would. Prints CPU seconds and the real-time factor (sim/CPU) - a
// portable proxy for the on-board steady-state cost.
static int run_bench(const radio_config &cfg, double seconds) {
	printf("aisd bench: channels=%zu rate=%.3f MSps out=%.1f kHz (decim %d) sim=%.1fs\n",
	       cfg.freqs.size(), cfg.rate / 1e6, cfg.out_rate / 1e3, cfg.decim, seconds);
	fflush(stdout);

	std::vector<std::unique_ptr<ddc>> dcs;
	std::vector<std::unique_ptr<channel_receiver>> rxs;
	std::atomic<uint64_t> detections{0};
	for(size_t j = 0; j < cfg.freqs.size(); j++) {
		dcs.emplace_back(new ddc(cfg.rate, cfg.out_rate, cfg.freqs[j] - cfg.center));
		rxs.emplace_back(new channel_receiver(cfg.freqs[j], cfg.labels[j], cfg.out_rate,
		                                      [&](const ais_packet &) { detections++; }));
	}

	const size_t B = 4096;
	uint64_t nblocks = (uint64_t)(seconds * cfg.rate) / B;
	if(nblocks == 0) {
		nblocks = 1;
	}
	std::mt19937 rng(0xA15CAFE);
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
	printf("aisd bench: cpu=%.3fs realtime_factor=%.3fx (sim/cpu) sim=%.3fs samples=%llu detections=%llu\n",
	       cpu, rtf, sim, (unsigned long long)fed, (unsigned long long)detections.load());
	fflush(stdout);
	return 0;
}

// one worker per channel: the capture thread fans raw blocks out to bounded queues, each
// worker runs its own DDC + receiver so the channels spread across cores
struct channel_worker {
	ddc dc;
	channel_receiver rx;
	std::mutex mtx;
	std::condition_variable cv;
	std::deque<std::shared_ptr<std::vector<cf>>> q;
	uint64_t drops = 0;
	std::atomic<uint64_t> messages{0};
	std::atomic<float> last_level{0};
	std::thread thread;

	channel_worker(double in_rate, double out_rate, double offset_hz, double freq_hz, char label,
	               std::function<void(const ais_packet &)> on_packet)
		: dc(in_rate, out_rate, offset_hz), rx(freq_hz, label, out_rate, std::move(on_packet)) {
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
	        "usage: aisd [--center HZ] [--rate HZ] [--channel A|B|both]\n"
	        "            [--gain dB] [--agc 0|1] [--antenna name] [--port n] [--bench [secs]] [--selftest]\n");
}

int main(int argc, char **argv) {
	radio_config cfg;
	int gain = 40;
	bool agc = false;
	std::string antenna;
	int port = 7686;
	std::string channel = "both";
	bool selftest = false, bench = false;
	double bench_secs = 5;

	for(int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if(a == "--selftest") {
			selftest = true;
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
		} else if(a == "--center" && i + 1 < argc) {
			cfg.center = atof(argv[++i]);
		} else if(a == "--rate" && i + 1 < argc) {
			cfg.rate = atof(argv[++i]);
		} else if(a == "--channel" && i + 1 < argc) {
			channel = argv[++i];
		} else if(a == "--gain" && i + 1 < argc) {
			gain = atoi(argv[++i]);
		} else if(a == "--agc" && i + 1 < argc) {
			agc = atoi(argv[++i]) != 0;
		} else if(a == "--antenna" && i + 1 < argc) {
			antenna = argv[++i];
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

	if(channel == "A" || channel == "a" || channel == "both") {
		cfg.freqs.push_back(AIS_CH_A_HZ);
		cfg.labels.push_back('A');
	}
	if(channel == "B" || channel == "b" || channel == "both") {
		cfg.freqs.push_back(AIS_CH_B_HZ);
		cfg.labels.push_back('B');
	}
	if(cfg.freqs.empty()) {
		fprintf(stderr, "aisd: --channel must be A, B or both\n");
		return 2;
	}
	resolve_rates(cfg);

	if(bench) {
		return run_bench(cfg, bench_secs);
	}

#ifndef WITH_RFNM
	(void)gain;
	(void)agc;
	(void)port;
	(void)on_signal;
	fprintf(stderr, "aisd: built without librfnm (WITH_RFNM=0); only --selftest and --bench work in this build\n");
	return 2;
#else
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	message_ring ring;
	auto t_start = std::chrono::steady_clock::now();
	std::atomic<uint64_t> samples{0};

	auto on_packet = [&](const ais_packet &pkt) {
		message_entry e;
		e.time = message_ring::now_string();
		e.channel = pkt.channel;
		e.type = pkt.type;
		e.mmsi = pkt.mmsi;
		e.level = pkt.level_db;
		ais_fields f = ais_parse(pkt.logical);
		e.has_pos = f.has_pos; e.lat = f.lat; e.lon = f.lon;
		e.has_sog = f.has_sog; e.sog = f.sog;
		e.has_cog = f.has_cog; e.cog = f.cog;
		e.has_hdg = f.has_hdg; e.hdg = f.hdg;
		e.shiptype = f.shiptype;
		if(f.has_nav) { e.nav_status = ais_nav_status(f.nav_status); }
		e.name = f.name;
		e.callsign = f.callsign;
		e.destination = f.destination;
		e.info = ais_info(pkt.logical);
		std::vector<std::string> lines = ais_aivdm(pkt.logical, pkt.channel);
		for(size_t i = 0; i < lines.size(); i++) {
			e.nmea += (i ? "\n" : "") + lines[i];
		}
		printf("ch%c %s\n", pkt.channel, e.info.c_str());
		for(const auto &l : lines) {
			printf("  %s\n", l.c_str());
		}
		fflush(stdout);
		ring.push(std::move(e));
	};

	std::vector<std::unique_ptr<channel_worker>> workers;
	for(size_t i = 0; i < cfg.freqs.size(); i++) {
		auto per_chan = [&workers, i, &on_packet](const ais_packet &pkt) {
			workers[i]->messages++;
			workers[i]->last_level = pkt.level_db;
			on_packet(pkt);
		};
		workers.emplace_back(new channel_worker(cfg.rate, cfg.out_rate, cfg.freqs[i] - cfg.center,
		                                        cfg.freqs[i], cfg.labels[i], per_chan));
	}

	rfnm::httpd server(port, "ais", 1,
	                   { { "GET", "/status", "receiver + per-channel decode state" }, { "GET", "/messages", "decoded AIS message ring" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method != "GET") {
			return {};
		}
		const std::string &path = req.path;
		if(path == "/messages") {
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
				uint64_t msg = w->messages.load();
				char cb[192];
				snprintf(cb, sizeof(cb), "%s{\"ch\":\"%c\",\"freq\":%.0f,\"messages\":%llu,\"drops\":%llu,\"queue\":%llu",
				         i ? "," : "", cfg.labels[i], cfg.freqs[i], (unsigned long long)msg,
				         (unsigned long long)wdrops, (unsigned long long)wq);
				chans += cb;
				if(msg > 0) {
					snprintf(cb, sizeof(cb), ",\"last_level\":%.1f", (double)w->last_level.load());
					chans += cb;
				}
				chans += "}";
			}
			chans += "]";
			char b[320];
			snprintf(b, sizeof(b),
			         "{\"center\":%.0f,\"rate\":%.0f,\"out_rate\":%.0f,\"channels\":%zu,\"samples\":%llu,"
			         "\"uptime\":%.1f,\"queue_drops\":%llu,\"total\":%llu,\"chans\":",
			         cfg.center, cfg.rate, cfg.out_rate, cfg.freqs.size(), (unsigned long long)samples.load(),
			         up, (unsigned long long)drops, (unsigned long long)ring.total());
			return { 200, std::string(b) + chans + "}" };
		}
		return {};
	});
	if(!server.ok()) {
		fprintf(stderr, "aisd: http bind failed on 127.0.0.1:%d\n", port);
		g_stop = true;
		for(auto &w : workers) {
			w->join();
		}
		return 1;
	}

	printf("aisd: center=%.3f MHz rate=%.3f MSps out=%.1f kHz (decim %d) channels=%zu gain=%d antenna=%s http=:%d\n",
	       cfg.center / 1e6, cfg.rate / 1e6, cfg.out_rate / 1e3, cfg.decim, cfg.freqs.size(), gain,
	       antenna.empty() ? "preferred" : antenna.c_str(), port);
	printf("aisd: channels:");
	for(size_t i = 0; i < cfg.freqs.size(); i++) {
		printf(" %c=%.3f", cfg.labels[i], cfg.freqs[i] / 1e6);
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
	printf("aisd: exiting (%s), %llu samples, %llu messages\n", ok ? "clean" : "capture failed",
	       (unsigned long long)samples.load(), (unsigned long long)ring.total());
	return ok ? 0 : 1;
#endif
}

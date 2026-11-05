#include <algorithm>
#include <complex>
#include <functional>
// lorad: RX-only LoRa sniffer daemon for the RFNM web GUI.
//   --mode lorawan     - LoRaWAN uplink channels for the selected --region (default)
//   --mode meshtastic  - Meshtastic LongFast preset for the selected --region
//   --mode meshcore    - MeshCore narrow preset (62.5 kHz) for the selected --region
//   --region eu868|us915 selects the band plan (default eu868).
// Manual overrides (any region/plan): --center HZ, --freqs f1,f2,.. (MHz),
//   --sync 0xNN, and for LoRaWAN us915 --subband N (1..8).
// HTTP on 127.0.0.1:7684 behind nginx /lorapi/: /status, /packets.
#include <atomic>
#include <cctype>
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
#include "meshcore.h"
#include "rfnm_httpd.h"
#include "rfnm_boardconf.h"
#include "rfnm_json.h"
#include "meshcore_tx.h"
#include "meshtastic_tx.h"
#include "packets.h"
#include "parse.h"
#include "receiver.h"

#include "capture.h"

namespace lorad {
int run_selftest();
int run_selfloop_offline(double snr_db, double cfo_hz);
}

using namespace lorad;

static std::atomic<bool> g_stop{false};
static std::string g_iqfile;    // --iqfile: replay interleaved CS16 instead of opening the radio

// Feed a recorded CS16 capture through the same channel workers as live RX, at 4x real time so
// the worker queues keep up, then give them time to drain.
static bool replay_iq(const std::string &path, double rate_hz, std::atomic<bool> &stop, std::atomic<uint64_t> &samples,
                      const std::function<void(const cf *, size_t)> &cb) {
	FILE *f = fopen(path.c_str(), "rb");
	if(!f) {
		fprintf(stderr, "lorad: cannot open %s\n", path.c_str());
		return false;
	}
	std::vector<int16_t> raw(1 << 17);
	std::vector<cf> iq(raw.size() / 2);
	const float scale = 1.0f / 32768.0f;
	const auto block = std::chrono::duration<double>(iq.size() / rate_hz / 4.0);
	size_t n;
	while(!stop && (n = fread(raw.data(), 2 * sizeof(int16_t), iq.size(), f)) > 0) {
		for(size_t i = 0; i < n; i++) {
			iq[i] = cf(raw[2 * i] * scale, raw[2 * i + 1] * scale);
		}
		samples += n;
		cb(iq.data(), n);
		std::this_thread::sleep_for(block);
	}
	fclose(f);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	return true;
}

static void on_signal(int) {
	g_stop = true;
}

// ---- radio configuration ---------------------------------------------------------
// A single resolved plan drives both the live capture path and --bench, so the
// benchmark exercises exactly the channel set the daemon would run.
struct radio_config {
	std::string mode = "lorawan";   // "lorawan" | "meshtastic" | "meshcore"
	std::string region = "eu868";   // "eu868" | "us915"
	double center = 0;
	double rate = 2000000;
	double out_rate = 0;            // channel rate == LoRa BW: 125 / 250 / 62.5 kHz
	std::vector<double> freqs;
	uint8_t sync = 0x34;
	unsigned sf_mask = 0;
	int subband = 1;               // us915 lorawan sub-band 1..8
	double lo_offset = 0;           // LO placed this far below the channel midpoint
	int os = 1;                     // receiver input at os * out_rate
	double ddc_cutoff = 0.45;       // ddc lowpass edge, fraction of its output rate
	int ddc_taps = 4;               // ddc taps per decimation step
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
		c.sync = 0x2B;
		c.sf_mask = 1u << 11; // LongFast: SF11
		c.lo_offset = 250e3;  // keep the receiver's DC out of the channel
		// tuned on five minutes of captured traffic: 14 of 14 frames with a valid CRC, where the
		// 1x path got about a third. CR 4/5 corrects nothing, so one mistimed symbol loses the frame.
		c.os = 4;
		c.ddc_cutoff = 0.13;
		c.ddc_taps = 128;
		double f;
		if(c.region == "eu868") {
			f = 869.525e6;
		} else if(c.region == "us915") {
			f = 906.875e6; // US LongFast default
		} else {
			return false;
		}
		c.freqs = { f };
		c.center = f - c.lo_offset;
	} else if(c.mode == "meshcore") {
		// RadioLib private sync word, explicit header, payload CRC, no IQ inversion. The
		// coding rate comes from the PHY header, so CR 4/5 and 4/8 presets both work.
		c.out_rate = MESHCORE_BW;
		c.sync = 0x12;
		c.lo_offset = MESHCORE_LO_OFFSET;
		c.os = MESHCORE_OS;
		c.ddc_cutoff = MESHCORE_DDC_CUTOFF;
		c.ddc_taps = MESHCORE_DDC_TAPS;
		double f;
		if(c.region == "eu868") {
			f = 869.618e6;          // EU/UK Narrow + firmware default: SF8 (CR 4/8 resp. 4/5)
			c.sf_mask = (1u << 7) | (1u << 8); // NL, HU, SK, PT use SF7 on the same frequency
		} else if(c.region == "us915") {
			f = 910.525e6;          // USA / Canada: SF7 CR 4/5
			c.sf_mask = 1u << 7;
		} else {
			return false;
		}
		c.freqs = { f };
		c.center = f - c.lo_offset;
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
	printf("lorad bench: mode=%s region=%s channels=%zu rate=%.3f MSps out=%g kHz sim=%.1fs\n",
	       cfg.mode.c_str(), cfg.region.c_str(), cfg.freqs.size(), cfg.rate / 1e6, cfg.out_rate / 1e3, seconds);
	fflush(stdout);

	std::vector<std::unique_ptr<ddc>> dcs;
	std::vector<std::unique_ptr<channel_receiver>> rxs;
	std::atomic<uint64_t> detections{0};
	for(double f : cfg.freqs) {
		dcs.emplace_back(new ddc(cfg.rate, cfg.out_rate * cfg.os, f - cfg.center, cfg.ddc_cutoff, cfg.ddc_taps));
		rxs.emplace_back(new channel_receiver(f, cfg.sync, cfg.sf_mask, cfg.out_rate,
		                                      [&](const rx_packet &) { detections++; }, cfg.os));
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
	std::atomic<int64_t> busy_ms{0};        // steady clock of the last sign of channel activity
	std::thread thread;

	channel_worker(const radio_config &c, double freq_hz, std::function<void(const rx_packet &)> on_packet)
		: dc(c.rate, c.out_rate * c.os, freq_hz - c.center, c.ddc_cutoff, c.ddc_taps),
		  rx(freq_hz, c.sync, c.sf_mask, c.out_rate, std::move(on_packet), c.os) {
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
			uint64_t c0 = rx.candidates();
			rx.feed(base.data(), base.size());
			double p = 0;
			for(const cf &z : base) {
				p += std::norm(z);
			}
			p /= base.empty() ? 1 : base.size();
			floor_p = floor_p > 0 ? std::min(p, floor_p * 1.002) : p;
			if(rx.candidates() != c0 || p > 4.0 * floor_p) {   // preamble, or 6 dB over the slow floor
				busy_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				              std::chrono::steady_clock::now().time_since_epoch()).count();
			}
		}
	}
	double floor_p = 0;

	void join() {
		cv.notify_one();
		if(thread.joinable()) {
			thread.join();
		}
	}
};

static void usage() {
	fprintf(stderr,
	        "usage: lorad [--mode lorawan|meshtastic|meshcore] [--region eu868|us915] [--subband N]\n"
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
	bool tx_flag = false;
	bool tx_armed = false;
	std::string node_name = "RFNM", node_key = "/rfnm/apps/lora/node.key", tx_scope = "#de-nord";
	std::string channels_file = "/rfnm/apps/lora/channels.conf";
	std::string mt_channel_file = "/rfnm/apps/lora/mt_channel.conf";
	std::string contacts_file = "/rfnm/apps/lora/contacts.conf";
	std::string render_out, render_text, render_channel = "public";
	bool render_advert = false;
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
		} else if(a == "--iqfile" && i + 1 < argc) {
			g_iqfile = argv[++i];
		} else if(a == "--tx" && i + 1 < argc) {
			tx_flag = atoi(argv[++i]) != 0;
		} else if(a == "--tx-armed" && i + 1 < argc) {
			tx_armed = atoi(argv[++i]) != 0;
		} else if(a == "--node-name" && i + 1 < argc) {
			node_name = argv[++i];
		} else if(a == "--mt-channel-file" && i + 1 < argc) {
			mt_channel_file = argv[++i];
		} else if(a == "--node-key" && i + 1 < argc) {
			node_key = argv[++i];
		} else if(a == "--scope" && i + 1 < argc) {
			tx_scope = argv[++i];
		} else if(a == "--channels-file" && i + 1 < argc) {
			channels_file = argv[++i];
		} else if(a == "--contacts-file" && i + 1 < argc) {
			contacts_file = argv[++i];
		} else if(a == "--render-iq" && i + 1 < argc) {       // writes a file, never touches a radio
			render_out = argv[++i];
		} else if(a == "--render-text" && i + 1 < argc) {
			render_text = argv[++i];
		} else if(a == "--render-channel" && i + 1 < argc) {
			render_channel = argv[++i];
		} else if(a == "--render-advert") {
			render_advert = true;
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
			cfg.center = (lo + hi) / 2 - cfg.lo_offset; // default center = midpoint of the given channels
		}
	}
	if(have_center) {
		cfg.center = ov_center;
	}
	if(fmod(cfg.rate, cfg.out_rate * cfg.os) != 0) {
		fprintf(stderr, "lorad: rate %.0f is not an integer multiple of the channel rate %.0f\n", cfg.rate, cfg.out_rate * cfg.os);
		return 2;
	}

	if(bench) {
		return run_bench(cfg, bench_secs);
	}
	if(cfg.mode == "meshcore") {
		mc_channels_load(channels_file);
		mc_contacts_open(contacts_file);
	}
	if(!render_out.empty()) {
		mc_identity id;
		std::string ierr;
		if(!mc_identity_load(node_key, id, ierr)) {
			fprintf(stderr, "lorad: %s\n", ierr.c_str());
			return 1;
		}
		uint8_t key[16];
		std::vector<uint8_t> frame;
		uint32_t ts = (uint32_t)time(nullptr);
		if(render_advert) {
			frame = mc_build_advert(id, ts, node_name, false, 0, 0, mc_scope_named(tx_scope));
		} else if(!mc_channel_key(render_channel, key) ||
		          !mc_build_group_text(key, ts, node_name, render_text, mc_scope_named(tx_scope), frame)) {
			fprintf(stderr, "lorad: cannot build that frame\n");
			return 1;
		}
		std::vector<int16_t> iq = mc_render_iq(frame, cfg.rate, cfg.freqs[0] - cfg.center);
		iq.insert(iq.begin(), 400000, 0);
		iq.insert(iq.end(), 400000, 0);
		FILE *f = fopen(render_out.c_str(), "wb");
		if(!f || fwrite(iq.data(), 2, iq.size(), f) != iq.size()) {
			fprintf(stderr, "lorad: cannot write %s\n", render_out.c_str());
			return 1;
		}
		fclose(f);
		printf("lorad: rendered %zu byte frame, %.3f s airtime, to %s (no RF)\n", frame.size(), mc_airtime_s(frame.size()), render_out.c_str());
		return 0;
	}

#ifndef WITH_RFNM
	if(g_iqfile.empty()) {
		fprintf(stderr, "lorad: built without librfnm (WITH_RFNM=0); only --selftest, --bench and --iqfile work in this build\n");
		return 2;
	}
#endif
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	packet_ring ring;
	auto t_start = std::chrono::steady_clock::now();
	std::atomic<uint64_t> samples{0};

	std::function<void(const std::vector<uint8_t> &, packet_entry &)> mt_hook;   // set when Meshtastic can send
	auto on_packet = [&](const rx_packet &pkt) {
		if(!g_iqfile.empty()) {
			printf("packet t=%.3f f=%.3f sf=%d cr=%d snr=%.1f crc=%d len=%zu\n", pkt.sample_pos / pkt.bw_hz,
			       pkt.freq_hz / 1e6, pkt.sf, pkt.cr, pkt.snr_db, pkt.crc_ok ? 1 : 0, pkt.payload.size());
			fflush(stdout);
		}
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
			if(mt_hook && pkt.crc_ok) {
				mt_hook(pkt.payload, e);
			}
		} else if(cfg.mode == "meshcore") {
			meshcore_fill(e, pkt.payload, pkt.crc_ok);
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
		workers.emplace_back(new channel_worker(cfg, f, per_chan));
	}

	// ---- MeshCore TX: everything below is inert unless all gates open ----
	static tx_request txreq;
	mc_identity tx_id;
	mt_channel mtch;
	mt_keys mtkeys;
	if(cfg.mode == "meshtastic" && !mt_channel_file.empty()) {
		std::string merr;
		if(mt_channel_load(mt_channel_file, mtch, merr)) {
			meshtastic_set_channel(mtch.hash, mtch.psk);
		} else {
			fprintf(stderr, "lorad: %s\n", merr.c_str());
		}
	}
	const bool mt_tx = cfg.mode == "meshtastic" && mtch.ok;
	const bool tx_mode = cfg.mode == "meshcore" || mt_tx;
	auto air_of = [&](size_t n) { return mt_tx ? mt_airtime_s(n) : mc_airtime_s(n); };
	auto mt_ids = [&](uint32_t &from, uint32_t &id) {
		memcpy(&from, tx_id.pub, 4);
		std::random_device rd;
		id = rd();
	};
	std::mutex tx_mtx;       // one send at a time
	mc_duty tx_duty;
	std::atomic<int64_t> armed_until{0};
	const int tx_power_dbm = tx_power;
	bool board_all = false;
	{
		char bn[32] = "";
		board_all = vsg_boardconf_parse("/rfnm/board.conf", bn, sizeof(bn)) == VSG_TXPOL_ALL;
	}
	auto mono_s = []() {
		return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	};
	// A node that must confirm DMs at any hour cannot wait for someone to arm it: on 2026-11-01 a
	// DM went unanswered for 40 minutes after the 60 minute arm ran out. Only the time lock goes;
	// board policy, --tx 1, duty budget, listen-before-talk and /tx/disarm all stay in force.
	if(tx_armed && tx_flag && cfg.mode == "meshcore") {
		armed_until = (int64_t)(mono_s() + 3.15e8);
	}
	auto gate = [&]() {
		mc_gate_in g;
		g.board_all = board_all;
		g.tx_flag = tx_flag;
		g.armed = mono_s() < (double)armed_until.load();
		g.power_dbm = tx_power_dbm;
		g.tx_mode = tx_mode;
		return mc_tx_gate(g);
	};
	// listen-before-talk: the channel must have been quiet for a random 200..700 ms
	auto wait_clear = [&]() {
		auto now_ms = []() {
			return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		};
		int64_t quiet = 200 + rand() % 500, limit = now_ms() + 8000;
		while(now_ms() < limit) {
			if(now_ms() - workers[0]->busy_ms.load() > quiet) {
				return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		return false;
	};
	// Outgoing DMs wait here for their ACK. {"sent":true} only says the board transmitted: on
	// 2026-11-01 a reply to a node two hops away got no repeater echo and never arrived.
	struct dm_out {
		int id;
		std::string to;
		uint8_t peer[32];
		uint32_t ts;
		std::string text;
		std::string scope;
		std::vector<uint8_t> frame;   // Meshtastic: every try repeats the same packet and id
		std::vector<uint32_t> acks;   // one hash per attempt: any of them means delivered
		double t0, next;
		std::string state;            // pending, delivered, failed
	};
	std::mutex dm_m;
	std::deque<dm_out> dm_box;
	int dm_next_id = 1;
	const int DM_TRIES = 3;
	// calcFloodTimeoutMillisFor of the firmware: 500 ms + 16 airtimes
	// Meshtastic: the ACK only has to come straight back, airtime and listen-before-talk included
	auto dm_timeout = [&](size_t frame_len) { return mt_tx ? 3.0 + 6.0 * mt_airtime_s(frame_len) : 0.5 + 16.0 * mc_airtime_s(frame_len); };

	auto send_frame = [&](const std::vector<uint8_t> &frame) -> rfnm::http_resp {
		std::lock_guard<std::mutex> lk(tx_mtx);
		const char *why = gate();
		if(why) {
			return { 403, std::string("{\"error\":\"") + why + "\"}" };
		}
		double air = air_of(frame.size());
		if(!tx_duty.allow(mono_s(), air)) {
			return { 429, "{\"error\":\"duty cycle budget used up\"}" };
		}
		if(!wait_clear()) {
			return { 503, "{\"error\":\"channel busy\"}" };
		}
		if(gate()) {
			return { 403, "{\"error\":\"disarmed while waiting\"}" };
		}
		std::unique_lock<std::mutex> rl(txreq.m);
		txreq.iq = mt_tx ? lora_render_iq(mt_phy(), 250000.0, frame, cfg.rate, cfg.freqs[0] - cfg.center)
		                 : mc_render_iq(frame, cfg.rate, cfg.freqs[0] - cfg.center);
		txreq.power_dbm = tx_power_dbm;
		txreq.done = false;
		txreq.pending = true;
		tx_duty.commit(mono_s(), air);
		bool fin = txreq.cv.wait_for(rl, std::chrono::seconds(10), [] { return txreq.done; });
		if(!fin || !txreq.ok) {
			return { 500, "{\"error\":\"" + (fin ? txreq.err : std::string("tx timed out")) + "\"}" };
		}
		char b[96];
		snprintf(b, sizeof(b), "{\"sent\":true,\"bytes\":%zu,\"airtime\":%.3f}", frame.size(), air);
		return { 200, b };
	};

	rfnm::httpd server(port, "lora", 1,
	                   { { "GET", "/status", "scanner + per-channel decode state" }, { "GET", "/dms", "delivery state of outgoing DMs" }, { "GET", "/tx", "MeshCore TX gates and duty" }, { "POST", "/tx/arm", "arm TX for N seconds" }, { "POST", "/tx/disarm", "disarm TX" }, { "POST", "/send", "send channel text" }, { "POST", "/tx/advert", "send a signed advert" }, { "GET", "/packets", "decoded packet ring" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp {
		if(req.method == "POST" && (req.path == "/channels/add" || req.path == "/channels/remove")) {
			rfnm::jv v;
			if(!rfnm::jparse(req.body, v)) {
				return { 400, "{\"error\":\"bad json\"}" };
			}
			std::string name = v.str("name");
			if(req.path == "/channels/remove") {
				if(!mc_channel_remove(name) || !mc_channels_save(channels_file)) {
					return { 404, "{\"error\":\"no such channel\"}" };
				}
				return { 200, "{\"ok\":true}" };
			}
			if(!mc_channel_name_ok(name)) {
				return { 400, "{\"error\":\"name: letters, digits, - and _, optional leading #, at most 32\"}" };
			}
			uint8_t key[16];
			std::string hex = v.str("key");
			if(hex.empty()) {
				if(name[0] != '#') {
					return { 400, "{\"error\":\"a private channel needs its 32 hex digit key\"}" };
				}
				mc_channel_remove(name);
				mc_channel_key(name, key);
			} else {
				bool ok = hex.size() == 32;
				for(int i = 0; ok && i < 16; i++) {
					unsigned b = 0;
					ok = isxdigit((unsigned char)hex[2 * i]) && isxdigit((unsigned char)hex[2 * i + 1]) && sscanf(hex.c_str() + 2 * i, "%2x", &b) == 1;
					key[i] = (uint8_t)b;
				}
				if(!ok) {
					return { 400, "{\"error\":\"key must be 32 hex digits\"}" };
				}
			}
			if(!mc_channel_add(name, key) || !mc_channels_save(channels_file)) {
				return { 500, "{\"error\":\"could not store the channel\"}" };
			}
			return { 200, "{\"ok\":true}" };
		}
		if(req.method == "POST" && (req.path == "/send" || req.path == "/tx/advert" || req.path == "/tx/arm" || req.path == "/tx/disarm")) {
			if(req.path == "/tx/disarm") {
				armed_until = 0;
				return { 200, "{\"armed\":false}" };
			}
			rfnm::jv v;
			if(!req.body.empty() && !rfnm::jparse(req.body, v)) {
				return { 400, "{\"error\":\"bad json\"}" };
			}
			if(req.path == "/tx/arm") {
				mc_gate_in g;
				g.board_all = board_all;
				g.tx_flag = tx_flag;
				g.armed = true;
				g.power_dbm = tx_power_dbm;
				g.tx_mode = tx_mode;
				const char *why = mc_tx_gate(g);
				if(why) {
					return { 403, std::string("{\"error\":\"") + why + "\"}" };
				}
				std::string ierr;
				if(!tx_id.ok && !mc_identity_load(node_key, tx_id, ierr)) {
					return { 500, "{\"error\":\"" + ierr + "\"}" };
				}
				double sec = std::max(1.0, std::min(3600.0, v.num("seconds", 120)));
				armed_until = (int64_t)(mono_s() + sec);
				char b[64];
				snprintf(b, sizeof(b), "{\"armed\":true,\"seconds\":%.0f}", sec);
				return { 200, b };
			}
			if(!tx_id.ok) {
				return { 403, "{\"error\":\"not armed\"}" };
			}
			if(mt_tx) {
				// channel traffic at hop 0 by default: only nodes in direct range hear a test
				std::string to = v.str("to");
				int hop = std::max(0, std::min(3, (int)v.num("hop", to.empty() ? 0 : 3)));
				uint32_t from, id;
				mt_ids(from, id);
				std::vector<uint8_t> mf;
				if(!to.empty()) {
					uint32_t peer = 0;
					uint8_t pub[32];
					bool has_key = false;
					if(!mt_node_find(to, peer, pub, has_key)) {
						return { 404, "{\"error\":\"no such node\"}" };
					}
					if(!mt_build_dm(mtkeys, from, id, peer, has_key ? pub : nullptr, hop, mtch, v.str("text"), mf)) {
						return { 400, "{\"error\":\"text does not fit\"}" };
					}
					int dm;
					{
						std::lock_guard<std::mutex> lk(dm_m);
						dm = dm_next_id++;
						dm_out d;
						d.id = dm;
						d.to = to;
						d.ts = 0;
						d.frame = mf;
						d.acks.push_back(id);
						d.t0 = mono_s();
						d.next = 1e18;
						d.state = "pending";
						dm_box.push_back(d);
						if(dm_box.size() > 32) {
							dm_box.pop_front();
						}
					}
					rfnm::http_resp r = send_frame(mf);
					std::lock_guard<std::mutex> lk(dm_m);
					for(dm_out &d : dm_box) {
						if(d.id == dm) {
							if(r.code != 200) {
								d.state = "failed";
							} else if(d.state == "pending") {
								d.next = mono_s() + dm_timeout(mf.size());
							}
						}
					}
					if(r.code == 200 && !r.body.empty() && r.body.back() == '}') {
						r.body.pop_back();
						r.body += ",\"dm\":" + std::to_string(dm) + "}";
					}
					return r;
				}
				bool built = req.path == "/tx/advert" ? mt_build_nodeinfo(from, id, hop, mtch, node_name, node_name.substr(0, 4), mf,
				                                                          mtkeys.ok ? mtkeys.pub : nullptr, true)
				                                      : mt_build_text(from, id, hop, mtch, v.str("text"), mf);
				if(!built) {
					return { 400, "{\"error\":\"text does not fit\"}" };
				}
				return send_frame(mf);
			}
			mc_scope scope = mc_scope_named(v.str("scope", tx_scope));
			uint32_t ts = (uint32_t)time(nullptr);
			if(req.path == "/tx/advert") {
				return send_frame(mc_build_advert(tx_id, ts, node_name, false, 0, 0, scope));
			}
			uint8_t key[16];
			std::vector<uint8_t> frame;
			std::string to = v.str("to");
			if(!to.empty()) {
				uint8_t peer[32];
				if(!mc_contact_find(to, peer)) {
					return { 404, "{\"error\":\"no such contact\"}" };
				}
				uint32_t want = 0;
				std::string dm_scope = v.str("scope", "off");
				bool routed = false;
				if(!mc_build_dm(tx_id, peer, ts, v.str("text"), mc_scope_named(dm_scope), frame, 0, &want, &routed)) {
					return { 400, "{\"error\":\"text does not fit\"}" };
				}
				int id;
				{
					// listed before the send, or a quick ACK could find nothing to match
					std::lock_guard<std::mutex> lk(dm_m);
					id = dm_next_id++;
					dm_out d;
					d.id = id;
					d.to = to;
					memcpy(d.peer, peer, 32);
					d.ts = ts;
					d.text = v.str("text");
					d.scope = dm_scope;
					d.acks.push_back(want);
					d.t0 = mono_s();
					d.next = 1e18;        // not before the frame has left
					d.state = "pending";
					dm_box.push_back(d);
					if(dm_box.size() > 32) {
						dm_box.pop_front();
					}
				}
				rfnm::http_resp r = send_frame(frame);
				std::lock_guard<std::mutex> lk(dm_m);
				for(dm_out &d : dm_box) {
					if(d.id == id) {
						if(r.code != 200) {
							d.state = "failed";
						} else {
							// a routed try only has to go there and back; if it fails, the retries flood
							d.next = mono_s() + (routed ? 0.5 + 8.0 * mc_airtime_s(frame.size()) : dm_timeout(frame.size()));
						}
					}
				}
				if(r.code == 200 && !r.body.empty() && r.body.back() == '}') {
					r.body.pop_back();
					r.body += ",\"dm\":" + std::to_string(id) + "}";
				}
				return r;
			}
			if(!mc_channel_key(v.str("channel", "public"), key)) {
				return { 400, "{\"error\":\"channel must be public or #name\"}" };
			}
			if(!mc_build_group_text(key, ts, node_name, v.str("text"), scope, frame)) {
				return { 400, "{\"error\":\"text does not fit\"}" };
			}
			return send_frame(frame);
		}
		if(req.method != "GET") {
			return {};
		}
		const std::string &path = req.path;
		if(path == "/contacts") {        // chat nodes from verified adverts: public data
			std::string j = "{\"contacts\":[";
			bool first = true;
			for(const auto &c : (mt_tx ? mt_nodes() : mc_contacts())) {
				std::string n;
				for(char ch : c.first) {
					if(ch == '"' || ch == '\\') {
						n += '\\';
					}
					if((unsigned char)ch >= 0x20) {
						n += ch;
					}
				}
				j += std::string(first ? "" : ",") + "{\"name\":\"" + n + "\",\"key\":\"" + c.second + "\"}";
				first = false;
			}
			return { 200, j + "]}" };
		}
		if(path == "/dms") {             // delivery state of outgoing DMs, oldest first; no text
			std::string j = "{\"dms\":[";
			std::lock_guard<std::mutex> lk(dm_m);
			bool first = true;
			for(const dm_out &d : dm_box) {
				char b[128];
				snprintf(b, sizeof(b), "%s{\"dm\":%d,\"to\":\"%.16s\",\"state\":\"%s\",\"attempts\":%zu,\"age_s\":%.0f}", first ? "" : ",",
				         d.id, d.to.c_str(), d.state.c_str(), d.acks.size(), mono_s() - d.t0);
				j += b;
				first = false;
			}
			return { 200, j + "]}" };
		}
		if(path == "/channels") {        // names only: keys never leave the daemon
			std::string j = "{\"channels\":[";
			bool first = true;
			for(const std::string &n : mc_channel_names()) {
				j += (first ? "\"" : ",\"") + n + "\"";
				first = false;
			}
			return { 200, j + "]}" };
		}
		if(path == "/tx") {
			const char *why = gate();
			double remaining = std::max(0.0, (double)armed_until.load() - mono_s());
			char pk[20] = "";
			for(int i = 0; i < 6 && tx_id.ok; i++) {
				snprintf(pk + 2 * i, 3, "%02x", tx_id.pub[i]);
			}
			char b[420];
			snprintf(b, sizeof(b), "{\"enabled\":%s,\"armed\":%s,\"armed_s\":%.0f,\"power_dbm\":%d,\"node\":\"%s\",\"pubkey\":\"%s\",\"scope\":\"%s\",\"duty_used_s\":%.1f,\"duty_cap_s\":%.0f,\"blocked\":\"%s\"}",
			         (board_all && tx_flag && tx_mode) ? "true" : "false", remaining > 0 ? "true" : "false", remaining,
			         tx_power_dbm, node_name.c_str(), pk, tx_scope.c_str(), tx_duty.used(mono_s()), tx_duty.cap(), why ? why : "");
			return { 200, b };
		}
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
	printf("lorad: mode=%s region=%s%s center=%.3f MHz rate=%.3f MSps out=%g kHz channels=%zu sync=0x%02x gain=%d antenna=%s http=:%d\n",
	       cfg.mode.c_str(), cfg.region.c_str(), sb.c_str(),
	       cfg.center / 1e6, cfg.rate / 1e6, cfg.out_rate / 1e3, cfg.freqs.size(), cfg.sync, gain,
	       antenna.empty() ? "preferred" : antenna.c_str(), port);
	printf("lorad: channels:");
	for(double f : cfg.freqs) {
		printf(" %.3f", f / 1e6);
	}
	printf(" MHz\n");
	fflush(stdout);

	// private messages: the identity must be loaded to read them, and each one owes an ACK. The
	// ACK goes out through the normal gates, so an unarmed node reads but does not confirm.
	std::mutex ack_m;
	std::deque<std::pair<int, std::vector<uint8_t>>> ack_q;   // delay in ms, frame
	if(tx_flag && cfg.mode == "meshcore") {
		std::string ierr;
		if(mc_identity_load(node_key, tx_id, ierr)) {
			mc_dm_set_identity(tx_id);
			mc_ack_set_callback([&](uint32_t ack) {
				std::lock_guard<std::mutex> lk(dm_m);
				for(dm_out &d : dm_box) {
					if(d.state == "pending" && std::find(d.acks.begin(), d.acks.end(), ack) != d.acks.end()) {
						d.state = "delivered";
						printf("lorad: dm %d delivered, attempt %zu sent\n", d.id, d.acks.size());
						fflush(stdout);
					}
				}
			});
			mc_dm_set_callback([&](const mc_dm_event &e) {
				// answer in the form it arrived: repeaters drop a scoped ACK for a plain flood sender
				mc_scope sc = e.scoped ? mc_scope_named(tx_scope) : mc_scope_named("off");
				std::lock_guard<std::mutex> lk(ack_m);
				std::vector<uint8_t> f = e.flood ? mc_build_path_ack(tx_id, e, sc) : e.has_route ? mc_build_direct_ack(e) : mc_build_flood_ack(e, sc);
				ack_q.push_back({ (e.flood || !e.has_route) ? 2000 : 200, f });
			});
		} else {
			fprintf(stderr, "lorad: %s\n", ierr.c_str());
		}
	}
	if(tx_flag && mt_tx) {
		std::string ierr;
		if(mc_identity_load(node_key, tx_id, ierr) && mt_keys_load("/rfnm/apps/lora/mt.key", mtkeys, ierr)) {
			mt_nodes_open("/rfnm/apps/lora/mt_nodes.conf");
			uint32_t self;
			memcpy(&self, tx_id.pub, 4);
			mt_hook = [&, self](const std::vector<uint8_t> &frame, packet_entry &e) {
				mt_event ev = mt_receive(frame, self, mtkeys, mtch);
				uint32_t id;
				std::random_device rd;
				id = rd();
				std::vector<uint8_t> out;
				if(ev.unreadable && ev.want_ack) {
					// PKI_UNKNOWN_PUBKEY: the sender answers with its NODEINFO, and resends
					if(mt_build_ack(self, id, ev.from, ev.hop_start, mtch, ev.id, 35, out)) {
						std::lock_guard<std::mutex> lk(ack_m);
						ack_q.push_back({ 300, out });
					}
					return;
				}
				if(!ev.valid) {
					return;
				}
				if(ev.port == 1 && ev.for_us) {
					e.m_decoded = true;
					e.m_has_text = true;
					e.m_text = ev.text;
					e.info = mesh_node_string(ev.from) + " DM" + (ev.sealed ? " sealed \"" : " \"") + ev.text + "\"";
				}
				if(ev.for_us && ev.want_ack && mt_build_ack(self, id, ev.from, ev.hop_start, mtch, ev.id, 0, out)) {
					// every copy is answered: the sender repeats because the last ACK got lost
					std::lock_guard<std::mutex> lk(ack_m);
					ack_q.push_back({ 300, out });
				}
				if(ev.want_response && ev.port == 4 && mt_build_nodeinfo(self, id ^ 1, ev.hop_start, mtch, node_name, node_name.substr(0, 4), out, mtkeys.pub)) {
					std::lock_guard<std::mutex> lk(ack_m);
					ack_q.push_back({ 1500, out });
				}
				if(ev.for_us && ev.routing_error >= 0 && ev.request_id) {
					std::lock_guard<std::mutex> lk(dm_m);
					for(dm_out &d : dm_box) {
						if(d.state == "pending" && !d.frame.empty() && d.acks[0] == ev.request_id) {
							d.state = ev.routing_error == 0 ? "delivered" : "failed";
							printf("lorad: dm %d %s, error %d, attempt %zu sent\n", d.id, d.state.c_str(), ev.routing_error, d.acks.size());
							fflush(stdout);
						}
					}
				}
			};
		} else {
			fprintf(stderr, "lorad: %s\n", ierr.c_str());
		}
	}
	std::thread ack_thread([&] {
		while(!g_stop) {
			std::vector<uint8_t> f;
			int wait_ms = 0;
			{
				std::lock_guard<std::mutex> lk(ack_m);
				if(!ack_q.empty()) {
					wait_ms = ack_q.front().first;
					f = ack_q.front().second;
					ack_q.pop_front();
				}
			}
			if(f.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}
			// The sender gives up after about 16 airtimes of a short message, some 6 s, and repeaters
			// are still re-sending a flood for the first second or two and cannot hear us then. A
			// routed DM floods nothing, so its ACK can go at once.
			std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
			rfnm::http_resp r = send_frame(f);
			printf("lorad: dm ack %d\n", r.code);
			fflush(stdout);
		}
	});

	// Same timestamp and text with the next attempt number, as the companion app resends: the
	// recipient sees one message, and each try has its own ACK.
	std::thread dm_thread([&] {
		while(!g_stop) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			dm_out d;
			bool due = false;
			{
				std::lock_guard<std::mutex> lk(dm_m);
				for(dm_out &x : dm_box) {
					if(x.state == "pending" && mono_s() >= x.next) {
						if((int)x.acks.size() >= DM_TRIES) {
							x.state = "failed";
							printf("lorad: dm %d failed, no ack after %zu attempts\n", x.id, x.acks.size());
							fflush(stdout);
						} else {
							x.next = 1e18;   // held until this try has left
							d = x;
							due = true;
							break;
						}
					}
				}
			}
			if(!due) {
				continue;
			}
			std::vector<uint8_t> frame;
			uint32_t want = 0;
			int attempt = (int)d.acks.size();
			bool built = true;
			if(d.frame.empty()) {
				built = mc_build_dm(tx_id, d.peer, d.ts, d.text, mc_scope_named(d.scope), frame, attempt, &want);
			} else {
				frame = d.frame;
				want = d.acks[0];
			}
			rfnm::http_resp r;
			if(built) {
				std::lock_guard<std::mutex> lk(dm_m);
				for(dm_out &x : dm_box) {
					if(x.id == d.id) {
						x.acks.push_back(want);   // before the send, as the first try
					}
				}
			}
			if(built) {
				r = send_frame(frame);
			}
			std::lock_guard<std::mutex> lk(dm_m);
			for(dm_out &x : dm_box) {
				if(x.id == d.id && x.state == "pending") {
					if(built && r.code == 200) {
						x.next = mono_s() + dm_timeout(frame.size());
						printf("lorad: dm %d resent, attempt %d\n", x.id, attempt + 1);
					} else if(r.code == 503) {
						x.acks.pop_back();        // channel busy: nothing left the antenna, try again soon
						x.next = mono_s() + 3.0;
					} else {
						x.state = "failed";
						printf("lorad: dm %d failed, resend answered %d\n", x.id, r.code);
					}
					fflush(stdout);
				}
			}
		}
	});

	auto cb = [&](const cf *buf, size_t n) {
		auto block = std::make_shared<std::vector<cf>>(buf, buf + n);
		for(auto &w : workers) {
			w->push(block);
		}
	};
#ifdef WITH_RFNM
	bool ok = g_iqfile.empty() ? run_capture(cfg.center, cfg.rate, gain, agc, antenna, g_stop, samples, cb,
	                                         (tx_flag && tx_mode) ? &txreq : nullptr)
	                           : replay_iq(g_iqfile, cfg.rate, g_stop, samples, cb);
#else
	(void)gain;
	(void)agc;
	bool ok = replay_iq(g_iqfile, cfg.rate, g_stop, samples, cb);
#endif

	g_stop = true;
	ack_thread.join();
	dm_thread.join();
	for(auto &w : workers) {
		w->join();
	}
	printf("lorad: exiting (%s), %llu samples, %llu packets\n", ok ? "clean" : "capture failed",
	       (unsigned long long)samples.load(), (unsigned long long)ring.total());
	return ok ? 0 : 1;
}

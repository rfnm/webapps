// elrsd: the daemon - config, the ONE rfnm device, capture/DSP/HTTP/TX threads,
// the HTTP/JSON API on 127.0.0.1:7694 (nginx /api/app/elrs/), the optional WebSocket push,
// the TX arm state machine (arm = stop RX source -> radio_tx::start; disarm/auto-disarm/
// SIGTERM = radio_tx::stop -> restart RX) and the host-only --loopback integration test.
// Entered from the permanent bootstrap src/main.cpp for everything except --selftest.
// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp src/dsp.cpp src/rx_chain.cpp src/source_replay.cpp src/source_buffer.cpp src/tx_chain.cpp src/config.cpp src/tracker.cpp src/packets.cpp src/status.cpp src/replay.cpp src/ws_server.cpp
// 
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "daemon.h"
#include "daemon_internal.h"
#include "rfnm_httpd.h"
#include "rfnm_json.h"

// the compile-time TX ceiling of src/tx_rfnm.cpp (reported in the self-loop JSON; the adapter enforces it)
#ifndef ELRSD_TX_HARD_CAP_DBM
#define ELRSD_TX_HARD_CAP_DBM 0
#endif
#define ELRSD_TX_HARD_CAP_DBM_VALUE (ELRSD_TX_HARD_CAP_DBM)

namespace elrsd {

bool resolve_uid(const daemon_config &cfg, uid_t6 &out);            // replay.cpp
__attribute__((weak)) int run_bench_rx(double seconds);              // B's bench driver (rx_selftest.cpp), optional

static std::atomic<bool> g_stop{ false };
static void on_signal(int) { g_stop = true; }

static rfnm::http_resp err(int code, const std::string &msg) {
	return { code, std::string("{\"err\":\"") + rfnm::jesc(msg) + "\"}" };
}

// ---- bounded capture -> DSP queue (64 blocks; overflow drops the OLDEST block) ----------------------
struct block_item {
	std::vector<int16_t> iq;
	uint64_t sample_index;
	int64_t ts_ns;
};
class block_queue {
public:
	explicit block_queue(size_t cap = 64) : cap_(cap) {}
	void push(const source_block &b) {
		block_item it;
		it.iq.assign(b.iq, b.iq + b.count * 2);
		it.sample_index = b.sample_index;
		it.ts_ns = b.ts_ns;
		std::lock_guard<std::mutex> lk(mtx_);
		if(closed_) return;
		if(q_.size() >= cap_) { q_.pop_front(); drops_++; }
		q_.push_back(std::move(it));
		cv_.notify_one();
	}
	bool pop(block_item &out) {
		std::unique_lock<std::mutex> lk(mtx_);
		cv_.wait_for(lk, std::chrono::milliseconds(100), [&]() { return closed_ || !q_.empty(); });
		if(q_.empty()) return false;
		out = std::move(q_.front());
		q_.pop_front();
		return true;
	}
	void close() { std::lock_guard<std::mutex> lk(mtx_); closed_ = true; cv_.notify_all(); }
	void reopen() { std::lock_guard<std::mutex> lk(mtx_); closed_ = false; q_.clear(); }
	bool closed() const { std::lock_guard<std::mutex> lk(mtx_); return closed_; }
	uint64_t drops() const { std::lock_guard<std::mutex> lk(mtx_); return drops_; }
private:
	mutable std::mutex mtx_;
	std::condition_variable cv_;
	std::deque<block_item> q_;
	size_t cap_;
	bool closed_ = false;
	uint64_t drops_ = 0;
};

// ---- the daemon state --------------------------------------------------------------------------------
struct daemon {
	daemon_config cfg;
	std::mutex mtx;                       // guards everything below that the HTTP thread touches
	std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
	std::string state = "idle";
	std::string last_error;
	std::atomic<int> exit_code{ 0 };
	std::atomic<bool> capture_failed{ false };

	uid_t6 uid{};
	bool uid_set = false;
	const rate_params *rp = &RATE_TABLE[2];
	bool auto_rate = false;

	packet_ring ring{ 500 };
	std::unique_ptr<rx_chain> chain;
	std::unique_ptr<rx_tracker> tracker;
	std::unique_ptr<ws_server> ws;
	std::array<uint16_t, 16> tx_rc{};

	// device + radios
	void *dev = nullptr;
	tx_policy policy = tx_policy::NONE;
	std::string board = "unknown";
	std::unique_ptr<radio_tx> radio;
	std::unique_ptr<elrs_transmitter> tx;
	bool tx_armed_state = false;         // daemon's view (RX stopped, tx thread active)
	std::chrono::steady_clock::time_point arm_deadline;

	// rx session
	std::unique_ptr<sample_source> src;
	std::atomic<bool> src_stop{ false };
	std::thread th_capture, th_dsp;
	block_queue queue;
	bool rx_running = false;
	source_stats src_stats_last;
	// self-loop only: RX on a literal path, on channel rx_channel, WITHOUT parking
	// the TX before the RX apply (the TX is live on purpose; order TX on first, then RX apply Wiki)
	bool rx_park_tx = true;
	bool rx_strict_path = false;
	int rx_channel = 0;
	std::string rx_path_override;

	rx_chain_config chain_cfg() const {
		rx_chain_config rc;
		rc.fs_hz = cfg.fs_hz;
		rc.fc_hz = cfg.fc_hz;
		rc.os = cfg.os;
		rc.rate_index = auto_rate ? 0 : rp->index;
		rc.conj = cfg.conj;
		rc.auto_conj = cfg.conj_auto;
		return rc;
	}

	// (re)build the chain and point the tracker at it (fc / os / rate change)
	void rebuild_chain_locked() {
		chain = std::make_unique<rx_chain>(chain_cfg());
		chain->set_on_burst([this](const rx_burst &b) { tracker->on_burst(b); });
		rx_tracker_hooks::set_chain(*tracker, chain.get());
		int lo, hi;
		channel_window(cfg.fc_hz, cfg.fs_hz, lo, hi);
		rx_tracker_hooks::set_window(*tracker, lo, hi);
	}

	// ---- RX session (mtx held by the caller) ----
	FILE *dump_fp = nullptr; size_t dump_left = 0;   // --selfloop-dump
	std::atomic<uint64_t> rx_blocks{ 0 };            // blocks delivered by the source (self-loop waits for the first)
	bool start_rx_locked() {
		if(rx_running) return true;
		rfnm_rx_config rc;
		rc.fs_hz = cfg.fs_hz;
		rc.fc_hz = cfg.fc_hz;
		rc.gain_db = cfg.gain_db;
		rc.agc = cfg.agc;
		rc.antenna = rx_path_override.empty() ? cfg.antenna : rx_path_override;
		rc.park_tx = rx_park_tx;
		rc.strict_path = rx_strict_path;
		rc.rx_channel = rx_channel;
		src.reset(dev ? make_rfnm_rx_source(rc, dev) : nullptr);
		if(!src) {
			state = "idle";
			if(last_error.empty()) last_error = dev ? "rx source unavailable" : "no radio (built without librfnm or device open failed)";
			return false;
		}
		src_stop = false;
		queue.reopen();
		rx_running = true;
		state = "listening";
		th_capture = std::thread([this]() {
			bool ok = src->run(src_stop, [this](const source_block &b) {
				rx_blocks++;
				if(dump_fp && dump_left > 0) {
					size_t n = std::min(b.count, dump_left); fwrite(b.iq, 4, n, dump_fp); dump_left -= n;
					if(dump_left == 0) { fclose(dump_fp); dump_fp = nullptr; fprintf(stderr, "elrsd: selfloop dump complete\n"); }
				}
				queue.push(b);
			});
			queue.close();
			// never take `mtx` here: stop_rx_locked() joins this thread while holding it
			if(!ok && !src_stop) capture_failed = true;
		});
		th_dsp = std::thread([this]() {
			block_item it;
			for(;;) {
				if(!queue.pop(it)) {
					if(queue.closed()) break;
					continue;
				}
				chain->feed(it.iq.data(), it.iq.size() / 2, it.sample_index, it.ts_ns);
			}
		});
		return true;
	}
	void stop_rx_locked() {
		if(!rx_running) return;
		src_stop = true;
		queue.close();
		if(th_capture.joinable()) th_capture.join();
		if(th_dsp.joinable()) th_dsp.join();
		if(src) src->stats(src_stats_last);
		src.reset();
		rx_running = false;
		if(state == "listening") state = "idle";
	}

	// ---- TX arm / disarm (mtx held) ----
	void disarm_locked(const char *why) {
		if(tx) tx->disarm();
		if(tx_armed_state) {
			fprintf(stderr, "elrsd: tx disarmed (%s), restarting rx\n", why);
			tx_armed_state = false;
			last_error.clear();
			start_rx_locked();
		}
	}

	status_snapshot snapshot_locked() {
		status_snapshot s;
		s.state = tx_armed_state ? "tx_armed" : state;
		s.uptime_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
		s.rate = rate_by_index(rx_tracker_hooks::rate_index_now(*tracker));
		s.uid_set = uid_set;
		s.cfg = cfg;
		if(src) src->stats(s.src); else s.src = src_stats_last;
		if(chain) chain->get_stats(s.rx);
		s.link = tracker->stats();
		fhss_view v = rx_tracker_hooks::view(*tracker);
		s.lock = v.lock;
		s.slot = v.slot;
		s.nonce = v.nonce;
		s.channel = v.channel;
		s.nonce_conf = v.nonce_conf;
		s.fhss_index = v.fhss_index;
		s.next_sync_slot = v.next_sync_slot;
		s.interval_est_us = v.interval_est_us;
		if(tx) s.tx = tx->status();
		s.tx.enabled = cfg.tx_enabled;
		s.tx.policy = policy;
		s.tx.board = board;
		s.tx.power_cap_dbm = cfg.tx_cap_dbm;
		if(!tx && s.tx.last_error.empty() && policy != tx_policy::NONE) s.tx.last_error = "no tx radio";
		s.board = board;
		s.last_error = last_error;
		s.total_packets = ring.total();
		s.queue_drops = queue.drops();
		packet_entry rc;
		if(rx_tracker_hooks::last_rc_entry(*tracker, rc)) { s.last_rc = rc; s.last_rc_valid = true; }
		s.ws_clients = ws ? ws->clients() : 0;
		return s;
	}

	std::string channels_json_locked() {
		link_stats st = tracker->stats();
		fhss_view v = rx_tracker_hooks::view(*tracker);
		std::string j = "{\"per_channel\":[";
		for(int i = 0; i < FHSS_NUM_CHANNELS; i++) j += (i ? "," : "") + jint(st.per_channel[i]);
		j += "],\"last_seen_s\":[";
		for(int i = 0; i < FHSS_NUM_CHANNELS; i++) j += (i ? "," : "") + jnum(st.last_seen_s[i], 3);
		j += "],\"window\":{\"ch_lo\":" + jint(st.window_ch_lo) + ",\"ch_hi\":" + jint(st.window_ch_hi) + "}";
		bool known = v.lock.state != lock_state::NONE;
		j += ",\"sequence_known\":" + jbool(known);
		j += ",\"current\":" + jint(v.channel);
		j += ",\"next\":[";
		std::vector<int> nx = rx_tracker_hooks::next_channels(*tracker, 4);
		for(size_t i = 0; i < nx.size(); i++) j += (i ? "," : "") + jint(nx[i]);
		j += "],\"slot\":" + jint((long long)v.slot) + ",\"sync_channel\":" + jint(FHSS_SYNC_CHANNEL) + "}";
		return j;
	}
};

// ---- --loopback: TX synthesis -> buffer source + rx_chain (B) -> tracker (C) -> decode (A) ------------
static int run_loopback(daemon &d) {
	const rate_params &rp = *d.rp;
	uid_t6 uid = d.uid_set ? d.uid : test_uid();
	if(!d.uid_set) fprintf(stderr, "elrsd: loopback: no runtime UID configured, using the TEST-UID of the vectors\n");
	const double fs = d.cfg.fs_hz, fc = d.cfg.fc_hz;
	const int n_slots = 12 * rp.hop_interval;   // 12 dwells -> Lock B needs >= 8 in-window dwells
	const uint64_t slot_samples = (uint64_t)llround(rp.interval_us * 1e-6 * fs);
	const uint64_t lead = slot_samples / 4;
	std::array<uint8_t, FHSS_SEQ_LEN> seq = fhss_sequence(uid);
	li_codec codec(rp);
	burst_synth_config sc;
	sc.fs_hz = fs;
	burst_synth synth(rp, sc);
	std::vector<int16_t> stream((size_t)((n_slots + 1) * slot_samples) * 2, 0);
	std::vector<int16_t> burst;
	// link timeline: nonce0 = 0, ptr0 = 0 (seq[0] = 41, the SYNC channel: Lock A gets exercised
	// when 41 is inside the window)
	int ptr = 0;
	int visible = 0, syncs = 0;
	std::vector<std::array<uint8_t, 8>> truth(n_slots);
	std::vector<bool> aired(n_slots, false);
	for(int s = 0; s < n_slots; s++) {
		uint8_t nonce = (uint8_t)s;
		int ch = seq[ptr];
		double f_ch = fhss_channel_freq_hz(ch);
		std::array<uint16_t, 16> chv;
		for(int i = 0; i < 16; i++) chv[i] = (uint16_t)(172 + (lowbias32((uint32_t)(s * 16 + i)) % 1640));
		std::array<uint8_t, 8> pkt;
		bool is_sync = (ch == FHSS_SYNC_CHANNEL) && (nonce % rp.hop_interval == 0);
		if(is_sync) pkt = ota4_build_sync(uid, (uint8_t)ptr, nonce, rp.index, 2 /* tlm 64 */);
		else pkt = ota4_build_rc(uid, chv, nonce, rp.hop_interval, rp.tlm_denom_default, 2);
		truth[s] = pkt;
		double off = f_ch - fc;
		if(std::fabs(off) <= fs / 2 - 1.0e6) {
			std::vector<uint16_t> sym = codec.encode(pkt.data(), 8, 0);
			size_t n = synth.synth(sym, off, burst);
			size_t pos = (size_t)(s * slot_samples + lead) * 2;
			for(size_t i = 0; i < n * 2 && pos + i < stream.size(); i++) {
				int v = stream[pos + i] + burst[i];
				stream[pos + i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
			}
			aired[s] = true;
			visible++;
			if(is_sync) syncs++;
		}
		if((nonce + 1) % rp.hop_interval == 0) ptr = (ptr + 1) % FHSS_SEQ_LEN;
	}
	packet_ring ring(500);
	rx_chain_config rc;
	rc.fs_hz = fs;
	rc.fc_hz = fc;
	rc.os = d.cfg.os;
	rc.rate_index = rp.index;
	rc.conj = false;
	rx_chain chain(rc);
	rx_tracker tracker(ring, &chain);
	tracker.set_uid(uid);
	tracker.set_rate(rp);
	int lo, hi;
	channel_window(fc, fs, lo, hi);
	rx_tracker_hooks::set_window(tracker, lo, hi);
	uint64_t tchk = 0, tmatch = 0;
	tracker.set_truth_provider([&](const rx_burst &b, std::array<uint8_t, 8> &out) {
		long long k = llround(((double)b.sample_start - (double)lead) / (double)slot_samples);
		if(k < 0 || k >= n_slots) return false;
		out = truth[(size_t)k];
		return true;
	});
	rx_tracker_hooks::set_packet_sink(tracker, [&](const packet_entry &e) {
		if(e.truth_known) { tchk++; if(e.truth_match) tmatch++; }
	});
	chain.set_on_burst([&](const rx_burst &b) { tracker.on_burst(b); });
	auto t0 = std::chrono::steady_clock::now();
	std::unique_ptr<sample_source> src(make_buffer_source(stream.data(), stream.size() / 2, fs, fc));
	std::atomic<bool> stop{ false };
	src->run(stop, [&](const source_block &b) { chain.feed(b.iq, b.count, b.sample_index, b.ts_ns); });
	double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	rx_chain::stats cs;
	chain.get_stats(cs);
	link_stats st = tracker.stats();
	link_lock lk = tracker.lock();
	bool ok = st.crc_ok >= (uint64_t)(visible * 9 / 10) && tmatch == tchk && lk.state != lock_state::NONE;
	printf("{\"loopback\":true,\"rate\":%d,\"sf\":%d,\"fc_hz\":%s,\"slots\":%d,\"visible\":%d,\"syncs_aired\":%d,"
	       "\"bursts\":%llu,\"found\":%llu,\"decoded\":%llu,\"crc_ok\":%llu,\"rc\":%llu,\"sync\":%llu,"
	       "\"truth_checked\":%llu,\"truth_match\":%llu,\"lock\":\"%s\",\"lock_score\":%d,\"lq_pct\":%s,"
	       "\"signal_s\":%s,\"wall_s\":%s,\"ok\":%s}\n",
	       rp.index, rp.sf, jnum(fc, 0).c_str(), n_slots, visible, syncs,
	       (unsigned long long)cs.bursts, (unsigned long long)cs.demod_found, (unsigned long long)st.decoded,
	       (unsigned long long)st.crc_ok, (unsigned long long)st.rc, (unsigned long long)st.sync,
	       (unsigned long long)tchk, (unsigned long long)tmatch, lock_state_name(lk.state), lk.score,
	       jnum(st.lq_pct, 1).c_str(), jnum((double)(n_slots + 1) * slot_samples / fs, 3).c_str(), jnum(wall, 3).c_str(),
	       ok ? "true" : "false");
	printf("loopback %s\n", ok ? "OK" : "FAILED");
	return ok ? 0 : 1;
}

// ---- --selfloop-device: TX ch0 (normal elrs_transmitter through the librfnm adapter, every gate) -> the
//      Lime card's chip-internal loopback tap (RX path "loopback") -> normal RX source + rx_chain + tracker
//      on the SAME device at the same time (the one exception to "RX stops while armed").
//      Nothing is meant to leave the board: the TX runs at power_range.min (never above the caps) and
//      the chip tap is non-diverting, so SMA_A must carry NO antenna (open or 50 Ohm load).
static int run_selfloop_device(daemon &d) {
	const double seconds = d.cfg.selfloop_s;
	auto refuse = [&](const char *why, int code) {
		fprintf(stderr, "elrsd: selfloop-device: refused: %s\n", why);
		printf("{\"selfloop_device\":true,\"ok\":false,\"err\":%s}\n", jstr(why).c_str());
		return code;
	};
	if(!(seconds > 0)) return refuse("invalid --selfloop-device SECONDS (1..3600)", 2);
	if(d.auto_rate) return refuse("rate auto is not a transmit rate (use --rate 250hz etc.)", 2);
	const rate_params &rp = *d.rp;
	uid_t6 uid = d.uid_set ? d.uid : test_uid();
	if(!d.uid_set) fprintf(stderr, "elrsd: selfloop-device: no runtime UID configured, using the TEST-UID of the vectors (fine for a loopback)\n");
	// gates 1 + 2 before any device access (gate 3 = this run's arm request, gate 4 = the adapter)
	std::string board_s;
	d.policy = tx_policy_from_boardconf(d.cfg.boardconf.c_str(), &board_s);
	d.board = board_s.empty() ? "unknown" : board_s;
	if(d.policy == tx_policy::NONE) return refuse("tx not allowed on this board (board.conf TX_ALLOWED)", 2);
	if(!d.cfg.tx_enabled) return refuse("tx disabled (start with --tx 1)", 2);
	std::string derr;
	d.dev = rfnm_device_open(d.cfg.fs_hz, derr);
	if(!d.dev) return refuse(("no device: " + (derr.empty() ? std::string("open failed") : derr)).c_str(), 2);
	d.radio.reset(make_rfnm_tx(d.dev, d.policy));
	if(!d.radio) { rfnm_device_close(d.dev); d.dev = nullptr; return refuse("no tx radio", 2); }
	tx_plan_config plan;
	plan.fs_hz = d.cfg.fs_hz;
	plan.window_hz = d.cfg.fs_hz;
	d.tx = std::make_unique<elrs_transmitter>(*d.radio, plan, d.cfg.tx_cap_dbm, d.cfg.tx_enabled);
	d.tx->set_uid(uid);
	const std::array<uint16_t, 16> pattern = selfloop_rc_pattern();
	d.tx->set_rc_channels(pattern);
	d.tracker->set_uid(uid);
	d.tracker->set_rate(rp);
	rx_tracker_hooks::set_wallclock(*d.tracker, true);
	d.rebuild_chain_locked();
	selfloop_checker checker(uid, rp, pattern, rp.tlm_denom_default);
	std::mutex cmtx;
	rx_tracker_hooks::set_packet_sink(*d.tracker, [&](const packet_entry &e) {
		uint8_t b[8];
		bool have = selfloop_checker::hex_to_bytes(e.hex, b);
		std::lock_guard<std::mutex> lk(cmtx);
		checker.on_packet(e.kind, e.crc_ok, have ? b : nullptr);
	});
	// ---- 1. TX RF first (gate 3: this request; the adapter re-checks policy/path/caps and clamps to
	//         power_range.min) - RF_ON + apply only, the feed session is started in step 3 ----
	tx_arm_request r;
	r.power_dbm = d.cfg.selfloop_tx_dbm; // default -100 = "as low as the board goes": clamped UP to power_range.min, never above a cap
	r.antenna = "sma_a";                // the Lime TX has exactly one path; policy embed is refused by the adapter
	r.duration_s = std::min(seconds, d.cfg.tx_max_duration_s);
	r.rate_index = rp.index;
	r.sync_packets = true;
	r.tlm_denom = rp.tlm_denom_default;
	r.cw_only = d.cfg.selfloop_cw;        // --selfloop-cw: the idle tone is the only content, no bursts
	r.cw_offset_hz = d.cfg.selfloop_cw_offset_hz;
	r.cw_amplitude = 14000;
	std::string reason;
	if(!d.tx->arm(r, reason, /*start_session=*/false)) {
		d.tx.reset(); d.radio.reset();
		rfnm_device_close(d.dev); d.dev = nullptr;
		return refuse(("tx arm refused: " + reason).c_str(), 2);
	}
	tx_status ts0 = d.tx->status();
	fprintf(stderr, "elrsd: selfloop-device: TX RF ARMED path %s power %d dBm (range %d..%d, cap %d, hard cap %d) lo %.1f MHz for %.0f s%s\n",
	        ts0.antenna.c_str(), ts0.power_dbm_applied, ts0.power_min_dbm, ts0.power_max_dbm, d.cfg.tx_cap_dbm, ELRSD_TX_HARD_CAP_DBM_VALUE,
	        ts0.freq_lo_hz / 1e6, r.duration_s, d.cfg.selfloop_cw ? " [CW: no bursts, tone at LO+offset]" : "");
	// ---- 2. THEN the RX apply on the loopback path, TX left alone (no park), same device ----
	int rc_exit = 0;
	{
		std::lock_guard<std::mutex> lk(d.mtx);
		d.rx_park_tx = false;
		d.rx_strict_path = true;
		d.rx_channel = d.cfg.selfloop_rx_channel;
		d.rx_path_override = d.cfg.selfloop_rx_path;
		d.tx_armed_state = true;
		if(!d.cfg.selfloop_dump.empty()) {
			d.dump_fp = fopen(d.cfg.selfloop_dump.c_str(), "wb");
			d.dump_left = d.dump_fp ? (size_t)(d.cfg.selfloop_dump_s * d.cfg.fs_hz) : 0;
			fprintf(stderr, "elrsd: selfloop dump -> %s (%zu samples)\n", d.cfg.selfloop_dump.c_str(), d.dump_left);
		}
		if(!d.start_rx_locked()) {
			d.tx->disarm();
			d.tx.reset(); d.radio.reset();
			rfnm_device_close(d.dev); d.dev = nullptr;
			return refuse(("rx start failed: " + d.last_error).c_str(), 2);
		}
	}
	// ---- 3. the RX applies run on the capture thread: wait until samples flow, THEN anchor the TX feed
	//         session (a session latched before the RX apply feeds a grid the device re-minted; the
	//         2026-09-14 run: host "sent" 2770 bursts, the RX heard only the LO leak) ----
	{
		auto until = std::chrono::steady_clock::now() + std::chrono::seconds(8);
		while(d.rx_blocks.load() == 0 && !d.capture_failed && !g_stop && std::chrono::steady_clock::now() < until)
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		auto fail = [&](const std::string &why) {
			d.tx->disarm();
			{ std::lock_guard<std::mutex> lk(d.mtx); d.stop_rx_locked(); d.tx_armed_state = false; }
			d.tx.reset(); d.radio.reset();
			rfnm_device_close(d.dev); d.dev = nullptr;
			return refuse(why.c_str(), 2);
		};
		if(d.rx_blocks.load() == 0) return fail(d.capture_failed ? "rx source failed before the first block (see log)" : "rx delivered no samples within 8 s");
		std::string why;
		if(!d.tx->session_start(why)) return fail("tx feed session refused: " + why);
		tx_status ts1 = d.tx->status();
		fprintf(stderr, "elrsd: selfloop-device: TX FEED SESSION anchored after the RX apply (%s)%s\n", d.radio->describe().c_str(),
		        d.cfg.selfloop_cw ? " - CW tone running" : " - bursts from the next slot");
		(void)ts1;
	}
	auto t0 = std::chrono::steady_clock::now();
	auto deadline = t0 + std::chrono::milliseconds((long)(seconds * 1000) + 1500);   // RX tail after the TX auto-disarm
	auto last_log = t0;
	std::string rx_err;
	while(!g_stop) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		auto now = std::chrono::steady_clock::now();
		if(d.capture_failed) { rx_err = "rx source failed (see log)"; break; }
		bool armed = d.tx->armed();
		if(!armed && now - t0 > std::chrono::seconds(1) && d.tx->status().last_error.rfind("auto-disarm", 0) != 0) { rx_err = "tx stopped: " + d.tx->status().last_error; break; }
		if(now >= deadline) break;
		if(now - last_log >= std::chrono::seconds(5)) {
			last_log = now;
			tx_status ts = d.tx->status();
			selfloop_counts k;
			{ std::lock_guard<std::mutex> lk(cmtx); k = checker.counts(); }
			link_stats st = d.tracker->stats();
			fprintf(stderr, "elrsd: selfloop-device: %.0f s | tx sent %llu dropped %llu retunes %llu | feed fed %llu late %llu pkts %llu drops %llu lead %lld us | dev placed %llu late %llu misal %llu underruns %llu rebases %llu | rx bursts %llu crc_ok %llu bitexact %llu mismatch %llu | lock %s\n",
			        std::chrono::duration<double>(now - t0).count(), (unsigned long long)ts.bursts_sent, (unsigned long long)ts.bursts_dropped,
			        (unsigned long long)ts.retunes, (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.bursts_late,
			        (unsigned long long)ts.health.packets, (unsigned long long)ts.health.drops, (long long)ts.health.frontier_lead_us,
			        (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.health.pos_late, (unsigned long long)ts.health.pos_misaligned,
			        (unsigned long long)ts.health.underruns, (unsigned long long)ts.health.rebases,
			        (unsigned long long)st.bursts, (unsigned long long)k.crc_ok,
			        (unsigned long long)(k.rc_bitexact + k.sync_bitexact), (unsigned long long)k.mismatch, lock_state_name(d.tracker->lock().state));
		}
	}
	// ---- 4. stop: TX first (RF_OFF), then the RX session (status snapshot BEFORE disarm: the adapter's
	//         health/feed counters are only read while armed) ----
	tx_status ts = d.tx->status();
	d.tx->disarm();
	{
		std::lock_guard<std::mutex> lk(d.mtx);
		d.stop_rx_locked();
		d.tx_armed_state = false;
	}
	double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	selfloop_counts k;
	{ std::lock_guard<std::mutex> lk(cmtx); k = checker.counts(); }
	rx_chain::stats cs;
	d.chain->get_stats(cs);
	link_stats st = d.tracker->stats();
	link_lock lk = d.tracker->lock();
	// how many of the transmitted slots fell into the RX window (the TX hops over all 80 channels,
	// the RX window sees ~25 of them): tx timeline nonce0 = 0 / ptr0 = 0, slot s -> ptr (s / hop) % 240
	int lo, hi;
	channel_window(d.cfg.fc_hz, d.cfg.fs_hz, lo, hi);
	std::array<uint8_t, FHSS_SEQ_LEN> seq = fhss_sequence(uid);
	uint64_t in_window = 0;
	for(int64_t s = 0; s <= ts.slot; s++) {
		int ch = seq[(size_t)((s / rp.hop_interval) % FHSS_SEQ_LEN)];
		if(ch >= lo && ch <= hi) in_window++;
	}
	bool ok = rx_err.empty() && (d.cfg.selfloop_cw ? ts.health.packets > 0 && ts.health.drops == 0 : k.ok());
	if(!rx_err.empty()) { fprintf(stderr, "elrsd: selfloop-device: %s\n", rx_err.c_str()); rc_exit = 1; }
	printf("{\"selfloop_device\":true,\"cw\":%s,\"cw_offset_hz\":%s,\"board\":%s,\"policy\":%s,\"rate\":%d,\"sf\":%d,\"fs_hz\":%s,\"fc_hz\":%s,\"seconds\":%s,\"wall_s\":%s,"
	       "\"rx\":{\"path\":%s,\"channel\":%d,\"gain_db\":%d,\"window\":{\"ch_lo\":%d,\"ch_hi\":%d},\"samples\":%llu,\"drops\":%llu,\"queue_drops\":%llu,"
	       "\"bursts\":%llu,\"found\":%llu,\"decoded\":%llu,\"crc_ok\":%llu,\"crc_fail\":%llu,\"rc\":%llu,\"sync\":%llu,\"floor\":%s,\"cpu_busy\":%s},"
	       "\"tx\":{\"path\":%s,\"power_dbm_applied\":%d,\"power_range\":[%d,%d],\"power_cap_dbm\":%d,\"hard_cap_dbm\":%d,\"freq_lo_hz\":%s,"
	       "\"sent\":%llu,\"syncs\":%llu,\"dropped\":%llu,\"retunes\":%llu,\"retune_ms\":{\"p50\":%s,\"p99\":%s,\"max\":%s},\"slots\":%lld,\"slots_in_rx_window\":%llu,"
	       "\"health\":{\"writes\":%llu,\"packets\":%llu,\"drops\":%llu,\"stalls\":%llu,\"underruns\":%llu,\"pos_placed\":%llu,\"pos_late\":%llu,\"pos_misaligned\":%llu,"
	       "\"bursts_queued\":%llu,\"bursts_fed\":%llu,\"bursts_late\":%llu,\"zero_samples\":%llu,\"burst_samples\":%llu,\"frontier_lead_us\":%lld,\"anchor_moves\":%llu,\"rebases\":%llu,\"refusals\":%llu},\"last_error\":%s},"
	       "\"verify\":%s,\"lock\":%s,\"lock_score\":%d,\"lq_pct\":%s,\"err\":%s,\"ok\":%s}\n",
	       d.cfg.selfloop_cw ? "true" : "false", jnum(d.cfg.selfloop_cw ? d.cfg.selfloop_cw_offset_hz : 0.0, 0).c_str(),
	       jstr(d.board).c_str(), jstr(policy_name(d.policy)).c_str(), rp.index, rp.sf, jnum(d.cfg.fs_hz, 0).c_str(), jnum(d.cfg.fc_hz, 0).c_str(),
	       jnum(seconds, 1).c_str(), jnum(wall, 2).c_str(),
	       jstr(d.cfg.selfloop_rx_path).c_str(), d.cfg.selfloop_rx_channel, d.cfg.gain_db, lo, hi,
	       (unsigned long long)d.src_stats_last.samples, (unsigned long long)d.src_stats_last.drops, (unsigned long long)d.queue.drops(),
	       (unsigned long long)cs.bursts, (unsigned long long)cs.demod_found, (unsigned long long)st.decoded, (unsigned long long)st.crc_ok,
	       (unsigned long long)st.crc_fail, (unsigned long long)st.rc, (unsigned long long)st.sync, jnum(cs.floor_level, 1).c_str(), jnum(cs.cpu_busy, 3).c_str(),
	       jstr(ts.antenna).c_str(), ts.power_dbm_applied, ts.power_min_dbm, ts.power_max_dbm, d.cfg.tx_cap_dbm, ELRSD_TX_HARD_CAP_DBM_VALUE, jnum(ts.freq_lo_hz, 0).c_str(),
	       (unsigned long long)ts.bursts_sent, (unsigned long long)ts.syncs_sent, (unsigned long long)ts.bursts_dropped, (unsigned long long)ts.retunes,
	       jnum(ts.retune_ms_p50, 2).c_str(), jnum(ts.retune_ms_p99, 2).c_str(), jnum(ts.retune_ms_max, 2).c_str(), (long long)ts.slot, (unsigned long long)in_window,
	       (unsigned long long)ts.health.writes, (unsigned long long)ts.health.packets, (unsigned long long)ts.health.drops, (unsigned long long)ts.health.stalls,
	       (unsigned long long)ts.health.underruns, (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.health.pos_late, (unsigned long long)ts.health.pos_misaligned,
	       (unsigned long long)ts.health.bursts_queued, (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.bursts_late,
	       (unsigned long long)ts.health.zero_samples, (unsigned long long)ts.health.burst_samples, (long long)ts.health.frontier_lead_us,
	       (unsigned long long)ts.health.anchor_moves, (unsigned long long)ts.health.rebases,
	       (unsigned long long)ts.health.refusals, jstr(ts.last_error).c_str(),
	       selfloop_counts_json(k).c_str(), jstr(lock_state_name(lk.state)).c_str(), lk.score, jnum(st.lq_pct, 1).c_str(), jstr(rx_err).c_str(), ok ? "true" : "false");
	printf("selfloop-device %s (%llu crc-ok, %llu bit-exact, %llu mismatch, tx sent %llu, feed fed %llu pkts %llu drops %llu, dev placed %llu late %llu)\n", ok ? "OK" : "FAILED",
	       (unsigned long long)k.crc_ok, (unsigned long long)(k.rc_bitexact + k.sync_bitexact), (unsigned long long)k.mismatch, (unsigned long long)ts.bursts_sent,
	       (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.packets, (unsigned long long)ts.health.drops,
	       (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.health.pos_late);
	d.tx.reset();
	d.radio.reset();
	rfnm_device_close(d.dev);
	d.dev = nullptr;
	return ok ? 0 : (rc_exit ? rc_exit : 1);
}

// ---- --tx-only SECONDS: PURE TRANSMIT, no RX path - the SEND half of the two-board
//      proof. Unlike --selfloop-device this opens NO RX source, calls no start_rx_locked and issues
//      no RX apply, so the librfnm TX anchor (latched once by rfnm::tx_stream) is never moved: the
//      feed session is anchored immediately in arm(start_session=true) and the stamped bursts reach
//      the DAC (pos_placed > 0). The 2026-09-14 two-board attempt failed because the daemon ran RX
//      while transmitting; the concurrent RX apply shifted the anchor -> every burst "late", pos_placed
//      0. A second board receives on the planned channels reported below.
static int run_tx_only(daemon &d) {
	const double seconds = d.cfg.tx_only_s;
	auto refuse = [&](const char *why, int code) {
		fprintf(stderr, "elrsd: tx-only: refused: %s\n", why);
		printf("{\"tx_only\":true,\"ok\":false,\"pos_placed\":0,\"err\":%s}\n", jstr(why).c_str());
		return code;
	};
	if(!(seconds > 0)) return refuse("invalid --tx-only SECONDS (1..3600)", 2);
	if(d.auto_rate) return refuse("rate auto is not a transmit rate (use --rate 250hz etc.)", 2);
	const rate_params &rp = *d.rp;
	uid_t6 uid = d.uid_set ? d.uid : test_uid();
	if(!d.uid_set) fprintf(stderr, "elrsd: tx-only: WARNING no runtime UID configured, using the TEST-UID of the vectors "
	                                "(a real two-board test needs the real binding via ELRS_BINDING_PHRASE / PUT /config)\n");
	// gates 1 + 2 before any device access (gate 3 = this run's arm request, gate 4 = the adapter)
	std::string board_s;
	d.policy = tx_policy_from_boardconf(d.cfg.boardconf.c_str(), &board_s);
	d.board = board_s.empty() ? "unknown" : board_s;
	if(d.policy == tx_policy::NONE) return refuse("tx not allowed on this board (board.conf TX_ALLOWED)", 2);
	if(!d.cfg.tx_enabled) return refuse("tx disabled (start with --tx 1)", 2);
	std::string derr;
	d.dev = rfnm_device_open(d.cfg.fs_hz, derr);
	if(!d.dev) return refuse(("no device: " + (derr.empty() ? std::string("open failed") : derr)).c_str(), 2);
	d.radio.reset(make_rfnm_tx(d.dev, d.policy));
	if(!d.radio) { rfnm_device_close(d.dev); d.dev = nullptr; return refuse("no tx radio", 2); }
	tx_plan_config plan;
	plan.fs_hz = d.cfg.fs_hz;
	plan.window_hz = d.cfg.fs_hz;
	d.tx = std::make_unique<elrs_transmitter>(*d.radio, plan, d.cfg.tx_cap_dbm, d.cfg.tx_enabled);
	d.tx->set_uid(uid);
	const std::array<uint16_t, 16> pattern = selfloop_rc_pattern();
	d.tx->set_rc_channels(pattern);
	// ---- arm: RF bring-up AND the feed session IMMEDIATELY (start_session=true). No RX apply ever
	//      runs on this device, so the anchor latched here stays valid for the whole run. ----
	tx_arm_request r;
	r.power_dbm = d.cfg.tx_only_dbm;    // default -100 = "as low as the board goes": clamped UP to power_range.min, never above a cap
	r.antenna = "sma_a";               // the Lime TX has exactly one path
	r.duration_s = std::min(seconds, d.cfg.tx_max_duration_s);
	r.rate_index = rp.index;
	r.sync_packets = true;
	r.tlm_denom = rp.tlm_denom_default;
	std::string reason;
	if(!d.tx->arm(r, reason, /*start_session=*/true)) {
		d.tx.reset(); d.radio.reset();
		rfnm_device_close(d.dev); d.dev = nullptr;
		return refuse(("tx arm refused: " + reason).c_str(), 2);
	}
	tx_status ts0 = d.tx->status();
	fprintf(stderr, "elrsd: tx-only: TX RF ARMED + FEED SESSION path %s power %d dBm (range %d..%d, cap %d, hard cap %d) lo %.1f MHz for %.0f s - NO RX PATH (anchor stable)\n",
	        ts0.antenna.c_str(), ts0.power_dbm_applied, ts0.power_min_dbm, ts0.power_max_dbm, d.cfg.tx_cap_dbm, ELRSD_TX_HARD_CAP_DBM_VALUE,
	        ts0.freq_lo_hz / 1e6, r.duration_s);
	// ---- run for SECONDS (the transmitter auto-disarms itself at the duration); SIGINT stops early ----
	auto t0 = std::chrono::steady_clock::now();
	auto deadline = t0 + std::chrono::milliseconds((long)(seconds * 1000) + 500);
	auto last_log = t0;
	std::string tx_err;
	while(!g_stop) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		auto now = std::chrono::steady_clock::now();
		bool armed = d.tx->armed();
		if(!armed && now - t0 > std::chrono::seconds(1) && d.tx->status().last_error.rfind("auto-disarm", 0) != 0) { tx_err = "tx stopped: " + d.tx->status().last_error; break; }
		if(now >= deadline) break;
		if(now - last_log >= std::chrono::seconds(5)) {
			last_log = now;
			tx_status ts = d.tx->status();
			fprintf(stderr, "elrsd: tx-only: %.0f s | tx sent %llu dropped %llu syncs %llu retunes %llu | feed fed %llu late %llu pkts %llu drops %llu lead %lld us | dev PLACED %llu late %llu misal %llu underruns %llu rebases %llu anchor_moves %llu\n",
			        std::chrono::duration<double>(now - t0).count(), (unsigned long long)ts.bursts_sent, (unsigned long long)ts.bursts_dropped,
			        (unsigned long long)ts.syncs_sent, (unsigned long long)ts.retunes, (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.bursts_late,
			        (unsigned long long)ts.health.packets, (unsigned long long)ts.health.drops, (long long)ts.health.frontier_lead_us,
			        (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.health.pos_late, (unsigned long long)ts.health.pos_misaligned,
			        (unsigned long long)ts.health.underruns, (unsigned long long)ts.health.rebases, (unsigned long long)ts.health.anchor_moves);
		}
	}
	// ---- stop: status snapshot BEFORE disarm (the adapter's health/feed counters are only read while
	//      armed), then TX RF_OFF. No RX session to stop. ----
	tx_status ts = d.tx->status();
	d.tx->disarm();
	double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	// planned send channels: for the receiving board. Slot s -> ptr (s / hop) % 240 -> channel; the
	// transmitter starts nonce0 = 0 / ptr0 = 0 at slot 0 (same link_model). freq_hz = nominal channel
	// centre (what a receiver tunes to); freq_air_hz = the SX1280-exact centre actually transmitted.
	int lo, hi;
	channel_window(d.cfg.fc_hz, d.cfg.fs_hz, lo, hi);
	std::array<uint8_t, FHSS_SEQ_LEN> seq = fhss_sequence(uid);
	const int n_plan = 20;
	std::string planned = "[";
	for(int s = 0; s < n_plan; s++) {
		int ptr = (int)((s / rp.hop_interval) % FHSS_SEQ_LEN);
		int ch = seq[(size_t)ptr];
		bool in_win = ch >= lo && ch <= hi;
		planned += (s ? "," : "");
		planned += "{\"slot\":" + jint(s) + ",\"fhss_ptr\":" + jint(ptr) + ",\"channel\":" + jint(ch) +
		           ",\"freq_hz\":" + jnum(fhss_channel_freq_hz(ch), 0) + ",\"freq_air_hz\":" + jnum(fhss_channel_freq_hz_sx1280(ch), 0) +
		           ",\"sync\":" + jbool(ch == FHSS_SYNC_CHANNEL) + ",\"in_rx_window\":" + jbool(in_win) + "}";
	}
	planned += "]";
	// how many of the transmitted slots would fall into a 25 MHz RX window centred at fc (parity with
	// --selfloop-device; a second board with a wider/steered window can see more)
	uint64_t in_window = 0;
	for(int64_t s = 0; s <= ts.slot; s++) {
		int ch = seq[(size_t)((s / rp.hop_interval) % FHSS_SEQ_LEN)];
		if(ch >= lo && ch <= hi) in_window++;
	}
	// SUCCESS CRITERION: samples reached the DAC = the device placed stamped bursts (pos_placed > 0)
	// and the transmitter did not stop with an error.
	bool ok = tx_err.empty() && ts.health.pos_placed > 0;
	int rc_exit = ok ? 0 : 1;
	if(!tx_err.empty()) fprintf(stderr, "elrsd: tx-only: %s\n", tx_err.c_str());
	// pos_placed is FIRST after ok: it is the success criterion (samples at the DAC).
	printf("{\"tx_only\":true,\"ok\":%s,\"pos_placed\":%llu,\"board\":%s,\"policy\":%s,\"rate\":%d,\"sf\":%d,\"fs_hz\":%s,\"fc_hz\":%s,\"seconds\":%s,\"wall_s\":%s,\"uid\":%s,"
	       "\"tx\":{\"path\":%s,\"power_dbm_applied\":%d,\"power_range\":[%d,%d],\"power_cap_dbm\":%d,\"hard_cap_dbm\":%d,\"freq_lo_hz\":%s,"
	       "\"sent\":%llu,\"syncs\":%llu,\"dropped\":%llu,\"retunes\":%llu,\"retune_ms\":{\"p50\":%s,\"p99\":%s,\"max\":%s},\"slots\":%lld,\"slots_in_rx_window\":%llu,"
	       "\"health\":{\"writes\":%llu,\"packets\":%llu,\"drops\":%llu,\"stalls\":%llu,\"underruns\":%llu,\"pos_placed\":%llu,\"pos_late\":%llu,\"pos_misaligned\":%llu,"
	       "\"bursts_queued\":%llu,\"bursts_fed\":%llu,\"bursts_late\":%llu,\"zero_samples\":%llu,\"burst_samples\":%llu,\"frontier_lead_us\":%lld,\"anchor_moves\":%llu,\"rebases\":%llu,\"refusals\":%llu},\"last_error\":%s},"
	       "\"freq_lo_hz\":%s,\"rx_window\":{\"ch_lo\":%d,\"ch_hi\":%d},\"planned_slots\":%s,\"err\":%s}\n",
	       ok ? "true" : "false", (unsigned long long)ts.health.pos_placed,
	       jstr(d.board).c_str(), jstr(policy_name(d.policy)).c_str(), rp.index, rp.sf, jnum(d.cfg.fs_hz, 0).c_str(), jnum(d.cfg.fc_hz, 0).c_str(),
	       jnum(seconds, 1).c_str(), jnum(wall, 2).c_str(), jstr(d.uid_set ? "runtime" : "test-uid").c_str(),
	       jstr(ts.antenna).c_str(), ts.power_dbm_applied, ts.power_min_dbm, ts.power_max_dbm, d.cfg.tx_cap_dbm, ELRSD_TX_HARD_CAP_DBM_VALUE, jnum(ts.freq_lo_hz, 0).c_str(),
	       (unsigned long long)ts.bursts_sent, (unsigned long long)ts.syncs_sent, (unsigned long long)ts.bursts_dropped, (unsigned long long)ts.retunes,
	       jnum(ts.retune_ms_p50, 2).c_str(), jnum(ts.retune_ms_p99, 2).c_str(), jnum(ts.retune_ms_max, 2).c_str(), (long long)ts.slot, (unsigned long long)in_window,
	       (unsigned long long)ts.health.writes, (unsigned long long)ts.health.packets, (unsigned long long)ts.health.drops, (unsigned long long)ts.health.stalls,
	       (unsigned long long)ts.health.underruns, (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.health.pos_late, (unsigned long long)ts.health.pos_misaligned,
	       (unsigned long long)ts.health.bursts_queued, (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.bursts_late,
	       (unsigned long long)ts.health.zero_samples, (unsigned long long)ts.health.burst_samples, (long long)ts.health.frontier_lead_us,
	       (unsigned long long)ts.health.anchor_moves, (unsigned long long)ts.health.rebases, (unsigned long long)ts.health.refusals, jstr(ts.last_error).c_str(),
	       jnum(ts.freq_lo_hz, 0).c_str(), lo, hi, planned.c_str(), jstr(tx_err).c_str());
	printf("tx-only %s (pos_placed %llu <- SAMPLES AT THE DAC, tx sent %llu, feed fed %llu late %llu, pos_late %llu, dropped %llu, syncs %llu, retunes %llu)\n",
	       ok ? "OK" : "FAILED", (unsigned long long)ts.health.pos_placed, (unsigned long long)ts.bursts_sent,
	       (unsigned long long)ts.health.bursts_fed, (unsigned long long)ts.health.bursts_late, (unsigned long long)ts.health.pos_late,
	       (unsigned long long)ts.bursts_dropped, (unsigned long long)ts.syncs_sent, (unsigned long long)ts.retunes);
	d.tx.reset();
	d.radio.reset();
	rfnm_device_close(d.dev);
	d.dev = nullptr;
	return ok ? 0 : rc_exit;
}

// ---- HTTP handler --------------------------------------------------------------------------------------------
static rfnm::http_resp handle(daemon &d, const rfnm::http_req &req) {
	auto q_param = [&](const char *key) -> std::string {
		std::string k = std::string(key) + "=";
		size_t p = 0;
		while(p < req.query.size()) {
			size_t e = req.query.find('&', p);
			if(e == std::string::npos) e = req.query.size();
			std::string kv = req.query.substr(p, e - p);
			if(kv.rfind(k, 0) == 0) return kv.substr(k.size());
			p = e + 1;
		}
		return "";
	};
	if(req.method == "GET" && (req.path == "/status" || req.path == "/" || req.path.empty())) {
		std::lock_guard<std::mutex> lk(d.mtx);
		return { 200, status_to_json(d.snapshot_locked()) };
	}
	if(req.method == "GET" && req.path == "/packets") {
		uint64_t since = strtoull(q_param("since").c_str(), nullptr, 10);
		size_t max = (size_t)strtoul(q_param("max").c_str(), nullptr, 10);
		if(!max) max = 500;
		return { 200, d.ring.to_json(since, max) };
	}
	if(req.method == "GET" && req.path == "/rc") {
		std::lock_guard<std::mutex> lk(d.mtx);
		packet_entry rc;
		const packet_entry *p = rx_tracker_hooks::last_rc_entry(*d.tracker, rc);
		link_stats st = d.tracker->stats();
		std::string j = "{\"rc\":" + rc_json(p) + ",\"link\":{\"lq_pct\":" + jnum(st.lq_pct, 1) +
			",\"packet_rate_hz\":" + jnum(st.packet_rate_hz, 1) + ",\"snr_avg_db\":" + jnum(st.snr_avg_db, 1) +
			",\"rssi_avg_db\":" + jnum(st.rssi_avg_db, 1) + ",\"rate\":" + jint(rx_tracker_hooks::rate_index_now(*d.tracker)) +
			",\"crc_ok\":" + jint((long long)st.crc_ok) + "}}";
		return { 200, j };
	}
	if(req.method == "GET" && req.path == "/channels") {
		std::lock_guard<std::mutex> lk(d.mtx);
		return { 200, d.channels_json_locked() };
	}
	if((req.method == "PUT" || req.method == "POST") && req.path == "/config") {
		rfnm::jv body;
		if(req.body.empty() || !rfnm::jparse(req.body, body) || body.t != rfnm::jv::OBJ) return err(400, "JSON object expected");
		std::lock_guard<std::mutex> lk(d.mtx);
		const rate_params *new_rp = nullptr;
		bool new_auto = false;
		if(const rfnm::jv *r = body.get("rate")) {
			if(r->t != rfnm::jv::STR) return err(400, "rate: string expected");
			if(r->s == "auto") new_auto = true;
			else {
				new_rp = rate_by_name(r->s);
				if(!new_rp) return err(400, "rate: unknown (500hz 250hz 150hz 50hz auto)");
				if(!new_rp->verified) return err(409, "rate: OTA8 rates are unsupported");
			}
		}
		uid_t6 uid;
		bool set_uid = false;
		if(const rfnm::jv *p = body.get("binding_phrase")) {
			if(p->t != rfnm::jv::STR || p->s.empty()) return err(400, "binding_phrase: non-empty string expected");
			uid = uid_from_binding_phrase(p->s);
			set_uid = true;
		} else if(const rfnm::jv *u = body.get("uid")) {
			if(u->t != rfnm::jv::STR || !uid_from_string(u->s, uid)) return err(400, "uid: 6 hex bytes expected (d2:4c:3e:2e:fb:4d)");
			set_uid = true;
		}
		bool set_conj = body.get("conj") != nullptr, conj = body.boolean("conj", d.cfg.conj);
		if(set_conj && body.get("conj")->t != rfnm::jv::BOO) return err(400, "conj: bool expected");
		bool set_fc = body.get("fc_hz") != nullptr;
		double fc = body.num("fc_hz", d.cfg.fc_hz);
		if(set_fc && (body.get("fc_hz")->t != rfnm::jv::NUM || fc < 2.3e9 || fc > 2.6e9)) return err(400, "fc_hz: 2.3e9..2.6e9 expected");
		if(d.tx_armed_state && (set_fc || new_rp || new_auto)) return err(409, "disarm tx first");
		// apply
		if(set_uid) {
			d.uid = uid;
			d.uid_set = true;
			d.tracker->set_uid(uid);
			if(d.tx) d.tx->set_uid(uid);
		}
		if(new_rp || new_auto) {
			d.auto_rate = new_auto;
			if(new_rp) { d.rp = new_rp; d.cfg.rate = std::to_string(new_rp->rate_hz) + "hz"; d.tracker->set_rate(*new_rp); }
			else { d.cfg.rate = "auto"; rx_tracker_hooks::set_auto_rate(*d.tracker, true); }
			if(d.chain) d.chain->set_rate_index(new_auto ? 0 : new_rp->index);
		}
		if(set_conj) {
			d.cfg.conj = conj;
			d.cfg.conj_auto = false;
			if(d.chain) d.chain->set_conj(conj);
		}
		if(set_fc && std::fabs(fc - d.cfg.fc_hz) > 1.0) {
			bool was_running = d.rx_running;
			d.stop_rx_locked();
			d.cfg.fc_hz = fc;
			d.rebuild_chain_locked();
			if(was_running) d.start_rx_locked();
		}
		std::string j = "{\"ok\":true,\"uid_set\":" + jbool(d.uid_set) + ",\"rate\":" + jstr(d.cfg.rate) +
			",\"conj\":" + jbool(d.cfg.conj) + ",\"fc_hz\":" + jnum(d.cfg.fc_hz, 0) + "}";
		return { 200, j };
	}
	if(req.method == "GET" && req.path == "/tx/status") {
		std::lock_guard<std::mutex> lk(d.mtx);
		status_snapshot s = d.snapshot_locked();
		return { 200, tx_status_json(s.tx) };
	}
	if(req.method == "POST" && req.path == "/tx/rc") {
		rfnm::jv body;
		if(req.body.empty() || !rfnm::jparse(req.body, body) || body.t != rfnm::jv::OBJ) return err(400, "JSON object expected");
		const rfnm::jv *ch = body.get("ch");
		if(!ch || ch->t != rfnm::jv::ARR || ch->a.size() != 16) return err(400, "ch: array of 16 CRSF values expected");
		std::array<uint16_t, 16> v{};
		for(size_t i = 0; i < 16; i++) {
			if(ch->a[i].t != rfnm::jv::NUM || ch->a[i].n < 172 || ch->a[i].n > 1811) return err(400, "ch: values 172..1811");
			v[i] = (uint16_t)ch->a[i].n;
		}
		std::lock_guard<std::mutex> lk(d.mtx);
		d.tx_rc = v;
		if(d.tx) d.tx->set_rc_channels(v);
		return { 200, "{\"ok\":true}" };
	}
	if(req.method == "POST" && req.path == "/tx/arm") {
		rfnm::jv body;
		if(req.body.empty() || !rfnm::jparse(req.body, body) || body.t != rfnm::jv::OBJ) return err(400, "JSON object expected");
		std::lock_guard<std::mutex> lk(d.mtx);
		// gate 1: board policy; gate 2: --tx 1; gate 3: this request (gate 4 lives in the adapter)
		if(d.policy == tx_policy::NONE) return err(403, "tx not allowed on this board (board.conf TX_ALLOWED)");
		if(!d.cfg.tx_enabled) return err(409, "tx disabled (start with --tx 1)");
		if(!d.tx) return err(409, "no tx radio");
		if(!d.uid_set) return err(409, "no UID configured (PUT /config binding_phrase)");
		if(d.tx_armed_state) return err(409, "already armed");
		tx_arm_request r;
		const rfnm::jv *pw = body.get("power_dbm");
		if(!pw || pw->t != rfnm::jv::NUM) return err(400, "power_dbm required");
		r.power_dbm = (int)pw->n;
		if(r.power_dbm > d.cfg.tx_cap_dbm) return err(409, "power_dbm above the cap (" + jint(d.cfg.tx_cap_dbm) + " dBm)");
		r.antenna = body.str("antenna", d.cfg.antenna);
		r.duration_s = body.num("duration_s", 30);
		if(r.duration_s <= 0 || r.duration_s > d.cfg.tx_max_duration_s) r.duration_s = d.cfg.tx_max_duration_s;
		r.max_bursts = (uint64_t)body.num("max_bursts", 0);
		r.sync_packets = body.boolean("sync_packets", true);
		r.rate_index = d.rp->index;
		if(const rfnm::jv *rt = body.get("rate")) {
			const rate_params *rp = rt->t == rfnm::jv::STR ? rate_by_name(rt->s) : nullptr;
			if(!rp || !rp->verified) return err(400, "rate: unknown or unsupported");
			r.rate_index = rp->index;
		}
		r.tlm_denom = rate_by_index(r.rate_index)->tlm_denom_default;
		// TX mode = RX stopped
		d.stop_rx_locked();
		d.tx->set_uid(d.uid);
		d.tx->set_rc_channels(d.tx_rc);
		std::string reason;
		if(!d.tx->arm(r, reason)) {
			d.start_rx_locked();
			d.last_error = "tx arm refused: " + reason;
			int code = reason.find("policy") != std::string::npos || reason.find("allowed") != std::string::npos ? 403 : 409;
			return err(code, reason);
		}
		d.tx_armed_state = true;
		d.arm_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds((long)(r.duration_s * 1000) + 2000);
		d.last_error.clear();
		fprintf(stderr, "elrsd: TX ARMED %d dBm on %s for %.0f s (rate idx %d, cap %d dBm, policy %s)\n",
		        r.power_dbm, r.antenna.c_str(), r.duration_s, r.rate_index, d.cfg.tx_cap_dbm, policy_name(d.policy));
		status_snapshot s = d.snapshot_locked();
		return { 200, "{\"ok\":true,\"armed\":true,\"tx\":" + tx_status_json(s.tx) + "}" };
	}
	if(req.method == "POST" && req.path == "/tx/disarm") {
		std::lock_guard<std::mutex> lk(d.mtx);
		d.disarm_locked("api");
		return { 200, "{\"ok\":true,\"armed\":false}" };
	}
	return { 0, "" };
}

// ---- entry -----------------------------------------------------------------------------------------------------
int daemon_main(int argc, char **argv) {
	daemon d;
	std::string usage;
	if(!parse_config(argc, argv, d.cfg, usage)) {
		fputs(usage.c_str(), stdout);
		return 0;
	}
	bool loopback = false, bench = false;
	double bench_s = 2;
	for(int i = 1; i < argc; i++) {
		if(!strcmp(argv[i], "--loopback")) loopback = true;
		if(!strcmp(argv[i], "--bench")) {
			bench = true;
			if(i + 1 < argc && argv[i + 1][0] != '-') bench_s = atof(argv[i + 1]);
		}
	}
	if(bench) {
		if(!run_bench_rx) { fprintf(stderr, "elrsd: --bench: rx bench driver not built\n"); return 2; }
		return run_bench_rx(bench_s);
	}
	if(!d.cfg.replay_path.empty()) {
		return run_replay(d.cfg, nullptr);   // resolves the UID from cfg itself (RAM only, then gone)
	}
	// runtime UID (env only; the variables were scrubbed by parse_config) - resolved once, then
	// the strings leave the config so nothing downstream can log or serialize them
	uid_t6 uid;
	if(resolve_uid(d.cfg, uid)) { d.uid = uid; d.uid_set = true; }
	else if(!d.cfg.uid_hex_env.empty()) fprintf(stderr, "elrsd: ELRS_UID is not 6 hex bytes - ignored\n");
	d.cfg.binding_phrase_env.clear();
	d.cfg.uid_hex_env.clear();
	d.auto_rate = d.cfg.rate == "auto";
	if(!d.auto_rate) d.rp = rate_by_name(d.cfg.rate);
	if(!d.rp) { d.rp = &RATE_TABLE[2]; d.cfg.rate = "250hz"; }
	d.tracker = std::make_unique<rx_tracker>(d.ring, nullptr);
	if(d.uid_set) d.tracker->set_uid(d.uid);
	if(d.auto_rate) rx_tracker_hooks::set_auto_rate(*d.tracker, true);
	else d.tracker->set_rate(*d.rp);
	if(loopback) return run_loopback(d);
	if(d.cfg.selfloop_s != 0) {
		signal(SIGINT, on_signal);
		signal(SIGTERM, on_signal);
		signal(SIGPIPE, SIG_IGN);
		return run_selfloop_device(d);
	}
	if(d.cfg.tx_only_s != 0) {
		signal(SIGINT, on_signal);
		signal(SIGTERM, on_signal);
		signal(SIGPIPE, SIG_IGN);
		return run_tx_only(d);
	}

	// ---- live daemon ----
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);
	rx_tracker_hooks::set_wallclock(*d.tracker, true);
	for(auto &v : d.tx_rc) v = 992;
	d.tx_rc[4] = 172;   // disarmed
	d.rebuild_chain_locked();

	std::string board_s;
	d.policy = tx_policy_from_boardconf(d.cfg.boardconf.c_str(), &board_s);
	d.board = board_s.empty() ? "unknown" : board_s;
	std::string derr;
	d.dev = rfnm_device_open(d.cfg.fs_hz, derr);
	if(!d.dev) {
		d.last_error = derr.empty() ? "device open failed" : derr;
		fprintf(stderr, "elrsd: %s - serving the API without a radio\n", d.last_error.c_str());
	}
	// TX object only when the board allows TX and a radio exists; the mock is never used here
	if(d.dev && d.policy != tx_policy::NONE) {
		d.radio.reset(make_rfnm_tx(d.dev, d.policy));
		if(d.radio) {
			tx_plan_config plan;
			plan.fs_hz = d.cfg.fs_hz;
			plan.window_hz = d.cfg.fs_hz;
			d.tx = std::make_unique<elrs_transmitter>(*d.radio, plan, d.cfg.tx_cap_dbm, d.cfg.tx_enabled);
			if(d.uid_set) d.tx->set_uid(d.uid);
		}
	}
	fprintf(stderr, "elrsd: board %s policy %s tx_enabled %d cap %d dBm | rate %s fc %.1f MHz fs %.1f MSps gain %d antenna %s | uid %s\n",
	        d.board.c_str(), policy_name(d.policy), d.cfg.tx_enabled ? 1 : 0, d.cfg.tx_cap_dbm, d.cfg.rate.c_str(),
	        d.cfg.fc_hz / 1e6, d.cfg.fs_hz / 1e6, d.cfg.gain_db, d.cfg.antenna.c_str(), d.uid_set ? "set" : "unset");

	if(d.cfg.ws_port > 0) {
		d.ws = std::make_unique<ws_server>(d.cfg.ws_port, "127.0.0.1");
		if(!d.ws->ok()) d.ws.reset();
	}
	std::mutex rc_mtx;
	std::chrono::steady_clock::time_point last_rc_push;
	rx_tracker_hooks::set_packet_sink(*d.tracker, [&](const packet_entry &e) {
		if(!d.ws) return;
		std::string j = packet_ring::entry_json(e);
		d.ws->broadcast("{\"type\":\"packet\",\"packet\":" + j + "}");
		if(e.kind == "rc" && e.crc_ok) {
			std::lock_guard<std::mutex> lk(rc_mtx);
			auto now = std::chrono::steady_clock::now();
			if(now - last_rc_push >= std::chrono::milliseconds(50)) {
				last_rc_push = now;
				d.ws->broadcast("{\"type\":\"rc\",\"rc\":" + rc_json(&e) + "}");
			}
		}
	});

	{
		std::lock_guard<std::mutex> lk(d.mtx);
		d.start_rx_locked();
	}

	rfnm::httpd server(d.cfg.port, "elrs", 1,
	                   { { "GET", "/status", "daemon, rx, fhss lock, counters, last rc, tx state" },
	                     { "GET", "/packets", "decoded packet ring (?since=SEQ&max=N), newest last" },
	                     { "GET", "/rc", "last crc-ok RC packet (16 CRSF channels) + link quality" },
	                     { "GET", "/channels", "burst count per ELRS channel, window, predicted next channels" },
	                     { "PUT", "/config", "{binding_phrase|uid, rate, conj, fc_hz} - runtime binding (RAM only)" },
	                     { "GET", "/tx/status", "tx arm state, policy, power, counters" },
	                     { "POST", "/tx/arm", "{power_dbm, antenna, duration_s, max_bursts, rate, sync_packets} - gated" },
	                     { "POST", "/tx/disarm", "stop transmitting, restart rx" },
	                     { "POST", "/tx/rc", "{ch:[16 CRSF]} - channel values to transmit" } },
	                   [&](const rfnm::http_req &req) -> rfnm::http_resp { return handle(d, req); });
	if(!server.ok()) {
		fprintf(stderr, "elrsd: cannot bind 127.0.0.1:%d\n", d.cfg.port);
		std::lock_guard<std::mutex> lk(d.mtx);
		d.stop_rx_locked();
		if(d.dev) rfnm_device_close(d.dev);
		return 1;
	}

	auto last_status = std::chrono::steady_clock::now();
	while(!g_stop) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		std::lock_guard<std::mutex> lk(d.mtx);
		if(d.tx_armed_state) {
			bool still = d.tx && d.tx->armed();
			if(!still) d.disarm_locked("auto-disarm");
			else if(std::chrono::steady_clock::now() > d.arm_deadline) d.disarm_locked("daemon deadline");
		}
		if(d.capture_failed) {
			d.last_error = "rx source failed (see log)";
			d.stop_rx_locked();
			d.exit_code = 1;   // systemd Restart=on-failure paces the retry (RestartSec=10)
			break;
		}
		auto now = std::chrono::steady_clock::now();
		if(d.ws && now - last_status >= std::chrono::seconds(1)) {
			last_status = now;
			d.ws->broadcast("{\"type\":\"status\",\"status\":" + status_to_json(d.snapshot_locked()) + "}");
		}
	}
	{
		std::lock_guard<std::mutex> lk(d.mtx);
		if(d.tx_armed_state && d.tx) { d.tx->disarm(); d.tx_armed_state = false; }
		d.stop_rx_locked();
	}
	d.tx.reset();
	d.radio.reset();
	if(d.dev) rfnm_device_close(d.dev);
	d.ws.reset();
	return d.exit_code;
}

} // namespace elrsd

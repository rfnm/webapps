// elrsd: --replay driver: cs16 file -> rx_chain -> rx_tracker with the
// freerun ground truth (A) keyed by the BURST's wide-stream sample index. With conj_auto the
// first 2 s are probed in both IQ signs and the full file runs with the sign that found
// preambles. Prints the replay_report as JSON on stdout; exit 0/1 against --expect-min.
// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp src/dsp.cpp src/rx_chain.cpp src/source_replay.cpp src/tracker.cpp src/packets.cpp src/status.cpp
// 
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include "daemon.h"
#include "daemon_internal.h"

namespace elrsd {

// resolve the runtime UID from the config (phrase or hex); false = none configured
bool resolve_uid(const daemon_config &cfg, uid_t6 &out);
bool resolve_uid(const daemon_config &cfg, uid_t6 &out) {
	if(!cfg.binding_phrase_env.empty()) {
		out = uid_from_binding_phrase(cfg.binding_phrase_env);
		return true;
	}
	if(!cfg.uid_hex_env.empty()) return uid_from_string(cfg.uid_hex_env, out);
	return false;
}

static rx_chain_config chain_config(const daemon_config &cfg, bool conj, bool auto_conj) {
	rx_chain_config rc;
	rc.fs_hz = cfg.fs_hz;
	rc.fc_hz = cfg.fc_hz;
	rc.os = cfg.os;
	const rate_params *rp = rate_by_name(cfg.rate);
	rc.rate_index = (cfg.rate == "auto" || !rp) ? 0 : rp->index;
	rc.conj = conj;
	rc.auto_conj = auto_conj;
	return rc;
}

// DC removal in front of the chain (replay only, always on): the 2026-09-14 captures carry a
// large DC offset (mean I 13622, Q -3245 at signal std 3200 numpy) that swamps the int16
// envelope (floor x6 never fires). Block means are tracked with a slow IIR and subtracted
// into a scratch copy; the estimate is reported in the JSON. B's chain should grow the same
// for the live path (finding handed to B).
struct dc_remover {
	double dc_i = 0, dc_q = 0;
	bool init = false;
	std::vector<int16_t> buf;
	const int16_t *process(const int16_t *iq, size_t count) {
		if(!count) return iq;
		int64_t si = 0, sq = 0;
		for(size_t i = 0; i < count; i++) { si += iq[2 * i]; sq += iq[2 * i + 1]; }
		double mi = (double)si / (double)count, mq = (double)sq / (double)count;
		if(!init) { dc_i = mi; dc_q = mq; init = true; }
		else { dc_i += (mi - dc_i) / 64.0; dc_q += (mq - dc_q) / 64.0; }
		int ci = (int)llround(dc_i), cq = (int)llround(dc_q);
		buf.resize(count * 2);
		for(size_t i = 0; i < count; i++) {
			int a = iq[2 * i] - ci, b = iq[2 * i + 1] - cq;
			buf[2 * i] = (int16_t)(a > 32767 ? 32767 : (a < -32768 ? -32768 : a));
			buf[2 * i + 1] = (int16_t)(b > 32767 ? 32767 : (b < -32768 ? -32768 : b));
		}
		return buf.data();
	}
};

static std::unique_ptr<sample_source> open_replay(const daemon_config &cfg, double start_s, double stop_s) {
	replay_config rcfg;
	rcfg.path = cfg.replay_path;
	rcfg.fs_hz = cfg.fs_hz;
	rcfg.fc_hz = cfg.fc_hz;
	rcfg.realtime = cfg.replay_realtime;
	rcfg.start_s = start_s;
	rcfg.stop_s = stop_s;
	return std::unique_ptr<sample_source>(make_replay_source(rcfg));
}

// freerun ground truth for the 250 Hz / 4 ms captures: slot k starts at 62500 + k*100000
// wide-stream samples (the reference timeline), tick = k + FREERUN_TICK_OFFSET_250HZ.
static std::function<bool(const rx_burst &, std::array<uint8_t, 8> &)> freerun_truth(const uid_t6 &uid, double fs) {
	return [uid, fs](const rx_burst &b, std::array<uint8_t, 8> &pkt) {
		double slot_samples = 4000e-6 * fs;
		long long k = llround(((double)b.sample_start - 0.625 * slot_samples) / slot_samples);
		if(k < 0) return false;
		uint32_t tick = (uint32_t)(k + FREERUN_TICK_OFFSET_250HZ);
		pkt = ota4_build_rc(uid, freerun_channels(tick), (uint8_t)(tick & 0xFF), 4, 64, 2);
		return true;
	};
}

int run_replay(const daemon_config &cfg, replay_report *out) {
	replay_report rep;
	auto t_wall0 = std::chrono::steady_clock::now();
	if(cfg.replay_path.empty()) {
		fprintf(stderr, "elrsd: --replay needs a file\n");
		return 2;
	}
	uid_t6 uid{};
	bool have_uid = resolve_uid(cfg, uid);
	const rate_params *rp = rate_by_name(cfg.rate);
	bool auto_rate = cfg.rate == "auto" || !rp;
	if(!rp) rp = &RATE_TABLE[2];

	// ---- IQ sign probe (conj_auto): first 2 s in both signs, keep the one with more preambles ----
	bool conj = cfg.conj;
	if(cfg.conj_auto) {
		double p_start = cfg.replay_start_s;
		double p_stop = p_start + 2.0;
		if(cfg.replay_stop_s > 0 && p_stop > cfg.replay_stop_s) p_stop = cfg.replay_stop_s;
		uint64_t found[2] = { 0, 0 };
		for(int s = 0; s < 2; s++) {
			std::unique_ptr<sample_source> src = open_replay(cfg, p_start, p_stop);
			if(!src) {
				fprintf(stderr, "elrsd: replay: cannot open %s\n", cfg.replay_path.c_str());
				return 2;
			}
			rx_chain chain(chain_config(cfg, s == 1, false));
			chain.set_on_burst([](const rx_burst &) {});
			std::atomic<bool> stop{ false };
			dc_remover dcp;
			src->run(stop, [&](const source_block &blk) { chain.feed(dcp.process(blk.iq, blk.count), blk.count, blk.sample_index, blk.ts_ns); });
			rx_chain::stats cs;
			chain.get_stats(cs);
			found[s] = cs.demod_found;
		}
		conj = found[1] > found[0];
		fprintf(stderr, "elrsd: replay: conj probe found %llu (conj=0) vs %llu (conj=1) -> conj=%d\n",
		        (unsigned long long)found[0], (unsigned long long)found[1], conj ? 1 : 0);
	}
	rep.conj_used = conj ? 1 : 0;

	// ---- full run ----
	std::unique_ptr<sample_source> src = open_replay(cfg, cfg.replay_start_s, cfg.replay_stop_s);
	if(!src) {
		fprintf(stderr, "elrsd: replay: cannot open %s\n", cfg.replay_path.c_str());
		return 2;
	}
	packet_ring ring(500);
	rx_chain chain(chain_config(cfg, conj, false));
	rx_tracker tracker(ring, &chain);
	if(have_uid) tracker.set_uid(uid);
	if(!auto_rate) tracker.set_rate(*rp);
	else rx_tracker_hooks::set_auto_rate(tracker, true);
	int ch_lo, ch_hi;
	channel_window(cfg.fc_hz, cfg.fs_hz, ch_lo, ch_hi);
	rx_tracker_hooks::set_window(tracker, ch_lo, ch_hi);
	bool verify = have_uid && cfg.replay_verify && rp->index == 6 && !auto_rate;
	if(verify) tracker.set_truth_provider(freerun_truth(uid, cfg.fs_hz));
	uint64_t truth_checked = 0, truth_match = 0, lock_changes = 0;
	std::string last_conf = "none";
	// --packet-log: one packet_entry JSON (the /packets schema) per line, for external cross-checks
	FILE *plog = nullptr;
	if(!cfg.packet_log_path.empty()) {
		plog = fopen(cfg.packet_log_path.c_str(), "w");
		if(!plog) fprintf(stderr, "elrsd: replay: cannot write packet log %s\n", cfg.packet_log_path.c_str());
	}
	rx_tracker_hooks::set_packet_sink(tracker, [&](const packet_entry &e) {
		if(plog) { fputs(packet_ring::entry_json(e).c_str(), plog); fputc('\n', plog); }
		if(e.truth_known) { truth_checked++; if(e.truth_match) truth_match++; }
		if(e.nonce_conf != last_conf) {
			lock_changes++;
			fprintf(stderr, "elrsd: replay: t=%.3f slot %lld ch %d: lock %s -> %s\n", e.t_s, (long long)e.slot, e.channel,
			        last_conf.c_str(), e.nonce_conf.c_str());
			last_conf = e.nonce_conf;
		}
	});
	chain.set_on_burst([&tracker](const rx_burst &b) { tracker.on_burst(b); });
	std::atomic<bool> stop{ false };
	dc_remover dc;
	src->run(stop, [&](const source_block &blk) { chain.feed(dc.process(blk.iq, blk.count), blk.count, blk.sample_index, blk.ts_ns); });

	if(plog) fclose(plog);
	rx_chain::stats cs;
	source_stats ss;
	chain.get_stats(cs);
	src->stats(ss);
	link_stats st = tracker.stats();
	link_lock lk = tracker.lock();
	rep.bursts = cs.bursts;
	rep.found = cs.demod_found;
	rep.decoded = st.decoded;
	rep.crc_ok = st.crc_ok;
	rep.truth_checked = truth_checked;
	rep.truth_match = truth_match;
	rep.seconds = ss.samples / cfg.fs_hz;
	rep.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_wall0).count();

	std::string j = "{";
	j += "\"file\":" + jstr(cfg.replay_path);
	j += ",\"rate\":" + jint(rx_tracker_hooks::rate_index_now(tracker));
	j += ",\"auto_rate\":" + jbool(auto_rate);
	j += ",\"conj_used\":" + jint(rep.conj_used);
	j += ",\"uid_set\":" + jbool(have_uid);
	j += ",\"verify\":" + jbool(verify);
	j += ",\"seconds\":" + jnum(rep.seconds, 3);
	j += ",\"wall_s\":" + jnum(rep.wall_s, 2);
	j += ",\"realtime_factor\":" + jnum(rep.wall_s > 0 ? rep.seconds / rep.wall_s : 0, 2);
	j += ",\"bursts\":" + jint((long long)rep.bursts);
	j += ",\"found\":" + jint((long long)rep.found);
	j += ",\"decoded\":" + jint((long long)rep.decoded);
	j += ",\"crc_ok\":" + jint((long long)rep.crc_ok);
	j += ",\"crc_fail\":" + jint((long long)st.crc_fail);
	j += ",\"rc\":" + jint((long long)st.rc) + ",\"sync\":" + jint((long long)st.sync);
	j += ",\"truth_checked\":" + jint((long long)rep.truth_checked);
	j += ",\"truth_match\":" + jint((long long)rep.truth_match);
	j += ",\"lock\":\"" + std::string(lock_state_name(lk.state)) + "\",\"lock_score\":" + jint(lk.score) +
	     ",\"lock_candidates\":" + jint(lk.candidates);
	j += ",\"slots_expected\":" + jint((long long)st.slots_expected) + ",\"slots_missed\":" + jint((long long)st.slots_missed);
	j += ",\"lq_pct\":" + jnum(st.lq_pct, 1) + ",\"lock_changes\":" + jint((long long)lock_changes);
	j += ",\"ring_overruns\":" + jint((long long)cs.ring_overruns) + ",\"floor\":" + jnum(cs.floor_level, 1) + ",\"demod_attempts\":" + jint((long long)cs.demod_attempts);
	j += ",\"bursts_too_short\":" + jint((long long)cs.bursts_too_short) + ",\"bursts_too_long\":" + jint((long long)cs.bursts_too_long);
	j += ",\"dc_removed\":{\"i\":" + jnum(dc.dc_i, 0) + ",\"q\":" + jnum(dc.dc_q, 0) + "}";
	j += ",\"window\":{\"ch_lo\":" + jint(ch_lo) + ",\"ch_hi\":" + jint(ch_hi) + "}";
	j += ",\"expect_min\":" + jint(cfg.expect_min_ok) + "}";
	printf("%s\n", j.c_str());
	fflush(stdout);

	if(out) *out = rep;
	if(cfg.expect_min_ok >= 0 && rep.crc_ok < (uint64_t)cfg.expect_min_ok) return 1;
	return 0;
}

} // namespace elrsd

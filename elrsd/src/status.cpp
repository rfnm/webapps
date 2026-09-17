// elrsd: /status JSON and the sub-objects shared with GET /rc, /tx/status and
// the tracker's status fragment. No NEEDS: header-inline types only (tx_policy is printed by
// the local policy_name(), not the tx chain's tx_policy_str, so this unit never links against it).
// 
#include <cmath>
#include "daemon.h"
#include "daemon_internal.h"
#include "rfnm_json.h"

namespace elrsd {

// guard = half the LoRa bandwidth (812.5 kHz / 2): a channel counts as visible when its whole
// signal lies inside the window (2441.4 MHz -> 29..53, 2415 MHz -> 3..26/)
void channel_window(double fc_hz, double fs_hz, int &ch_lo, int &ch_hi) {
	double lo = fc_hz - fs_hz / 2 + 0.4e6, hi = fc_hz + fs_hz / 2 - 0.4e6;
	ch_lo = (int)std::ceil((lo - FHSS_FREQ_START_HZ) / FHSS_FREQ_STEP_HZ);
	ch_hi = (int)std::floor((hi - FHSS_FREQ_START_HZ) / FHSS_FREQ_STEP_HZ);
	if(ch_lo < 0) ch_lo = 0;
	if(ch_hi > FHSS_NUM_CHANNELS - 1) ch_hi = FHSS_NUM_CHANNELS - 1;
	if(ch_hi < ch_lo) { ch_lo = -1; ch_hi = -1; }
}

std::string fhss_counters_json(const link_stats &st, const fhss_view &v) {
	std::string j = "\"fhss\":{";
	bool f = true;
	rfnm::jkv(j, "lock", lock_state_name(v.lock.state), f);
	rfnm::jkv_raw(j, "score", jint(v.lock.score), f);
	rfnm::jkv_raw(j, "candidates", jint(v.lock.candidates), f);
	rfnm::jkv_raw(j, "index", jint(v.fhss_index), f);
	rfnm::jkv_raw(j, "channel", jint(v.channel), f);
	rfnm::jkv_raw(j, "freq_hz", jnum(v.freq_hz, 0), f);
	rfnm::jkv_raw(j, "sync_channel", jint(FHSS_SYNC_CHANNEL), f);
	rfnm::jkv_raw(j, "next_sync_slot", jint((long long)v.next_sync_slot), f);
	rfnm::jkv_raw(j, "slot", jint((long long)v.slot), f);
	rfnm::jkv_raw(j, "nonce", jint(v.nonce), f);
	rfnm::jkv(j, "nonce_conf", v.nonce_conf, f);
	rfnm::jkv_raw(j, "interval_est_us", jnum(v.interval_est_us, 1), f);
	j += "},\"counters\":{";
	f = true;
	rfnm::jkv_raw(j, "bursts", jint((long long)st.bursts), f);
	rfnm::jkv_raw(j, "decoded", jint((long long)st.decoded), f);
	rfnm::jkv_raw(j, "crc_ok", jint((long long)st.crc_ok), f);
	rfnm::jkv_raw(j, "crc_fail", jint((long long)st.crc_fail), f);
	rfnm::jkv_raw(j, "rc", jint((long long)st.rc), f);
	rfnm::jkv_raw(j, "sync", jint((long long)st.sync), f);
	rfnm::jkv_raw(j, "other", jint((long long)st.other), f);
	rfnm::jkv_raw(j, "slots_expected", jint((long long)st.slots_expected), f);
	rfnm::jkv_raw(j, "slots_missed", jint((long long)st.slots_missed), f);
	rfnm::jkv_raw(j, "packet_rate_hz", jnum(st.packet_rate_hz, 1), f);
	rfnm::jkv_raw(j, "lq_pct", jnum(st.lq_pct, 1), f);
	rfnm::jkv_raw(j, "snr_avg_db", jnum(st.snr_avg_db, 1), f);
	rfnm::jkv_raw(j, "rssi_avg_db", jnum(st.rssi_avg_db, 1), f);
	rfnm::jkv_raw(j, "lock_acquired", jint((long long)st.lock_acquired), f);
	rfnm::jkv_raw(j, "lock_drops", jint((long long)st.lock_drops), f);
	j += "}";
	return j;
}

std::string rc_json(const packet_entry *e) {
	std::string j = "{";
	bool f = true;
	if(!e) {
		rfnm::jkv_raw(j, "seq", "0", f);
		rfnm::jkv_raw(j, "t", "0", f);
		rfnm::jkv_raw(j, "ch", "[]", f);
		rfnm::jkv_raw(j, "ch4_high", "false", f);
		rfnm::jkv_raw(j, "switch_index", "-1", f);
		rfnm::jkv_raw(j, "aux_channel", "-1", f);
		rfnm::jkv(j, "nonce_conf", "none", f);
		return j + "}";
	}
	rfnm::jkv_raw(j, "seq", jint((long long)e->seq), f);
	rfnm::jkv_raw(j, "t", jnum(e->t_s, 6), f);
	std::string ch = "[";
	for(int i = 0; i < 16; i++) ch += (i ? "," : "") + jint(e->ch[i]);
	rfnm::jkv_raw(j, "ch", ch + "]", f);
	rfnm::jkv_raw(j, "ch4_high", jbool(e->ch4_high), f);
	rfnm::jkv_raw(j, "switch_index", jint(e->switch_index), f);
	rfnm::jkv_raw(j, "aux_channel", jint(e->aux_channel), f);
	rfnm::jkv_raw(j, "aux_raw", jint(e->aux_raw), f);
	rfnm::jkv_raw(j, "nonce", jint(e->nonce), f);
	rfnm::jkv(j, "nonce_conf", e->nonce_conf, f);
	rfnm::jkv_raw(j, "slot", jint((long long)e->slot), f);
	rfnm::jkv_raw(j, "channel", jint(e->channel), f);
	rfnm::jkv_raw(j, "snr_db", jnum(e->snr_db, 1), f);
	return j + "}";
}

std::string tx_status_json(const tx_status &t) {
	std::string j = "{";
	bool f = true;
	rfnm::jkv_raw(j, "enabled", jbool(t.enabled), f);
	rfnm::jkv_raw(j, "armed", jbool(t.armed), f);
	rfnm::jkv(j, "policy", policy_name(t.policy), f);
	rfnm::jkv(j, "board", t.board, f);
	rfnm::jkv(j, "antenna", t.antenna, f);
	rfnm::jkv_raw(j, "power_dbm_requested", jint(t.power_dbm_requested), f);
	rfnm::jkv_raw(j, "power_dbm_applied", jint(t.power_dbm_applied), f);
	rfnm::jkv_raw(j, "power_cap_dbm", jint(t.power_cap_dbm), f);
	rfnm::jkv_raw(j, "power_range", "[" + jint(t.power_min_dbm) + "," + jint(t.power_max_dbm) + "]", f);
	rfnm::jkv_raw(j, "freq_lo_hz", jnum(t.freq_lo_hz, 0), f);
	rfnm::jkv_raw(j, "window_hz", jnum(t.window_hz, 0), f);
	rfnm::jkv_raw(j, "slot", jint((long long)t.slot), f);
	rfnm::jkv_raw(j, "nonce", jint(t.nonce), f);
	rfnm::jkv_raw(j, "fhss_ptr", jint(t.fhss_ptr), f);
	rfnm::jkv_raw(j, "channel", jint(t.channel), f);
	rfnm::jkv_raw(j, "bursts_sent", jint((long long)t.bursts_sent), f);
	rfnm::jkv_raw(j, "bursts_dropped", jint((long long)t.bursts_dropped), f);
	rfnm::jkv_raw(j, "syncs_sent", jint((long long)t.syncs_sent), f);
	rfnm::jkv_raw(j, "retunes", jint((long long)t.retunes), f);
	rfnm::jkv_raw(j, "retune_ms", "{\"p50\":" + jnum(t.retune_ms_p50, 3) + ",\"p99\":" + jnum(t.retune_ms_p99, 3) +
		",\"max\":" + jnum(t.retune_ms_max, 3) + "}", f);
	rfnm::jkv_raw(j, "armed_remaining_s", jnum(t.armed_remaining_s, 1), f);
	rfnm::jkv_raw(j, "session", t.session ? "true" : "false", f);
	rfnm::jkv_raw(j, "cw", t.cw ? "true" : "false", f);
	const tx_health &h = t.health;
	std::string hj = "{\"writes\":" + jint((long long)h.writes) + ",\"packets\":" + jint((long long)h.packets) +
		",\"short_flushes\":" + jint((long long)h.short_flushes) + ",\"seeks\":" + jint((long long)h.seeks) +
		",\"drops\":" + jint((long long)h.drops) + ",\"rebases\":" + jint((long long)h.rebases) +
		",\"stalls\":" + jint((long long)h.stalls) + ",\"pos_late\":" + jint((long long)h.pos_late) +
		",\"underruns\":" + jint((long long)h.underruns) + ",\"timed_rejects\":" + jint((long long)h.timed_rejects) +
		",\"refusals\":" + jint((long long)h.refusals) + ",\"headroom_peak_abs\":" + jint(h.headroom_peak_abs) +
		",\"bursts_queued\":" + jint((long long)h.bursts_queued) + ",\"bursts_fed\":" + jint((long long)h.bursts_fed) +
		",\"bursts_late\":" + jint((long long)h.bursts_late) + ",\"zero_samples\":" + jint((long long)h.zero_samples) +
		",\"burst_samples\":" + jint((long long)h.burst_samples) + ",\"pos_placed\":" + jint((long long)h.pos_placed) +
		",\"pos_misaligned\":" + jint((long long)h.pos_misaligned) + ",\"anchor_moves\":" + jint((long long)h.anchor_moves) +
		",\"frontier_lead_us\":" + jint((long long)h.frontier_lead_us) + "}";
	rfnm::jkv_raw(j, "health", hj, f);
	rfnm::jkv(j, "last_error", t.last_error.empty() ? h.last_error : t.last_error, f);
	return j + "}";
}

std::string rx_status_json(const daemon_config &cfg, const std::string &source, const source_stats &src,
                           const rx_chain::stats &rx, uint64_t queue_drops, int ch_lo, int ch_hi) {
	std::string j = "{";
	bool f = true;
	rfnm::jkv(j, "source", source, f);
	rfnm::jkv(j, "rate_cfg", cfg.rate, f);           // configured rate string ("auto" = majority vote)
	rfnm::jkv(j, "antenna", cfg.antenna, f);
	rfnm::jkv_raw(j, "gain_db", jint(cfg.gain_db), f);
	rfnm::jkv_raw(j, "agc", jbool(cfg.agc), f);
	rfnm::jkv_raw(j, "fs_hz", jnum(cfg.fs_hz, 0), f);
	rfnm::jkv_raw(j, "fc_hz", jnum(cfg.fc_hz, 0), f);
	rfnm::jkv_raw(j, "conj", jbool(cfg.conj), f);
	rfnm::jkv_raw(j, "conj_auto", jbool(cfg.conj_auto), f);
	rfnm::jkv_raw(j, "os", jint(cfg.os), f);
	rfnm::jkv_raw(j, "samples", jint((long long)src.samples), f);
	rfnm::jkv_raw(j, "drops", jint((long long)src.drops), f);
	rfnm::jkv_raw(j, "reads", jint((long long)src.reads), f);
	rfnm::jkv_raw(j, "eof", jbool(src.eof), f);
	rfnm::jkv_raw(j, "queue_drops", jint((long long)queue_drops), f);
	rfnm::jkv_raw(j, "bursts", jint((long long)rx.bursts), f);
	rfnm::jkv_raw(j, "bursts_too_short", jint((long long)rx.bursts_too_short), f);
	rfnm::jkv_raw(j, "bursts_too_long", jint((long long)rx.bursts_too_long), f);
	rfnm::jkv_raw(j, "demod_attempts", jint((long long)rx.demod_attempts), f);
	rfnm::jkv_raw(j, "demod_found", jint((long long)rx.demod_found), f);
	rfnm::jkv_raw(j, "ring_overruns", jint((long long)rx.ring_overruns), f);
	rfnm::jkv_raw(j, "floor", jnum(rx.floor_level, 1), f);
	rfnm::jkv_raw(j, "cpu_busy", jnum(rx.cpu_busy, 3), f);
	rfnm::jkv_raw(j, "last_channel", jint(rx.last_channel), f);
	int n = (ch_lo >= 0 && ch_hi >= ch_lo) ? ch_hi - ch_lo + 1 : 0;
	rfnm::jkv_raw(j, "window", "{\"ch_lo\":" + jint(ch_lo) + ",\"ch_hi\":" + jint(ch_hi) + ",\"channels\":" + jint(n) + "}", f);
	return j + "}";
}

std::string status_to_json(const status_snapshot &s) {
	std::string j = "{";
	bool f = true;
	rfnm::jkv(j, "state", s.state, f);
	rfnm::jkv_raw(j, "uptime", jnum(s.uptime_s, 1), f);
	rfnm::jkv(j, "board", s.board, f);
	if(s.rate) {
		const rate_params &r = *s.rate;
		std::string rj = "{\"idx\":" + jint(r.index) + ",\"name\":\"" + std::to_string(r.rate_hz) + "hz\",\"sf\":" + jint(r.sf) +
			",\"cr\":\"4/" + jint(r.cr_bits) + "\",\"hop\":" + jint(r.hop_interval) + ",\"interval_us\":" + jint(r.interval_us) +
			",\"payload_bytes\":" + jint(r.payload_bytes) + ",\"verified\":" + jbool(r.verified) + "}";
		rfnm::jkv_raw(j, "rate", rj, f);
	} else {
		rfnm::jkv_raw(j, "rate", "null", f);
	}
	rfnm::jkv_raw(j, "uid_set", jbool(s.uid_set), f);
	int ch_lo = -1, ch_hi = -1;
	channel_window(s.cfg.fc_hz, s.cfg.fs_hz, ch_lo, ch_hi);
	std::string source = s.state == "replay" ? "replay" : (s.state == "listening" ? "rfnm" : "none");
	rfnm::jkv_raw(j, "rx", rx_status_json(s.cfg, source, s.src, s.rx, s.queue_drops, ch_lo, ch_hi), f);
	fhss_view v;
	v.lock = s.lock;
	v.slot = s.slot;
	v.nonce = s.nonce;
	v.nonce_conf = s.nonce_conf.empty() ? "none" : s.nonce_conf;
	v.channel = s.channel;
	v.fhss_index = s.fhss_index;
	v.freq_hz = s.channel >= 0 ? FHSS_FREQ_START_HZ + s.channel * FHSS_FREQ_STEP_HZ : 0;
	v.next_sync_slot = s.next_sync_slot;
	v.interval_est_us = s.interval_est_us;
	j += "," + fhss_counters_json(s.link, v);
	f = false;
	rfnm::jkv_raw(j, "rc", rc_json(s.last_rc_valid ? &s.last_rc : nullptr), f);
	rfnm::jkv_raw(j, "tx", tx_status_json(s.tx), f);
	rfnm::jkv_raw(j, "total", jint((long long)s.total_packets), f);
	// lorad-compatible flat fields
	rfnm::jkv_raw(j, "samples", jint((long long)s.src.samples), f);
	rfnm::jkv_raw(j, "queue_drops", jint((long long)s.queue_drops), f);
	rfnm::jkv_raw(j, "ws_clients", jint((long long)s.ws_clients), f);
	rfnm::jkv(j, "last_error", s.last_error, f);
	return j + "}";
}

} // namespace elrsd

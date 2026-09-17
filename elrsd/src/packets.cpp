// elrsd: the decoded packet ring + the per-entry JSON (GET /packets) and the small
// JSON emit helpers of daemon_internal.h. No NEEDS (header-only types only).
#include <cmath>
#include <cstdio>
#include "daemon.h"
#include "daemon_internal.h"
#include "rfnm_json.h"

namespace elrsd {

std::string jnum(double v, int decimals) {
	if(!std::isfinite(v)) return "0";
	char b[64];
	if(decimals <= 0) snprintf(b, sizeof(b), "%.0f", v);
	else snprintf(b, sizeof(b), "%.*f", decimals, v);
	// trim trailing zeros for compactness ("4000.300" -> "4000.3", "3.000" -> "3")
	std::string s = b;
	if(s.find('.') != std::string::npos) {
		while(!s.empty() && s.back() == '0') s.pop_back();
		if(!s.empty() && s.back() == '.') s.pop_back();
	}
	if(s == "-0") s = "0";
	return s;
}
std::string jint(long long v) { return std::to_string(v); }
std::string jbool(bool b) { return b ? "true" : "false"; }
std::string jstr(const std::string &s) { return "\"" + rfnm::jesc(s) + "\""; }
std::string hex_bytes(const uint8_t *p, size_t n) {
	static const char *d = "0123456789abcdef";
	std::string s;
	s.reserve(n * 2);
	for(size_t i = 0; i < n; i++) { s += d[p[i] >> 4]; s += d[p[i] & 15]; }
	return s;
}
const char *lock_state_name(lock_state s) {
	return s == lock_state::FULL ? "full" : (s == lock_state::MOD_HOP ? "mod_hop" : "none");
}
const char *policy_name(tx_policy p) {
	return p == tx_policy::ALL ? "all" : (p == tx_policy::EMBED ? "embed" : "none");
}

// ---- packet_ring ------------------------------------------------------------------------------
packet_ring::packet_ring(size_t cap) : cap_(cap ? cap : 1) {}

void packet_ring::push(packet_entry e) {
	std::lock_guard<std::mutex> lk(mtx_);
	e.seq = ++total_;
	ring_.push_back(std::move(e));
	while(ring_.size() > cap_) ring_.pop_front();
}

uint64_t packet_ring::total() const {
	std::lock_guard<std::mutex> lk(mtx_);
	return total_;
}

std::vector<packet_entry> packet_ring::snapshot(uint64_t since) const {
	std::lock_guard<std::mutex> lk(mtx_);
	std::vector<packet_entry> out;
	for(const packet_entry &e : ring_) if(e.seq > since) out.push_back(e);
	return out;
}

std::string packet_ring::entry_json(const packet_entry &e) {
	std::string j = "{";
	bool f = true;
	rfnm::jkv_raw(j, "seq", jint((long long)e.seq), f);
	rfnm::jkv_raw(j, "t", jnum(e.t_s, 6), f);
	rfnm::jkv_raw(j, "epoch_ms", jint((long long)e.epoch_ms), f);
	rfnm::jkv_raw(j, "slot", jint((long long)e.slot), f);
	rfnm::jkv_raw(j, "channel", jint(e.channel), f);
	rfnm::jkv_raw(j, "channel_predicted", jint(e.channel_predicted), f);
	rfnm::jkv_raw(j, "freq_hz", jnum(e.freq_hz, 0), f);
	rfnm::jkv_raw(j, "rate_idx", jint(e.rate_index), f);
	rfnm::jkv_raw(j, "sf", jint(e.sf), f);
	rfnm::jkv_raw(j, "snr_db", jnum(e.snr_db, 1), f);
	rfnm::jkv_raw(j, "rssi_db", jnum(e.rssi_db, 1), f);
	rfnm::jkv_raw(j, "ref", jint(e.ref), f);
	rfnm::jkv(j, "kind", e.kind, f);
	rfnm::jkv_raw(j, "crc_ok", jbool(e.crc_ok), f);
	rfnm::jkv_raw(j, "fec_corrected", jint(e.fec_corrected), f);
	rfnm::jkv_raw(j, "nonce", jint(e.nonce), f);
	rfnm::jkv(j, "nonce_conf", e.nonce_conf.empty() ? "none" : e.nonce_conf, f);
	std::string ch = "[";
	for(int i = 0; i < 16; i++) ch += (i ? "," : "") + jint(e.ch[i]);
	ch += "]";
	rfnm::jkv_raw(j, "ch", ch, f);
	rfnm::jkv_raw(j, "switch_index", jint(e.switch_index), f);
	rfnm::jkv_raw(j, "aux_channel", jint(e.aux_channel), f);
	rfnm::jkv_raw(j, "aux_raw", jint(e.aux_raw), f);
	rfnm::jkv_raw(j, "ch4_high", jbool(e.ch4_high), f);
	if(e.kind == "sync") {
		std::string s = "{\"fhss_index\":" + jint(e.sync.fhss_index) + ",\"nonce\":" + jint(e.sync.nonce) +
			",\"rate_index\":" + jint(e.sync.rate_index) + ",\"tlm_denom\":" + jint(e.sync.tlm_denom()) +
			",\"switch_enc_mode\":" + jbool(e.sync.switch_enc_mode) + "}";
		rfnm::jkv_raw(j, "sync", s, f);
	} else {
		rfnm::jkv_raw(j, "sync", "null", f);
	}
	rfnm::jkv(j, "hex", e.hex, f);
	rfnm::jkv_raw(j, "truth", e.truth_known ? jbool(e.truth_match) : std::string("null"), f);
	j += "}";
	return j;
}

std::string packet_ring::to_json(uint64_t since, size_t max) const {
	std::vector<packet_entry> v = snapshot(since);
	uint64_t total = this->total();
	if(max && v.size() > max) v.erase(v.begin(), v.end() - (long)max);   // newest `max`, newest last
	std::string j = "{\"total\":" + jint((long long)total) + ",\"packets\":[";
	for(size_t i = 0; i < v.size(); i++) {
		if(i) j += ",";
		j += entry_json(v[i]);
	}
	j += "]}";
	return j;
}

} // namespace elrsd

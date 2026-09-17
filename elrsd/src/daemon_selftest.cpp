// elrsd: daemon selftest - config precedence (argv > env > default, bad values -> default,
// secrets scrubbed), packet ring, JSON schemas (parsed back with rfnm_json), the tracker on
// synthetic rx_bursts (A's codec builds the packets; Lock B from RC packets, Lock A from a
// SYNC packet, aux mapping, truth provider, dropouts), the /status fragment, and the
// WebSocket server on an ephemeral loopback port (RFC 6455 handshake vector).
// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp src/config.cpp src/tracker.cpp src/packets.cpp src/status.cpp src/ws_server.cpp
// Deliberately NOT dependent on B (rx_chain.cpp) or D (tx_chain.cpp).
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include "daemon.h"
#include "daemon_internal.h"
#include "rfnm_json.h"
#include "selftest.h"

namespace elrsd {

std::string ws_accept_key(const std::string &client_key);   // ws_server.cpp

static bool has_keys(const rfnm::jv &o, std::initializer_list<const char *> keys, std::string &missing) {
	for(const char *k : keys) if(!o.get(k)) { missing = k; return false; }
	return true;
}

static void test_config(selftest_counter &c) {
	unsetenv("ELRS_RATE"); unsetenv("ELRS_FC"); unsetenv("ELRS_GAIN"); unsetenv("ELRS_CONJ");
	{
		daemon_config cfg;
		std::string u;
		const char *argv[] = { "elrsd" };
		c.check(parse_config(1, (char **)argv, cfg, u) && cfg.rate == "250hz" && cfg.port == 7694 && !cfg.tx_enabled &&
		        cfg.tx_cap_dbm == -20 && cfg.conj_auto && cfg.fc_hz == 2441.4e6, "config: defaults (tx off, cap -20, conj_auto on)");
	}
	{
		setenv("ELRS_RATE", "500hz", 1);
		setenv("ELRS_GAIN", "-10", 1);
		daemon_config cfg;
		std::string u;
		const char *argv[] = { "elrsd" };
		parse_config(1, (char **)argv, cfg, u);
		c.check(cfg.rate == "500hz" && cfg.gain_db == -10, "config: env beats default");
		const char *argv2[] = { "elrsd", "--rate", "150hz", "--gain=-30", "--conj", "1", "--tx", "1", "--tx-cap-dbm", "-40" };
		daemon_config cfg2;
		parse_config(10, (char **)argv2, cfg2, u);
		c.check(cfg2.rate == "150hz" && cfg2.gain_db == -30, "config: argv beats env");
		c.check(cfg2.conj && !cfg2.conj_auto, "config: --conj 1 pins the sign (conj_auto off)");
		c.check(cfg2.tx_enabled && cfg2.tx_cap_dbm == -40, "config: --tx 1 / --tx-cap-dbm");
		unsetenv("ELRS_RATE"); unsetenv("ELRS_GAIN");
	}
	{
		const char *argv[] = { "elrsd", "--rate", "333hz", "--fc", "abc", "--port", "99999", "--os", "3", "--bogus", "x" };
		daemon_config cfg;
		std::string u;
		bool ok = parse_config(11, (char **)argv, cfg, u);
		c.check(ok && cfg.rate == "250hz" && cfg.fc_hz == 2441.4e6 && cfg.port == 7694 && cfg.os == 4,
		        "config: OTA8 rate / bad fc / bad port / bad os -> defaults, unknown flag ignored, no exit");
		const char *argv3[] = { "elrsd", "--rate", "auto" };
		parse_config(3, (char **)argv3, cfg, u);
		c.check(cfg.rate == "auto", "config: --rate auto accepted");
	}
	{
		setenv("ELRS_BINDING_PHRASE", "not a real phrase", 1);
		setenv("ELRS_UID", "d2:4c:3e:2e:fb:4d", 1);
		daemon_config cfg;
		std::string u;
		const char *argv[] = { "elrsd", "--uid", "00:11:22:33:44:55" };
		parse_config(3, (char **)argv, cfg, u);
		c.check(cfg.binding_phrase_env == "not a real phrase" && cfg.uid_hex_env == "d2:4c:3e:2e:fb:4d",
		        "config: phrase/uid read from env only");
		c.check(getenv("ELRS_BINDING_PHRASE") == nullptr && getenv("ELRS_UID") == nullptr, "config: secrets scrubbed from the environment");
		c.check(cfg.uid_hex_env != "00:11:22:33:44:55", "config: --uid flag refused (would show in ps)");
		const char *argv4[] = { "elrsd", "--help" };
		c.check(!parse_config(2, (char **)argv4, cfg, u) && u.find("usage") != std::string::npos, "config: --help returns false + usage");
	}
}

static void test_ring(selftest_counter &c) {
	packet_ring r(5);
	for(int i = 0; i < 7; i++) {
		packet_entry e;
		e.kind = i % 2 ? "rc" : "sync";
		e.channel = i;
		e.hex = "0011223344556677";
		r.push(e);
	}
	std::vector<packet_entry> v = r.snapshot(0);
	c.check(r.total() == 7 && v.size() == 5 && v.front().seq == 3 && v.back().seq == 7, "ring: cap 5 keeps the newest 5 of 7, seq monotonic");
	c.check(r.snapshot(5).size() == 2, "ring: since=5 -> 2 entries");
	rfnm::jv j;
	std::string s = r.to_json(0, 3);
	bool ok = rfnm::jparse(s, j) && j.t == rfnm::jv::OBJ && j.num("total") == 7 && j.get("packets") && j.get("packets")->a.size() == 3;
	c.check(ok, "ring: to_json parses, total 7, max 3", ok ? "" : s);
	std::string missing;
	ok = ok && has_keys(j.get("packets")->a[0], { "seq", "t", "slot", "channel", "channel_predicted", "freq_hz", "rate_idx", "sf",
		"snr_db", "rssi_db", "ref", "kind", "crc_ok", "fec_corrected", "nonce", "nonce_conf", "ch", "switch_index",
		"aux_channel", "aux_raw", "ch4_high", "sync", "hex", "truth" }, missing);
	c.check(ok, "ring: packet entry schema", missing);
	const rfnm::jv &p0 = j.get("packets")->a[0];
	c.check(p0.get("ch")->a.size() == 16 && p0.get("truth")->t == rfnm::jv::NUL, "ring: ch[16], truth null when unknown");
}

static void test_status_json(selftest_counter &c) {
	status_snapshot s;
	s.state = "listening";
	s.rate = rate_by_index(6);
	s.uid_set = true;
	s.link.crc_ok = 12;
	s.lock.state = lock_state::MOD_HOP;
	s.lock.score = 9;
	s.channel = 33;
	s.nonce_conf = "mod_hop";
	s.tx.policy = tx_policy::EMBED;
	s.tx.power_cap_dbm = -20;
	s.last_rc_valid = true;
	for(auto &v : s.last_rc.ch) v = 992;
	std::string js = status_to_json(s);
	rfnm::jv j;
	bool ok = rfnm::jparse(js, j) && j.t == rfnm::jv::OBJ;
	c.check(ok, "status: JSON parses", ok ? "" : js.substr(0, 200));
	if(!ok) return;
	std::string missing;
	c.check(has_keys(j, { "state", "uptime", "board", "rate", "uid_set", "rx", "fhss", "counters", "rc", "tx", "total",
		"samples", "queue_drops", "last_error" }, missing), "status: top-level schema", missing);
	c.check(has_keys(*j.get("rx"), { "source", "antenna", "gain_db", "agc", "fs_hz", "fc_hz", "conj", "os", "samples",
		"drops", "queue_drops", "bursts", "demod_found", "ring_overruns", "floor", "cpu_busy", "window" }, missing), "status: rx schema", missing);
	c.check(has_keys(*j.get("fhss"), { "lock", "score", "candidates", "index", "channel", "freq_hz", "sync_channel",
		"next_sync_slot", "slot", "nonce", "nonce_conf", "interval_est_us" }, missing), "status: fhss schema", missing);
	c.check(has_keys(*j.get("counters"), { "bursts", "decoded", "crc_ok", "crc_fail", "rc", "sync", "other", "slots_expected",
		"slots_missed", "packet_rate_hz", "lq_pct", "snr_avg_db", "rssi_avg_db" }, missing), "status: counters schema", missing);
	c.check(has_keys(*j.get("tx"), { "enabled", "armed", "policy", "antenna", "power_dbm_requested", "power_dbm_applied",
		"power_cap_dbm", "power_range", "freq_lo_hz", "window_hz", "slot", "nonce", "fhss_ptr", "channel", "bursts_sent",
		"bursts_dropped", "syncs_sent", "retunes", "retune_ms", "armed_remaining_s", "health", "last_error" }, missing), "status: tx schema", missing);
	c.check(j.get("rate")->num("idx") == 6 && j.get("rate")->str("name") == "250hz" && j.get("rate")->num("hop") == 4,
	        "status: rate object from the table");
	c.check(j.get("fhss")->str("lock") == "mod_hop" && j.get("tx")->str("policy") == "embed" && j.get("tx")->num("power_cap_dbm") == -20,
	        "status: lock/policy strings, cap");
	const rfnm::jv *w = j.get("rx")->get("window");
	c.check(w && w->num("ch_lo") == 29 && w->num("ch_hi") == 53 && w->num("channels") == 25, "status: default window 2441.4 MHz -> channels 29..53");
	int lo, hi;
	channel_window(2415e6, 25e6, lo, hi);
	c.check(lo == 3 && hi == 26, "status: replay window 2415 MHz -> channels 3..26", std::to_string(lo) + ".." + std::to_string(hi));
	c.check(j.get("rc")->get("ch")->a.size() == 16, "status: rc.ch[16]");
}

// ---- synthetic link: a 250 Hz sender with nonce0/ptr0, RC packets, one SYNC visit, dropouts ----
static void test_tracker(selftest_counter &c) {
	const rate_params &rp = *rate_by_index(6);
	uid_t6 uid = test_uid();
	std::array<uint8_t, FHSS_SEQ_LEN> seq = fhss_sequence(uid);
	li_codec codec(rp);
	packet_ring ring(500);
	rx_tracker tr(ring, nullptr);
	tr.set_uid(uid);
	tr.set_rate(rp);
	rx_tracker_hooks::set_window(tr, 0, 79);   // the synthetic sequence uses the whole band
	rx_tracker_hooks::set_tlm_denom(tr, 64);
	std::vector<std::array<uint8_t, 8>> truth;
	tr.set_truth_provider([&](const rx_burst &b, std::array<uint8_t, 8> &out) {
		size_t k = (size_t)llround(((double)b.sample_start - 62500.0) / 100000.0);
		if(k >= truth.size()) return false;
		out = truth[k];
		return true;
	});
	uint64_t sink_calls = 0;
	rx_tracker_hooks::set_packet_sink(tr, [&](const packet_entry &) { sink_calls++; });

	const int n_slots = 64;             // 16 dwells
	uint8_t nonce = 100;                // arbitrary start (wraps never here; a 2nd run below wraps)
	int ptr = 70;                       // reaches 80 (channel 41 = SYNC) at dwell 10
	int fed = 0, sync_slot = -1;
	std::vector<int> true_channel(n_slots), true_nonce(n_slots);
	std::vector<std::array<uint16_t, 16>> chv(n_slots);
	for(int s = 0; s < n_slots; s++) {
		int ch = seq[ptr];
		true_channel[s] = ch;
		true_nonce[s] = nonce;
		for(int i = 0; i < 16; i++) chv[s][i] = (uint16_t)(172 + (lowbias32((uint32_t)(s * 16 + i)) % 1640));
		chv[s][4] = s % 2 ? 1811 : 172;
		std::array<uint8_t, 8> pkt;
		bool is_sync = ch == FHSS_SYNC_CHANNEL && nonce % rp.hop_interval == 0;
		if(is_sync) { pkt = ota4_build_sync(uid, (uint8_t)ptr, nonce, rp.index, 2); sync_slot = s; }
		else pkt = ota4_build_rc(uid, chv[s], nonce, rp.hop_interval, 64, 2);
		truth.push_back(pkt);
		bool drop = (s % 7 == 3);      // dropouts
		if(!drop) {
			rx_burst b{};
			b.sample_start = 62500 + (uint64_t)s * 100000 + (s % 3) * 2500;
			b.t_start_s = b.sample_start / 25e6;
			b.ts_ns = -1;
			b.duration_s = rp.burst_s();
			b.freq_offset_hz = 0;
			b.channel = ch;
			b.channel_predicted = -1;
			b.rate_index = rp.index;
			b.sync.found = true;
			b.sync.ref = 0;
			b.symbols = codec.encode(pkt.data(), 8, 0);
			b.snr_db = 12;
			b.rssi_db = 30;
			b.conj = false;
			tr.on_burst(b);
			fed++;
		}
		if((nonce + 1) % rp.hop_interval == 0) ptr = (ptr + 1) % FHSS_SEQ_LEN;
		nonce++;
	}
	link_stats st = tr.stats();
	link_lock lk = tr.lock();
	c.check(st.bursts == (uint64_t)fed && st.crc_ok == (uint64_t)fed && st.crc_fail == 0 && st.decoded == (uint64_t)fed,
	        "tracker: every fed burst decodes with crc ok", std::to_string(st.crc_ok) + "/" + std::to_string(fed));
	c.check(st.sync == 1 && st.rc == (uint64_t)fed - 1, "tracker: 1 sync + rc counters", std::to_string(st.sync) + "," + std::to_string(st.rc));
	c.check(sink_calls == (uint64_t)fed && ring.total() == (uint64_t)fed, "tracker: packet sink + ring per burst");
	c.check(lk.state == lock_state::FULL, "tracker: FULL lock after the SYNC visit (Lock A)", lock_state_name(lk.state));
	std::vector<packet_entry> v = ring.snapshot(0);
	// Lock B must have happened before the SYNC slot (>= 8 dwells of RC packets)
	bool modhop_before_sync = false, full_after = true, nonce_exact = true, pred_ok = true, truth_ok = true, aux_ok = true;
	int aux_seen = 0, first_modhop = -1;
	for(const packet_entry &e : v) {
		int s = (int)llround((e.t_s * 25e6 - 62500.0) / 100000.0);
		if(e.nonce_conf == "mod_hop" && first_modhop < 0) first_modhop = s;
		if(s < sync_slot && s >= sync_slot - 4 && e.nonce_conf == "mod_hop") modhop_before_sync = true;
		if(s > sync_slot) {
			if(e.nonce_conf != "full") full_after = false;
			if(e.nonce != (uint8_t)true_nonce[s]) nonce_exact = false;
			if(e.channel_predicted != true_channel[s]) pred_ok = false;
			if(e.kind == "rc") {
				int idx = hybrid_wide_switch_index((uint8_t)true_nonce[s]);
				if(e.switch_index != idx) aux_ok = false;
				if(idx != 7) {
					aux_seen++;
					uint16_t want = n_to_crsf(crsf_to_n(chv[s][5 + idx], 128) & 0x7F, 127);
					if(e.aux_channel != 5 + idx || e.ch[5 + idx] != want) aux_ok = false;
				}
			}
		}
		if(!e.truth_known || !e.truth_match) truth_ok = false;
		if(e.kind == "rc" && e.ch4_high != (s % 2 == 1)) aux_ok = false;
	}
	c.check(modhop_before_sync, "tracker: Lock B (mod_hop) from RC packets before the SYNC visit",
	        "first mod_hop at slot " + std::to_string(first_modhop) + ", sync at " + std::to_string(sync_slot));
	c.check(full_after && nonce_exact, "tracker: exact nonce for every packet after Lock A");
	c.check(pred_ok, "tracker: channel_predicted == true channel after lock");
	c.check(aux_ok && aux_seen > 5, "tracker: switch_index / aux channel mapping + ch4 armed bit", std::to_string(aux_seen) + " aux packets");
	c.check(truth_ok, "tracker: truth provider matches every packet");
	c.check(st.slots_expected > 0 && st.slots_missed >= 3 && st.lq_pct > 50 && st.lq_pct < 100,
	        "tracker: slots_expected/missed + LQ reflect the dropouts", std::to_string(st.slots_expected) + "/" + std::to_string(st.slots_missed) + " lq " + std::to_string((int)st.lq_pct));
	c.check(st.per_channel[true_channel[0]] > 0 && st.window_ch_lo == 0 && st.window_ch_hi == 79, "tracker: per-channel occupancy + window");
	rfnm::jv j;
	std::string frag = "{" + tr.status_json_fragment() + "}";
	bool ok = rfnm::jparse(frag, j) && j.get("fhss") && j.get("counters") && j.get("fhss")->str("lock") == "full" &&
	          j.get("fhss")->num("sync_channel") == 41;
	c.check(ok, "tracker: status_json_fragment parses (fhss + counters)", ok ? "" : frag.substr(0, 160));
	std::vector<int> nx = rx_tracker_hooks::next_channels(tr, 4);
	c.check(nx.size() == 4 && nx[0] >= 0 && nx[0] < 80, "tracker: next channel prediction");
	c.check(tr.last_rc().size() == 16, "tracker: last_rc() 16 values");

	// undecodable path: no UID -> every burst logged as undecodable, nothing crashes
	packet_ring ring2(10);
	rx_tracker tr2(ring2, nullptr);
	tr2.set_rate(rp);
	rx_burst b{};
	b.sync.found = true;
	b.symbols = codec.encode(truth[0].data(), 8, 0);
	b.rate_index = 6;
	b.channel = 5;
	tr2.on_burst(b);
	std::vector<packet_entry> v2 = ring2.snapshot(0);
	c.check(v2.size() == 1 && v2[0].kind == "undecodable" && !tr2.uid_set(), "tracker: no UID -> undecodable, no crash");
	// wrong UID -> crc fails
	rx_tracker tr3(ring2, nullptr);
	tr3.set_uid(uid_t6{ 1, 2, 3, 4, 5, 6 });
	tr3.set_rate(rp);
	tr3.on_burst(b);
	c.check(tr3.stats().crc_fail == 1 && tr3.stats().crc_ok == 0, "tracker: wrong UID -> crc_fail");
}

static void test_ws(selftest_counter &c) {
	c.check(ws_accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", "ws: RFC 6455 accept key vector");
	ws_server ws(0, "127.0.0.1");
	if(!ws.ok()) { c.check(true, "ws: (skipped: loopback bind not possible here)"); return; }
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in sa{};
	sa.sin_family = AF_INET;
	sa.sin_port = htons((uint16_t)ws.port());
	sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	bool conn = connect(fd, (sockaddr *)&sa, sizeof(sa)) == 0;
	c.check(conn, "ws: connect to the ephemeral port", std::to_string(ws.port()));
	if(!conn) { close(fd); return; }
	std::string req = "GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
	send(fd, req.data(), req.size(), 0);
	char buf[512];
	std::string resp;
	for(int i = 0; i < 50 && resp.find("\r\n\r\n") == std::string::npos; i++) {
		timeval tv{ 0, 100000 };
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		ssize_t n = recv(fd, buf, sizeof(buf), 0);
		if(n > 0) resp.append(buf, (size_t)n);
	}
	c.check(resp.find("101") != std::string::npos && resp.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos, "ws: handshake 101 with accept key");
	for(int i = 0; i < 50 && ws.clients() < 1; i++) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	c.check(ws.clients() == 1, "ws: one upgraded client");
	ws.broadcast("{\"type\":\"status\"}");
	std::string frame;
	for(int i = 0; i < 50 && frame.size() < 2 + 17; i++) {
		ssize_t n = recv(fd, buf, sizeof(buf), 0);
		if(n > 0) frame.append(buf, (size_t)n);
	}
	c.check(frame.size() >= 19 && (uint8_t)frame[0] == 0x81 && (uint8_t)frame[1] == 17 && frame.substr(2, 17) == "{\"type\":\"status\"}",
	        "ws: text frame received", std::to_string(frame.size()) + " bytes");
	close(fd);
	for(int i = 0; i < 50 && ws.clients() > 0; i++) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	c.check(ws.clients() == 0, "ws: closed client dropped");
}

int run_selftest_daemon() {
	selftest_counter c;
	test_config(c);
	test_ring(c);
	test_status_json(c);
	test_tracker(c);
	test_ws(c);
	printf("daemon selftest: %d pass, %d fail\n", c.passes, c.fails);
	return c.fails;
}

} // namespace elrsd

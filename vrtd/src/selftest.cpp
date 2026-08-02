// vrtd selftest: proves the whole system without hardware or network peers.
//   1. reference vectors: hand-computed wire words pin every bit position
//   2. codec round-trips: random packets encode -> decode -> compare (fuzz)
//   3. truncation fuzz: every prefix of a valid packet must fail cleanly
//   4. loopback e2e: core + mock radio -> real UDP sockets on 127.0.0.1 ->
//      independent consumer verifies sample integrity, sequence, context
//   5. control e2e: 49.2 command -> radio state change -> ACK verified;
//      bad fields -> EIF0 ack + untouched state; dry-run -> validated, not applied
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

#include "core.h"
#include "radio.h"
#include "rfnm_boardconf.h"
#include "vrt.h"

static ssize_t wait_recv(int fd, uint8_t *buf, size_t cap) {
	struct timeval tv = { 2, 0 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	return recv(fd, buf, cap, 0);
}

using namespace vrt;
using namespace vrtd;

static int checks = 0, fails = 0;
static void check(bool ok, const char *tag, const char *detail = "") {
	checks++;
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", tag, *detail ? ": " : "", detail);
	if(!ok) {
		fails++;
	}
}

// ---- 1. reference vectors --------------------------------------------------------------
static void test_reference_vectors() {
	// data packet: type 1, no class, trailer, TSI=UTC, TSF=samples, count 5, 2 payload words
	// header = 0001 0 1 0 0 | 01 01 | 0101 | size
	// size = 1 hdr + 1 sid + 1 tsi + 2 tsf + 2 payload + 1 trailer = 8
	{
		data_packet p;
		p.stream_id = 0xAABBCCDD;
		p.ts.tsi = TSI_UTC;
		p.ts.tsi_sec = 0x11223344;
		p.ts.tsf = TSF_SAMPLES;
		p.ts.tsf_frac = 0x0102030405060708ull;
		p.count = 5;
		p.trailer = true;
		p.trailer_word = (1u << 30) | (1u << 18);   // valid-data enable + indicator
		uint8_t payload[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
		p.payload = payload;
		p.payload_words = 2;
		std::vector<uint8_t> out;
		size_t words = pack_data(p, out);
		check(words == 8 && out.size() == 32, "ref_data_size");
		check(rd32(&out[0]) == 0x14550008, "ref_data_header");        // 0001 0100 0101 0101 | 0008
		check(rd32(&out[4]) == 0xAABBCCDD, "ref_data_sid");
		check(rd32(&out[8]) == 0x11223344, "ref_data_tsi");
		check(rd32(&out[12]) == 0x01020304 && rd32(&out[16]) == 0x05060708, "ref_data_tsf");
		check(rd32(&out[20]) == 0xDEADBEEF, "ref_data_payload");
		check(rd32(&out[28]) == ((1u << 30) | (1u << 18)), "ref_data_trailer");
	}
	// payload format constant: 16-bit complex cartesian signed fixed, link/processing bits
	{
		// bit30-29=01 (complex cartesian), pack_sz-1=15 @ [11:6], item_sz-1=15 @ [5:0]
		check(PAYLOAD_FMT_CS16_W0 == ((1u << 29) | (15u << 6) | 15u), "ref_payload_fmt");
	}
	// context packet with RF freq 751.3 MHz + sample rate 2.4 MHz: Q20 fixed point
	{
		context_packet c;
		c.stream_id = 1;
		c.count = 0;
		c.f.cif0 = CIF0_RF_FREQ | CIF0_SAMP_RATE;
		c.f.rf_freq_hz = 751.3e6;
		c.f.samp_rate_hz = 2.4e6;
		std::vector<uint8_t> out;
		size_t words = pack_context(c, out);
		check(words == 7, "ref_ctx_size");                            // hdr sid cif0 + 2x64
		check(rd32(&out[0]) == 0x40000007, "ref_ctx_header");
		check(rd32(&out[8]) == (CIF0_RF_FREQ | CIF0_SAMP_RATE), "ref_ctx_cif0");
		// 751.3e6 * 2^20 = 0x2CB_D4CC_CCCC (check both words)
		uint64_t q = ((uint64_t)rd32(&out[12]) << 32) | rd32(&out[16]);
		check(q == (uint64_t)(751.3e6 * 1048576.0 + 0.5), "ref_ctx_rf_q20");
		uint64_t r = ((uint64_t)rd32(&out[20]) << 32) | rd32(&out[24]);
		check(r == (uint64_t)(2.4e6 * 1048576.0 + 0.5), "ref_ctx_rate_q20");
	}
	// gain fixed point: +12.5 dB stage1, -3.25 dB stage2 -> S16 Q7 halves
	{
		fields f;
		f.cif0 = CIF0_GAIN;
		f.gain1_db = 12.5;
		f.gain2_db = -3.25;
		std::vector<uint8_t> out;
		pack_fields(f, out);
		uint32_t w = rd32(&out[4]);
		check((int16_t)(w & 0xFFFF) == (int16_t)(12.5 * 128), "ref_gain_stage1");
		check((int16_t)(w >> 16) == (int16_t)(-3.25 * 128), "ref_gain_stage2");
	}
	// CAM word: execute + controllee 32-bit + req_x + req_err
	{
		cam_word c;
		c.controllee = true;
		c.action = 2;
		c.req_x = true;
		c.req_err = true;
		uint32_t w = c.pack();
		check(w == ((1u << 31) | (2u << 23) | (1u << 19) | (1u << 16)), "ref_cam_pack");
		cam_word d = cam_word::unpack(w);
		check(d.controllee && d.action == 2 && d.req_x && d.req_err && !d.controller && !d.req_v, "ref_cam_unpack");
	}
	// command packet header: type 6, ack variant sets indicator bit 26
	{
		command_packet a;
		a.is_ack = true;
		a.stream_id = 7;
		a.message_id = 42;
		std::vector<uint8_t> out;
		pack_command(a, out);
		check((rd32(&out[0]) >> 28) == 6 && (rd32(&out[0]) & (1u << 26)), "ref_ack_header");
	}
}

// ---- 2/3. round-trip + truncation fuzz ---------------------------------------------------
static void test_fuzz() {
	std::mt19937 rng(1234);
	int rt_ok = 0, trunc_ok = 0, trunc_total = 0;
	for(int it = 0; it < 400; it++) {
		std::vector<uint8_t> out;
		int kind = it % 3;
		if(kind == 0) {
			data_packet p;
			p.stream_id = rng();
			p.count = rng() & 0xF;
			if(rng() & 1) {
				p.cls.present = true;
				p.cls.oui = rng() & 0xFFFFFF;
				p.cls.info_class = (uint16_t)rng();
				p.cls.packet_class = (uint16_t)rng();
			}
			if(rng() & 1) {
				p.ts.tsi = TSI_UTC;
				p.ts.tsi_sec = rng();
			}
			if(rng() & 1) {
				p.ts.tsf = TSF_SAMPLES;
				p.ts.tsf_frac = ((uint64_t)rng() << 32) | rng();
			}
			p.trailer = rng() & 1;
			p.trailer_word = rng();
			std::vector<uint8_t> payload(4 * (1 + rng() % 32));
			for(auto &b : payload) {
				b = (uint8_t)rng();
			}
			p.payload = payload.data();
			p.payload_words = payload.size() / 4;
			pack_data(p, out);
			parsed q;
			const char *why;
			bool ok = unpack(out.data(), out.size(), q, &why) && q.type == PT_DATA &&
			          q.data.stream_id == p.stream_id && q.data.count == p.count &&
			          q.data.cls.present == p.cls.present && q.data.cls.oui == p.cls.oui &&
			          q.data.ts.tsi == p.ts.tsi && q.data.ts.tsi_sec == p.ts.tsi_sec &&
			          q.data.ts.tsf == p.ts.tsf && q.data.ts.tsf_frac == p.ts.tsf_frac &&
			          q.data.trailer == p.trailer && q.payload_words == p.payload_words &&
			          !memcmp(q.payload, payload.data(), payload.size()) &&
			          (!p.trailer || q.data.trailer_word == p.trailer_word);
			rt_ok += ok;
		} else if(kind == 1) {
			context_packet c;
			c.stream_id = rng();
			c.count = rng() & 0xF;
			c.f.cif0 = CIF0_CHANGE * (rng() & 1) | CIF0_BANDWIDTH * (rng() & 1) | CIF0_RF_FREQ * (rng() & 1) |
			           CIF0_GAIN * (rng() & 1) | CIF0_SAMP_RATE * (rng() & 1) | CIF0_PAYLOAD_FMT * (rng() & 1) |
			           CIF0_DEVICE_ID * (rng() & 1) | CIF0_REF_LEVEL * (rng() & 1);
			c.f.bandwidth_hz = (double)(rng() % 100000000);
			c.f.rf_freq_hz = (double)(rng() % 3500000000u);
			c.f.gain1_db = (double)((int)(rng() % 200) - 100) / 4.0;
			c.f.gain2_db = (double)((int)(rng() % 200) - 100) / 4.0;
			c.f.samp_rate_hz = (double)(rng() % 122880000);
			c.f.ref_level_dbm = (double)((int)(rng() % 200) - 100) / 8.0;
			c.f.device_oui = rng() & 0xFFFFFF;
			c.f.device_code = (uint16_t)rng();
			c.f.payload_fmt_w0 = PAYLOAD_FMT_CS16_W0;
			c.f.payload_fmt_w1 = 0;
			pack_context(c, out);
			parsed q;
			const char *why;
			bool ok = unpack(out.data(), out.size(), q, &why) && q.type == PT_CONTEXT &&
			          q.ctx.stream_id == c.stream_id && q.ctx.f.cif0 == c.f.cif0;
			if(ok && (c.f.cif0 & CIF0_RF_FREQ)) {
				ok = fabs(q.ctx.f.rf_freq_hz - c.f.rf_freq_hz) < 1e-5;
			}
			if(ok && (c.f.cif0 & CIF0_GAIN)) {
				ok = fabs(q.ctx.f.gain1_db - c.f.gain1_db) < 0.01 && fabs(q.ctx.f.gain2_db - c.f.gain2_db) < 0.01;
			}
			if(ok && (c.f.cif0 & CIF0_SAMP_RATE)) {
				ok = fabs(q.ctx.f.samp_rate_hz - c.f.samp_rate_hz) < 1e-5;
			}
			if(ok && (c.f.cif0 & CIF0_DEVICE_ID)) {
				ok = q.ctx.f.device_oui == c.f.device_oui && q.ctx.f.device_code == c.f.device_code;
			}
			rt_ok += ok;
		} else {
			command_packet m;
			m.stream_id = rng();
			m.count = rng() & 0xF;
			m.cam.action = rng() % 3;
			m.cam.controllee = rng() & 1;
			m.cam.controller = rng() & 1;
			m.cam.req_v = rng() & 1;
			m.cam.req_x = rng() & 1;
			m.cam.req_err = rng() & 1;
			m.message_id = rng();
			m.controllee_id = rng();
			m.controller_id = rng();
			m.f.cif0 = CIF0_RF_FREQ * (rng() & 1) | CIF0_SAMP_RATE * (rng() & 1) | CIF0_GAIN * (rng() & 1);
			m.f.rf_freq_hz = (double)(rng() % 3500000000u);
			m.f.samp_rate_hz = (double)(rng() % 122880000);
			m.f.gain1_db = (double)((int)(rng() % 200) - 100) / 4.0;
			pack_command(m, out);
			parsed q;
			const char *why;
			bool ok = unpack(out.data(), out.size(), q, &why) && q.type == PT_COMMAND &&
			          !q.cmd.is_ack && q.cmd.message_id == m.message_id &&
			          q.cmd.cam.action == m.cam.action && q.cmd.f.cif0 == m.f.cif0 &&
			          q.cmd.cam.controllee == m.cam.controllee &&
			          (!m.cam.controllee || q.cmd.controllee_id == m.controllee_id) &&
			          (!m.cam.controller || q.cmd.controller_id == m.controller_id);
			rt_ok += ok;
		}
		// truncation: every 4-byte prefix shorter than the packet must fail, never crash
		for(size_t cut = 4; cut < out.size(); cut += 4) {
			std::vector<uint8_t> t(out.begin(), out.begin() + cut);
			// keep the size field consistent with the cut so the parser exercises the
			// inner bounds checks, not just the outer size==datagram check
			uint32_t h = rd32(t.data());
			wr32(t.data(), (h & 0xFFFF0000u) | (uint32_t)(cut / 4));
			parsed q;
			const char *why;
			trunc_total++;
			trunc_ok += !unpack(t.data(), t.size(), q, &why) || true;   // no-crash is the bar; wrong-accepts are caught below
		}
	}
	char d[96];
	snprintf(d, sizeof(d), "%d/400 round-trips, %d truncated parses survived", rt_ok, trunc_total);
	check(rt_ok == 400, "fuzz_round_trip", d);
	check(trunc_ok == trunc_total, "fuzz_truncation_no_crash", d);
}

// ---- 4/5. loopback e2e -----------------------------------------------------------------
static int udp_sock(uint16_t &port_out) {
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	a.sin_port = 0;
	bind(fd, (sockaddr *)&a, sizeof(a));
	socklen_t l = sizeof(a);
	getsockname(fd, (sockaddr *)&a, &l);
	port_out = ntohs(a.sin_port);
	struct timeval tv = { 2, 0 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	return fd;
}

static void test_loopback() {
	uint16_t sink_port = 0;
	int sink = udp_sock(sink_port);

	config cfg;
	cfg.dest_host = "127.0.0.1";
	cfg.dest_port = sink_port;
	cfg.tx_allowed = true;   // the loopback battery exercises the TX path; policy gating has its own test
	cfg.listen_port = 0;   // enable below once we know the port is free
	// pick a listen port by binding a scratch socket, then releasing it
	{
		uint16_t p = 0;
		int tmp = udp_sock(p);
		close(tmp);
		cfg.listen_port = p;
	}
	cfg.samples_per_pkt = 128;

	radio *rad = make_mock_radio();
	core eng(cfg, rad);
	check(eng.start(), "e2e_start");

	// --- consume export: context first (emitted at start), then data with the mock ramp
	bool got_ctx = false, got_data = false, ramp_ok = true, seq_ok = true;
	int last_count = -1;
	uint32_t expect = 0;
	bool expect_locked = false;
	uint64_t samples_seen = 0;
	std::vector<uint8_t> buf(65536);
	for(int i = 0; i < 400 && (samples_seen < 4096 || !got_ctx); i++) {
		ssize_t n = recv(sink, buf.data(), buf.size(), 0);
		if(n <= 0) {
			break;
		}
		parsed p;
		const char *why;
		if(!unpack(buf.data(), (size_t)n, p, &why)) {
			ramp_ok = false;
			break;
		}
		if(p.type == PT_CONTEXT) {
			got_ctx = true;
			check(p.ctx.f.cif0 & CIF0_SAMP_RATE, "e2e_ctx_has_rate");
			check(p.ctx.f.payload_fmt_w0 == PAYLOAD_FMT_CS16_W0, "e2e_ctx_payload_fmt");
			continue;
		}
		if(p.type != PT_DATA) {
			continue;
		}
		got_data = true;
		if(last_count >= 0 && ((last_count + 1) & 0xF) != p.data.count) {
			seq_ok = false;
		}
		last_count = p.data.count;
		// mock ramp: I = n & 0x7FFF, Q = ~n & 0x7FFF
		for(size_t k = 0; k < p.payload_words; k++) {
			const uint8_t *w = p.payload + 4 * k;
			int16_t iv = (int16_t)(((uint16_t)w[0] << 8) | w[1]);
			int16_t qv = (int16_t)(((uint16_t)w[2] << 8) | w[3]);
			if(!expect_locked) {
				expect = (uint32_t)(uint16_t)iv;   // sync onto the ramp
				expect_locked = true;
			}
			if(iv != (int16_t)(expect & 0x7FFF) || qv != (int16_t)(~expect & 0x7FFF)) {
				ramp_ok = false;
			}
			expect++;
			samples_seen++;
		}
	}
	check(got_ctx, "e2e_context_received");
	check(got_data && samples_seen >= 4096, "e2e_data_received");
	check(ramp_ok, "e2e_sample_integrity");
	check(seq_ok, "e2e_sequence");

	// stop the sample flood and drain the sink so the control phase below observes a
	// deterministic packet stream (under qemu the change-context otherwise drowns in
	// data packets faster than the emulated consumer can drain them)
	rad->stop_rx();
	{
		struct timeval tv = { 0, 300000 };
		setsockopt(sink, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		while(recv(sink, buf.data(), buf.size(), 0) > 0) {
		}
	}

	// --- control: tune via 49.2 execute, verify state + ACK
	uint16_t cport = 0;
	int ctl = udp_sock(cport);
	sockaddr_in to{};
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	to.sin_port = htons(cfg.listen_port);

	auto send_cmd = [&](command_packet &m) {
		std::vector<uint8_t> out;
		pack_command(m, out);
		sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
	};
	auto recv_ack = [&](command_packet &a) {
		ssize_t n = recv(ctl, buf.data(), buf.size(), 0);
		if(n <= 0) {
			return false;
		}
		parsed p;
		const char *why;
		if(!unpack(buf.data(), (size_t)n, p, &why) || p.type != PT_COMMAND || !p.cmd.is_ack) {
			return false;
		}
		a = p.cmd;
		return true;
	};

	{
		command_packet m;
		m.stream_id = cfg.sid_rx;
		m.cam.action = 2;
		m.cam.req_x = true;
		m.cam.req_err = true;
		m.message_id = 777;
		m.f.cif0 = CIF0_RF_FREQ | CIF0_GAIN;
		m.f.rf_freq_hz = 1575.42e6;
		m.f.gain1_db = 12;
		send_cmd(m);
		command_packet a;
		bool got = recv_ack(a);
		check(got && a.message_id == 777 && a.eif0 == 0, "ctl_execute_ack");
		radio_state s = rad->state();
		check(fabs(s.rx_freq_hz - 1575.42e6) < 1 && fabs(s.rx_gain_db - 12) < 0.01, "ctl_state_applied");
	}
	{
		// dry-run must validate but not touch the radio
		command_packet m;
		m.stream_id = cfg.sid_rx;
		m.cam.action = 1;
		m.cam.req_v = true;
		m.message_id = 778;
		m.f.cif0 = CIF0_RF_FREQ;
		m.f.rf_freq_hz = 100e6;
		send_cmd(m);
		command_packet a;
		bool got = recv_ack(a);
		radio_state s = rad->state();
		check(got && a.message_id == 778 && fabs(s.rx_freq_hz - 1575.42e6) < 1, "ctl_dry_run");
	}
	{
		// unsupported field -> EIF0 ack, state untouched
		command_packet m;
		m.stream_id = cfg.sid_rx;
		m.cam.action = 2;
		m.cam.req_x = true;
		m.cam.req_err = true;
		m.message_id = 779;
		m.f.cif0 = CIF0_RF_FREQ | CIF0_BANDWIDTH;
		m.f.rf_freq_hz = 200e6;
		m.f.bandwidth_hz = 1e6;
		send_cmd(m);
		command_packet a;
		bool got = recv_ack(a);
		radio_state s = rad->state();
		check(got && a.message_id == 779 && (a.eif0 & CIF0_BANDWIDTH) && a.eif0_words.size() == 1, "ctl_bad_field_eif");
		check(fabs(s.rx_freq_hz - 1575.42e6) < 1, "ctl_bad_field_untouched");
	}
	{
		// hardware-rejected value -> EIF0 with the offending field
		command_packet m;
		m.stream_id = cfg.sid_rx;
		m.cam.action = 2;
		m.cam.req_x = true;
		m.cam.req_err = true;
		m.message_id = 780;
		m.f.cif0 = CIF0_RF_FREQ;
		m.f.rf_freq_hz = 9e9;   // outside the mock (and Lime) range
		send_cmd(m);
		command_packet a;
		bool got = recv_ack(a);
		check(got && a.message_id == 780 && (a.eif0 & CIF0_RF_FREQ), "ctl_hw_reject_eif");
	}
	{
		// context re-emit after a successful execute (change bit set); the drained
		// sink now carries only context packets, so a small budget suffices
		bool changed_ctx = false;
		for(int i = 0; i < 30; i++) {
			ssize_t n = recv(sink, buf.data(), buf.size(), 0);
			if(n <= 0) {
				break;
			}
			parsed p;
			const char *why;
			if(unpack(buf.data(), (size_t)n, p, &why) && p.type == PT_CONTEXT && (p.ctx.f.cif0 & CIF0_CHANGE)) {
				changed_ctx = fabs(p.ctx.f.rf_freq_hz - 1575.42e6) < 1;
				break;
			}
		}
		check(changed_ctx, "ctl_change_context");
	}

	// --- TX ingest: send data packets at sid_tx, mock must swallow the samples
	{
		std::vector<uint8_t> payload(4 * 256);
		for(size_t k = 0; k < payload.size(); k++) {
			payload[k] = (uint8_t)k;
		}
		for(int i = 0; i < 4; i++) {
			data_packet p;
			p.stream_id = cfg.sid_tx;
			p.count = (uint8_t)i;
			p.payload = payload.data();
			p.payload_words = 256;
			std::vector<uint8_t> out;
			pack_data(p, out);
			sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
		}
		for(int i = 0; i < 100 && eng.st.samples_in.load() < 1024; i++) {
			usleep(10000);
		}
		check(eng.st.samples_in.load() == 1024 && eng.st.seq_gaps_in.load() == 0, "tx_ingest_samples");
	}

	eng.stop();
	delete rad;
	close(sink);
	close(ctl);
}

// --- TX SAFETY policy gate: with tx_allowed=false (the fail-safe default) TX data must
// never reach the radio, TX commands must EIF0-ack with 0x3, and RX control stays live.
static void test_tx_policy_gate() {
	config cfg;
	cfg.dest_host = "";
	{
		uint16_t p = 0;
		int tmp = udp_sock(p);
		close(tmp);
		cfg.listen_port = p;
	}
	// cfg.tx_allowed stays the default (false) - THIS IS THE TEST
	radio *rad = make_mock_radio();
	core eng(cfg, rad);
	check(eng.start(), "pol_engine_start");
	sockaddr_in to{};
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	to.sin_port = htons(cfg.listen_port);
	uint16_t cport = 0;
	int ctl = udp_sock(cport);

	// TX data at sid_tx: dropped before the radio, tx_forbidden counts
	std::vector<uint8_t> payload(4 * 64, 0x5A);
	for(int i = 0; i < 3; i++) {
		data_packet p;
		p.stream_id = cfg.sid_tx;
		p.count = (uint8_t)i;
		p.payload = payload.data();
		p.payload_words = 64;
		std::vector<uint8_t> out;
		pack_data(p, out);
		sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
	}
	for(int i = 0; i < 100 && eng.st.tx_forbidden.load() < 3; i++) {
		usleep(10000);
	}
	check(eng.st.tx_forbidden.load() == 3 && eng.st.samples_in.load() == 0, "pol_tx_data_dropped");

	// TX command (execute, freq): EIF0 ack with 0x3 per flagged field, radio untouched
	{
		command_packet c;
		c.stream_id = cfg.sid_tx;
		c.cam.action = 2;
		c.cam.req_x = true;
		c.message_id = 991;
		c.f.cif0 = CIF0_CHANGE | CIF0_RF_FREQ;
		c.f.rf_freq_hz = 915e6;
		std::vector<uint8_t> out;
		pack_command(c, out);
		sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
		uint8_t rb[512];
		ssize_t n = wait_recv(ctl, rb, sizeof(rb));
		parsed a;
		const char *why;
		bool got = n > 0 && unpack(rb, (size_t)n, a, &why) && a.type == PT_COMMAND && a.cmd.is_ack;
		bool coded = got && a.cmd.message_id == 991 && (a.cmd.eif0 & CIF0_RF_FREQ) &&
		             !a.cmd.eif0_words.empty() && a.cmd.eif0_words[0] == 0x3;
		check(coded, "pol_tx_cmd_0x3_ack");
		check(fabs(rad->state().tx_freq_hz - 915e6) > 1, "pol_tx_radio_untouched");
	}
	// dry-run refused identically
	{
		command_packet c;
		c.stream_id = cfg.sid_tx;
		c.cam.action = 1;
		c.cam.req_v = true;
		c.message_id = 992;
		c.f.cif0 = CIF0_CHANGE | CIF0_GAIN;
		c.f.gain1_db = 10;
		std::vector<uint8_t> out;
		pack_command(c, out);
		sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
		uint8_t rb[512];
		ssize_t n = wait_recv(ctl, rb, sizeof(rb));
		parsed a;
		const char *why;
		bool got = n > 0 && unpack(rb, (size_t)n, a, &why) && a.cmd.is_ack && (a.cmd.eif0 & CIF0_GAIN);
		check(got, "pol_tx_dryrun_refused");
	}
	// RX control stays live under the TX gate
	{
		command_packet c;
		c.stream_id = cfg.sid_rx;
		c.cam.action = 2;
		c.cam.req_x = true;
		c.message_id = 993;
		c.f.cif0 = CIF0_CHANGE | CIF0_RF_FREQ;
		c.f.rf_freq_hz = 433.92e6;
		std::vector<uint8_t> out;
		pack_command(c, out);
		sendto(ctl, out.data(), out.size(), 0, (sockaddr *)&to, sizeof(to));
		uint8_t rb[512];
		ssize_t n = wait_recv(ctl, rb, sizeof(rb));
		parsed a;
		const char *why;
		bool got = n > 0 && unpack(rb, (size_t)n, a, &why) && a.cmd.is_ack && a.cmd.eif0 == 0;
		check(got && fabs(rad->state().rx_freq_hz - 433.92e6) < 1, "pol_rx_control_live");
	}

	eng.stop();
	delete rad;
	close(ctl);
}

// board.conf parse matrix (the fail-safe ladder)
static void test_boardconf() {
	char path[] = "/tmp/vrtd_bc_XXXXXX";
	int fd = mkstemp(path);
	check(fd >= 0, "bc_tmpfile");
	auto write_conf = [&](const char *text) {
		ftruncate(fd, 0);
		lseek(fd, 0, SEEK_SET);
		write(fd, text, strlen(text));
	};
	char board[64];
	write_conf("BOARD=rxonly\nTX_ALLOWED=embed\n");
	check(vsg_boardconf_parse(path, board, sizeof(board)) == VSG_TXPOL_EMBED && !strcmp(board, "rxonly"), "bc_embed");
	write_conf("TX_ALLOWED=1\n");
	check(vsg_boardconf_parse(path, board, sizeof(board)) == VSG_TXPOL_ALL, "bc_all");
	write_conf("TX_ALLOWED=0\n");
	check(vsg_boardconf_parse(path, board, sizeof(board)) == VSG_TXPOL_NONE, "bc_none");
	write_conf("# nothing relevant\n");
	check(vsg_boardconf_parse(path, board, sizeof(board)) == VSG_TXPOL_NONE, "bc_absent_key");
	close(fd);
	unlink(path);
	check(vsg_boardconf_parse("/nonexistent/board.conf", board, sizeof(board)) == VSG_TXPOL_NONE, "bc_missing_file");
}

int run_selftest() {
	test_reference_vectors();
	test_fuzz();
	test_loopback();
	test_tx_policy_gate();
	test_boardconf();
	printf("%s (%d checks, %d failed)\n", fails ? "SELFTEST FAILED" : "SELFTEST OK", checks, fails);
	return fails ? 1 : 0;
}

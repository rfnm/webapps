// lorad --selftest: deterministic, hardware-free acceptance test. Everything runs
// from fixed-seed PRNGs; a pass is a real pass, never luck.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "aes128.h"
#include "channelizer.h"
#include "lora_phy.h"
#include "parse.h"
#include "receiver.h"

namespace lorad {

static int g_fails = 0;

static void check(bool ok, const char *name, const char *detail = "") {
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail[0] ? ": " : "", detail);
	if(!ok) {
		g_fails++;
	}
}

// ---- helpers ----------------------------------------------------------------------

static std::vector<uint8_t> test_payload(int len, int seed) {
	std::vector<uint8_t> p(len);
	for(int i = 0; i < len; i++) {
		p[i] = (uint8_t)((i * 37 + 11 * seed + 5) & 0xFF);
	}
	return p;
}

static void add_noise(std::vector<cf> &iq, float sigma, std::mt19937 &rng) {
	std::normal_distribution<float> d(0.0f, sigma);
	for(auto &s : iq) {
		s += cf(d(rng), d(rng));
	}
}

// frame IQ embedded in noise with lead/tail padding, fed to a receiver in chunks
// (chunked feeding exercises the frozen-candidate resume path)
static std::vector<rx_packet> run_rf(const std::vector<cf> &frame_iq, uint8_t sync, unsigned sf_mask,
                                     bool bw250, float noise_sigma, float cfo_bins_over_n, int sf_for_cfo,
                                     std::mt19937 &rng, size_t lead = 1377, size_t tail_syms_n = 8192) {
	std::vector<cf> iq(lead, cf(0, 0));
	iq.insert(iq.end(), frame_iq.begin(), frame_iq.end());
	iq.resize(iq.size() + tail_syms_n, cf(0, 0));
	if(cfo_bins_over_n != 0) {
		int n = 1 << sf_for_cfo;
		double w = 2.0 * M_PI * cfo_bins_over_n / n;
		for(size_t i = 0; i < iq.size(); i++) {
			double ph = w * (double)i;
			iq[i] *= cf((float)cos(ph), (float)sin(ph));
		}
	}
	add_noise(iq, noise_sigma, rng);

	std::vector<rx_packet> out;
	channel_receiver rx(868.1e6, sync, sf_mask, bw250, [&](const rx_packet &p) { out.push_back(p); });
	const size_t chunk = 4096;
	for(size_t i = 0; i < iq.size(); i += chunk) {
		rx.feed(iq.data() + i, std::min(chunk, iq.size() - i));
	}
	return out;
}

// oversampled frame synth at fs = os * BW with a frequency offset (cycles per capture
// sample). Instantaneous frequency wraps at Nyquist like the real chirp; the +0.5/n
// term matches chirp_mod's closed-form half-bin convention.
static std::vector<cf> synth_frame_os(const phy_params &p, const std::vector<uint16_t> &symbols, int os, double f_off_cps) {
	int n = 1 << p.sf;
	std::vector<cf> iq;
	double ph = 0;
	auto emit_up = [&](int s) {
		for(int k = 0; k < n * os; k++) {
			double pos = fmod((double)s + (double)k / os, (double)n);
			double f = (pos + 0.5) / n - 0.5;
			ph += 2.0 * M_PI * (f / os + f_off_cps);
			iq.push_back(cf((float)cos(ph), (float)sin(ph)));
		}
	};
	auto emit_down = [&](int quarter) {
		int len = quarter ? n * os / 4 : n * os;
		for(int k = 0; k < len; k++) {
			double f = 0.5 - ((double)k / os + 0.5) / n;
			ph += 2.0 * M_PI * (f / os + f_off_cps);
			iq.push_back(cf((float)cos(ph), (float)sin(ph)));
		}
	};
	for(int i = 0; i < p.preamble_len; i++) {
		emit_up(0);
	}
	emit_up(((p.sync >> 4) & 0xF) << 3);
	emit_up((p.sync & 0xF) << 3);
	emit_down(0);
	emit_down(0);
	emit_down(1);
	for(uint16_t s : symbols) {
		emit_up(s);
	}
	return iq;
}

// ---- cases ------------------------------------------------------------------------

static void test_aes_kat() {
	// FIPS-197 appendix C.1 known-answer vector: proves the homegrown AES core
	static const uint8_t KEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
	static const uint8_t PT[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
	static const uint8_t CT[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
	uint8_t out[16];
	aes128 aes(KEY);
	aes.encrypt_block(PT, out);
	check(memcmp(out, CT, 16) == 0, "aes_fips197_kat");

	// CTR must be self-inverse
	uint8_t nonce[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 0, 0, 0 };
	std::vector<uint8_t> msg = test_payload(45, 3), orig = msg;
	aes.ctr_crypt(nonce, msg.data(), msg.size());
	bool changed = msg != orig;
	aes.ctr_crypt(nonce, msg.data(), msg.size());
	check(changed && msg == orig, "aes_ctr_roundtrip");
}

static void test_header() {
	uint8_t nib[5];
	bool ok = true;
	for(int len = 1; len < 256; len += 17) {
		for(int cr = 1; cr <= 4; cr++) {
			header_build(len, cr, (len & 1) != 0, nib);
			phy_header h = header_parse(nib);
			ok &= h.valid && h.payload_len == len && h.cr == cr && h.has_crc == ((len & 1) != 0);
		}
	}
	check(ok, "header_roundtrip");
	header_build(32, 2, true, nib);
	nib[4] ^= 0x3;
	check(!header_parse(nib).valid, "header_bad_checksum_rejected");
}

static void test_phy_roundtrip() {
	struct tc {
		int sf, cr, len;
		bool ldro;
	};
	static const tc CASES[] = {
		{ 7, 1, 12, false }, { 7, 4, 1, false }, { 8, 2, 23, false }, { 9, 3, 51, false },
		{ 10, 4, 64, false }, { 11, 1, 20, true }, { 12, 4, 33, true }, { 12, 1, 8, true },
		{ 7, 4, 255, false },   // max payload at the worst symbol count (~600 syms)
	};
	for(const auto &c : CASES) {
		phy_params p;
		p.sf = c.sf;
		p.cr = c.cr;
		p.ldro = c.ldro;
		p.has_crc = true;
		std::vector<uint8_t> pl = test_payload(c.len, c.sf + c.cr);
		std::vector<uint16_t> syms = encode_frame(p, pl.data(), (int)pl.size());
		frame_decoder dec(c.sf, c.ldro);
		for(uint16_t s : syms) {
			if(!dec.feed(s)) {
				break;
			}
		}
		char name[64];
		snprintf(name, sizeof(name), "phy_roundtrip_sf%d_cr%d_len%d%s", c.sf, c.cr, c.len, c.ldro ? "_ldro" : "");
		check(dec.done() && dec.crc_ok() && dec.payload() == pl, name);
	}
}

static void test_iq_cases() {
	struct tc {
		const char *name;
		int sf, cr, len;
		float sigma;   // signal amplitude is 1.0
		float cfo;     // bins
	};
	static const tc CASES[] = {
		{ "iq_sf7_clean", 7, 1, 16, 0.05f, 0.0f },
		{ "iq_sf7_maxlen", 7, 4, 255, 0.05f, 0.0f },   // longest frame: the decode cap must not truncate it
		{ "iq_sf8_cfo", 8, 4, 24, 0.10f, 2.3f },
		{ "iq_sf9_noisy", 9, 1, 32, 0.70f, -1.4f },
		{ "iq_sf10_cr2", 10, 2, 12, 0.20f, 0.7f },
		{ "iq_sf11_ldro", 11, 1, 18, 0.10f, 1.2f },
		{ "iq_sf12_ldro", 12, 1, 8, 0.10f, -2.0f },
	};
	std::mt19937 rng(12345);
	for(const auto &c : CASES) {
		phy_params p;
		p.sf = c.sf;
		p.cr = c.cr;
		p.ldro = c.sf >= 11; // BW125 convention, must match the receiver
		p.has_crc = true;
		p.sync = 0x34;
		std::vector<uint8_t> pl = test_payload(c.len, c.sf);
		chirp_mod mod(c.sf);
		std::vector<cf> frame = mod.frame(p, encode_frame(p, pl.data(), (int)pl.size()));
		auto pkts = run_rf(frame, 0x34, 0x1F80, false, c.sigma, c.cfo, c.sf, rng);
		bool ok = pkts.size() == 1 && pkts[0].sf == c.sf && pkts[0].crc_ok && pkts[0].payload == pl;
		char detail[64];
		snprintf(detail, sizeof(detail), "%zu pkts", pkts.size());
		check(ok, c.name, ok ? "" : detail);
	}
}

static void test_iq_multiframe() {
	std::mt19937 rng(777);
	phy_params p;
	p.sf = 7;
	p.cr = 2;
	p.has_crc = true;
	p.sync = 0x34;
	std::vector<uint8_t> a = test_payload(10, 1), b = test_payload(14, 2);
	chirp_mod mod(7);
	std::vector<cf> iq = mod.frame(p, encode_frame(p, a.data(), (int)a.size()));
	iq.resize(iq.size() + 3 * 128, cf(0, 0)); // inter-frame gap
	std::vector<cf> f2 = mod.frame(p, encode_frame(p, b.data(), (int)b.size()));
	iq.insert(iq.end(), f2.begin(), f2.end());
	auto pkts = run_rf(iq, 0x34, 0x1F80, false, 0.05f, 0, 7, rng);
	check(pkts.size() == 2 && pkts[0].payload == a && pkts[1].payload == b && pkts[0].crc_ok && pkts[1].crc_ok, "iq_two_frames");
}

static void test_iq_sync_reject() {
	std::mt19937 rng(4242);
	phy_params p;
	p.sf = 8;
	p.cr = 1;
	p.has_crc = true;
	p.sync = 0x12; // private network: the 0x34 receiver must ignore it
	std::vector<uint8_t> pl = test_payload(20, 9);
	chirp_mod mod(8);
	std::vector<cf> frame = mod.frame(p, encode_frame(p, pl.data(), (int)pl.size()));
	auto pkts = run_rf(frame, 0x34, 0x1F80, false, 0.05f, 0, 8, rng);
	check(pkts.empty(), "iq_sync_reject");
}

static void test_ddc_e2e() {
	// full as-flown chain: 2 MSps capture with the channel at +300 kHz -> ddc -> receiver
	std::mt19937 rng(31337);
	phy_params p;
	p.sf = 7;
	p.cr = 1;
	p.has_crc = true;
	p.sync = 0x34;
	std::vector<uint8_t> pl = test_payload(16, 4);
	const int os = 16; // 2 MSps / 125 kHz
	std::vector<cf> cap = synth_frame_os(p, encode_frame(p, pl.data(), (int)pl.size()), os, 300e3 / 2e6);
	std::vector<cf> lead(20000, cf(0, 0));
	cap.insert(cap.begin(), lead.begin(), lead.end());
	cap.resize(cap.size() + 40000, cf(0, 0));
	add_noise(cap, 0.02f, rng);

	std::vector<rx_packet> pkts;
	channel_receiver rx(868.1e6, 0x34, 0x1F80, false, [&](const rx_packet &pk) { pkts.push_back(pk); });
	ddc dc(2e6, 125e3, 300e3);
	std::vector<cf> base;
	const size_t chunk = 65536;
	for(size_t i = 0; i < cap.size(); i += chunk) {
		base.clear();
		dc.process(cap.data() + i, std::min(chunk, cap.size() - i), base);
		rx.feed(base.data(), base.size());
	}
	bool ok = pkts.size() == 1 && pkts[0].sf == 7 && pkts[0].crc_ok && pkts[0].payload == pl;
	check(ok, "ddc_e2e_2msps_offset300k");
}

static std::vector<uint8_t> build_meshtastic_packet(const char *text, uint32_t from, uint32_t id);

static void test_us915_lorawan_e2e() {
	// US915 sub-band 1: 8 x 125 kHz channels 902.3..903.7 MHz, center 903.0 MHz, sync
	// 0x34, SF7..SF10 only. Synthesize an uplink at 2 MSps on one channel's offset and
	// run it through the full 8-channel us915 receiver bank; exactly the home channel
	// must decode it. This proves US LoRaWAN works luck-independently.
	double center = 903.0e6;
	std::vector<double> freqs;
	for(int j = 0; j < 8; j++) {
		freqs.push_back(902.3e6 + j * 0.2e6);
	}
	struct tc {
		int sf, cr, len, ch;
	};
	static const tc CASES[] = {
		{ 7, 1, 16, 5 },   // SF7 (DR3) on channel 5 = 903.3 MHz
		{ 10, 2, 12, 2 },  // SF10 (DR0) on channel 2 = 902.7 MHz
	};
	std::mt19937 rng(90210);
	for(const auto &c : CASES) {
		phy_params p;
		p.sf = c.sf;
		p.cr = c.cr;
		p.ldro = false; // BW125 SF7..SF10: no LDRO
		p.has_crc = true;
		p.sync = 0x34;
		std::vector<uint8_t> pl = test_payload(c.len, c.sf + c.ch);
		const int os = 16; // 2 MSps / 125 kHz
		double f_off = (freqs[c.ch] - center) / 2e6;
		std::vector<cf> cap = synth_frame_os(p, encode_frame(p, pl.data(), (int)pl.size()), os, f_off);
		std::vector<cf> lead(20000, cf(0, 0));
		cap.insert(cap.begin(), lead.begin(), lead.end());
		cap.resize(cap.size() + 40000, cf(0, 0));
		add_noise(cap, 0.02f, rng);

		std::vector<rx_packet> pkts;
		std::vector<std::unique_ptr<ddc>> dcs;
		std::vector<std::unique_ptr<channel_receiver>> rxs;
		for(int j = 0; j < 8; j++) {
			dcs.emplace_back(new ddc(2e6, 125e3, freqs[j] - center));
			rxs.emplace_back(new channel_receiver(freqs[j], 0x34, 0x0780, false, [&](const rx_packet &pk) { pkts.push_back(pk); }));
		}
		std::vector<cf> base;
		const size_t chunk = 65536;
		for(size_t i = 0; i < cap.size(); i += chunk) {
			size_t m = std::min(chunk, cap.size() - i);
			for(int j = 0; j < 8; j++) {
				base.clear();
				dcs[j]->process(cap.data() + i, m, base);
				rxs[j]->feed(base.data(), base.size());
			}
		}
		bool ok = pkts.size() == 1 && pkts[0].sf == c.sf && pkts[0].crc_ok && pkts[0].payload == pl &&
		          std::abs(pkts[0].freq_hz - freqs[c.ch]) < 1.0;
		char name[64], detail[64];
		snprintf(name, sizeof(name), "us915_lorawan_e2e_sf%d_ch%d", c.sf, c.ch);
		snprintf(detail, sizeof(detail), "%zu pkts", pkts.size());
		check(ok, name, ok ? "" : detail);
	}
}

static void test_us915_meshtastic_e2e() {
	// US Meshtastic LongFast: 906.875 MHz, BW250 SF11 (no LDRO below SF12), sync 0x2B.
	// Full 2 MSps -> 250 kHz DDC into the single-channel us915 receiver.
	std::mt19937 rng(60657);
	std::vector<uint8_t> pkt = build_meshtastic_packet("us mesh", 0xcafef00d, 0x00abcdef);
	phy_params p;
	p.sf = 11;
	p.cr = 1;
	p.ldro = false;
	p.has_crc = true;
	p.sync = 0x2B;
	const int os = 8; // 2 MSps / 250 kHz
	std::vector<cf> cap = synth_frame_os(p, encode_frame(p, pkt.data(), (int)pkt.size()), os, 0.0);
	std::vector<cf> lead(20000, cf(0, 0));
	cap.insert(cap.begin(), lead.begin(), lead.end());
	cap.resize(cap.size() + 40000, cf(0, 0));
	add_noise(cap, 0.02f, rng);

	std::vector<rx_packet> pkts;
	channel_receiver rx(906.875e6, 0x2B, 1u << 11, true, [&](const rx_packet &pk) { pkts.push_back(pk); });
	ddc dc(2e6, 250e3, 0.0);
	std::vector<cf> base;
	const size_t chunk = 65536;
	for(size_t i = 0; i < cap.size(); i += chunk) {
		base.clear();
		dc.process(cap.data() + i, std::min(chunk, cap.size() - i), base);
		rx.feed(base.data(), base.size());
	}
	bool ok = pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pkt;
	std::string info = ok ? meshtastic_info(pkts[0].payload) : "";
	check(ok && info.find("\"us mesh\"") != std::string::npos, "us915_meshtastic_e2e", info.c_str());
}

static void test_lorawan_parse() {
	// unconfirmed uplink: devaddr 26011f2e, ADR, fcnt 258, port 1, 4-byte payload, MIC
	std::vector<uint8_t> mac = { 0x40, 0x2e, 0x1f, 0x01, 0x26, 0x80, 0x02, 0x01, 0x01, 0xde, 0xad, 0xbe, 0xef, 0x11, 0x22, 0x33, 0x44 };
	std::string info = lorawan_info(mac);
	bool ok = info.find("UnconfUp") != std::string::npos && info.find("dev=26011f2e") != std::string::npos &&
	          info.find("fcnt=258") != std::string::npos && info.find("port=1") != std::string::npos &&
	          info.find("adr") != std::string::npos;
	check(ok, "lorawan_parse", info.c_str());

	std::vector<uint8_t> join(23, 0);
	join[0] = 0x00;
	for(int i = 0; i < 8; i++) {
		join[1 + i] = (uint8_t)(0xA0 + i); // JoinEUI LE
		join[9 + i] = (uint8_t)(0xB0 + i); // DevEUI LE
	}
	join[17] = 0x34;
	join[18] = 0x12;
	info = lorawan_info(join);
	check(info.find("JoinReq") != std::string::npos && info.find("deveui=b7b6b5b4b3b2b1b0") != std::string::npos &&
	      info.find("nonce=1234") != std::string::npos, "lorawan_parse_join", info.c_str());
}

// build a Meshtastic radio packet: 16-byte header + default-PSK-encrypted Data proto
static std::vector<uint8_t> build_meshtastic_packet(const char *text, uint32_t from, uint32_t id) {
	static const uint8_t PSK[16] = {
		0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
	};
	std::vector<uint8_t> pkt = {
		0xff, 0xff, 0xff, 0xff, // dest: broadcast
		(uint8_t)(from), (uint8_t)(from >> 8), (uint8_t)(from >> 16), (uint8_t)(from >> 24),
		(uint8_t)(id), (uint8_t)(id >> 8), (uint8_t)(id >> 16), (uint8_t)(id >> 24),
		0x63, // flags: hop_limit 3, hop_start 3
		0x08, // channel hash
		0x00, 0x00,
	};
	std::vector<uint8_t> proto = { 0x08, 0x01, 0x12, (uint8_t)strlen(text) }; // Data{portnum:1, payload:text}
	proto.insert(proto.end(), text, text + strlen(text));
	uint8_t nonce[16];
	memset(nonce, 0, sizeof(nonce));
	memcpy(nonce, &pkt[8], 4);
	memcpy(nonce + 8, &pkt[4], 4);
	aes128 aes(PSK);
	aes.ctr_crypt(nonce, proto.data(), proto.size());
	pkt.insert(pkt.end(), proto.begin(), proto.end());
	return pkt;
}

static void test_meshtastic_parse() {
	std::vector<uint8_t> pkt = build_meshtastic_packet("hi lorad", 0xa1b2c3d4, 0x12345678);
	std::string info = meshtastic_info(pkt);
	bool ok = info.find("!a1b2c3d4") != std::string::npos && info.find("^all") != std::string::npos &&
	          info.find("\"hi lorad\"") != std::string::npos;
	check(ok, "meshtastic_parse", info.c_str());
}

// build a Meshtastic POSITION frame: 16-byte header + default-PSK-encrypted Data{port=3,
// payload=Position{latitude_i, longitude_i, altitude}} and check the structured decode.
static void test_meshtastic_position() {
	static const uint8_t PSK[16] = {
		0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
	};
	int32_t lat_i = (int32_t)llround(40.6413 * 1e7), lon_i = (int32_t)llround(-73.7781 * 1e7);
	std::vector<uint8_t> pos = { 0x0d };
	for(int i = 0; i < 4; i++) { pos.push_back((uint8_t)(lat_i >> (8 * i))); }
	pos.push_back(0x15);
	for(int i = 0; i < 4; i++) { pos.push_back((uint8_t)(lon_i >> (8 * i))); }
	pos.push_back(0x18);
	pos.push_back(42); // altitude 42 m
	std::vector<uint8_t> proto = { 0x08, 0x03, 0x12, (uint8_t)pos.size() };
	proto.insert(proto.end(), pos.begin(), pos.end());
	std::vector<uint8_t> pkt = {
		0xff, 0xff, 0xff, 0xff, 0x7e, 0x1e, 0x50, 0x0b, 0x33, 0x03, 0x00, 0x00, 0x63, 0x08, 0x00, 0x00,
	};
	uint8_t nonce[16];
	memset(nonce, 0, sizeof(nonce));
	memcpy(nonce, &pkt[8], 4);
	memcpy(nonce + 8, &pkt[4], 4);
	aes128 aes(PSK);
	aes.ctr_crypt(nonce, proto.data(), proto.size());
	pkt.insert(pkt.end(), proto.begin(), proto.end());

	mesh_fields mf = meshtastic_decode(pkt);
	bool ok = mf.valid && mf.decoded && mf.port == 3 && mf.portname == "POSITION" && mf.has_pos &&
	          std::fabs(mf.lat - 40.6413) < 1e-5 && std::fabs(mf.lon + 73.7781) < 1e-5 &&
	          mf.has_alt && mf.alt == 42 && mesh_node_string(mf.from) == "!0b501e7e";
	char d[96];
	snprintf(d, sizeof(d), "lat=%.4f lon=%.4f alt=%d", mf.lat, mf.lon, mf.alt);
	check(ok, "meshtastic_position", d);
}

static void test_meshtastic_e2e() {
	// LongFast preset as flown: BW250 SF11 (no LDRO below SF12), sync 0x2B, CR 4/5
	std::mt19937 rng(2026);
	std::vector<uint8_t> pkt = build_meshtastic_packet("mesh e2e", 0xdeadbeef, 0x00c0ffee);
	phy_params p;
	p.sf = 11;
	p.cr = 1;
	p.ldro = false;
	p.has_crc = true;
	p.sync = 0x2B;
	chirp_mod mod(11);
	std::vector<cf> frame = mod.frame(p, encode_frame(p, pkt.data(), (int)pkt.size()));
	auto pkts = run_rf(frame, 0x2B, 1u << 11, true, 0.1f, 0.8f, 11, rng);
	bool ok = pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pkt;
	std::string info = ok ? meshtastic_info(pkts[0].payload) : "";
	check(ok && info.find("\"mesh e2e\"") != std::string::npos, "meshtastic_e2e", info.c_str());
}

int run_selftest() {
	printf("lorad selftest\n");
	test_aes_kat();
	test_header();
	test_phy_roundtrip();
	test_iq_cases();
	test_iq_multiframe();
	test_iq_sync_reject();
	test_ddc_e2e();
	test_us915_lorawan_e2e();
	test_us915_meshtastic_e2e();
	test_lorawan_parse();
	test_meshtastic_parse();
	test_meshtastic_position();
	test_meshtastic_e2e();
	printf(g_fails ? "SELFTEST FAILED: %d case(s)\n" : "SELFTEST OK\n", g_fails);
	return g_fails ? 1 : 0;
}

} // namespace lorad

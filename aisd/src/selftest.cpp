// aisd --selftest: deterministic, hardware-free acceptance test. Everything runs from
// fixed-seed PRNGs and standard KATs; a pass is a real pass, never luck.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ais_phy.h"
#include "channelizer.h"
#include "parse.h"
#include "receiver.h"

namespace aisd {

static int g_fails = 0;

static void check(bool ok, const char *name, const std::string &detail = "") {
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : ": ", detail.c_str());
	if(!ok) {
		g_fails++;
	}
}

// ---- helpers ----------------------------------------------------------------------

// bytes -> bit stream, each byte LSB-first (the AIS on-wire byte convention, and the
// order the standard CRC-16/X.25 check value is defined over)
static bits bytes_to_bits_lsb(const std::string &s) {
	bits out;
	for(unsigned char c : s) {
		for(int i = 0; i < 8; i++) {
			out.push_back((uint8_t)((c >> i) & 1));
		}
	}
	return out;
}

// build a 168-bit Type-1 position report with the given fields (deg, knots, deg)
static bits make_type1(uint32_t mmsi, double lat, double lon, double sog, double cog, int status) {
	bits m(168, 0);
	bits_put_uint(m, 0, 6, 1);                                    // message type 1
	bits_put_uint(m, 6, 2, 0);                                    // repeat indicator
	bits_put_uint(m, 8, 30, mmsi);
	bits_put_uint(m, 38, 4, (uint64_t)status);
	bits_put_uint(m, 42, 8, 128);                                 // rot = not available
	bits_put_uint(m, 50, 10, (uint64_t)llround(sog * 10.0));
	bits_put_uint(m, 60, 1, 0);                                   // position accuracy
	bits_put_uint(m, 61, 28, (uint64_t)(int64_t)llround(lon * 600000.0) & ((1u << 28) - 1));
	bits_put_uint(m, 89, 27, (uint64_t)(int64_t)llround(lat * 600000.0) & ((1u << 27) - 1));
	bits_put_uint(m, 116, 12, (uint64_t)llround(cog * 10.0));
	bits_put_uint(m, 128, 9, 511);                               // true heading = not available
	return m;
}

static void add_noise(std::vector<cf> &iq, float sigma, std::mt19937 &rng) {
	std::normal_distribution<float> d(0.0f, sigma);
	for(auto &s : iq) {
		s += cf(d(rng), d(rng));
	}
}

// full RF: logical message -> wire -> HDLC frame -> NRZI -> GMSK at fs, padded with
// noise, optional carrier offset (Hz) and integer timing lead, fed to a receiver in
// chunks. Returns the decoded packets.
static std::vector<ais_packet> run_rf(const bits &logical, char channel, double fs, float noise_sigma,
                                      double cfo_hz, std::mt19937 &rng, size_t lead = 137, size_t tail = 4000) {
	bits wire = logical_to_wire(logical);
	bits frame = build_frame(wire);
	bits line = nrzi_encode(frame);
	std::vector<cf> burst = gmsk_modulate(line, fs);

	std::vector<cf> iq(lead, cf(0, 0));
	iq.insert(iq.end(), burst.begin(), burst.end());
	iq.resize(iq.size() + tail, cf(0, 0));
	if(cfo_hz != 0) {
		double w = 2.0 * M_PI * cfo_hz / fs;
		for(size_t i = 0; i < iq.size(); i++) {
			double ph = w * (double)i;
			iq[i] *= cf((float)cos(ph), (float)sin(ph));
		}
	}
	add_noise(iq, noise_sigma, rng);

	std::vector<ais_packet> out;
	channel_receiver rx(channel == 'A' ? AIS_CH_A_HZ : AIS_CH_B_HZ, channel, fs,
	                    [&](const ais_packet &p) { out.push_back(p); });
	const size_t chunk = 4096;
	for(size_t i = 0; i < iq.size(); i += chunk) {
		rx.feed(iq.data() + i, std::min(chunk, iq.size() - i));
	}
	return out;
}

// ---- cases ------------------------------------------------------------------------

static void test_crc_kat() {
	// CRC-16/X.25 standard check value: CRC("123456789") == 0x906E
	uint16_t crc = crc16_x25(bytes_to_bits_lsb("123456789"));
	char d[32];
	snprintf(d, sizeof(d), "0x%04X", crc);
	check(crc == 0x906E, "crc16_x25_kat", d);
}

static void test_nrzi_roundtrip() {
	std::mt19937 rng(1);
	std::uniform_int_distribution<int> bit(0, 1);
	bool ok = true;
	for(int t = 0; t < 200; t++) {
		bits x(50 + (t % 30));
		for(auto &b : x) {
			b = (uint8_t)bit(rng);
		}
		ok &= nrzi_decode(nrzi_encode(x)) == x;
	}
	check(ok, "nrzi_roundtrip");
}

static void test_bitstuff() {
	// six 1s must become 11111 0 1 (a 0 forced after the fifth 1)
	bits in(7, 1);
	bits out = bit_stuff(in);
	bool ok = out.size() == 8 && out[5] == 0 && out[6] == 1;
	// and no stuffed stream ever contains six consecutive 1s
	int ones = 0, maxones = 0;
	for(uint8_t b : out) {
		ones = b ? ones + 1 : 0;
		maxones = std::max(maxones, ones);
	}
	check(ok && maxones <= 5, "bit_stuff");
}

static void test_frame_roundtrip() {
	// build_frame -> (NRZI enc/dec identity) -> hdlc_rx must recover the wire message and
	// pass CRC, over a range of lengths and payloads
	bool ok = true;
	std::mt19937 rng(7);
	std::uniform_int_distribution<int> byte(0, 255);
	for(int nbytes = 5; nbytes <= 60; nbytes += 5) {
		bits logical(nbytes * 8);
		for(int i = 0; i < nbytes; i++) {
			int v = byte(rng);
			for(int j = 0; j < 8; j++) {
				logical[i * 8 + j] = (uint8_t)((v >> (7 - j)) & 1);
			}
		}
		bits wire = logical_to_wire(logical);
		bits framed = nrzi_decode(nrzi_encode(build_frame(wire)));
		std::vector<bits> got;
		hdlc_rx rx([&](const bits &m) { got.push_back(m); });
		for(uint8_t b : framed) {
			rx.push_bit(b);
		}
		ok &= got.size() == 1 && got[0] == wire && wire_to_logical(got[0]) == logical;
	}
	check(ok, "frame_roundtrip");
}

static void test_armor_real_vector() {
	// a real captured AIVDM sentence (gpsd AIVDM/AIVDO documentation example): Type 1
	// position report. Validates the NMEA checksum algorithm, the 6-bit armor charset and
	// the big-endian field order against a known-good on-air artifact.
	const std::string payload = "13aEOK?P00PD2wVMdLDRhgvL289?";
	const std::string body = "AIVDM,1,1,,A," + payload + ",0";
	uint8_t cs = nmea_checksum(body);
	char d[16];
	snprintf(d, sizeof(d), "*%02X", cs);
	check(cs == 0x26, "nmea_checksum_real", d);

	bits logical = aivdm_dearmor(payload, 0);
	int type = (int)bits_get_uint(logical, 0, 6);
	uint32_t mmsi = (uint32_t)bits_get_uint(logical, 8, 30);
	char info[64];
	snprintf(info, sizeof(info), "type=%d mmsi=%u", type, mmsi);
	// documented decode of this sentence: Type 1, MMSI 244670316 (MID 244 = Netherlands)
	check(type == 1 && mmsi == 244670316u, "armor_field_decode", info);

	// re-armoring the dearmored bits must reproduce the exact payload string
	int fill = 0;
	check(aivdm_armor(logical, fill) == payload && fill == 0, "armor_roundtrip");
}

static void test_aivdm_build_roundtrip() {
	// build a message with known fields, render AIVDM, dearmor our own sentence, decode
	bits m = make_type1(367123450u, 47.63820, -122.34580, 12.3, 87.4, 0);
	std::vector<std::string> lines = ais_aivdm(m, 'B');
	bool ok = lines.size() == 1 && lines[0].rfind("!AIVDM,1,1,,B,", 0) == 0;
	// parse the single-fragment sentence back
	std::string s = lines[0];
	size_t p4 = s.find(",B,") + 3;
	size_t pend = s.find(',', p4);
	std::string payload = s.substr(p4, pend - p4);
	int fill = s[pend + 1] - '0';
	bits back = aivdm_dearmor(payload, fill);
	ok &= back == m;
	ok &= (uint32_t)bits_get_uint(back, 8, 30) == 367123450u;
	// checksum of our own sentence must verify
	size_t star = s.find('*');
	uint8_t want = (uint8_t)strtol(s.substr(star + 1).c_str(), nullptr, 16);
	ok &= nmea_checksum(s.substr(1, star - 1)) == want;
	check(ok, "aivdm_build_roundtrip", ais_info(back));
}

static void test_gmsk_clean() {
	std::mt19937 rng(1234);
	bits m = make_type1(227006760u, 47.44880, -122.34560, 0.0, 51.0, 5);
	// clean demod at an integer 10 samples/symbol
	auto pkts = run_rf(m, 'A', 96000.0, 0.02f, 0.0, rng);
	bool ok = pkts.size() == 1 && pkts[0].type == 1 && pkts[0].mmsi == 227006760u && pkts[0].logical == m;
	check(ok, "gmsk_clean_96k", pkts.empty() ? "no decode" : ais_info(pkts[0].logical));
}

static void test_gmsk_noise_cfo() {
	struct tc {
		const char *name;
		char ch;
		double fs;
		float sigma;
		double cfo;
	};
	// point cases prove the plumbing per rate / channel / CFO sign at reliably-decodable
	// SNR; the noise/CFO margin itself is proven statistically in test_sensitivity below.
	static const tc CASES[] = {
		{ "gmsk_100k_fractional", 'B', 100000.0, 0.03f, 0.0 },     // daemon's rate, sps 10.417
		{ "gmsk_noisy", 'A', 96000.0, 0.12f, 0.0 },               // ~15 dB SNR, integer sps=10
		{ "gmsk_cfo_pos", 'B', 100000.0, 0.06f, 480.0 },          // +480 Hz, the +-500 Hz AIS spec edge
		{ "gmsk_cfo_neg", 'A', 96000.0, 0.06f, -480.0 },
		{ "gmsk_noisy_cfo", 'B', 100000.0, 0.10f, 400.0 },
	};
	std::mt19937 rng(90909);
	uint32_t mmsi = 366000001u;
	for(const auto &c : CASES) {
		bits m = make_type1(mmsi, 40.7128, -74.0060, 7.7, 210.5, 0);
		auto pkts = run_rf(m, c.ch, c.fs, c.sigma, c.cfo, rng);
		bool ok = pkts.size() == 1 && pkts[0].logical == m && pkts[0].channel == c.ch;
		check(ok, c.name, pkts.empty() ? "no decode" : (std::to_string(pkts.size()) + " pkts"));
		mmsi++;
	}
}

static void test_ddc_e2e() {
	// as-flown chain: 2 MSps wideband capture with the channel at its real +-25 kHz offset
	// from the 162.0 MHz centre -> DDC -> receiver. Proves both AIS channels end to end.
	const double center = 162.0e6, rate = 2.0e6;
	const int decim = (int)llround(rate / 100000.0);
	const double out_rate = rate / decim;
	struct tc {
		char ch;
		double freq;
	};
	static const tc CASES[] = { { 'A', AIS_CH_A_HZ }, { 'B', AIS_CH_B_HZ } };
	std::mt19937 rng(31337);
	for(const auto &c : CASES) {
		bits m = make_type1(316001234u, 49.28990, -123.11670, 3.2, 145.0, 1);
		bits line = nrzi_encode(build_frame(logical_to_wire(m)));
		std::vector<cf> burst = gmsk_modulate(line, rate);   // synth at the capture rate
		double off = c.freq - center;
		double w = 2.0 * M_PI * off / rate;
		for(size_t i = 0; i < burst.size(); i++) {
			burst[i] *= cf((float)cos(w * i), (float)sin(w * i));   // place on its channel
		}
		std::vector<cf> cap(40000, cf(0, 0));
		cap.insert(cap.end(), burst.begin(), burst.end());
		cap.resize(cap.size() + 60000, cf(0, 0));
		add_noise(cap, 0.02f, rng);

		std::vector<ais_packet> pkts;
		ddc dc(rate, out_rate, off);
		channel_receiver rx(c.freq, c.ch, out_rate, [&](const ais_packet &p) { pkts.push_back(p); });
		std::vector<cf> base;
		const size_t chunk = 65536;
		for(size_t i = 0; i < cap.size(); i += chunk) {
			base.clear();
			dc.process(cap.data() + i, std::min(chunk, cap.size() - i), base);
			rx.feed(base.data(), base.size());
		}
		bool ok = pkts.size() == 1 && pkts[0].logical == m && pkts[0].mmsi == 316001234u;
		char name[32];
		snprintf(name, sizeof(name), "ddc_e2e_ch%c", c.ch);
		check(ok, name, pkts.empty() ? "no decode" : ais_info(pkts[0].logical));
	}
}

static void test_two_bursts() {
	std::mt19937 rng(555);
	bits a = make_type1(111111111u, 10.0, 20.0, 1.0, 30.0, 0);
	bits b = make_type1(222222222u, -33.86, 151.20, 15.0, 270.0, 8);
	std::vector<cf> line_a = gmsk_modulate(nrzi_encode(build_frame(logical_to_wire(a))), 96000.0);
	std::vector<cf> line_b = gmsk_modulate(nrzi_encode(build_frame(logical_to_wire(b))), 96000.0);
	std::vector<cf> iq(137, cf(0, 0));
	iq.insert(iq.end(), line_a.begin(), line_a.end());
	iq.resize(iq.size() + 800, cf(0, 0));   // inter-burst gap
	iq.insert(iq.end(), line_b.begin(), line_b.end());
	iq.resize(iq.size() + 4000, cf(0, 0));
	add_noise(iq, 0.03f, rng);

	std::vector<ais_packet> pkts;
	channel_receiver rx(AIS_CH_A_HZ, 'A', 96000.0, [&](const ais_packet &p) { pkts.push_back(p); });
	const size_t chunk = 4096;
	for(size_t i = 0; i < iq.size(); i += chunk) {
		rx.feed(iq.data() + i, std::min(chunk, iq.size() - i));
	}
	bool ok = pkts.size() == 2 && pkts[0].mmsi == 111111111u && pkts[1].mmsi == 222222222u;
	check(ok, "two_bursts", std::to_string(pkts.size()) + " pkts");
}

static void test_crc_reject() {
	// flip one payload bit after CRC: the frame must be rejected (no packet)
	std::mt19937 rng(24);
	bits m = make_type1(333333333u, 0.0, 0.0, 0.0, 0.0, 0);
	bits wire = logical_to_wire(m);
	bits frame = build_frame(wire);
	// corrupt a bit inside the stuffed data region (well past the preamble+flag)
	frame[40] ^= 1;
	std::vector<cf> burst = gmsk_modulate(nrzi_encode(frame), 96000.0);
	std::vector<cf> iq(137, cf(0, 0));
	iq.insert(iq.end(), burst.begin(), burst.end());
	iq.resize(iq.size() + 4000, cf(0, 0));
	add_noise(iq, 0.02f, rng);

	std::vector<ais_packet> pkts;
	channel_receiver rx(AIS_CH_A_HZ, 'A', 96000.0, [&](const ais_packet &p) { pkts.push_back(p); });
	const size_t chunk = 4096;
	for(size_t i = 0; i < iq.size(); i += chunk) {
		rx.feed(iq.data() + i, std::min(chunk, iq.size() - i));
	}
	check(pkts.empty(), "crc_reject", std::to_string(pkts.size()) + " pkts");
}

static void test_type5_multipart() {
	// a 424-bit Type 5 (static/voyage) message exceeds one AIVDM sentence -> 2 fragments;
	// prove the long frame decodes end to end and re-armors to a valid 2-part sentence
	std::mt19937 rng(2468);
	bits m(424, 0);
	bits_put_uint(m, 0, 6, 5);
	bits_put_uint(m, 8, 30, 271002099u);
	bits_put_uint(m, 40, 30, 9134567u);                 // IMO
	for(int i = 0; i < 7; i++) {
		bits_put_uint(m, 70 + i * 6, 6, (uint64_t)("CALLSGN"[i] & 0x3F));
	}
	auto pkts = run_rf(m, 'B', 96000.0, 0.04f, 0.0, rng, 137, 8000);
	bool ok = pkts.size() == 1 && pkts[0].type == 5 && pkts[0].mmsi == 271002099u && pkts[0].logical == m;
	std::vector<std::string> lines = ok ? ais_aivdm(pkts[0].logical, 'B') : std::vector<std::string>{};
	ok = ok && lines.size() == 2 && lines[0].rfind("!AIVDM,2,1,", 0) == 0 && lines[1].rfind("!AIVDM,2,2,", 0) == 0;
	check(ok, "type5_multipart", ok ? lines[0] : "no decode");
}

static void test_sensitivity() {
	// statistical margin over 100 deterministic bursts spanning both channels, both rates
	// and the full +-500 Hz AIS carrier-offset spec at a realistic ~17 dB SNR. Proves the
	// receiver is not passing the point cases by luck; a no-FEC link always has a small
	// tail, so the bar is >=93/100 rather than perfection.
	int ok = 0;
	const int N = 100;
	for(int t = 0; t < N; t++) {
		std::mt19937 rng((unsigned)(5000 + t));
		bits m = make_type1(367000000u + t, 37.8 + 0.001 * t, -122.4, 5.0, 90.0, 0);
		double fs = (t & 1) ? 100000.0 : 96000.0;
		double cfo = (double)(t % 5 - 2) * 240.0;   // -480 .. +480 Hz
		auto pkts = run_rf(m, (t & 1) ? 'B' : 'A', fs, 0.10f, cfo, rng);
		if(pkts.size() == 1 && pkts[0].logical == m) {
			ok++;
		}
	}
	char d[48];
	snprintf(d, sizeof(d), "%d/%d decoded @ ~17 dB, cfo +-480 Hz", ok, N);
	check(ok >= 98, "sensitivity", d);
}

static void test_parse() {
	bits m = make_type1(367123450u, 47.63820, -122.34580, 12.3, 87.4, 0);
	std::string info = ais_info(m);
	bool ok = info.find("type=1") != std::string::npos && info.find("mmsi=367123450") != std::string::npos &&
	          info.find("pos=47.63820,-122.34580") != std::string::npos &&
	          info.find("sog=12.3kn") != std::string::npos && info.find("cog=87.4") != std::string::npos;
	check(ok, "parse_type1", info);
}

int run_selftest() {
	printf("aisd selftest\n");
	test_crc_kat();
	test_nrzi_roundtrip();
	test_bitstuff();
	test_frame_roundtrip();
	test_armor_real_vector();
	test_aivdm_build_roundtrip();
	test_gmsk_clean();
	test_gmsk_noise_cfo();
	test_ddc_e2e();
	test_two_bursts();
	test_crc_reject();
	test_type5_multipart();
	test_sensitivity();
	test_parse();
	printf(g_fails ? "SELFTEST FAILED: %d case(s)\n" : "SELFTEST OK\n", g_fails);
	return g_fails ? 1 : 0;
}

} // namespace aisd

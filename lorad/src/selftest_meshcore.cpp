// lorad --selftest, MeshCore part: crypto known answers, real over-the-air packets, and
// synthesized IQ through the daemon's exact chain at MeshCore parameters.
//
// The packets and their expected values come from the test suite of the independent
// MIT-licensed decoder github.com/michaelhart/meshcore-decoder (tests/advert.test.ts,
// grouptext-decryption.test.ts, packet-structure.test.ts, region-transport.test.ts). The
// MeshCore repository itself ships no packet vectors.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <random>
#include <string>
#include <vector>

#include "aes128.h"
#include "channelizer.h"
#include "lora_phy.h"
#include "meshcore.h"
#include "meshcore_tx.h"
#include "meshtastic_tx.h"
#include "packets.h"
#include "parse.h"
#include "receiver.h"
#include "sha256.h"

namespace lorad {

static void (*g_check)(bool, const char *, const char *);

static void check(bool ok, const char *name, const char *detail = "") {
	g_check(ok, name, detail);
}

static const char *MC_ADVERT =
	"11007E7662676F7F0850A8A355BAAFBFC1EB7B4174C340442D7D7161C9474A2C94006CE7CF682E58408DD8FCC51906ECA98EBF94A037886BDADE"
	"7ECD09FD92B839491DF3809C9454F5286D1D3370AC31A34593D569E9A042A3B41FD331DFFB7E18599CE1E60992A076D50238C5B8F85757375354"
	"522F50756765744D65736820436F75676172";
static const char *MC_PUBLIC = "150011C3C1354D619BAE9590E4D177DB7EEAF982F5BDCF78005D75157D9535FA90178F785D";
static const char *MC_OTHER_CHANNEL = "15833fa002860ccae0eed9ca78b9ab0775d477c1f6490a398bf4edc75240"; // #bot, 3-byte hashes
static const char *MC_TWO_BYTE = "1540cab3b15626481a5ba64247ab25766e410b026e0678a32da9f0c3946fae5b714cab170f";
static const char *MC_TEXT_MSG = "09046F17C47ED00A13E16AB5B94B1CC2D1A5059C6E5A6253C60D";
static const char *MC_RESPONSE = "0600DE1FDFCAD56E6C38B756FEE81C24199C6043AC5B";
static const char *MC_TRANSPORT =
	"14FA1A0000034E927D596EA23622BCB4D5945E49348165AF7DABA3F5DCEED85F430E0856DB5B591E86AB3363BC00E1BA30776698F72FC57C71"
	"68E66A4875CDB710F3C175FC2B3FE75A036EF14FA59A709062D3A9FF7014F2E7A8512C";

static std::vector<uint8_t> from_hex(const char *h) {
	std::vector<uint8_t> v;
	for(size_t i = 0; h[i] && h[i + 1]; i += 2) {
		char b[3] = { h[i], h[i + 1], 0 };
		v.push_back((uint8_t)strtol(b, nullptr, 16));
	}
	return v;
}

static std::string digest_hex(const uint8_t *d) {
	return hex_string(std::vector<uint8_t>(d, d + 32));
}

static void add_noise(std::vector<cf> &iq, float sigma, std::mt19937 &rng) {
	std::normal_distribution<float> d(0.0f, sigma);
	for(auto &s : iq) {
		s += cf(d(rng), d(rng));
	}
}

static void test_crypto_kat() {
	// FIPS-197 C.1 through the inverse cipher
	static const uint8_t KEY[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
	static const uint8_t PT[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
	static const uint8_t CT[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
	uint8_t out[16];
	aes128 aes(KEY);
	aes.decrypt_block(CT, out);
	check(memcmp(out, PT, 16) == 0, "aes_fips197_inverse_kat");

	// FIPS 180-4 examples: one and two blocks
	uint8_t d[32];
	sha256 a;
	a.update((const uint8_t *)"abc", 3);
	a.finish(d);
	check(digest_hex(d) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "sha256_abc_kat");
	const char *m2 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
	sha256 b;
	b.update((const uint8_t *)m2, strlen(m2));
	b.finish(d);
	check(digest_hex(d) == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", "sha256_two_block_kat");

	// RFC 4231 test case 2
	const char *msg = "what do ya want for nothing?";
	hmac_sha256((const uint8_t *)"Jefe", 4, (const uint8_t *)msg, strlen(msg), d);
	check(digest_hex(d) == "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", "hmac_sha256_rfc4231_kat");
}

static void test_vectors() {
	{
		meshcore_fields f = meshcore_decode(from_hex(MC_ADVERT));
		bool ok = f.valid && f.type == 4 && f.route == 1 && f.hops == 0 && f.has_advert && f.node_type == 2 &&
		          f.pubkey == "7e7662676f7f" && f.adv_time == 1758455660 && f.name == "WW7STR/PugetMesh Cougar" &&
		          f.has_pos && std::fabs(f.lat - 47.543968) < 1e-6 && std::fabs(f.lon + 122.108616) < 1e-6 && !f.truncated;
		check(ok, "meshcore_real_advert", meshcore_info(f).c_str());
	}
	{
		meshcore_fields f = meshcore_decode(from_hex(MC_PUBLIC));
		bool ok = f.valid && f.type == 5 && f.route == 1 && f.channel == 0x11 && f.public_channel && f.has_text &&
		          f.msg_time == 1758484279 && f.sender == "\xf0\x9f\x8c\xb2 Tree" && f.text == "\xe2\x98\x81\xef\xb8\x8f" && !f.encrypted;
		check(ok, "meshcore_real_public_channel", meshcore_info(f).c_str());
	}
	{
		// another channel's key: the header decodes, the content must stay ciphertext
		meshcore_fields f = meshcore_decode(from_hex(MC_OTHER_CHANNEL));
		bool ok = f.valid && f.type == 5 && f.hash_size == 3 && f.hops == 3 && f.path.size() == 3 &&
		          f.path[0] == "3fa002" && f.path[2] == "e0eed9" && f.encrypted && !f.has_text && !f.public_channel;
		check(ok, "meshcore_real_other_channel_stays_encrypted", meshcore_info(f).c_str());
		meshcore_fields g = meshcore_decode(from_hex(MC_TWO_BYTE));
		check(g.valid && g.hash_size == 2 && g.hops == 0 && g.channel == 0xca && g.encrypted, "meshcore_real_two_byte_hash_mode");
	}
	{
		// private message and response: routing metadata only
		meshcore_fields f = meshcore_decode(from_hex(MC_TEXT_MSG));
		bool ok = f.valid && f.type == 2 && f.route == 1 && f.hops == 4 && f.path[0] == "6f" && f.path[3] == "7e" &&
		          f.dst == 0xd0 && f.src == 0x0a && f.encrypted && !f.has_text;
		check(ok, "meshcore_real_private_message_stays_encrypted", meshcore_info(f).c_str());
		meshcore_fields g = meshcore_decode(from_hex(MC_RESPONSE));
		check(g.valid && g.type == 1 && g.route == 2 && g.dst == 0xde && g.src == 0x1f && g.encrypted, "meshcore_real_response");
	}
	{
		meshcore_fields f = meshcore_decode(from_hex(MC_TRANSPORT));
		bool ok = f.valid && f.route == 0 && f.has_transport && f.transport[0] == 0x1afa && f.transport[1] == 0 &&
		          f.type == 5 && f.hops == 3 && f.path[0] == "4e" && f.channel == 0x59 && f.encrypted;
		check(ok, "meshcore_real_transport_flood", meshcore_info(f).c_str());
	}
	{
		// the packet hash ignores the path, so relayed copies of one flood compare equal
		std::vector<uint8_t> a = from_hex(MC_PUBLIC), b = a;
		b[1] = 2;
		b.insert(b.begin() + 2, { 0xaa, 0xbb });
		meshcore_fields fa = meshcore_decode(a), fb = meshcore_decode(b);
		check(fa.hash.size() == 16 && fa.hash == fb.hash && fb.hops == 2 && fb.has_text, "meshcore_hash_ignores_path", fa.hash.c_str());
	}
}

static void test_malformed() {
	check(!meshcore_decode(from_hex("15C1FF00")).valid, "meshcore_reserved_hash_size_rejected");
	check(!meshcore_decode(from_hex("11")).valid && !meshcore_decode({}).valid, "meshcore_too_short_rejected");
	check(!meshcore_decode(from_hex("1100")).valid, "meshcore_empty_payload_rejected");
	std::vector<uint8_t> big = { 0x11, 0xbf }; // 63 hops x 3 bytes > MAX_PATH_SIZE
	big.resize(250, 0x55);
	check(!meshcore_decode(big).valid, "meshcore_oversize_path_rejected");
	std::vector<uint8_t> bigpl = { 0x11, 0x00 };
	bigpl.resize(2 + 185, 0x55);
	check(!meshcore_decode(bigpl).valid, "meshcore_oversize_payload_rejected");

	std::vector<uint8_t> adv = from_hex(MC_ADVERT);
	adv.resize(2 + 50);
	meshcore_fields f = meshcore_decode(adv);
	check(f.valid && f.truncated && !f.has_advert, "meshcore_truncated_advert");
	adv = from_hex(MC_ADVERT);
	adv.resize(2 + 32 + 4 + 64 + 5); // the flags byte promises lat/lon, the frame ends inside them
	f = meshcore_decode(adv);
	check(f.valid && f.truncated && !f.has_pos && f.name.empty(), "meshcore_advert_flags_past_end");

	// every prefix of every vector walks all bounds checks
	bool ok = true;
	for(const char *h : { MC_ADVERT, MC_PUBLIC, MC_OTHER_CHANNEL, MC_TEXT_MSG, MC_TRANSPORT }) {
		std::vector<uint8_t> v = from_hex(h);
		for(size_t n = 0; n <= v.size(); n++) {
			std::vector<uint8_t> cut(v.begin(), v.begin() + (long)n);
			ok &= meshcore_info(meshcore_decode(cut)).size() < 1024;
		}
	}
	check(ok, "meshcore_all_prefixes_safe");

	// a flipped ciphertext bit fails the MAC: no text, and it stays "encrypted"
	std::vector<uint8_t> bad = from_hex(MC_PUBLIC);
	bad[10] ^= 0x01;
	f = meshcore_decode(bad);
	check(f.valid && f.channel == 0x11 && !f.public_channel && !f.has_text && f.encrypted, "meshcore_bad_mac_not_decrypted");

	// an unknown payload version is reported, its payload is not interpreted
	std::vector<uint8_t> ver = from_hex(MC_ADVERT);
	ver[0] |= 0x40;
	f = meshcore_decode(ver);
	check(f.valid && f.version == 1 && !f.has_advert, "meshcore_future_version_header_only");
}

static void test_json() {
	packet_ring ring;
	std::vector<uint8_t> adv = from_hex(MC_ADVERT);
	packet_entry e;
	e.time = "00:00:00";
	e.hex = hex_string(adv);
	meshcore_fill(e, adv, true);
	ring.push(e);
	packet_entry b;
	b.time = "00:00:01";
	meshcore_fill(b, adv, false);
	ring.push(b);
	std::string js = ring.to_json();
	bool ok = js.find("\"meshcore\":{\"type\":\"ADVERT\"") != std::string::npos &&
	          js.find("\"name\":\"WW7STR/PugetMesh Cougar\"") != std::string::npos &&
	          js.find("\"pubkey\":\"7e7662676f7f\"") != std::string::npos &&
	          js.find("\"lat\":47.543968,\"lon\":-122.108616") != std::string::npos &&
	          js.find("\"role\":\"repeater\"") != std::string::npos;
	check(ok, "meshcore_json_fields");
	// the CRC-failed copy must not add a second structured object
	size_t first = js.find("\"meshcore\"");
	check(first != std::string::npos && js.find("\"meshcore\"", first + 1) == std::string::npos, "meshcore_crc_bad_gets_no_structured_decode");
}

// frame synth at fs = os * BW for a transmitter whose crystal runs clk_ppm fast: the chirps
// run on its time base, and f_off_cps carries the matching carrier offset. Phase is
// continuous from symbol to symbol, as an SX126x emits.
static std::vector<cf> synth_frame_clk(const phy_params &p, const std::vector<uint16_t> &symbols, int os, double f_off_cps, double clk_ppm) {
	const int n = 1 << p.sf;
	const double step = (1.0 + clk_ppm * 1e-6) / os; // TX chips per capture sample
	std::vector<cf> iq;
	double ph = 0, t = 0;
	auto emit = [&](double chips, bool down, int s) {
		while(t < chips) {
			double f = down ? 0.5 - (t + 0.5) / n : fmod((double)s + t + 0.5, (double)n) / n - 0.5;
			ph += 2.0 * M_PI * (f * step + f_off_cps);
			iq.push_back(cf((float)cos(ph), (float)sin(ph)));
			t += step;
		}
		t -= chips;
	};
	for(int i = 0; i < p.preamble_len; i++) {
		emit(n, false, 0);
	}
	emit(n, false, ((p.sync >> 4) & 0xF) << 3);
	emit(n, false, (p.sync & 0xF) << 3);
	emit(n, true, 0);
	emit(n, true, 0);
	emit(n / 4.0, true, 0);
	for(uint16_t s : symbols) {
		emit(n, false, s);
	}
	return iq;
}

static const double MC_FC = 869.618e6, MC_LO = MC_FC - MESHCORE_LO_OFFSET, MC_FS = 2e6;
static const unsigned MC_SF_MASK = (1u << 7) | (1u << 8);

// the daemon's exact chain for --mode meshcore --region eu868, LO offset included
static std::vector<rx_packet> mc_chain(std::vector<cf> &cap, std::mt19937 &rng, float sigma, size_t extra_lead = 0) {
	std::vector<cf> lead(150000 + extra_lead, cf(0, 0));
	cap.insert(cap.begin(), lead.begin(), lead.end());
	cap.resize(cap.size() + 300000, cf(0, 0));
	add_noise(cap, sigma, rng);
	std::vector<rx_packet> pkts;
	channel_receiver rx(MC_FC, 0x12, MC_SF_MASK, MESHCORE_BW, [&](const rx_packet &pk) { pkts.push_back(pk); }, MESHCORE_OS);
	ddc dc(MC_FS, MESHCORE_BW * MESHCORE_OS, MC_FC - MC_LO, MESHCORE_DDC_CUTOFF, MESHCORE_DDC_TAPS);
	std::vector<cf> base;
	const size_t chunk = 16384;
	for(size_t i = 0; i < cap.size(); i += chunk) {
		base.clear();
		dc.process(cap.data() + i, std::min(chunk, cap.size() - i), base);
		rx.feed(base.data(), base.size());
	}
	return pkts;
}

static phy_params mc_phy(int sf, int cr) {
	phy_params p;
	p.sf = sf;
	p.cr = cr;
	p.ldro = false;
	p.has_crc = true;
	p.sync = 0x12;
	p.preamble_len = sf <= 8 ? 32 : 16; // RadioLibWrapper::preambleLengthForSF
	return p;
}

static void test_e2e() {
	struct tc {
		const char *name;
		int sf, cr;
		const char *pkt;
		double ppm;
		float sigma;
	};
	// the ppm cases bracket real crystal error: 10 ppm is +8.7 kHz at 869.6 MHz, 14 % of the
	// 62.5 kHz channel, and a shared time base also slides the symbol timing
	static const tc CASES[] = {
		{ "meshcore_e2e_eu_narrow_sf8_cr8_advert", 8, 4, MC_ADVERT, 0, 0.02f },
		{ "meshcore_e2e_fw_default_sf8_cr5_advert", 8, 1, MC_ADVERT, 0, 0.02f },
		{ "meshcore_e2e_sf7_cr5_public", 7, 1, MC_PUBLIC, 0, 0.02f },
		{ "meshcore_e2e_sf8_cr8_public_plus5ppm", 8, 4, MC_PUBLIC, 5, 0.02f },
		{ "meshcore_e2e_sf8_cr8_advert_plus10ppm", 8, 4, MC_ADVERT, 10, 0.02f },
		{ "meshcore_e2e_sf8_cr8_advert_minus10ppm", 8, 4, MC_ADVERT, -10, 0.02f },
		{ "meshcore_e2e_sf8_cr5_advert_plus10ppm", 8, 1, MC_ADVERT, 10, 0.02f },
		{ "meshcore_e2e_sf8_cr8_advert_snr_minus9db", 8, 4, MC_ADVERT, 3, 9.0f }, // sigma per 2 MSps I/Q rail
		{ "meshcore_e2e_sf8_cr8_advert_plus15ppm", 8, 4, MC_ADVERT, 15, 0.02f },
	};
	std::mt19937 rng(0x4d43);
	for(const auto &c : CASES) {
		std::vector<uint8_t> pl = from_hex(c.pkt);
		phy_params p = mc_phy(c.sf, c.cr);
		double f_off = ((MC_FC - MC_LO) + c.ppm * 1e-6 * MC_FC) / MC_FS;
		std::vector<cf> cap = synth_frame_clk(p, encode_frame(p, pl.data(), (int)pl.size()), 32, f_off, c.ppm);
		auto pkts = mc_chain(cap, rng, c.sigma);
		bool ok = pkts.size() == 1 && pkts[0].sf == c.sf && pkts[0].cr == c.cr && pkts[0].crc_ok && pkts[0].payload == pl &&
		          pkts[0].bw_hz == 62500.0;
		char d[96];
		snprintf(d, sizeof(d), "%zu pkts sf=%d crc=%d snr=%.1f", pkts.size(), pkts.empty() ? 0 : pkts[0].sf,
		         pkts.empty() ? 0 : (int)pkts[0].crc_ok, pkts.empty() ? 0.0 : (double)pkts[0].snr_db);
		check(ok, c.name, d);
	}
}

// a frame starts at an arbitrary instant: walk it across one symbol-rate sample (32 capture
// samples) at the worst crystal error. A fixed lead would only prove one timing phase.
static void test_timing_phases() {
	std::mt19937 rng(0x4d46);
	std::vector<uint8_t> pl = from_hex(MC_ADVERT);
	phy_params p = mc_phy(8, 4);
	std::vector<uint16_t> syms = encode_frame(p, pl.data(), (int)pl.size());
	int good = 0, total = 0;
	float worst = 1e9f;
	for(double ppm : { 0.0, 10.0, -10.0 }) {
		for(size_t lead = 0; lead < 32; lead += 3) {
			double f_off = ((MC_FC - MC_LO) + ppm * 1e-6 * MC_FC) / MC_FS;
			std::vector<cf> cap = synth_frame_clk(p, syms, 32, f_off, ppm);
			auto pkts = mc_chain(cap, rng, 0.02f, lead);
			total++;
			if(pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pl) {
				good++;
				worst = std::min(worst, pkts[0].snr_db);
			}
		}
	}
	char d[64];
	snprintf(d, sizeof(d), "%d/%d decoded, worst snr=%.1f", good, total, worst);
	check(good == total, "meshcore_all_timing_phases_0_and_10ppm", d);
}

static void test_neighbours() {
	std::mt19937 rng(0x4d44);
	std::vector<uint8_t> pl = from_hex(MC_PUBLIC);
	phy_params p = mc_phy(8, 4);
	std::vector<uint16_t> syms = encode_frame(p, pl.data(), (int)pl.size());
	const double home = (MC_FC - MC_LO) / MC_FS;

	// the same frame on another network's sync word is not ours
	{
		phy_params q = p;
		q.sync = 0x2B;
		std::vector<cf> cap = synth_frame_clk(q, encode_frame(q, pl.data(), (int)pl.size()), 32, home, 0);
		check(mc_chain(cap, rng, 0.02f).empty(), "meshcore_rejects_meshtastic_sync");
	}
	// equal-power 125 kHz LoRa neighbour 200 kHz up, on air through the whole frame
	{
		std::vector<cf> cap = synth_frame_clk(p, syms, 32, home, 0);
		phy_params q = mc_phy(7, 1);
		q.sync = 0x34;
		q.preamble_len = 8;
		std::vector<uint8_t> junk(250, 0x5a);
		std::vector<uint16_t> qs = encode_frame(q, junk.data(), (int)junk.size());
		std::vector<cf> other;
		while(other.size() < cap.size()) {
			std::vector<cf> o = synth_frame_clk(q, qs, 16, (MC_FC + 200e3 - MC_LO) / MC_FS, 0);
			other.insert(other.end(), o.begin(), o.end());
		}
		for(size_t i = 0; i < cap.size(); i++) {
			cap[i] += other[i];
		}
		auto pkts = mc_chain(cap, rng, 0.02f);
		check(pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pl, "meshcore_survives_lora_neighbour_200k");
	}
	// Meshtastic LongFast at 869.525 MHz overlaps our band in EU868; 10 dB weaker here
	{
		std::vector<cf> cap = synth_frame_clk(p, syms, 32, home, 0);
		phy_params q = mc_phy(11, 1);
		q.sync = 0x2B;
		std::vector<uint8_t> junk(100, 0xa5);
		std::vector<cf> o = synth_frame_clk(q, encode_frame(q, junk.data(), (int)junk.size()), 8, (869.525e6 - MC_LO) / MC_FS, 0);
		for(size_t i = 0; i < cap.size() && i < o.size(); i++) {
			cap[i] += o[i] * 0.316f;
		}
		auto pkts = mc_chain(cap, rng, 0.02f);
		check(pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pl, "meshcore_survives_longfast_overlap_minus10db");
	}
}

static void test_ldro_rule() {
	// BW62.5: SF10 is the first LDRO spreading factor (16.4 ms symbol), the same rule as
	// BW125 SF11 and BW250 SF12
	std::mt19937 rng(0x4d45);
	std::vector<uint8_t> pl(20, 0x3c);
	phy_params p = mc_phy(10, 2);
	p.ldro = true;
	chirp_mod mod(10);
	std::vector<cf> iq(1377, cf(0, 0));
	std::vector<cf> frame = mod.frame(p, encode_frame(p, pl.data(), (int)pl.size()));
	iq.insert(iq.end(), frame.begin(), frame.end());
	iq.resize(iq.size() + 8192, cf(0, 0));
	add_noise(iq, 0.05f, rng);
	std::vector<rx_packet> pkts;
	channel_receiver rx(869.618e6, 0x12, 1u << 10, 62500.0, [&](const rx_packet &pk) { pkts.push_back(pk); });
	const size_t chunk = 4096;
	for(size_t i = 0; i < iq.size(); i += chunk) {
		rx.feed(iq.data() + i, std::min(chunk, iq.size() - i));
	}
	check(pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == pl, "ldro_sf10_bw62k5");
}

// ---- TX: everything below stays offline ----
static bool scope_code_ok(const std::vector<uint8_t> &b, const char *name) {
	uint16_t want = (uint16_t)(b[1] | (b[2] << 8));
	size_t n = 6 + (size_t)((b[5] >> 6) + 1) * (b[5] & 63);
	return mc_transport_code(mc_scope_named(name).key, (b[0] >> 2) & 15, &b[n], b.size() - n) == want;
}

static void test_tx() {
	// a real advert from another implementation must verify, a flipped bit must not
	std::vector<uint8_t> adv = from_hex(MC_ADVERT);
	check(mc_verify_advert(adv), "tx_real_advert_signature");
	adv[adv.size() - 1] ^= 1;
	check(!mc_verify_advert(adv), "tx_real_advert_tamper");

	uint8_t seed[32];
	for(int i = 0; i < 32; i++) {
		seed[i] = (uint8_t)(i * 7 + 1);
	}
	mc_identity id;
	mc_identity_from_seed(seed, id);
	for(const char *name : { "#region-a", "#region-b", "#region-c" }) {
		std::vector<uint8_t> scoped = mc_build_advert(id, 1793470000, "RFNM", true, 50.0, 8.0, mc_scope_named(name));
		check(scope_code_ok(scoped, name), "tx_scope_code");
		check(!scope_code_ok(scoped, "#other"), "tx_scope_code_wrong_region");
	}
	mc_scope sc = mc_scope_named("#region-a");
	std::vector<uint8_t> a = mc_build_advert(id, 1793470000, "RFNM", true, 50.1234, 8.5678, sc);
	meshcore_fields f = meshcore_decode(a);
	check(f.valid && f.type == 4 && f.route == 0 && f.has_transport && f.has_advert && f.name == "RFNM" && f.has_pos &&
	      std::fabs(f.lat - 50.1234) < 1e-5 && mc_verify_advert(a), "tx_own_advert", meshcore_info(f).c_str());
	std::vector<uint8_t> af = a;
	af.back() ^= 1;
	check(!mc_verify_advert(af), "tx_own_advert_tamper");

	// rebuild the real Public packet from its decoded fields: byte-exact crypto and framing
	{
		std::vector<uint8_t> real = from_hex(MC_PUBLIC);
		meshcore_fields r = meshcore_decode(real);
		uint8_t key[16];
		std::vector<uint8_t> mine;
		bool ok = mc_channel_key("public", key) && mc_build_group_text(key, r.msg_time, r.sender, r.text, mc_scope(), mine);
		check(ok && mine == real, "tx_public_channel_bytes_match_real_packet", meshcore_info(meshcore_decode(mine)).c_str());
	}
	{
		uint8_t key[16];
		std::vector<uint8_t> fr;
		check(mc_channel_key("public", key) && mc_build_group_text(key, 1793470000, "RFNM", "moin", sc, fr), "tx_group_build");
		meshcore_fields g = meshcore_decode(fr);
		check(g.valid && g.public_channel && g.has_text && g.sender == "RFNM" && g.text == "moin" && g.has_transport, "tx_group_decode", meshcore_info(g).c_str());
		check(!mc_build_group_text(key, 1, "RFNM", std::string(400, 'x'), sc, fr), "tx_group_too_long_refused");
		check(!mc_channel_key("nope", key), "tx_channel_name_refused");
		std::vector<uint8_t> ping;
		check(mc_channel_key("#ping", key) && mc_build_group_text(key, 1, "x", "y", sc, ping) && ping[6] == 0x28, "tx_hashtag_channel_hash_real");

		// through the daemon's receive chain, at the transmit waveform and sample rate
		std::mt19937 rng(77);
		std::vector<int16_t> iq = mc_render_iq(fr, MC_FS, MC_FC - MC_LO);
		std::vector<cf> cap(iq.size() / 2);
		for(size_t i = 0; i < cap.size(); i++) {
			cap[i] = cf(iq[2 * i] / 32768.0f, iq[2 * i + 1] / 32768.0f);
		}
		check((iq.size() / 2) % 256 == 0, "tx_iq_256_grid");
		std::vector<rx_packet> pk = mc_chain(cap, rng, 0.05f);
		bool hit = pk.size() == 1 && pk[0].crc_ok && pk[0].payload == fr && pk[0].cr == 4 && pk[0].sf == 8;
		check(hit, "tx_iq_roundtrip_through_receiver");
		double air = mc_airtime_s(fr.size());
		check(std::fabs(air - (double)iq.size() / 2 / MC_FS) < 0.02, "tx_airtime_matches_waveform");
	}
	{
		mc_duty d;
		check(d.allow(0, 1.0), "tx_duty_allows");
		d.commit(0, 359.5);
		check(d.allow(10, 0.4) && !d.allow(10, 0.6), "tx_duty_caps_at_10_percent");
		check(d.allow(3700, 300), "tx_duty_window_slides");
	}
	{
		mc_gate_in g;
		check(mc_tx_gate(g) != nullptr, "tx_gate_default_closed");
		g.tx_mode = g.board_all = g.tx_flag = true;
		check(mc_tx_gate(g) != nullptr, "tx_gate_needs_arming");
		g.armed = true;
		check(mc_tx_gate(g) == nullptr, "tx_gate_open");
		g.power_dbm = LORAD_TX_HARD_CAP_DBM + 1;
		check(mc_tx_gate(g) != nullptr, "tx_gate_power_cap");
		g.power_dbm = 0;
		g.board_all = false;
		check(mc_tx_gate(g) != nullptr, "tx_gate_board_policy");
	}
}

// A user channel decodes only with its key, and its messages name the channel.
static void test_user_channels() {
	const uint8_t key[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
	mc_scope sc;
	std::vector<uint8_t> fr;
	check(mc_build_group_text(key, 1793470000, "RFNM", "hello", sc, fr), "chan_build");
	meshcore_fields a = meshcore_decode(fr);
	check(a.encrypted && !a.has_text, "chan_unknown_stays_ciphertext");
	check(!mc_channel_name_ok("public") && !mc_channel_name_ok("a b") && !mc_channel_name_ok("a\"b") && mc_channel_name_ok("friends"), "chan_name_rules");
	check(mc_channel_add("friends", key), "chan_add");
	meshcore_fields b = meshcore_decode(fr);
	check(!b.encrypted && b.has_text && b.text == "hello" && b.sender == "RFNM" && b.channel_name == "friends", "chan_decodes");
	uint8_t other[16], got[16];
	memcpy(other, key, 16);
	other[0] ^= 1;
	check(mc_channel_add("other", other), "chan_add_other");
	check(mc_channel_key("friends", got) && memcmp(got, key, 16) == 0, "chan_send_key");
	check(mc_channels_save("/tmp/lorad-selftest-channels") && mc_channel_remove("friends") && mc_channel_remove("other"), "chan_save_remove");
	meshcore_fields c = meshcore_decode(fr);
	check(c.encrypted && !mc_channel_key("friends", got), "chan_removed");
	check(mc_channels_load("/tmp/lorad-selftest-channels") && meshcore_decode(fr).has_text, "chan_load");
	mc_channel_remove("friends");
	mc_channel_remove("other");
	unlink("/tmp/lorad-selftest-channels");
}

// A to B through the real frame builders; B's decoder and its ACK.
static void test_dm() {
	uint8_t sa[32], sb[32];
	memset(sa, 0x11, 32);
	memset(sb, 0x22, 32);
	mc_identity a, b;
	mc_identity_from_seed(sa, a);
	mc_identity_from_seed(sb, b);
	mc_scope sc;
	mc_contacts_clear();
	std::vector<uint8_t> fr;
	mc_dm_set_identity(a);
	uint32_t want_ack = 0, want_ack1 = 0;
	check(mc_build_dm(a, b.pub, 1793470000, "hallo", sc, fr, 0, &want_ack), "dm_build");
	std::vector<uint8_t> fr1;
	check(mc_build_dm(a, b.pub, 1793470000, "hallo", sc, fr1, 1, &want_ack1) && want_ack1 != want_ack, "dm_retry_has_own_ack");
	mc_dm_set_identity(b);
	meshcore_fields f0 = meshcore_decode(fr);
	check(f0.encrypted && !f0.has_text, "dm_unknown_sender_stays_ciphertext");
	mc_contact_add(a.pub, "alice");
	int acks = 0;
	mc_dm_event ev;
	mc_dm_set_callback([&](const mc_dm_event &e) { ev = e; acks++; });
	meshcore_fields f1 = meshcore_decode(fr);
	check(!f1.encrypted && f1.has_text && f1.text == "hallo" && f1.sender == "alice" && f1.channel_name == "DM", "dm_decodes");
	meshcore_decode(fr);
	check(acks == 1, "dm_acked_once");
	uint8_t want[32];
	{
		sha256 s;
		uint8_t head[5] = { (uint8_t)(1793470000u), (uint8_t)(1793470000u >> 8), (uint8_t)(1793470000u >> 16), (uint8_t)(1793470000u >> 24), 0 };
		s.update(head, 5);
		s.update((const uint8_t *)"hallo", 5);
		s.update(a.pub, 32);
		s.finish(want);
	}
	check(memcmp(ev.ack, want, 4) == 0 && ev.flood, "dm_ack_hash");
	std::vector<uint8_t> pa = mc_build_path_ack(b, ev, sc);
	meshcore_fields fp = meshcore_decode(pa);
	check(fp.valid && fp.dst == a.pub[0] && fp.src == b.pub[0], "dm_path_ack_frame");
	meshcore_decode(fr1);
	check(acks == 2 && memcmp(ev.ack, &want_ack1, 4) == 0, "dm_retry_acked_again");
	std::vector<uint8_t> pa1 = mc_build_path_ack(b, ev, sc);
	std::vector<uint8_t> fa = mc_build_flood_ack(ev, sc);
	mc_dm_set_identity(a);
	std::vector<uint32_t> heard;
	mc_ack_set_callback([&](uint32_t x) { heard.push_back(x); });
	meshcore_decode(pa);
	check(heard.empty(), "dm_path_ack_needs_contact");
	mc_contact_add(b.pub, "bob");
	meshcore_decode(pa);
	check(heard.size() == 1 && heard[0] == want_ack, "dm_path_ack_read");
	meshcore_decode(pa1);
	check(heard.size() == 2 && heard[1] == want_ack1, "dm_path_ack_retry_read");
	meshcore_decode(fa);
	check(heard.size() == 3 && heard[2] == want_ack1, "dm_flood_ack_read");
	mc_ack_set_callback(nullptr);
	bool routed = true;
	std::vector<uint8_t> rt;
	check(mc_build_dm(a, b.pub, 1793470001, "x", sc, rt, 0, nullptr, &routed) && !routed && (rt[0] & 3) != 2, "dm_no_route_floods");
	mc_dm_set_identity(b);
	meshcore_decode(fr);                                   // the flood teaches B the route to A, relayed copy or not
	check(mc_build_dm(b, a.pub, 1793470002, "y", sc, rt, 0, nullptr, &routed) && routed && (rt[0] & 3) == 2 && ((rt[0] >> 2) & 15) == 2, "dm_route_goes_direct");
	mc_dm_set_identity(a);
	std::vector<uint8_t> tampered = fr;
	tampered.back() ^= 1;
	check(meshcore_decode(tampered).encrypted, "dm_bad_mac_rejected");
	// a route survives the restart of the daemon
	char dir[] = "/tmp/lorad-rt-XXXXXX";
	check(mkdtemp(dir) != nullptr, "dm_route_tmpdir");
	std::string cf_path = std::string(dir) + "/contacts.conf";
	mc_contacts_clear();
	mc_contacts_open(cf_path);
	mc_dm_set_identity(b);
	mc_contact_add(a.pub, "alice");
	std::vector<uint8_t> fl;
	mc_dm_set_identity(a);
	mc_build_dm(a, b.pub, 1793470009, "z", sc, fl);
	mc_dm_set_identity(b);
	meshcore_decode(fl);
	mc_contacts_clear();
	mc_contacts_open(cf_path);
	bool rt2 = false;
	std::vector<uint8_t> dr;
	check(mc_build_dm(b, a.pub, 1793470010, "w", sc, dr, 0, nullptr, &rt2) && rt2 && (dr[0] & 3) == 2, "dm_route_survives_restart");
	FILE *rf = fopen((cf_path + ".routes").c_str(), "w");
	if(rf) {
		fprintf(rf, "not a route\n");
		fclose(rf);
	}
	mc_contacts_clear();
	mc_contacts_open(cf_path);
	check(mc_build_dm(b, a.pub, 1793470011, "v", sc, dr, 0, nullptr, &rt2) && !rt2, "dm_damaged_route_ignored");
	unlink((cf_path + ".routes").c_str());
	unlink(cf_path.c_str());
	rmdir(dir);
	mc_contacts_open("");
	mc_dm_set_identity(a);
	mc_dm_set_callback(nullptr);
	mc_contacts_clear();
}

static void test_meshtastic_dm(const mt_channel &ch) {
	// FIPS 197 AES-256 and the packet from the firmware's own test/test_crypto
	std::vector<uint8_t> key = from_hex("603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4");
	std::vector<uint8_t> pt = from_hex("6bc1bee22e409f96e93d7e117393172a"), want = from_hex("f3eed1bdb5d2a03c064b5a7e3db181f8");
	uint8_t out[16];
	aes256(key.data()).encrypt_block(pt.data(), out);
	check(memcmp(out, want.data(), 16) == 0, "mt_aes256_fips");
	std::vector<uint8_t> priv = from_hex("a00330633e63522f8a4d81ec6d9d1e6617f6c8ffd3a4c698229537d44e522277");
	std::vector<uint8_t> pub = from_hex("db18fc50eea47f00251cb784819a3cf5fc361882597f589f0d7ff820e8064457");
	std::vector<uint8_t> radio = from_hex("8c646d7a2909000062d6b2136b00000040df24abfcc30a17a3d9046726099e796a1c036a792b");
	mt_keys k;
	mt_keys_from_private(priv.data(), k);
	uint8_t shared[32];
	mt_shared_key(k, pub.data(), shared);
	std::vector<uint8_t> expect_s = from_hex("777b1545c9d6f9a2");
	check(memcmp(shared, expect_s.data(), 8) == 0, "mt_shared_key_vector");
	std::vector<uint8_t> plain;
	check(mt_pkc_open(shared, 0x0929, 0x13b2d662, radio.data() + 16, 22, plain) && plain == from_hex("08011204746573744800"), "mt_pkc_firmware_packet");
	std::vector<uint8_t> bad = radio;
	bad[20] ^= 1;
	check(!mt_pkc_open(shared, 0x0929, 0x13b2d662, bad.data() + 16, 22, plain), "mt_pkc_tamper_rejected");
	std::vector<uint8_t> sealed = mt_pkc_seal(shared, 0x0929, 0x13b2d662, from_hex("08011204746573744800"), 0x2b796a03);
	check(sealed == std::vector<uint8_t>(radio.begin() + 16, radio.end()), "mt_pkc_seal_matches_firmware");

	// two nodes: keys by NODEINFO, then a sealed DM, its ACK and an unknown sender
	uint8_t sa[32], sb[32];
	memset(sa, 0x31, 32);
	memset(sb, 0x42, 32);
	mt_keys a, b;
	mt_keys_from_private(sa, a);
	mt_keys_from_private(sb, b);
	const uint32_t ida = 0xaaaa0001, idb = 0xbbbb0002;
	mt_nodes_clear();
	std::vector<uint8_t> fr, ni;
	check(mt_build_dm(a, ida, 7, idb, b.pub, 0, ch, "geheim", fr), "mt_dm_build");
	check((fr[12] & 8) && fr[13] == 0, "mt_dm_header_sealed_want_ack");
	check(mt_receive(fr, idb, b, ch).unreadable, "mt_dm_unknown_sender_unreadable");
	check(mt_build_nodeinfo(ida, 8, 0, ch, "Alice", "ALC", ni, a.pub, true), "mt_nodeinfo_with_key");
	mt_event en = mt_receive(ni, idb, b, ch);
	check(en.valid && en.want_response && en.port == 4, "mt_nodeinfo_read");
	mt_event ed = mt_receive(fr, idb, b, ch);
	check(ed.valid && ed.sealed && ed.for_us && ed.want_ack && ed.text == "geheim", "mt_dm_decrypts");
	check(mt_receive(fr, 0x12345678, b, ch).text.empty(), "mt_dm_not_for_others");
	std::vector<uint8_t> ack;
	check(mt_build_ack(idb, 9, ida, 0, ch, 7, 0, ack), "mt_ack_build");
	mt_event ea = mt_receive(ack, ida, a, ch);
	check(ea.valid && ea.for_us && ea.port == 5 && ea.routing_error == 0 && ea.request_id == 7, "mt_ack_read");
	std::vector<uint8_t> forged;
	check(mt_build_nodeinfo(ida, 10, 0, ch, "Mallory", "MAL", forged, b.pub), "mt_forged_nodeinfo_build");
	mt_receive(forged, idb, b, ch);
	uint32_t found = 0;
	uint8_t kept[32];
	bool has = false;
	check(mt_node_find("aaaa", found, kept, has) && memcmp(kept, a.pub, 32) == 0, "mt_first_key_stays");
	mt_nodes_clear();
}

// The frame builder through the real render and the daemon's --mode meshtastic chain.
static void test_meshtastic_tx() {
	uint8_t def[16] = { 0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01 };
	check(mt_channel_hash("LongFast", def) == 0x08, "mt_default_channel_hash");
	mt_channel ch;
	std::vector<uint8_t> fr;
	check(!mt_build_text(0x12345678, 1, 0, ch, "x", fr), "mt_no_channel_no_frame");
	char path[] = "/tmp/lorad-mt-XXXXXX";
	int fd = mkstemp(path);
	check(fd >= 0, "mt_tmpfile");
	const char *pub = "Bad d4f1bb3a20290759f0bcffabcf4e6901\n";
	check(write(fd, pub, strlen(pub)) > 0, "mt_tmpfile_write");
	std::string err;
	check(!mt_channel_load(path, ch, err), "mt_public_key_refused");
	check(ftruncate(fd, 0) == 0 && lseek(fd, 0, SEEK_SET) == 0, "mt_tmpfile_reset");
	const char *priv = "RFNMTEST 00112233445566778899aabbccddeeff\n";
	check(write(fd, priv, strlen(priv)) > 0, "mt_tmpfile_write2");
	close(fd);
	check(mt_channel_load(path, ch, err) && ch.hash == mt_channel_hash("RFNMTEST", ch.psk), "mt_channel_loads");
	unlink(path);
	meshtastic_set_channel(ch.hash, ch.psk);
	check(mt_build_text(0xee5e51b2, 0x01020304, 0, ch, "Moin Meshtastic", fr) && fr.size() == 16 + 19, "mt_text_frame");
	check(fr[0] == 0xff && fr[4] == 0xb2 && fr[8] == 4 && fr[12] == 0 && fr[13] == ch.hash, "mt_header");
	check(meshtastic_info(fr).find("Moin Meshtastic") != std::string::npos, "mt_text_decodes", meshtastic_info(fr).c_str());
	std::vector<uint8_t> ni;
	check(mt_build_nodeinfo(0xee5e51b2, 2, 0, ch, "TEST-1", "RFNM", ni), "mt_nodeinfo_frame");
	check(meshtastic_info(ni).find("NODEINFO") != std::string::npos, "mt_nodeinfo_decodes", meshtastic_info(ni).c_str());
	// the render, offset as in the daemon: 250 kHz above the LO at 2 MSps
	const double fs = 2e6, f0 = 869.525e6, lo = f0 - 250e3;
	std::vector<int16_t> iq = lora_render_iq(mt_phy(), 250000.0, fr, fs, f0 - lo);
	std::vector<cf> cap(150000, cf(0, 0));
	for(size_t i = 0; i + 1 < iq.size(); i += 2) {
		cap.push_back(cf(iq[i] / 32768.0f, iq[i + 1] / 32768.0f));
	}
	cap.resize(cap.size() + 300000, cf(0, 0));
	std::mt19937 rng(0x4d54);
	add_noise(cap, 0.02f, rng);
	std::vector<rx_packet> pkts;
	channel_receiver rx(f0, 0x2B, 1u << 11, 250000, [&](const rx_packet &pk) { pkts.push_back(pk); }, 4);
	ddc dc(fs, 250000 * 4, f0 - lo, 0.13, 128);
	std::vector<cf> base;
	for(size_t i = 0; i < cap.size(); i += 16384) {
		base.clear();
		dc.process(cap.data() + i, std::min<size_t>(16384, cap.size() - i), base);
		rx.feed(base.data(), base.size());
	}
	check(pkts.size() == 1 && pkts[0].crc_ok && pkts[0].payload == fr, "mt_iq_through_rx_chain");
	double air = mt_airtime_s(fr.size());
	check(std::fabs(air - (iq.size() / 2) / fs) < 0.02, "mt_airtime_matches_render");
	test_meshtastic_dm(ch);
}

void run_meshcore_selftest(void (*check_fn)(bool, const char *, const char *)) {
	g_check = check_fn;
	test_meshtastic_tx();
	test_dm();
	test_user_channels();
	test_crypto_kat();
	test_vectors();
	test_malformed();
	test_json();
	test_e2e();
	test_timing_phases();
	test_neighbours();
	test_ldro_rule();
	test_tx();
}

} // namespace lorad

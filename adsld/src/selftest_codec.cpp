// adsld selftest: link-layer codec cases - CRC24 KATs, XXTEA vectors, exponential-field
// round trips, encode/decode round trips, 1-bit syndrome correction, and byte-exact
// cross-validation against an independent reference implementation.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

#include "adsl_codec.h"
#include "selftest.h"

namespace adsld {

// ---- cross-validation vectors --------------------------------------------------------
// test vectors generated with P. Jalocha's ads-l.h reference implementation (ogn-tracker
// project, GPL - generator lives outside this repo). Each row is a complete 25-byte wire
// packet (LEN + NetHdr + XXTEA-scrambled data + CRC24 big-endian) for the matching input
// in ref_input() below.

static constexpr int NUM_REF_VECS = 6;

static const uint8_t REF_WIRE[NUM_REF_VECS][ADSL_PKT_LEN] = {
	// alps_climb: FLARM glider airborne over the Alps, climbing
	{ 0x18,0x01,0x3C,0x43,0x0C,0x91,0xBE,0x64,0xEA,0x58,0xE3,0xE4,0xE0,0x68,0x84,0xB6,0xAA,0x7F,0x86,0xF0,0xF6,0x4F,0xC0,0xF6,0x76 },
	// south_descend: ICAO light plane, negative latitude, descending
	{ 0x18,0x01,0x39,0x10,0xC1,0xAF,0x87,0xC4,0x89,0xE8,0x05,0x50,0xA3,0x20,0x1C,0x61,0x77,0xA8,0x14,0x06,0x6B,0xE2,0xBD,0xA3,0x5A },
	// west_balloon: OGN balloon, negative longitude, relay flag set
	{ 0x18,0x01,0x28,0x33,0x7A,0xDB,0x40,0xC2,0xD1,0x8D,0x62,0x3C,0xEB,0xEE,0x23,0xA2,0xB6,0xF7,0x9A,0x49,0x8D,0xB7,0x95,0xA8,0x8F },
	// ground: on ground near the equator/meridian, zero speed, no track
	{ 0x18,0x01,0x04,0x5A,0xDD,0x6A,0x3D,0x4A,0x80,0x9A,0xA0,0x88,0xED,0x74,0x17,0x60,0xF7,0xDC,0x92,0x28,0x08,0xA4,0x5A,0x17,0xC5 },
	// no_fix: no GNSS fix, position and all kinematics invalid
	{ 0x18,0x01,0x58,0x45,0x02,0x80,0xF0,0xB5,0x50,0x66,0x95,0xD5,0x4C,0xFA,0x95,0x11,0x65,0xA8,0xDF,0x6A,0x33,0xDD,0xC1,0x40,0x74 },
	// fast_high: jet-like, high altitude/speed exercising the exponential upper ranges
	{ 0x18,0x01,0x96,0x05,0x12,0xBE,0x5E,0xD4,0x28,0xA5,0xCD,0x77,0x66,0xEE,0x72,0xDD,0x42,0xC1,0x68,0xA6,0xEA,0xBE,0x72,0xCA,0x39 },
};

static adsl_traffic mk(uint8_t amt, uint32_t addr, bool relay,
                       uint8_t ts, uint8_t fs, uint8_t cat, uint8_t emg,
                       bool posv, double lat, double lon, bool altv, float alt,
                       bool spdv, float spd, bool vrv, float vr, bool trkv, float trk,
                       uint8_t sil, uint8_t sda, uint8_t nic, uint8_t nacp,
                       uint8_t gva, uint8_t nacv) {
	adsl_traffic t;
	t.amt = amt;
	t.addr = addr;
	t.relay = relay;
	t.ts_qs = ts;
	t.flightstate = fs;
	t.category = cat;
	t.emergency = emg;
	t.pos_valid = posv;
	t.lat_deg = lat;
	t.lon_deg = lon;
	t.alt_valid = altv;
	t.alt_m = alt;
	t.speed_valid = spdv;
	t.speed_mps = spd;
	t.vrate_valid = vrv;
	t.vrate_mps = vr;
	t.track_valid = trkv;
	t.track_deg = trk;
	t.sil = sil;
	t.sda = sda;
	t.nic = nic;
	t.nacp = nacp;
	t.gva = gva;
	t.nacv = nacv;
	return t;
}

static adsl_traffic ref_input(int i) {
	switch(i) {
	case 0: return mk(6, 0x3D5E21, false, 17, 2, 4, 1,
	                  true, 47.252250, 11.343720, true, 2450.0f, true, 31.5f,
	                  true, 2.375f, true, 245.5f, 1, 1, 8, 5, 2, 2);
	case 1: return mk(5, 0x4CA7B0, false, 58, 2, 1, 1,
	                  true, -33.968710, 18.601520, true, 320.0f, true, 65.25f,
	                  true, -4.5f, true, 87.2f, 3, 2, 10, 7, 3, 3);
	case 2: return mk(7, 0x155AA0, true, 0, 2, 5, 1,
	                  true, 51.477930, -2.598720, true, 152.0f, true, 3.75f,
	                  true, -0.125f, true, 312.9f, 2, 1, 9, 6, 2, 1);
	case 3: return mk(0, 0xFFFFFF, false, 33, 1, 0, 0,
	                  true, -0.000870, -0.000340, true, -75.0f, true, 0.0f,
	                  true, 0.0f, false, 0.0f, 0, 0, 0, 0, 0, 0);
	case 4: return mk(1, 0x00007F, false, 59, 0, 13, 0,
	                  false, 0.0, 0.0, false, 0.0f, false, 0.0f,
	                  false, 0.0f, false, 0.0f, 0, 0, 0, 0, 0, 0);
	default: return mk(5, 0xAB0123, false, 42, 2, 2, 1,
	                   true, 63.985210, -22.605440, true, 12496.0f, true, 234.0f,
	                   true, 30.0f, true, 179.6f, 3, 3, 11, 7, 3, 3);
	}
}

static std::string hexdump(const uint8_t *b, size_t n) {
	std::string s;
	char tmp[4];
	for(size_t i = 0; i < n; i++) {
		snprintf(tmp, sizeof(tmp), "%02X", b[i]);
		s += tmp;
	}
	return s;
}

// natural-unit field comparison against the (pre-quantization) reference inputs; the
// tolerances are the worst-case quantization steps of each field at the tested values
static bool fields_match(const adsl_traffic &a, const adsl_traffic &b, std::string &why) {
	char msg[128];
	msg[0] = 0;
	if(a.amt != b.amt || a.addr != b.addr || a.relay != b.relay) {
		snprintf(msg, sizeof(msg), "sender %u:%06X:%d vs %u:%06X:%d",
		         a.amt, a.addr, a.relay, b.amt, b.addr, b.relay);
	} else if(a.ts_qs != b.ts_qs || a.flightstate != b.flightstate ||
	          a.category != b.category || a.emergency != b.emergency) {
		snprintf(msg, sizeof(msg), "time/state mismatch");
	} else if(a.pos_valid != b.pos_valid ||
	          (a.pos_valid && (std::fabs(a.lat_deg - b.lat_deg) > 0.6 / 93206.0 ||
	                           std::fabs(a.lon_deg - b.lon_deg) > 0.6 / 46603.0))) {
		snprintf(msg, sizeof(msg), "pos %d(%.6f,%.6f) vs %d(%.6f,%.6f)", a.pos_valid,
		         a.lat_deg, a.lon_deg, b.pos_valid, b.lat_deg, b.lon_deg);
	} else if(a.alt_valid != b.alt_valid ||
	          (a.alt_valid && std::fabs(a.alt_m - b.alt_m) > 8.0f)) {
		snprintf(msg, sizeof(msg), "alt %d:%.1f vs %d:%.1f", a.alt_valid, a.alt_m,
		         b.alt_valid, b.alt_m);
	} else if(a.speed_valid != b.speed_valid ||
	          (a.speed_valid && std::fabs(a.speed_mps - b.speed_mps) > 2.0f)) {
		snprintf(msg, sizeof(msg), "speed mismatch");
	} else if(a.vrate_valid != b.vrate_valid ||
	          (a.vrate_valid && std::fabs(a.vrate_mps - b.vrate_mps) > 1.0f)) {
		snprintf(msg, sizeof(msg), "vrate mismatch");
	} else if(a.track_valid != b.track_valid ||
	          (a.track_valid && std::fabs(a.track_deg - b.track_deg) > 0.5f)) {
		snprintf(msg, sizeof(msg), "track mismatch");
	} else if(a.sil != b.sil || a.sda != b.sda || a.nic != b.nic || a.nacp != b.nacp ||
	          a.gva != b.gva || a.nacv != b.nacv) {
		snprintf(msg, sizeof(msg), "quality mismatch");
	}
	why = msg;
	return msg[0] == 0;
}

int selftest_codec() {
	int fails = 0;

	// ---- CRC24 KATs (check values from the reference implementation) -----------------
	{
		uint8_t ramp[ADSL_CRC_SPAN];
		for(size_t i = 0; i < sizeof(ramp); i++) {
			ramp[i] = (uint8_t)(i * 7 + 3);
		}
		uint8_t zeros[ADSL_CRC_SPAN] = { 0 };
		uint8_t one = 0x01;
		check(fails, crc24(ramp, sizeof(ramp)) == 0xD1147D &&
		             crc24(zeros, sizeof(zeros)) == 0x000000 &&
		             crc24(&one, 1) == 0xFFF409, "codec: crc24 KATs");
		// appending the returned CRC big-endian must give a zero syndrome
		uint8_t buf[ADSL_CRC_SPAN + 3];
		memcpy(buf, ramp, sizeof(ramp));
		uint32_t c = crc24(ramp, sizeof(ramp));
		buf[21] = (uint8_t)(c >> 16);
		buf[22] = (uint8_t)(c >> 8);
		buf[23] = (uint8_t)c;
		check(fails, crc24_correct_1bit(buf, sizeof(buf)) == -1, "codec: crc24 of message plus its CRC is zero");
	}

	// ---- XXTEA -----------------------------------------------------------------------
	{
		// KAT generated with the reference XXTEA_Encrypt_Key0 (5 words, 6 rounds)
		uint32_t w[5] = { 0x00000000, 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
		const uint32_t expect[5] = { 0x5391E374, 0xFFCE2A63, 0x749099FF, 0x7589BAB6, 0x0327D8B2 };
		xxtea_scramble(w);
		check(fails, memcmp(w, expect, sizeof(expect)) == 0, "codec: xxtea scramble KAT");
		xxtea_descramble(w);
		const uint32_t orig[5] = { 0x00000000, 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
		check(fails, memcmp(w, orig, sizeof(orig)) == 0, "codec: xxtea descramble inverse");
		uint32_t z[5] = { 0, 0, 0, 0, 0 };
		const uint32_t zexpect[5] = { 0x699467EB, 0x0B7C7AE4, 0xD4E85D63, 0x382B0DB6, 0xE679295D };
		xxtea_scramble(z);
		check(fails, memcmp(z, zexpect, sizeof(zexpect)) == 0, "codec: xxtea all-zero KAT");

		std::mt19937 rng(12345);
		int bad = 0;
		for(int n = 0; n < 200; n++) {
			uint32_t a[5], b[5];
			for(int i = 0; i < 5; i++) {
				a[i] = b[i] = rng();
			}
			xxtea_scramble(b);
			xxtea_descramble(b);
			if(memcmp(a, b, sizeof(a)) != 0) {
				bad++;
			}
		}
		check(fails, bad == 0, "codec: xxtea random round trips");
	}

	// ---- exponential field coding (worked examples from the spec) --------------------
	{
		bool ok = exp_enc(0, 12) == 0x0000 &&      // alt -320 m
		          exp_enc(320, 12) == 0x0140 &&    // alt 0 m
		          exp_enc(1320, 12) == 0x0528 &&   // alt 1000 m
		          exp_enc(61424, 12) == 0x3FFE &&  // alt 61104 m (clamp point)
		          exp_enc(0, 6) == 0x00 &&         // speed 0
		          exp_enc(1, 6) == 0x01 &&         // speed 0.25 m/s
		          exp_enc(3, 6) == 0x03 &&         // speed 0.75 m/s
		          exp_enc(480, 6) == 0xC4 &&       // speed 120 m/s
		          exp_enc(944, 6) == 0xFE &&       // speed 236 m/s (clamp point)
		          exp_dec(0x0528, 12) == 1320 &&
		          exp_dec(0x3FFE, 12) == 61424 &&
		          exp_dec(0xC4, 6) == 480 &&
		          exp_dec(0xFE, 6) == 944;
		check(fails, ok, "codec: exponential coding spec examples");
		int bad = 0;
		for(uint32_t code = 0; code < 256; code++) {           // N=6: every code
			if(exp_enc(exp_dec(code, 6), 6) != code) {
				bad++;
			}
		}
		for(uint32_t code = 0; code < 16384; code++) {         // N=12: every code
			if(exp_enc(exp_dec(code, 12), 12) != code) {
				bad++;
			}
		}
		check(fails, bad == 0, "codec: exponential enc(dec) round trip");
	}

	// ---- cross-validation against the reference wire packets -------------------------
	{
		int enc_bad = 0, dec_bad = 0;
		std::string detail;
		for(int i = 0; i < NUM_REF_VECS; i++) {
			uint8_t pkt[ADSL_PKT_LEN];
			adsl_encode(ref_input(i), pkt);
			if(memcmp(pkt, REF_WIRE[i], ADSL_PKT_LEN) != 0) {
				enc_bad++;
				if(detail.empty()) {
					detail = "vec " + std::to_string(i) + " ours " +
					         hexdump(pkt, ADSL_PKT_LEN) + " ref " +
					         hexdump(REF_WIRE[i], ADSL_PKT_LEN);
				}
			}
			adsl_traffic t;
			std::string why;
			if(adsl_decode(REF_WIRE[i], t) != adsl_decode_result::ok ||
			   !fields_match(t, ref_input(i), why)) {
				dec_bad++;
				if(detail.empty()) {
					detail = "vec " + std::to_string(i) + " " + (why.empty() ? "crc" : why);
				}
			}
		}
		check(fails, enc_bad == 0, "codec: encode matches reference bytes", detail);
		check(fails, dec_bad == 0, "codec: decode of reference bytes", detail);
	}

	// ---- 1-bit syndrome correction over the full protected span ----------------------
	{
		const uint8_t *wire = REF_WIRE[0];
		int bad = 0;
		for(int bit = 0; bit < (int)(ADSL_CRC_SPAN + 3) * 8; bit++) {
			uint8_t buf[ADSL_CRC_SPAN + 3];
			memcpy(buf, wire + 1, sizeof(buf));
			buf[bit / 8] ^= (uint8_t)(0x80u >> (bit & 7));
			if(crc24_correct_1bit(buf, sizeof(buf)) != bit ||
			   memcmp(buf, wire + 1, sizeof(buf)) != 0) {
				bad++;
			}
			// same flip through the packet-level API
			uint8_t pkt[ADSL_PKT_LEN];
			memcpy(pkt, wire, ADSL_PKT_LEN);
			pkt[1 + bit / 8] ^= (uint8_t)(0x80u >> (bit & 7));
			adsl_traffic t;
			std::string why;
			if(adsl_decode(pkt, t) != adsl_decode_result::corrected_1bit ||
			   !fields_match(t, ref_input(0), why)) {
				bad++;
			}
		}
		check(fails, bad == 0, "codec: 1-bit correction, all 192 positions");

		// 2-bit errors must be rejected, never miscorrected
		std::mt19937 rng(777);
		int bad2 = 0;
		for(int n = 0; n < 500; n++) {
			int b1 = (int)(rng() % 192);
			int b2 = (int)(rng() % 192);
			if(b1 == b2) {
				continue;
			}
			uint8_t buf[ADSL_CRC_SPAN + 3];
			memcpy(buf, wire + 1, sizeof(buf));
			buf[b1 / 8] ^= (uint8_t)(0x80u >> (b1 & 7));
			buf[b2 / 8] ^= (uint8_t)(0x80u >> (b2 & 7));
			if(crc24_correct_1bit(buf, sizeof(buf)) != -2) {
				bad2++;
			}
		}
		check(fails, bad2 == 0, "codec: 2-bit errors rejected");

		// packet-level: 3 corrupted bits -> crc_fail, out untouched
		uint8_t pkt[ADSL_PKT_LEN];
		memcpy(pkt, wire, ADSL_PKT_LEN);
		pkt[5] ^= 0x01;
		pkt[9] ^= 0x40;
		pkt[20] ^= 0x08;
		adsl_traffic t;
		t.addr = 0xDEADBE;
		check(fails, adsl_decode(pkt, t) == adsl_decode_result::crc_fail && t.addr == 0xDEADBE,
		      "codec: crc_fail leaves output untouched");
		// wrong LEN byte is not a Traffic packet
		memcpy(pkt, wire, ADSL_PKT_LEN);
		pkt[0] = 0x17;
		check(fails, adsl_decode(pkt, t) == adsl_decode_result::crc_fail,
		      "codec: wrong LEN byte rejected");
	}

	// ---- randomized encode/decode/re-encode sweep ------------------------------------
	{
		std::mt19937 rng(31337);
		auto frand = [&](double lo, double hi) {
			return lo + (hi - lo) * (double)(rng() & 0xFFFFFF) / (double)0xFFFFFF;
		};
		int bad = 0;
		std::string detail;
		for(int n = 0; n < 500; n++) {
			adsl_traffic t;
			t.amt = (uint8_t)(rng() % 64);
			t.addr = rng() & 0xFFFFFF;
			t.relay = (rng() & 1) != 0;
			t.proto_ver = (uint8_t)(rng() % 16);
			t.hdr_flag = (rng() & 1) != 0;
			t.key_idx = (uint8_t)(rng() % 4);      // 3 = scrambling disabled path
			t.err_mode = (rng() & 1) != 0;
			t.ts_qs = (uint8_t)(rng() % 60);
			t.flightstate = (uint8_t)(rng() % 4);
			t.category = (uint8_t)(rng() % 32);
			t.emergency = (uint8_t)(rng() % 8);
			t.pos_valid = (rng() % 8) != 0;
			if(t.pos_valid) {
				t.lat_deg = frand(-90.0, 90.0);
				t.lon_deg = frand(-180.0, 180.0);
			}
			t.alt_valid = (rng() % 8) != 0;
			if(t.alt_valid) {
				t.alt_m = (float)frand(-320.0, 61104.0);
			}
			t.speed_valid = (rng() % 8) != 0;
			if(t.speed_valid) {
				t.speed_mps = (float)frand(1.0, 236.0);  // >=1 so the track stays defined
			}
			t.vrate_valid = (rng() % 8) != 0;
			if(t.vrate_valid) {
				t.vrate_mps = (float)frand(-118.0, 119.0);
			}
			t.track_valid = t.speed_valid && (rng() & 1) != 0;
			if(t.track_valid) {
				t.track_deg = (float)frand(0.0, 359.9);
			}
			t.sil = (uint8_t)(rng() % 4);
			t.sda = (uint8_t)(rng() % 4);
			t.nic = (uint8_t)(rng() % 16);
			t.nacp = (uint8_t)(rng() % 8);
			t.gva = (uint8_t)(rng() % 4);
			t.nacv = (uint8_t)(rng() % 4);

			uint8_t pkt[ADSL_PKT_LEN], pkt2[ADSL_PKT_LEN];
			adsl_encode(t, pkt);
			adsl_traffic r;
			if(adsl_decode(pkt, r) != adsl_decode_result::ok) {
				bad++;
				continue;
			}
			// validity flags survive the wire (track: absent input decodes as 0 deg)
			if(r.pos_valid != t.pos_valid || r.alt_valid != t.alt_valid ||
			   r.speed_valid != t.speed_valid || r.vrate_valid != t.vrate_valid ||
			   r.amt != t.amt || r.addr != t.addr || r.relay != t.relay ||
			   r.proto_ver != t.proto_ver || r.key_idx != t.key_idx ||
			   r.hdr_flag != t.hdr_flag || r.err_mode != t.err_mode) {
				bad++;
				if(detail.empty()) {
					detail = "flags/sender, iteration " + std::to_string(n);
				}
				continue;
			}
			// re-encoding the decoded fields must reproduce the exact wire bytes
			adsl_encode(r, pkt2);
			if(memcmp(pkt, pkt2, ADSL_PKT_LEN) != 0) {
				bad++;
				if(detail.empty()) {
					detail = "re-encode, iteration " + std::to_string(n);
				}
			}
		}
		check(fails, bad == 0, "codec: 500 random encode/decode/re-encode sweeps", detail);
	}

	return fails;
}

} // namespace adsld

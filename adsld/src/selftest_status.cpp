// adsld selftest: the ADS-L Status payload (ADS-L.4.SRD860.G.2) end to end, and the MAC
// rules that arrived with it. No reference implementation carries Status (the ogn-tracker
// reference has none), so the bit order is pinned to the spec's own worked example (B.3) and
// the scrambler to the spec's own sample code (E.2); every other case builds on that:
//   - bitfield access in the B.3 order; XXTEA against the E.2 sample code, verbatim
//   - the 17-byte packet: layout, random round trips, 1-bit correction, rejection of
//     two-bit errors and of the other decoder's frames - by length and by payload type
//   - the real modulator and correlator: a 17-byte frame next to 25-byte ones, 12 dB SNR
//     with carrier offset, and frames whose LEN chips cannot tell the length (the CRC does)
//   - the scheduler: Status every status_interval_s, Traffic and Status each in their own
//     channel alternation (C.2.5), >= 100 ms between frames on one channel (EN 300 220-2
//     V3.3.1 Table 18), one channel jammed at 1 Hz, both channels jammed, a reconfigure in the middle of a run, and a manual
//     beacon that has to obey the same rules
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "adsl_codec.h"
#include "adsl_phy.h"
#include "channelizer.h"
#include "gfsk_mod.h"
#include "receiver.h"
#include "selftest.h"
#include "txsched.h"

namespace adsld {

namespace {

constexpr double FS_IN = 2e6;      // capture rate
constexpr double FS_OUT = 400e3;   // demod rate (4 samples/chip)
constexpr double LO_HZ = 867.9e6;  // channels at +300 kHz / +500 kHz
constexpr double DDC_CUTOFF = 130e3;
constexpr float SIG_AMP = 0.25f;   // same burst amplitude as selftest_loopback

// the two chips of LEN bit 3 - the only bit in which 0x18 and 0x10 differ: preamble (24)
// and sync word (32) first, then the LEN byte MSB first, so bit 3 is its fifth bit
constexpr size_t LEN_BIT3_CHIP = (size_t)(ADSL_PREAMBLE_CHIPS + ADSL_SYNC_CHIPS + 2 * 4);

adsl_status random_status(std::mt19937 &rng) {
	adsl_status s;
	s.amt = (uint8_t)(rng() & 0x3F);
	s.addr = rng() & 0xFFFFFF;
	s.relay = (rng() & 1) != 0;
	s.proto_ver = (uint8_t)(rng() & 0xF);
	s.key_idx = (uint8_t)(rng() % 4);
	s.max_proto_ver = (uint8_t)(rng() & 0xF);
	s.rx_mband = (uint8_t)(rng() & 3);
	s.rx_oband_ldr = (uint8_t)(rng() & 3);
	s.rx_oband_hdr = (uint8_t)(rng() & 3);
	s.mobile = (uint8_t)(rng() & 3);
	s.ec_rx = (uint8_t)(rng() & 0xFF);
	s.ec_tx = (uint8_t)(rng() & 0xFF);
	s.xpdr = (uint8_t)(rng() & 3);
	s.uplink_client = (rng() & 1) != 0;
	s.fisb_client = (rng() & 1) != 0;
	return s;
}

bool same_status(const adsl_status &a, const adsl_status &b) {
	return a.amt == b.amt && a.addr == b.addr && a.relay == b.relay &&
	       a.proto_ver == b.proto_ver && a.key_idx == b.key_idx &&
	       a.max_proto_ver == b.max_proto_ver && a.rx_mband == b.rx_mband &&
	       a.rx_oband_ldr == b.rx_oband_ldr && a.rx_oband_hdr == b.rx_oband_hdr &&
	       a.mobile == b.mobile && a.ec_rx == b.ec_rx && a.ec_tx == b.ec_tx &&
	       a.xpdr == b.xpdr && a.uplink_client == b.uplink_client &&
	       a.fisb_client == b.fisb_client;
}

// The XXTEA sample code of ADS-L.4.SRD860.E.2, verbatim apart from C++ spelling - an oracle
// that shares no line with adsl_codec.cpp.
#define SPEC_MX ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))
void spec_xxtea_encrypt(uint32_t *data, uint32_t num_data_words, const uint32_t *key,
                        uint32_t round) {
	uint32_t z, y = data[0], sum = 0, e, DELTA = 0x9e3779b9;
	uint32_t p, q;
	z = data[num_data_words - 1];
	q = round;
	while(q-- > 0) {
		sum += DELTA;
		e = (sum >> 2) & 3;
		for(p = 0; p < num_data_words - 1; p++) {
			y = data[p + 1];
			data[p] += SPEC_MX;
			z = data[p];
		}
		y = data[0];
		data[num_data_words - 1] += SPEC_MX;
		z = data[num_data_words - 1];
	}
}
#undef SPEC_MX

// production TX chain at the channel offset
std::vector<cf> render_chips(int ch, const bits &chips, double cfo_hz = 0) {
	return gfsk_render_burst(chips, FS_IN, ADSL_CH_HZ[ch] - LO_HZ + cfo_hz, SIG_AMP);
}

std::vector<cf> render(int ch, const uint8_t *pkt, size_t len, double cfo_hz = 0) {
	return render_chips(ch, frame_build(pkt, len), cfo_hz);
}

// stimulus + optional AWGN (total complex rms) -> ddc -> receiver; every delivery kept
std::vector<std::vector<uint8_t>> run_rx(int ch, const std::vector<cf> &sig, double noise_rms,
                                         uint32_t seed) {
	std::vector<std::vector<uint8_t>> got;
	ddc chan(FS_IN, FS_OUT, ADSL_CH_HZ[ch] - LO_HZ, DDC_CUTOFF);
	adsl_receiver rx(ch, ADSL_CH_HZ[ch], FS_OUT,
	                 [&got](const uint8_t *p, const adsl_rx_meta &) {
		                 got.emplace_back(p, p + ADSL_PKT_LEN);
	                 });
	std::mt19937 rng(seed);
	std::normal_distribution<float> gauss(0.0f, (float)(noise_rms / std::sqrt(2.0)));
	const size_t lead = 4000, tail = 4000;
	const size_t total = sig.size() + lead + tail;
	std::vector<cf> chunk, out;
	for(size_t i = 0; i < total;) {
		chunk.clear();
		for(; i < total && chunk.size() < 8192; i++) {
			cf x = (i >= lead && i - lead < sig.size()) ? sig[i - lead] : cf(0, 0);
			if(noise_rms > 0) {
				x += cf(gauss(rng), gauss(rng));
			}
			chunk.push_back(x);
		}
		out.clear();
		chan.process(chunk.data(), chunk.size(), out);
		rx.feed(out.data(), out.size());
	}
	return got;
}

int count_matches(const std::vector<std::vector<uint8_t>> &got, const uint8_t *pkt) {
	int n = 0;
	for(const auto &p : got) {
		if(memcmp(p.data(), pkt, ADSL_PKT_LEN) == 0) {
			n++;
		}
	}
	return n;
}

// always-fresh fix; flightstate 2 = airborne (1 Hz Traffic)
class fixed_pos : public position_source {
public:
	explicit fixed_pos(uint8_t flightstate) : fs_(flightstate) {
	}

	bool get(uint64_t now_ms, position_fix &out) override {
		out = position_fix();
		out.valid = true;
		out.lat_deg = 53.55;
		out.lon_deg = 9.99;
		out.alt_m = 50;
		out.flightstate = fs_;
		out.time_ms = now_ms;
		out.ts_qs = (uint8_t)((now_ms / 250) % 60);
		return true;
	}

private:
	uint8_t fs_;
};

struct sent {
	uint64_t t;
	int ch;
	tx_kind kind;
	std::vector<uint8_t> pkt;
};

// drive the scheduler over [t0, t1] in `step` ms with a fixed busy/free state per channel
void drive(txsched &s, std::vector<sent> &log, uint64_t t0, uint64_t t1, uint64_t step,
           bool busy0, bool busy1) {
	const float noise[ADSL_NUM_CHANNELS] = { -95.0f, -95.0f };
	const float rssi[ADSL_NUM_CHANNELS] = { busy0 ? -75.0f : -95.0f, busy1 ? -75.0f : -95.0f };
	for(uint64_t t = t0; t <= t1; t += step) {
		auto r = s.tick(t, rssi, noise);
		if(r) {
			log.push_back({ t, r->channel, r->kind, r->packet });
		}
	}
}

size_t count_of(const std::vector<sent> &log, tx_kind k, int ch = -1) {
	size_t n = 0;
	for(const auto &e : log) {
		if(e.kind == k && (ch < 0 || e.ch == ch)) {
			n++;
		}
	}
	return n;
}

bool alternates(const std::vector<sent> &log, tx_kind k) {
	int last = -1;
	for(const auto &e : log) {
		if(e.kind != k) {
			continue;
		}
		if(e.ch == last) {
			return false;
		}
		last = e.ch;
	}
	return true;
}

uint64_t min_channel_gap(const std::vector<sent> &log) {
	uint64_t last[ADSL_NUM_CHANNELS] = { 0 };
	bool have[ADSL_NUM_CHANNELS] = { false };
	uint64_t gap = ~0ull;
	for(const auto &e : log) {
		if(have[e.ch]) {
			gap = std::min(gap, e.t - last[e.ch]);
		}
		have[e.ch] = true;
		last[e.ch] = e.t;
	}
	return gap;
}

bool decode_status(const sent &e, adsl_status &out) {
	if(e.pkt.size() != ADSL_STATUS_PKT_LEN) {
		return false;
	}
	uint8_t buf[ADSL_PKT_LEN] = { 0 };
	memcpy(buf, e.pkt.data(), e.pkt.size());
	return adsl_status_decode(buf, out) == adsl_decode_result::ok;
}

txsched_config beacon_cfg(position_source *pos, uint32_t seed) {
	txsched_config c;
	c.pos = pos;
	c.tmpl.amt = 0;
	c.tmpl.addr = 0xA1B2C3;
	c.tmpl.category = 11;
	c.seed = seed;
	c.status_tmpl.max_proto_ver = 1;
	c.status_tmpl.rx_mband = 3;
	return c;
}

} // namespace

int selftest_status() {
	int fails = 0;
	char detail[400];

	// ---- B.3 bit order ---------------------------------------------------------------
	{
		uint8_t d[3] = { 0 };
		adsl_put_bits(d, 0, 14, 0x12AA);
		adsl_put_bits(d, 14, 10, 0x355);
		bool ok = d[0] == 0xAA && d[1] == 0x52 && d[2] == 0xD5 &&
		          adsl_get_bits(d, 0, 14) == 0x12AA && adsl_get_bits(d, 14, 10) == 0x355;
		snprintf(detail, sizeof(detail), "%02X %02X %02X (B.3 bit string 10101010 01010010 "
		         "11010101; its prose says 0x55 for byte 2)", d[0], d[1], d[2]);
		check(fails, ok, "status: B.3 example 2 bit order", detail);
	}
	{
		uint8_t d[8];
		memset(d, 0xFF, sizeof(d));
		adsl_put_bits(d, 13, 7, 0);
		bool ok = adsl_get_bits(d, 0, 13) == 0x1FFF && adsl_get_bits(d, 13, 7) == 0 &&
		          adsl_get_bits(d, 20, 44) == 0xFFFFFFFFFFFull;
		const uint8_t w[4] = { 0x78, 0x56, 0x34, 0x12 };
		ok = ok && adsl_get_bits(w, 0, 32) == 0x12345678u;
		check(fails, ok, "status: bitfields leave their neighbours alone, 32 bits = LE word");
	}

	// ---- XXTEA against the spec's own sample code ------------------------------------
	{
		std::mt19937 rng(0x7EA);
		const uint32_t zero_key[4] = { 0, 0, 0, 0 };
		int bad = 0;
		for(int i = 0; i < 300; i++) {
			for(uint32_t n = 3; n <= 5; n += 2) {
				uint32_t spec[5], ours[5], orig[5];
				for(uint32_t k = 0; k < n; k++) {
					orig[k] = spec[k] = ours[k] = rng();
				}
				spec_xxtea_encrypt(spec, n, zero_key, 6);
				if(n == 5) {
					xxtea_scramble(ours);
				} else {
					xxtea_scramble_n(ours, n);
				}
				if(memcmp(spec, ours, n * sizeof(uint32_t)) != 0) {
					bad++;
				}
				xxtea_descramble_n(ours, n);
				if(memcmp(orig, ours, n * sizeof(uint32_t)) != 0) {
					bad++;
				}
			}
		}
		snprintf(detail, sizeof(detail), "%d mismatches in 600 blocks of 3 and 5 words", bad);
		check(fails, bad == 0, "status: XXTEA equals the E.2 sample code (6 rounds, zero key), "
		      "descramble inverts it", detail);
	}

	// ---- packet layout ---------------------------------------------------------------
	{
		adsl_status s;
		s.amt = 0;
		s.addr = 0xA1B2C3;
		s.max_proto_ver = 1;
		s.rx_mband = 3;
		s.key_idx = 3;   // scrambling off: the data block is readable on the wire
		uint8_t pkt[ADSL_PKT_LEN];
		memset(pkt, 0xEE, sizeof(pkt));
		size_t n = adsl_status_encode(s, pkt);
		bool ok = n == ADSL_STATUS_PKT_LEN && pkt[0] == ADSL_LEN_STATUS && pkt[1] == 0x61 &&
		          pkt[2] == ADSL_PAYLOAD_STATUS && adsl_get_bits(pkt + 2, 8, 6) == 0 &&
		          adsl_get_bits(pkt + 2, 14, 24) == 0xA1B2C3 && pkt[7] == 0x31;
		for(size_t i = 8; i < 14; i++) {
			ok = ok && pkt[i] == 0;          // rest of the payload + Reserved(24)
		}
		uint32_t c = crc24(pkt + 1, 1 + ADSL_STATUS_DATA_LEN);
		ok = ok && pkt[14] == (uint8_t)(c >> 16) && pkt[15] == (uint8_t)(c >> 8) &&
		     pkt[16] == (uint8_t)c;
		for(size_t i = ADSL_STATUS_PKT_LEN; i < ADSL_PKT_LEN; i++) {
			ok = ok && pkt[i] == 0;
		}
		snprintf(detail, sizeof(detail), "LEN %02X hdr %02X type %02X payload[0] %02X, %zu B",
		         pkt[0], pkt[1], pkt[2], pkt[7], n);
		check(fails, ok, "status: layout - LEN 0x10, header, payload from bit 40, CRC over "
		      "13 B, zero tail", detail);
	}

	// ---- random round trips ------------------------------------------------------------
	{
		std::mt19937 rng(0x57A7);
		int bad = 0;
		for(int i = 0; i < 500; i++) {
			adsl_status s = random_status(rng);
			uint8_t pkt[ADSL_PKT_LEN];
			adsl_status_encode(s, pkt);
			adsl_status d;
			if(adsl_status_decode(pkt, d) != adsl_decode_result::ok || !same_status(s, d)) {
				bad++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/500 bad", bad);
		check(fails, bad == 0, "status: 500 random encode/decode round trips (all key indices)",
		      detail);
	}

	// ---- error control -----------------------------------------------------------------
	{
		adsl_status s;
		s.amt = 2;
		s.addr = 0x3D5C01;
		s.max_proto_ver = 1;
		s.rx_mband = 3;
		uint8_t pkt[ADSL_PKT_LEN];
		adsl_status_encode(s, pkt);
		int bad = 0;
		const int nbits = (int)(1 + ADSL_STATUS_DATA_LEN + 3) * 8;   // 128
		for(int bit = 0; bit < nbits; bit++) {
			uint8_t p2[ADSL_PKT_LEN];
			memcpy(p2, pkt, sizeof(p2));
			p2[1 + bit / 8] ^= (uint8_t)(0x80u >> (bit % 8));
			adsl_status d;
			if(adsl_status_decode(p2, d) != adsl_decode_result::corrected_1bit ||
			   !same_status(s, d)) {
				bad++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/%d positions bad", bad, nbits);
		check(fails, bad == 0, "status: 1-bit correction at every position", detail);

		std::mt19937 rng(0xB17);
		int accepted = 0, tried = 0;
		while(tried < 300) {
			int b1 = (int)(rng() % (uint32_t)nbits), b2 = (int)(rng() % (uint32_t)nbits);
			if(b1 == b2) {
				continue;
			}
			tried++;
			uint8_t p2[ADSL_PKT_LEN];
			memcpy(p2, pkt, sizeof(p2));
			p2[1 + b1 / 8] ^= (uint8_t)(0x80u >> (b1 % 8));
			p2[1 + b2 / 8] ^= (uint8_t)(0x80u >> (b2 % 8));
			adsl_status d;
			if(adsl_status_decode(p2, d) != adsl_decode_result::crc_fail) {
				accepted++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/300 double flips accepted", accepted);
		check(fails, accepted == 0, "status: 2-bit errors rejected", detail);
	}

	// ---- the decoders keep to their own frames ----------------------------------------
	{
		adsl_status s;
		s.addr = 0x123456;
		uint8_t sp[ADSL_PKT_LEN];
		adsl_status_encode(s, sp);
		adsl_traffic t;
		t.addr = 0x123456;
		t.pos_valid = true;
		t.lat_deg = 1.0;
		t.lon_deg = 2.0;
		uint8_t tp[ADSL_PKT_LEN];
		adsl_encode(t, tp);
		adsl_traffic td;
		adsl_status sd;
		// by length: every decoder checks the LEN byte first
		bool ok = adsl_decode(sp, td) == adsl_decode_result::crc_fail &&
		          adsl_status_decode(tp, sd) == adsl_decode_result::crc_fail &&
		          adsl_onair_len(ADSL_LEN_TRAFFIC) == ADSL_PKT_LEN &&
		          adsl_onair_len(ADSL_LEN_STATUS) == ADSL_STATUS_PKT_LEN &&
		          adsl_onair_len(0x11) == 0;
		// by payload type, at the right length and with a valid CRC: a 16-byte packet of
		// another type is no Status report ...
		adsl_status clear = s;
		clear.key_idx = 3;
		uint8_t op[ADSL_PKT_LEN];
		adsl_status_encode(clear, op);
		op[2] = 4;   // Traffic Uplink's type number
		uint32_t c = crc24(op + 1, 1 + ADSL_STATUS_DATA_LEN);
		op[14] = (uint8_t)(c >> 16);
		op[15] = (uint8_t)(c >> 8);
		op[16] = (uint8_t)c;
		ok = ok && adsl_status_decode(op, sd) == adsl_decode_result::crc_fail;
		// ... and a 24-byte packet that says "Status" is no Traffic report
		adsl_traffic tclear = t;
		tclear.key_idx = 3;
		uint8_t xp[ADSL_PKT_LEN];
		adsl_encode(tclear, xp);
		xp[2] = ADSL_PAYLOAD_STATUS;
		c = crc24(xp + 1, ADSL_CRC_SPAN);
		xp[22] = (uint8_t)(c >> 16);
		xp[23] = (uint8_t)(c >> 8);
		xp[24] = (uint8_t)c;
		ok = ok && adsl_decode(xp, td) == adsl_decode_result::crc_fail;
		check(fails, ok, "status: Traffic and Status decoders reject each other's frames, "
		      "by length and by payload type");
		// the daemon labels what neither decoder takes as an unknown payload type
		check(fails, adsl_payload_type(tp) == ADSL_PAYLOAD_TRAFFIC &&
		             adsl_payload_type(sp) == ADSL_PAYLOAD_STATUS && adsl_payload_type(op) == 4 &&
		             adsl_payload_type(xp) == ADSL_PAYLOAD_STATUS,
		      "status: the payload type of a CRC-valid frame reads back, scrambled or not");
		uint8_t no[ADSL_PKT_LEN] = { 0x11 };
		check(fails, adsl_payload_type(no) == -1, "status: an unknown LEN has no payload type");
	}

	// ---- the real PHY: a 17-byte frame through modulator, DDC and correlator ----------
	adsl_status s_ref;
	s_ref.addr = 0xA1B2C3;
	s_ref.max_proto_ver = 1;
	s_ref.rx_mband = 3;
	uint8_t sp_ref[ADSL_PKT_LEN];
	const size_t sn = adsl_status_encode(s_ref, sp_ref);
	adsl_traffic t_ref;
	t_ref.addr = 0xA1B2C3;
	t_ref.flightstate = 2;
	t_ref.pos_valid = true;
	t_ref.lat_deg = 53.55;
	t_ref.lon_deg = 9.99;
	t_ref.alt_valid = true;
	t_ref.alt_m = 50;
	uint8_t tp_ref[ADSL_PKT_LEN];
	adsl_encode(t_ref, tp_ref);
	{
		bits chips = frame_build(sp_ref, sn);
		snprintf(detail, sizeof(detail), "%zu chips = %.2f ms", chips.size(),
		         (double)chips.size() / ADSL_CHIP_RATE_HZ * 1e3);
		check(fails, chips.size() == (size_t)(ADSL_PREAMBLE_CHIPS + (2 + sn) * 16),
		      "status: frame is 328 chips on air", detail);
		for(int ch = 0; ch < ADSL_NUM_CHANNELS; ch++) {
			auto got = run_rx(ch, render(ch, sp_ref, sn), 0.0, 1);
			snprintf(detail, sizeof(detail), "ch%d: %zu deliveries, %d byte-exact", ch,
			         got.size(), count_matches(got, sp_ref));
			check(fails, got.size() == 1 && count_matches(got, sp_ref) == 1,
			      "status: clean frame decoded once, byte-exact incl. zero tail", detail);
		}
	}
	{
		// Traffic, 5 ms, Status, 5 ms, the same Traffic again: the correlator has to pick the
		// length per frame, and nothing may be lost or invented
		std::vector<cf> sig;
		const std::vector<cf> gap((size_t)(0.005 * FS_IN), cf(0, 0));
		for(const auto &part : { render(0, tp_ref, ADSL_PKT_LEN), gap, render(0, sp_ref, sn), gap,
		                         render(0, tp_ref, ADSL_PKT_LEN) }) {
			sig.insert(sig.end(), part.begin(), part.end());
		}
		auto got = run_rx(0, sig, 0.0, 2);
		int nt = count_matches(got, tp_ref), ns = count_matches(got, sp_ref);
		snprintf(detail, sizeof(detail), "%zu deliveries: %d Traffic, %d Status", got.size(),
		         nt, ns);
		check(fails, got.size() == 3 && nt == 2 && ns == 1,
		      "status: Traffic/Status/Traffic back to back, each at its own length", detail);
	}
	{
		// 12 dB in-channel SNR (selftest_loopback's convention), carrier offset -15/0/+15 kHz
		const double snr_db = 12.0;
		const double rms = SIG_AMP * std::sqrt(FS_IN / FS_OUT) * std::pow(10.0, -snr_db / 20.0);
		const double cfos[] = { -15e3, 0.0, 15e3 };
		int good = 0, trials = 0;
		for(int k = 0; k < 3; k++) {
			const std::vector<cf> burst = render(1, sp_ref, sn, cfos[k]);
			for(uint32_t i = 0; i < 40; i++) {
				auto got = run_rx(1, burst, rms, 900 + (uint32_t)k * 100 + i);
				trials++;
				if(count_matches(got, sp_ref) >= 1) {
					good++;
				}
			}
		}
		snprintf(detail, sizeof(detail), "%d/%d decoded", good, trials);
		check(fails, good == trials, "status: 12 dB SNR with CFO -15/0/+15 kHz", detail);
	}
	{
		// LEN bit 3 transmitted as "00" or "11": one chip error against either length, a tie
		// the hard chips cannot resolve - both lengths are captured and the CRC decides
		const double rms = SIG_AMP * std::sqrt(FS_IN / FS_OUT) * std::pow(10.0, -12.0 / 20.0);
		int bad = 0, trials = 0;
		for(int which = 0; which < 2; which++) {
			const uint8_t *pkt = which ? sp_ref : tp_ref;
			const size_t len = which ? sn : ADSL_PKT_LEN;
			for(int level = 0; level < 2; level++) {
				bits chips = frame_build(pkt, len);
				chips[LEN_BIT3_CHIP] = (uint8_t)level;
				chips[LEN_BIT3_CHIP + 1] = (uint8_t)level;
				const std::vector<cf> burst = render_chips(0, chips);
				auto got = run_rx(0, burst, 0.0, 3);
				trials++;
				if(got.size() != 1 || count_matches(got, pkt) != 1) {
					bad++;
				}
				for(uint32_t i = 0; i < 10; i++) {
					auto g = run_rx(0, burst, rms, 700 + (uint32_t)(which * 40 + level * 20) + i);
					trials++;
					if(count_matches(g, pkt) < 1) {
						bad++;
					}
				}
			}
		}
		snprintf(detail, sizeof(detail), "%d/%d frames lost", bad, trials);
		check(fails, bad == 0, "status: ambiguous LEN chips - the CRC picks the length "
		      "(Traffic and Status, clean and 12 dB)", detail);
	}

	// ---- scheduler: free air --------------------------------------------------------------
	{
		fixed_pos pos(2);
		const txsched_config c = beacon_cfg(&pos, 77);
		txsched sched;
		sched.init(c);
		std::vector<sent> log;
		drive(sched, log, 0, 120500, 1, false, false);   // 1 ms ticks: Toff at ms resolution

		const uint64_t period = (uint64_t)std::llround(c.status_interval_s * 1000.0);
		std::vector<uint64_t> st;
		bool content = true;
		for(const auto &e : log) {
			if(e.kind != tx_kind::status) {
				continue;
			}
			st.push_back(e.t);
			adsl_status d;
			if(!decode_status(e, d) || d.amt != 0 || d.addr != 0xA1B2C3 || d.max_proto_ver != 1 ||
			   d.rx_mband != 3) {
				content = false;
			}
		}
		bool cadence = st.size() == (size_t)(120500 / period + 1) && st[0] < 1000;
		for(size_t i = 1; i < st.size() && cadence; i++) {
			const uint64_t dt = st[i] - st[i - 1];
			cadence = dt >= period && dt <= period + (uint64_t)txsched::TOFF_MIN_MS;
		}
		snprintf(detail, sizeof(detail), "%zu Status frames, period %llu ms, first at %llu ms",
		         st.size(), (unsigned long long)period,
		         st.empty() ? 0ull : (unsigned long long)st[0]);
		check(fails, cadence && content,
		      "status: every status_interval_s, carrying the beacon address and its capabilities",
		      detail);

		const size_t nt = count_of(log, tx_kind::traffic);
		snprintf(detail, sizeof(detail), "%zu Traffic, %zu Status", nt, st.size());
		check(fails, alternates(log, tx_kind::traffic) && alternates(log, tx_kind::status),
		      "status: Traffic and Status each alternate on every frame of their kind (C.2.5)",
		      detail);

		const uint64_t gap = min_channel_gap(log);
		snprintf(detail, sizeof(detail), "closest two frames on one channel: %llu ms",
		         (unsigned long long)gap);
		check(fails, gap >= (uint64_t)txsched::TOFF_MIN_MS,
		      "status: >= 100 ms between frames on the same channel (EN 300 220-2 Tab. 18)",
		      detail);

		snprintf(detail, sizeof(detail), "tx_count %llu, logged %zu, Traffic %zu, forced %llu",
		         (unsigned long long)sched.tx_count(), log.size(), nt,
		         (unsigned long long)sched.forced_count());
		check(fails, sched.tx_count() == log.size() && sched.status_frames() == st.size() &&
		                 sched.forced_count() == 0 && nt >= 120 && nt <= 121,
		      "status: counters match the air, the 1 Hz Traffic cadence is untouched", detail);
	}

	// ---- scheduler: one channel jammed at 1 Hz ------------------------------------------
	{
		// ch0 stays busy for 5 minutes, ch1 is free: Status must not be tied to Traffic's
		// channel, or it loses every backoff episode to the next Traffic slot.
		fixed_pos pos(2);
		txsched sched;
		sched.init(beacon_cfg(&pos, 78));
		std::vector<sent> log;
		drive(sched, log, 0, 300000, 10, true, false);
		const size_t st0 = count_of(log, tx_kind::status, 0), st1 = count_of(log, tx_kind::status, 1);
		size_t on_ch0 = 0;
		bool quiet_ok = true;
		for(size_t i = 0; i < log.size(); i++) {
			if(log[i].ch != 0) {
				continue;
			}
			on_ch0++;
			if(i + 1 < log.size() && log[i + 1].t - log[i].t < (uint64_t)txsched::LBT_QUIET_MS) {
				quiet_ok = false;
			}
		}
		snprintf(detail, sizeof(detail), "Status %zu on ch0 + %zu on ch1 (%llu forced); %zu frames "
		         "on ch0, %llu forced; Traffic %zu",
		         st0, st1, (unsigned long long)sched.status_forced(), on_ch0,
		         (unsigned long long)sched.forced_count(), count_of(log, tx_kind::traffic));
		check(fails, st0 + st1 >= 12 && st0 >= 1 && st1 >= 1 && alternates(log, tx_kind::status) &&
		                 alternates(log, tx_kind::traffic) && on_ch0 > 0 &&
		                 sched.forced_count() == on_ch0 && sched.status_forced() >= 1 && quiet_ok,
		      "status: one channel jammed at 1 Hz - Status keeps its alternation, forced only on "
		      "the jammed side", detail);
	}

	// ---- scheduler: both channels jammed ------------------------------------------------
	{
		fixed_pos pos(2);
		txsched sched;
		sched.init(beacon_cfg(&pos, 79));
		std::vector<sent> log;
		drive(sched, log, 0, 60000, 10, true, true);
		uint64_t spacing = ~0ull;
		for(size_t i = 1; i < log.size(); i++) {
			spacing = std::min(spacing, log[i].t - log[i - 1].t);
		}
		snprintf(detail, sizeof(detail), "%zu frames, %llu forced (%llu Status), closest %llu ms "
		         "apart", log.size(),
		         (unsigned long long)sched.forced_count(), (unsigned long long)sched.status_forced(),
		         (unsigned long long)spacing);
		check(fails, !log.empty() && sched.tx_count() == sched.forced_count() &&
		                 sched.status_forced() >= 1 && spacing >= (uint64_t)txsched::LBT_QUIET_MS,
		      "status: both channels jammed - Traffic and Status forced after their own 3 s, "
		      ">= 2 s apart", detail);
	}

	// ---- scheduler: reconfigure in the middle of a run ----------------------------------
	{
		fixed_pos pos(2);
		const txsched_config c = beacon_cfg(&pos, 80);
		txsched sched;
		sched.init(c);
		std::vector<sent> log;
		drive(sched, log, 0, 5003, 1, false, false);
		txsched_config c2 = c;
		c2.tmpl.category = 12;
		sched.reconfigure(c2);            // like POST /config {"category": 12}
		drive(sched, log, 5004, 12345, 1, false, false);
		txsched_config c3 = c2;
		c3.tmpl.addr = 0x0BADA5;
		sched.reconfigure(c3);            // like POST /config {"addr": "0BADA5"}
		drive(sched, log, 12346, 40000, 1, false, false);
		bool fields = true;
		size_t after_addr = 0;
		for(const auto &e : log) {
			if(e.kind == tx_kind::traffic) {
				adsl_traffic t;
				if(adsl_decode(e.pkt.data(), t) == adsl_decode_result::crc_fail ||
				   (e.t > 5003 && t.category != 12) || (e.t > 12345 && t.addr != 0x0BADA5)) {
					fields = false;
				}
				after_addr += e.t > 12345 ? 1 : 0;
			} else if(e.kind == tx_kind::status && e.t > 12345) {
				adsl_status s;
				fields = fields && decode_status(e, s) && s.addr == 0x0BADA5;
			}
		}
		const size_t ns = count_of(log, tx_kind::status);
		snprintf(detail, sizeof(detail), "%zu Status in 40 s, closest same-channel gap %llu ms, "
		         "%zu Traffic after the address change", ns,
		         (unsigned long long)min_channel_gap(log), after_addr);
		check(fails, alternates(log, tx_kind::traffic) && alternates(log, tx_kind::status) &&
		                 min_channel_gap(log) >= (uint64_t)txsched::TOFF_MIN_MS && ns == 3 &&
		                 fields && after_addr > 0,
		      "status: reconfigure keeps alternation, Toff and the Status schedule; the new "
		      "fields go on air", detail);
	}

	// ---- scheduler: a manual beacon obeys the same rules --------------------------------
	{
		fixed_pos pos(2);
		txsched_config c = beacon_cfg(&pos, 81);
		c.status_interval_s = 0;          // Traffic only: the question is Toff and alternation
		txsched sched;
		sched.init(c);
		std::vector<sent> log;
		drive(sched, log, 0, 1000, 1, false, false);   // Traffic at 0 on ch0, at 1000 on ch1
		const float quiet[ADSL_NUM_CHANNELS] = { -95.0f, -95.0f };
		const float busy[ADSL_NUM_CHANNELS] = { -75.0f, -75.0f };
		const float noise[ADSL_NUM_CHANNELS] = { -95.0f, -95.0f };
		bool ok = log.size() == 2 && log[1].ch == 1 && log[1].t == 1000;
		const bool toff_refused = !sched.claim_manual_traffic(1, 1050, quiet, noise);
		const bool lbt_refused = !sched.claim_manual_traffic(0, 1050, busy, noise);
		const bool claimed = sched.claim_manual_traffic(0, 1050, quiet, noise);
		ok = ok && toff_refused && lbt_refused && claimed && sched.next_channel() == 1;
		drive(sched, log, 1051, 2100, 1, false, false);
		ok = ok && log.size() == 3 && log[2].ch == 1 && log[2].t == 2000;
		snprintf(detail, sizeof(detail), "Toff refused %d, LBT refused %d, claimed %d, next "
		         "scheduled Traffic at %llu ms on ch%d", toff_refused, lbt_refused, claimed,
		         log.size() > 2 ? (unsigned long long)log[2].t : 0ull,
		         log.size() > 2 ? log[2].ch : -1);
		check(fails, ok, "status: a manual beacon obeys Toff and LBT and continues the Traffic "
		      "alternation", detail);
	}

	return fails;
}

} // namespace adsld

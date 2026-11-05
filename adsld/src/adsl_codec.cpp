// adsld: ADS-L link-layer codec implementation - see adsl_codec.h for the contract.
//
// Byte-exactness is cross-validated in selftest_codec.cpp against wire packets produced
// with P. Jalocha's ads-l reference implementation (ogn-tracker); the reference itself
// stays out of this repo. The reference has no Status payload - selftest_status.cpp holds
// that one to the bit order of B.3 instead.
#include "adsl_codec.h"

#include <cmath>
#include <cstring>

namespace adsld {

// ---- CRC24 ---------------------------------------------------------------------------
// Mode-S style CRC: the 32-bit register shifts the message through MSB-first with the
// left-aligned polynomial constant 0xFFFA0480 (equivalent generator 0x1FFF409); a byte
// enters the LOW eight bits and the register's top byte never holds message bits, hence
// the final >>8. Three zero flush bytes push the last message byte fully through.

static uint32_t crc24_pass(uint32_t crc, uint8_t byte) {
	crc |= byte;
	for(int bit = 0; bit < 8; bit++) {
		if(crc & 0x80000000u) {
			crc ^= 0xFFFA0480u;
		}
		crc <<= 1;
	}
	return crc;
}

uint32_t crc24(const uint8_t *data, size_t len) {
	uint32_t crc = 0;
	for(size_t i = 0; i < len; i++) {
		crc = crc24_pass(crc, data[i]);
	}
	crc = crc24_pass(crc, 0);
	crc = crc24_pass(crc, 0);
	crc = crc24_pass(crc, 0);
	return crc >> 8;
}

// check-form pass over message||CRC (no flush bytes): zero syndrome iff the CRC is good
static uint32_t crc24_syndrome(const uint8_t *buf, size_t len) {
	uint32_t crc = 0;
	for(size_t i = 0; i < len; i++) {
		crc = crc24_pass(crc, buf[i]);
	}
	return crc >> 8;
}

// Syndrome of a single flipped bit depends only on its distance from the end of the
// buffer: distance j (j=0 is the LSB of the last CRC byte) has syndrome x^j mod G.
// Table built lazily for the largest span we correct (24 bytes = 192 bits); shorter
// packets (Status: 16 bytes) use its first 128 entries.
static constexpr int CRC24_MAX_BITS = (int)(ADSL_CRC_SPAN + 3) * 8;

static const uint32_t *crc24_syndrome_table() {
	static uint32_t table[CRC24_MAX_BITS];
	static bool built = false;
	if(!built) {
		uint32_t s = 1;                      // j=0: flip of the very last bit
		for(int j = 0; j < CRC24_MAX_BITS; j++) {
			table[j] = s;
			s <<= 1;
			if(s & 0x1000000u) {
				s = (s & 0xFFFFFFu) ^ 0xFFF409u;
			}
		}
		built = true;
	}
	return table;
}

int crc24_correct_1bit(uint8_t *buf, size_t len) {
	uint32_t syn = crc24_syndrome(buf, len);
	if(syn == 0) {
		return -1;
	}
	int nbits = (int)(len * 8);
	int span = nbits < CRC24_MAX_BITS ? nbits : CRC24_MAX_BITS;
	const uint32_t *table = crc24_syndrome_table();
	for(int j = 0; j < span; j++) {
		if(table[j] == syn) {
			int bit = nbits - 1 - j;         // 0 = MSB of buf[0]
			buf[bit / 8] ^= (uint8_t)(0x80u >> (bit & 7));
			return bit;
		}
	}
	return -2;
}

// ---- XXTEA "scrambler" ---------------------------------------------------------------
// All-zero key, 6 rounds, DELTA 0x9E3779B9, over the ADS-L data block as little-endian
// words: FIVE for Traffic (the spec's table wrongly prints 6), THREE for Status. With a
// zero key the key-schedule term drops out of MX.

static inline uint32_t xxtea_mx(uint32_t y, uint32_t z, uint32_t sum) {
	return (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ ((sum ^ y) + z);
}

void xxtea_scramble_n(uint32_t *w, size_t n) {
	constexpr uint32_t DELTA = 0x9E3779B9u;
	uint32_t sum = 0;
	uint32_t z = w[n - 1];
	for(int round = 0; round < 6; round++) {
		sum += DELTA;
		for(size_t p = 0; p + 1 < n; p++) {
			z = w[p] += xxtea_mx(w[p + 1], z, sum);
		}
		z = w[n - 1] += xxtea_mx(w[0], z, sum);
	}
}

void xxtea_descramble_n(uint32_t *w, size_t n) {
	constexpr uint32_t DELTA = 0x9E3779B9u;
	uint32_t sum = 6 * DELTA;
	uint32_t y = w[0];
	for(int round = 0; round < 6; round++) {
		for(size_t p = n - 1; p > 0; p--) {
			y = w[p] -= xxtea_mx(y, w[p - 1], sum);
		}
		y = w[0] -= xxtea_mx(y, w[n - 1], sum);
		sum -= DELTA;
	}
}

void xxtea_scramble(uint32_t w[5]) {
	xxtea_scramble_n(w, 5);
}

void xxtea_descramble(uint32_t w[5]) {
	xxtea_descramble_n(w, 5);
}

// ---- exponential field coding --------------------------------------------------------
// <exp:2><base:N>: value = 2^e*(2^N+base) - 2^N. The spec's encoding algorithm: largest
// e with value >= 2^(N+e)-2^N, then base = (value - 2^(N+e) + 2^N) >> e (truncating).

uint32_t exp_enc(uint32_t value, int nbase) {
	uint32_t thres = 1u << nbase;
	int e = 3;
	while(e > 0 && value < (thres << e) - thres) {
		e--;
	}
	uint32_t base = (value - ((thres << e) - thres)) >> e;
	if(base > thres - 1) {
		base = thres - 1;                    // clamp to the top of the e=3 range
	}
	return ((uint32_t)e << nbase) | base;
}

uint32_t exp_dec(uint32_t code, int nbase) {
	uint32_t thres = 1u << nbase;
	uint32_t e = code >> nbase;
	uint32_t base = code & (thres - 1);
	return ((thres + base) << e) - thres;
}

// ---- little-endian word helpers ------------------------------------------------------

static void data_to_words(const uint8_t *d, uint32_t w[5]) {
	for(int i = 0; i < 5; i++) {
		w[i] = (uint32_t)d[4 * i] | ((uint32_t)d[4 * i + 1] << 8) |
		       ((uint32_t)d[4 * i + 2] << 16) | ((uint32_t)d[4 * i + 3] << 24);
	}
}

static void words_to_data(const uint32_t w[5], uint8_t *d) {
	for(int i = 0; i < 5; i++) {
		d[4 * i] = (uint8_t)w[i];
		d[4 * i + 1] = (uint8_t)(w[i] >> 8);
		d[4 * i + 2] = (uint8_t)(w[i] >> 16);
		d[4 * i + 3] = (uint8_t)(w[i] >> 24);
	}
}

// ---- bitfields (B.3) -----------------------------------------------------------------

void adsl_put_bits(uint8_t *d, unsigned offset, unsigned width, uint64_t value) {
	for(unsigned i = 0; i < width; i++) {
		unsigned bit = offset + i;
		uint8_t mask = (uint8_t)(1u << (bit & 7));
		if((value >> i) & 1u) {
			d[bit >> 3] |= mask;
		} else {
			d[bit >> 3] &= (uint8_t)~mask;
		}
	}
}

uint64_t adsl_get_bits(const uint8_t *d, unsigned offset, unsigned width) {
	uint64_t v = 0;
	for(unsigned i = 0; i < width; i++) {
		unsigned bit = offset + i;
		if((d[bit >> 3] >> (bit & 7)) & 1u) {
			v |= (uint64_t)1 << i;
		}
	}
	return v;
}

size_t adsl_onair_len(uint8_t len_byte) {
	switch(len_byte) {
	case ADSL_LEN_TRAFFIC:
		return ADSL_PKT_LEN;
	case ADSL_LEN_STATUS:
		return ADSL_STATUS_PKT_LEN;
	default:
		return 0;
	}
}

// ---- packet encode -------------------------------------------------------------------
// Field caps follow the spec examples: valid values never collide with the "invalid"
// all-ones patterns (speed caps at 0xFE, altitude at 0x3FFE, vertical rate at +0x0FF /
// -0x0FE), and a vertical rate of zero is always encoded non-negative.

void adsl_encode(const adsl_traffic &t, uint8_t out[ADSL_PKT_LEN]) {
	uint8_t d[ADSL_DATA_LEN] = { 0 };

	// data header: PayloadType(8), SenderAddr AMT(6)+addr(24), Reserved(1), Relay(1)
	d[0] = ADSL_PAYLOAD_TRAFFIC;
	uint32_t aw = (uint32_t)(t.amt & 0x3F) | ((t.addr & 0xFFFFFFu) << 6) |
	              (t.relay ? 0x80000000u : 0);
	d[1] = (uint8_t)aw;
	d[2] = (uint8_t)(aw >> 8);
	d[3] = (uint8_t)(aw >> 16);
	d[4] = (uint8_t)(aw >> 24);

	// iConspicuity: Timestamp(6) FlightState(2) | Category(5) Emergency(3)
	d[5] = (uint8_t)((t.ts_qs & 0x3F) | ((t.flightstate & 3) << 6));
	d[6] = (uint8_t)((t.category & 0x1F) | ((t.emergency & 7) << 5));

	// kinematics field codes
	uint32_t lat = 0x800000, lon = 0x800000;
	if(t.pos_valid) {
		lat = (uint32_t)lround(t.lat_deg * 93206.0) & 0xFFFFFF;
		lon = (uint32_t)lround(t.lon_deg * 46603.0) & 0xFFFFFF;
	}
	uint32_t speed = 0xFF;
	if(t.speed_valid) {
		long s = lround(t.speed_mps * 4.0);
		speed = exp_enc(s < 0 ? 0 : (uint32_t)s, 6);
		if(speed > 0xFE) {
			speed = 0xFE;                    // 236 m/s or more; 0xFF means invalid
		}
	}
	uint32_t alt = 0x3FFF;
	if(t.alt_valid) {
		long a = lround(t.alt_m) + 320;
		alt = exp_enc(a < 0 ? 0 : (uint32_t)a, 12);
		if(alt > 0x3FFE) {
			alt = 0x3FFE;                    // 61104 m or more; 0x3FFF means invalid
		}
	}
	uint32_t vrate = 0x1FF;
	if(t.vrate_valid) {
		long v = lround(t.vrate_mps * 8.0);
		uint32_t sign = 0;
		if(v < 0) {
			sign = 0x100;
			v = -v;
		}
		uint32_t mag = exp_enc((uint32_t)v, 6);
		if(sign && mag > 0xFE) {
			mag = 0xFE;                      // -118 m/s or less; 0x1FF means invalid
		}
		vrate = sign | mag;
	}
	uint32_t track = 0;
	if(t.track_valid) {
		long k = lround(t.track_deg * (512.0 / 360.0));
		track = (uint32_t)(((k % 512) + 512) % 512);
	}

	// position block, 88 bits LSB-up: Lat(24) Lon(24) Speed(8) Alt(14) VertRate(9) Track(9)
	d[7] = (uint8_t)lat;
	d[8] = (uint8_t)(lat >> 8);
	d[9] = (uint8_t)(lat >> 16);
	d[10] = (uint8_t)lon;
	d[11] = (uint8_t)(lon >> 8);
	d[12] = (uint8_t)(lon >> 16);
	d[13] = (uint8_t)speed;
	d[14] = (uint8_t)alt;
	d[15] = (uint8_t)((alt >> 8) | (vrate << 6));
	d[16] = (uint8_t)((vrate >> 2) | (track << 7));
	d[17] = (uint8_t)(track >> 1);

	// quality: SIL(2) SDA(2) NIC(4) | NACp(3) GVA(2) NACv(2) Reserved(1)
	d[18] = (uint8_t)((t.sil & 3) | ((t.sda & 3) << 2) | ((t.nic & 0xF) << 4));
	d[19] = (uint8_t)((t.nacp & 7) | ((t.gva & 3) << 3) | ((t.nacv & 3) << 5));

	// scramble (KeyIdx 3 = scrambling disabled), then LEN + NetHdr + data + CRC24 (BE)
	if((t.key_idx & 3) != 3) {
		uint32_t w[5];
		data_to_words(d, w);
		xxtea_scramble(w);
		words_to_data(w, d);
	}
	out[0] = ADSL_LEN_TRAFFIC;
	out[1] = (uint8_t)((t.proto_ver & 0xF) | (t.hdr_flag ? 0x10 : 0) |
	                   ((t.key_idx & 3) << 5) | (t.err_mode ? 0x80 : 0));
	memcpy(out + 2, d, ADSL_DATA_LEN);
	uint32_t c = crc24(out + 1, ADSL_CRC_SPAN);
	out[22] = (uint8_t)(c >> 16);
	out[23] = (uint8_t)(c >> 8);
	out[24] = (uint8_t)c;
}

// ---- packet decode -------------------------------------------------------------------

adsl_decode_result adsl_decode(const uint8_t in[ADSL_PKT_LEN], adsl_traffic &out) {
	if(in[0] != ADSL_LEN_TRAFFIC) {
		return adsl_decode_result::crc_fail; // not a Traffic-length packet
	}
	uint8_t buf[ADSL_CRC_SPAN + 3];
	memcpy(buf, in + 1, sizeof(buf));
	adsl_decode_result res = adsl_decode_result::ok;
	int bit = crc24_correct_1bit(buf, sizeof(buf));
	if(bit == -2) {
		return adsl_decode_result::crc_fail;
	}
	if(bit >= 0) {
		res = adsl_decode_result::corrected_1bit;
	}

	uint8_t hdr = buf[0];
	uint8_t key_idx = (hdr >> 5) & 3;
	uint8_t d[ADSL_DATA_LEN];
	memcpy(d, buf + 1, ADSL_DATA_LEN);
	if(key_idx != 3) {
		uint32_t w[5];
		data_to_words(d, w);
		xxtea_descramble(w);
		words_to_data(w, d);
	}
	if(d[0] != ADSL_PAYLOAD_TRAFFIC) {
		return adsl_decode_result::crc_fail; // CRC-valid but not an iConspicuity Traffic
	}

	out = adsl_traffic{};
	out.proto_ver = hdr & 0xF;
	out.hdr_flag = (hdr & 0x10) != 0;
	out.key_idx = key_idx;
	out.err_mode = (hdr & 0x80) != 0;

	uint32_t aw = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16) |
	              ((uint32_t)d[4] << 24);
	out.amt = (uint8_t)(aw & 0x3F);
	out.addr = (aw >> 6) & 0xFFFFFF;
	out.relay = (aw & 0x80000000u) != 0;

	out.ts_qs = (uint8_t)(d[5] & 0x3F);
	out.flightstate = (uint8_t)(d[5] >> 6);
	out.category = (uint8_t)(d[6] & 0x1F);
	out.emergency = (uint8_t)(d[6] >> 5);

	uint32_t lat = (uint32_t)d[7] | ((uint32_t)d[8] << 8) | ((uint32_t)d[9] << 16);
	uint32_t lon = (uint32_t)d[10] | ((uint32_t)d[11] << 8) | ((uint32_t)d[12] << 16);
	out.pos_valid = lat != 0x800000 && lon != 0x800000;
	if(out.pos_valid) {
		out.lat_deg = (double)((int32_t)(lat << 8) >> 8) / 93206.0;
		out.lon_deg = (double)((int32_t)(lon << 8) >> 8) / 46603.0;
	}

	uint32_t speed = d[13];
	uint32_t alt = (uint32_t)d[14] | (((uint32_t)d[15] & 0x3F) << 8);
	uint32_t vrate = ((uint32_t)d[15] >> 6) | (((uint32_t)d[16] & 0x7F) << 2);
	uint32_t track = ((uint32_t)d[16] >> 7) | ((uint32_t)d[17] << 1);

	out.speed_valid = speed != 0xFF;
	if(out.speed_valid) {
		out.speed_mps = 0.25f * (float)exp_dec(speed, 6);
	}
	out.alt_valid = alt != 0x3FFF;
	if(out.alt_valid) {
		out.alt_m = (float)exp_dec(alt & 0x3FFF, 12) - 320.0f;
	}
	out.vrate_valid = vrate != 0x1FF;
	if(out.vrate_valid) {
		float mag = 0.125f * (float)exp_dec(vrate & 0xFF, 6);
		out.vrate_mps = (vrate & 0x100) ? -mag : mag;
	}
	// the Track field carries no invalid code: it counts as invalid whenever the Ground
	// Speed field is zero or itself invalid (ADS-L.4.SRD860.G.1.10)
	out.track_valid = out.speed_valid && speed != 0;
	if(out.track_valid) {
		out.track_deg = (float)track * (360.0f / 512.0f);
	}

	out.sil = (uint8_t)(d[18] & 3);
	out.sda = (uint8_t)((d[18] >> 2) & 3);
	out.nic = (uint8_t)(d[18] >> 4);
	out.nacp = (uint8_t)(d[19] & 7);
	out.gva = (uint8_t)((d[19] >> 3) & 3);
	out.nacv = (uint8_t)((d[19] >> 5) & 3);
	return res;
}

// ---- Status (G.2) --------------------------------------------------------------------
// Same header, scrambling and CRC as Traffic, over 12 data bytes. The header's bit layout is
// the one adsl_encode() packs by hand; the payload starts at bit 40:
//   MaxProtoVer(4) M-Rx(2) O-LDR-Rx(2) O-HDR-Rx(2) Mobile(2) EC-Rx(8) EC-Tx(8) XPDR(2)
//   TrafficUplinkClient(1) FIS-B-Client(1) Reserved(24)

static constexpr unsigned STATUS_PAYLOAD_BIT = 40;
static constexpr size_t STATUS_WORDS = ADSL_STATUS_DATA_LEN / 4;

// the data block's little-endian words ARE its B.3 bitfields of width 32
static void status_xxtea(uint8_t *d, bool scramble) {
	uint32_t w[STATUS_WORDS];
	for(size_t i = 0; i < STATUS_WORDS; i++) {
		w[i] = (uint32_t)adsl_get_bits(d, (unsigned)(32 * i), 32);
	}
	if(scramble) {
		xxtea_scramble_n(w, STATUS_WORDS);
	} else {
		xxtea_descramble_n(w, STATUS_WORDS);
	}
	for(size_t i = 0; i < STATUS_WORDS; i++) {
		adsl_put_bits(d, (unsigned)(32 * i), 32, w[i]);
	}
}

size_t adsl_status_encode(const adsl_status &s, uint8_t out[ADSL_PKT_LEN]) {
	uint8_t d[ADSL_STATUS_DATA_LEN] = { 0 };
	adsl_put_bits(d, 0, 8, ADSL_PAYLOAD_STATUS);
	adsl_put_bits(d, 8, 6, s.amt & 0x3F);
	adsl_put_bits(d, 14, 24, s.addr & 0xFFFFFFu);
	adsl_put_bits(d, 39, 1, s.relay ? 1 : 0);      // bit 38 Reserved stays 0

	const unsigned p = STATUS_PAYLOAD_BIT;
	adsl_put_bits(d, p, 4, s.max_proto_ver & 0xF);
	adsl_put_bits(d, p + 4, 2, s.rx_mband & 3);
	adsl_put_bits(d, p + 6, 2, s.rx_oband_ldr & 3);
	adsl_put_bits(d, p + 8, 2, s.rx_oband_hdr & 3);
	adsl_put_bits(d, p + 10, 2, s.mobile & 3);
	adsl_put_bits(d, p + 12, 8, s.ec_rx);
	adsl_put_bits(d, p + 20, 8, s.ec_tx);
	adsl_put_bits(d, p + 28, 2, s.xpdr & 3);
	adsl_put_bits(d, p + 30, 1, s.uplink_client ? 1 : 0);
	adsl_put_bits(d, p + 31, 1, s.fisb_client ? 1 : 0);

	if((s.key_idx & 3) != 3) {
		status_xxtea(d, true);
	}
	memset(out, 0, ADSL_PKT_LEN);
	out[0] = ADSL_LEN_STATUS;
	out[1] = (uint8_t)((s.proto_ver & 0xF) | ((s.key_idx & 3) << 5));
	memcpy(out + 2, d, ADSL_STATUS_DATA_LEN);
	const size_t crc_at = 2 + ADSL_STATUS_DATA_LEN;
	uint32_t c = crc24(out + 1, 1 + ADSL_STATUS_DATA_LEN);
	out[crc_at] = (uint8_t)(c >> 16);
	out[crc_at + 1] = (uint8_t)(c >> 8);
	out[crc_at + 2] = (uint8_t)c;
	return ADSL_STATUS_PKT_LEN;
}

int adsl_payload_type(const uint8_t in[ADSL_PKT_LEN]) {
	if(in[0] == ADSL_LEN_TRAFFIC) {
		uint8_t d[ADSL_DATA_LEN];
		memcpy(d, in + 2, sizeof(d));
		if(((in[1] >> 5) & 3) != 3) {
			uint32_t w[5];
			data_to_words(d, w);
			xxtea_descramble(w);
			words_to_data(w, d);
		}
		return d[0];
	}
	if(in[0] == ADSL_LEN_STATUS) {
		uint8_t d[ADSL_STATUS_DATA_LEN];
		memcpy(d, in + 2, sizeof(d));
		if(((in[1] >> 5) & 3) != 3) {
			status_xxtea(d, false);
		}
		return (int)adsl_get_bits(d, 0, 8);
	}
	return -1;
}

adsl_decode_result adsl_status_decode(const uint8_t in[ADSL_PKT_LEN], adsl_status &out) {
	if(in[0] != ADSL_LEN_STATUS) {
		return adsl_decode_result::crc_fail;  // not a Status-length packet
	}
	uint8_t buf[1 + ADSL_STATUS_DATA_LEN + 3];
	memcpy(buf, in + 1, sizeof(buf));
	adsl_decode_result res = adsl_decode_result::ok;
	int bit = crc24_correct_1bit(buf, sizeof(buf));
	if(bit == -2) {
		return adsl_decode_result::crc_fail;
	}
	if(bit >= 0) {
		res = adsl_decode_result::corrected_1bit;
	}

	uint8_t hdr = buf[0];
	uint8_t key_idx = (hdr >> 5) & 3;
	uint8_t d[ADSL_STATUS_DATA_LEN];
	memcpy(d, buf + 1, sizeof(d));
	if(key_idx != 3) {
		status_xxtea(d, false);
	}
	if(adsl_get_bits(d, 0, 8) != ADSL_PAYLOAD_STATUS) {
		return adsl_decode_result::crc_fail;  // CRC-valid but some other 12-byte payload
	}

	out = adsl_status{};
	out.proto_ver = hdr & 0xF;
	out.key_idx = key_idx;
	out.amt = (uint8_t)adsl_get_bits(d, 8, 6);
	out.addr = (uint32_t)adsl_get_bits(d, 14, 24);
	out.relay = adsl_get_bits(d, 39, 1) != 0;

	const unsigned p = STATUS_PAYLOAD_BIT;
	out.max_proto_ver = (uint8_t)adsl_get_bits(d, p, 4);
	out.rx_mband = (uint8_t)adsl_get_bits(d, p + 4, 2);
	out.rx_oband_ldr = (uint8_t)adsl_get_bits(d, p + 6, 2);
	out.rx_oband_hdr = (uint8_t)adsl_get_bits(d, p + 8, 2);
	out.mobile = (uint8_t)adsl_get_bits(d, p + 10, 2);
	out.ec_rx = (uint8_t)adsl_get_bits(d, p + 12, 8);
	out.ec_tx = (uint8_t)adsl_get_bits(d, p + 20, 8);
	out.xpdr = (uint8_t)adsl_get_bits(d, p + 28, 2);
	out.uplink_client = adsl_get_bits(d, p + 30, 1) != 0;
	out.fisb_client = adsl_get_bits(d, p + 31, 1) != 0;
	return res;
}

} // namespace adsld

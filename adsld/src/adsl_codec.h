// adsld: ADS-L link-layer codec (EASA ADS-L 4 SRD-860 Issue 2, M-band Traffic and Status
// broadcasts). Everything byte-and-bit-exact about the on-air packets lives here so interop
// fixes stay one-line changes.
//
// On-air packet layout (25 bytes, after the PHY preamble + sync word):
//   [0]      LEN            = 0x18 (24: NetHdr + data + CRC, excludes itself)
//   [1]      Network header: ProtoVer(4)=1, flag(1), KeyIdx(2), ErrorControlMode(1)
//   [2..21]  ADS-L data, 20 bytes, XXTEA-"scrambled" on air:
//              header 5 B:  PayloadType(8)=2 Traffic, SenderAddr(30)=AMT(6)+addr(24),
//                           Reserved(1), Relay(1)
//              iConspicuity 15 B (120 bits): Timestamp(6) FlightState(2) Category(5)
//                           Emergency(3) Lat(24) Lon(24) GroundSpeed(8) Altitude(14)
//                           VertRate(9) Track(9) SIL(2) SDA(2) NIC(4) NACp(3) GVA(2)
//                           NACv(2) Reserved(1)
//   [22..24] CRC24 over bytes [1..21] (NetHdr + scrambled data), transmitted big-endian
//            (MSByte first) - the one big-endian exception in the packet.
// A Status packet (G.2) has the same structure with a 7-byte payload: LEN 0x10, 12 data
// bytes, 17 bytes on air.
//
// Bit conventions: multi-byte integers little-endian; bitfields pack LSB-upward across
// the little-endian byte stream; each byte goes MSB-first on air (that part is the PHY's
// business, this codec only ever sees whole bytes).
// Exponential encodings <exp:2><base:N>: value = 2^e*(2^N+base) - 2^N; N=12 for altitude
// (1 m LSB, +320 m offset), N=6 for speed (0.25 m/s LSB) and vertical rate (0.125 m/s
// LSB, signed variant = leading sign bit). Lat LSB = 1/93206 deg, Lon LSB = 1/46603 deg,
// both 24-bit signed; no-fix = 0x800000 in both.
#pragma once

#include <cstddef>
#include <cstdint>

namespace adsld {

constexpr size_t ADSL_PKT_LEN = 25;    // full on-air packet incl. the LEN byte
constexpr size_t ADSL_DATA_LEN = 20;   // the (scrambled) ADS-L data block
constexpr size_t ADSL_CRC_SPAN = 21;   // bytes covered by the CRC: NetHdr + data
constexpr uint8_t ADSL_LEN_TRAFFIC = 0x18;
constexpr uint8_t ADSL_PAYLOAD_TRAFFIC = 2;

// Status (ADS-L.4.SRD860.G.2): the same 5-byte ADS-L header plus a 56-bit payload = 12 data
// bytes (three XXTEA words). LEN = NetHdr 1 + data 12 + CRC 3 = 16, so 17 bytes fly after
// the sync word.
constexpr uint8_t ADSL_LEN_STATUS = 0x10;
constexpr uint8_t ADSL_PAYLOAD_STATUS = 3;
constexpr size_t ADSL_STATUS_DATA_LEN = 12;
constexpr size_t ADSL_STATUS_PKT_LEN = 17;   // on-air bytes incl. the LEN byte

// On-air byte count (LEN byte included) of a packet whose LEN byte this build captures:
// ADSL_PKT_LEN for Traffic, ADSL_STATUS_PKT_LEN for Status, 0 for anything else. Received packets travel in an ADSL_PKT_LEN buffer; the
// bytes past this count are zero.
size_t adsl_onair_len(uint8_t len_byte);

// One Traffic (iConspicuity) report in natural units. Encode fills the wire fields from
// these; decode fills these from the wire. Fields whose validity flag is false encode as
// the standard's "unavailable" code and vice versa.
struct adsl_traffic {
	// ---- sender (SenderAddr) ----
	uint8_t amt = 0;             // address-mapping table entry, 6 bits (0=random/privacy,
	                             // 5=ICAO, 6=FLARM, 7=OGN, 8=FANET, 9+=manufacturer pages)
	uint32_t addr = 0;           // 24-bit address
	bool relay = false;          // relay flag (data header bit 39)

	// ---- network header (decode fills; encode uses these, defaults are the normal TX) ----
	uint8_t proto_ver = 1;       // ProtoVer(4), always 1
	bool hdr_flag = false;       // network header bit 4, carried through unparsed, 0 on TX
	uint8_t key_idx = 0;         // KeyIdx(2)
	bool err_mode = false;       // ErrorControlMode(1)

	// ---- time / state ----
	uint8_t ts_qs = 0;           // Timestamp: quarter-seconds mod 60 (0..59, 15 s window)
	uint8_t flightstate = 0;     // FlightState(2): 0=unknown 1=ground 2=airborne
	uint8_t category = 0;        // Category(5) aircraft category
	uint8_t emergency = 0;       // Emergency(3): 0=none/unknown 1=no emergency ...

	// ---- kinematics, natural units ----
	bool pos_valid = false;      // false <-> lat/lon wire value 0x800000 (no fix)
	double lat_deg = 0;          // WGS-84, LSB 1/93206 deg on the wire
	double lon_deg = 0;          // WGS-84, LSB 1/46603 deg on the wire
	bool alt_valid = false;
	float alt_m = 0;             // ellipsoidal altitude, exponential N=12, offset +320 m
	bool speed_valid = false;
	float speed_mps = 0;         // ground speed, exponential N=6, LSB 0.25 m/s
	bool vrate_valid = false;
	float vrate_mps = 0;         // vertical rate, signed exponential N=6, LSB 0.125 m/s
	bool track_valid = false;
	float track_deg = 0;         // course over ground, LSB 360/512 deg

	// ---- quality indicators (raw wire codes) ----
	uint8_t sil = 0;             // SIL(2)
	uint8_t sda = 0;             // SDA(2)
	uint8_t nic = 0;             // NIC(4)
	uint8_t nacp = 0;            // NACp(3)
	uint8_t gva = 0;             // GVA(2)
	uint8_t nacv = 0;            // NACv(2)
};

// ---- packet codec --------------------------------------------------------------------

// Build the complete 25-byte on-air packet: LEN, NetHdr, XXTEA-scrambled data, CRC24
// (big-endian). The CRC is computed over the scrambled bytes, i.e. exactly what flies.
void adsl_encode(const adsl_traffic &t, uint8_t out[ADSL_PKT_LEN]);

enum class adsl_decode_result {
	ok,              // CRC good as received
	corrected_1bit,  // CRC failed, a single-bit syndrome correction made it pass
	crc_fail,        // uncorrectable
};

// Decode a received 25-byte packet (in[0] is the LEN byte). On ok/corrected_1bit fills
// `out` with descrambled, natural-unit fields; on crc_fail `out` is untouched. `in` is
// never modified (correction happens on an internal copy).
adsl_decode_result adsl_decode(const uint8_t in[ADSL_PKT_LEN], adsl_traffic &out);

// One Status report (ADS-L.4.SRD860.G.2): what the sender can receive and process, as raw
// wire codes (the tables are in the spec). A conformant device sends it at least once per
// 20 s (G.2.11) and names Traffic AND Status in its Declaration of Conformity.
struct adsl_status {
	// ---- sender + network header, as in adsl_traffic ----
	uint8_t amt = 0;
	uint32_t addr = 0;
	bool relay = false;
	uint8_t proto_ver = 1;       // ProtoVer(4)
	uint8_t key_idx = 0;         // KeyIdx(2); 3 = scrambling disabled

	// ---- Status payload, 56 bits ----
	// Max. Protocol Version(4): the highest version this sender transmits AND parses (G.2.1)
	uint8_t max_proto_ver = 1;
	// receive capability per band as average dwell time (G.2.2-G.2.4):
	// 0 none, 1 occasional > 5 %, 2 partial > 40 %, 3 full > 80 % - M-band "full" only
	// when both channels are received concurrently
	uint8_t rx_mband = 0;
	uint8_t rx_oband_ldr = 0;
	uint8_t rx_oband_hdr = 0;
	uint8_t mobile = 0;          // ADS-L 4 Mobile(2): 0 none, 1 offline, 2 online
	// other e-conspicuity systems on board (G.2.6/G.2.7), bit 0 FLARM, 1 PilotAware,
	// 2 FANET, 3 OGN tracker, 4 ADS-B 1090, 5 UAT 978, 6 Mode-S/PCAS (receive only);
	// unknown capabilities stay 0
	uint8_t ec_rx = 0;
	uint8_t ec_tx = 0;
	uint8_t xpdr = 0;            // XPDR Capabilities(2): 0 none/unknown, 1 Mode-C, 2 Mode-S
	bool uplink_client = false;  // processes Traffic Uplink payloads (G.2.9)
	bool fisb_client = false;    // processes FIS-B Uplink payloads (G.2.10)
	// Reserved(24) is sent as zero and ignored on receive (B.6)
};

// Build the Status packet into an ADSL_PKT_LEN buffer; returns the on-air length
// (ADSL_STATUS_PKT_LEN). The bytes past it are zeroed.
size_t adsl_status_encode(const adsl_status &s, uint8_t out[ADSL_PKT_LEN]);

// The PayloadType byte of a CRC-valid packet (in[0] selects Traffic or Status length, the data
// block is descrambled as usual), or -1 when the LEN byte matches neither. For labelling frames
// whose payload this build does not parse.
int adsl_payload_type(const uint8_t in[ADSL_PKT_LEN]);

// Decode a received Status packet (in[0] must be ADSL_LEN_STATUS). Same result contract as
// adsl_decode(); crc_fail also for a CRC-valid packet that is not payload type 3.
adsl_decode_result adsl_status_decode(const uint8_t in[ADSL_PKT_LEN], adsl_status &out);

// ---- building blocks (exposed for the selftest KATs) ---------------------------------

// CRC24 (Mode-S polynomial, left-aligned 0xFFFA0480, init 0, no final xor) over `len`
// message bytes followed by three implicit zero flush bytes; returns the 24-bit checksum
// to append (transmit MSByte first). Verify with crc24(msg, ADSL_CRC_SPAN) == received.
uint32_t crc24(const uint8_t *data, size_t len);

// Single-bit syndrome correction over buf = message || 3-byte big-endian CRC24, `len`
// bytes total (normally ADSL_CRC_SPAN + 3 = 24). Returns -1 if the CRC is already good,
// the index (0 = MSB of buf[0]) of the corrected bit if exactly one flip fixes it (buf is
// patched in place), or -2 if uncorrectable (buf unmodified).
int crc24_correct_1bit(uint8_t *buf, size_t len);

// XXTEA over the 20-byte data block as FIVE uint32 little-endian words (the spec's table
// wrongly prints 6): all-zero key, 6 rounds, DELTA 0x9E3779B9. There is NO PN whitening
// anywhere else in the chain.
void xxtea_scramble(uint32_t w[5]);
void xxtea_descramble(uint32_t w[5]);
// The same network over any block of n >= 2 words - the sample code in E.2 takes
// num_data_words as a parameter; a Status packet scrambles 3.
void xxtea_scramble_n(uint32_t *w, size_t n);
void xxtea_descramble_n(uint32_t *w, size_t n);

// Bitfield access in the ADS-L byte order (B.3): a field of `width` <= 64 bits starting at
// bit `offset` fills each byte least-significant bit first, and multi-byte fields run
// little-endian. B.3 example 2: A = 0x12AA (14 bits) then B = 0x355 (10 bits) -> AA 52 D5
// (the spec's prose says 0x55 for the last byte; its own bit string is 0xD5).
void adsl_put_bits(uint8_t *d, unsigned offset, unsigned width, uint64_t value);
uint64_t adsl_get_bits(const uint8_t *d, unsigned offset, unsigned width);

// ---- field-level exponential coding helpers (exposed for KATs) -----------------------

// <exp:2><base:N> unsigned exponential: value = 2^e*(2^N+base) - 2^N, in field LSB units.
// enc clamps to the representable range; field width is N+2 bits.
uint32_t exp_enc(uint32_t value, int nbase);
uint32_t exp_dec(uint32_t code, int nbase);

} // namespace adsld

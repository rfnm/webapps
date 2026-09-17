// elrsd: ExpressLRS 3.3.1 (2.4 GHz LoRa, OTA4, "Wide" switch mode) bit-level codec.
// Pure, deterministic, no I/O, no threads, no hardware. Everything in
// here is verified against tools/elrsd_vectors.json (checked-in vectors,
// TEST-UID d2:4c:3e:2e:fb:4d from the phrase "elrsd selftest phrase 2026"; the real
// binding phrase / UID is NEVER in this tree - it is runtime configuration).
//
// Evidence tiers used in comments: measured on the 2026-09-14 captures, derived from
// the 3.3.1 sources but unmeasured (OTA8 rates 5/8, CR4/5), to be measured.
//
// Conventions (all from):
//   * symbols are the raw dechirp argmax values 0..2^SF-1 (before Gray / ref removal)
//   * "ref" is the demodulated preamble up-chirp value under the SAME CFO/timing
//     correction as the payload (majority over inner preamble symbols; ideal sync -> 0)
//   * bit value of symbol s with width w:  v = gray_encode(((s - ref - 1) mod 2^SF) >> (SF - w))
//   * TX inverse:                          s = ((gray_decode(v) << (SF - w)) + ref + 1) mod 2^SF
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace elrsd {

// ---- UID / binding phrase ---------------------------------------------------------

using uid_t6 = std::array<uint8_t, 6>;

// uid = MD5('-DMY_BINDING_PHRASE="' + phrase + '"')[0:6], phrase byte-exact (no trim).
// Own MD5 implementation (RFC 1321) - no OpenSSL dependency on the board.
uid_t6 uid_from_binding_phrase(const std::string &phrase);
// "d2:4c:3e:2e:fb:4d" <-> bytes; parse accepts ':' '-' or no separator, returns false on error
std::string uid_to_string(const uid_t6 &uid);
bool uid_from_string(const std::string &s, uid_t6 &out);

static const uint8_t OTA_VERSION_ID = 3;                    // ELRS 3.x
inline uint16_t crc14_init_from_uid(const uid_t6 &u) { return (uint16_t)((((uint16_t)u[4] << 8) | u[5]) ^ OTA_VERSION_ID); }
inline uint32_t fhss_seed_from_uid(const uid_t6 &u) {
	return ((uint32_t)u[2] << 24) | ((uint32_t)u[3] << 16) | ((uint32_t)u[4] << 8) | (uint32_t)(u[5] ^ OTA_VERSION_ID);
}
inline bool iq_inverted_from_uid(const uid_t6 &u) { return (u[5] & 1) != 0; } // SetRFLinkRate invertIQ

// ---- FHSS 3.3.1 ------------------------------------------------------------------

static const int FHSS_NUM_CHANNELS = 80;
static const int FHSS_SEQ_LEN = 240;         // (256 / 80) * 80
static const int FHSS_SYNC_CHANNEL = 41;     // 80/2 + 1 (3.x; 4.x uses 40)
static const double FHSS_FREQ_START_HZ = 2400.4e6;
static const double FHSS_FREQ_STEP_HZ = 1.0e6;

// nominal grid 2400.4 MHz + ch * 1 MHz (what the capture shows)
double fhss_channel_freq_hz(int ch);
// SX1280 register-exact frequency (<= 327 Hz off nominal); use for TX synthesis
double fhss_channel_freq_hz_sx1280(int ch);
// nearest channel index for a frequency (nominal grid), -1 if > 0.5 MHz off any channel
int fhss_channel_from_freq(double f_hz);

// ELRS LCG: seed = (214013*seed + 2531011) mod 2^31 (uint32 wrap first); rng() = seed >> 16
struct elrs_rng {
	uint32_t state = 0;
	explicit elrs_rng(uint32_t seed = 0) : state(seed) {}
	uint32_t next();                          // 0..0x7FFF
	uint32_t next_n(uint32_t max) { return next() % max; }
};

// the 240-entry sequence for a UID; seq[0]=seq[80]=seq[160]=41 always, 41 never elsewhere
std::array<uint8_t, FHSS_SEQ_LEN> fhss_sequence(const uid_t6 &uid);

// ---- LoRa bit helpers / whitening ------------------------------------------------

inline uint32_t gray_encode(uint32_t v) { return v ^ (v >> 1); }
uint32_t gray_decode(uint32_t g);
// the 255-byte LoRa whitening sequence (FF FE FC F8 F0 E1 C2 85 0B 17 2F 5E BC ...)
const uint8_t *whitening_seq();            // 255 bytes
void whiten(uint8_t *data, size_t len);    // in place, data[i] ^= W[i], i < 255

// ---- air rates (3.3.1 airRateConfig, 2.4 GHz) ------------------------------------

enum class ota_kind : uint8_t { OTA4 = 0, OTA8 = 1 };

struct rate_params {
	int index;              // 4..9 (rateIndex in the SYNC packet)
	int rate_hz;            // 500 250 150 50 (333 / 100 for OTA8)
	int sf;                 // 5..8
	int cr_bits;            // 6 (CR4/6) or 8 (CR4/8) coded bits per nibble
	int payload_bytes;      // 8 (OTA4) or 13 (OTA8)
	int preamble_len;       // up-chirps before the 2 sync-word symbols
	int interval_us;        // slot length
	int hop_interval;       // packets per dwell (4, 2 for 50 Hz)
	int tlm_denom_default;  // 128 64 32 16
	double frame_offset_symbols; // payload start in symbols from preamble start: 18.25/20.25/16.25
	bool fine_sync;         // 2 extra symbols after SFD (SF5/SF6 only)
	bool verified;          // rate measured on air; false = hypothesis (OTA8)
	ota_kind kind;
	int n_cw;               // 2 * payload_bytes
	int n_coded;            // n_cw * cr_bits
	int n_sym;              // payload symbols
	int n_slots;            // sum of symbol widths
	int n_tail;             // n_slots - n_coded
	double t_sym_s() const { return (double)(1 << sf) / 812500.0; }
	double burst_s() const { return (frame_offset_symbols + n_sym) * t_sym_s(); }   // air time matches 3.3.1 TOA table
};

static const double ELRS_BW_HZ = 812500.0;

// The rate table is HEADER-ONLY (inline) so that B (rx_chain) and D (tx_chain) build and link
// without A's elrs_codec.cpp. Literal values = tools/elrsd_vectors.json rates[*] (A's selftest
// re-derives n_sym/n_slots/n_tail from the formulas and compares against these literals).
// Columns: index rate_hz sf cr_bits bytes preamble interval_us hop tlm_default frame_offset
//          fine_sync verified kind n_cw n_coded n_sym n_slots n_tail
inline const rate_params RATE_TABLE[6] = {
	{4, 500, 5, 6,  8, 12,  2000, 4, 128, 18.25, true,  true,  ota_kind::OTA4, 16,  96, 20, 100, 4},
	{5, 333, 5, 8, 13, 12,  3003, 4, 128, 18.25, true,  false, ota_kind::OTA8, 26, 208, 42, 210, 2},
	{6, 250, 6, 8,  8, 14,  4000, 4,  64, 20.25, true,  true,  ota_kind::OTA4, 16, 128, 22, 132, 4},
	{7, 150, 7, 8,  8, 12,  6666, 4,  32, 16.25, false, true,  ota_kind::OTA4, 16, 128, 21, 131, 3},
	{8, 100, 7, 8, 13, 12, 10000, 4,  32, 16.25, false, false, ota_kind::OTA8, 26, 208, 32, 208, 0},
	{9,  50, 8, 8,  8, 12, 20000, 2,  16, 16.25, false, true,  ota_kind::OTA4, 16, 128, 18, 128, 0},
};
inline const rate_params *rate_by_index(int idx) { return (idx >= 4 && idx <= 9) ? &RATE_TABLE[idx - 4] : nullptr; }
// "500hz" "250hz" "150hz" "50hz" ("333hz" "100hz" OTA8); nullptr if unknown
inline const rate_params *rate_by_name(const std::string &name) {
	for(const rate_params &r : RATE_TABLE) {
		if(name == std::to_string(r.rate_hz) + "hz" || name == std::to_string(r.rate_hz)) return &r;
	}
	return nullptr;
}
// SF5->4, SF6->6, SF7->7, SF8->9 (prefer_ota4 = false: SF5->5, SF7->8); nullptr otherwise
inline const rate_params *rate_from_sf(int sf, bool prefer_ota4 = true) {
	const rate_params *hit = nullptr;
	for(const rate_params &r : RATE_TABLE) {
		if(r.sf != sf) continue;
		if((r.kind == ota_kind::OTA4) == prefer_ota4) return &r;
		if(!hit) hit = &r;
	}
	return hit;
}
inline std::vector<int> rate_indices() { return {4, 5, 6, 7, 8, 9}; }
// the four rates the daemon supports and `auto` rate detection may choose
inline std::vector<int> rate_indices_verified() { return {4, 6, 7, 9}; }

// ---- LI codec (SX1280 CR_LI_4_x long interleaver) --------------------------------
// One instance per rate. Tables are generated from the formulas in
// and checked against the JSON tables in the selftest.

class li_codec {
public:
	explicit li_codec(const rate_params &rp);
	const rate_params &rate() const { return rp_; }

	// symbol widths (bits carried by payload symbol s): SF>=7 -> first 8 symbols SF-2, else SF
	const std::vector<int> &symbol_widths() const { return widths_; }
	// coded bit i -> slot index (table "coded_bit_to_slot" in the JSON)
	const std::vector<int> &coded_bit_to_slot() const { return cb2slot_; }
	// whitening pushed through the linear coding chain (table "whitening_coded_bits")
	const std::vector<uint8_t> &whitening_coded_bits() const { return wcoded_; }
	const std::array<uint8_t, 16> &hamming_table() const { return henc_; }  // nibble -> codeword (cr_bits wide, MSB first)
	// codeword -> nibble (0..15) or -1 (uncorrectable / detected error); CR4/8 corrects 1 bit,
	// CR4/6 and CR4/5 detect only. correct=false disables even the single-bit correction.
	int hamming_decode(uint32_t codeword, bool correct = true) const;

	// payload bytes -> n_sym raw symbol values (with the ref/+1 mapping applied)
	std::vector<uint16_t> encode(const uint8_t *payload, size_t len, int ref = 0) const;
	// raw symbols (>= n_sym) + ref -> payload; false if any codeword is uncorrectable.
	// fec_corrected (optional) counts single-bit corrections that were applied.
	bool decode(const uint16_t *symbols, size_t n_symbols, int ref, std::vector<uint8_t> &payload,
	            bool correct = true, int *fec_corrected = nullptr) const;
	// symbol-level helpers exposed for the selftest / tx chain
	uint32_t symbol_to_bits(uint16_t s, int sym_index, int ref) const;
	uint16_t bits_to_symbol(uint32_t v, int sym_index, int ref) const;

private:
	rate_params rp_;
	std::vector<int> widths_, off_, cb2slot_;
	std::vector<uint8_t> wcoded_;
	std::array<uint8_t, 16> henc_{};
	std::vector<int16_t> hdec_;   // 2^cr_bits entries, -1 = bad
	std::vector<std::pair<int, int>> tail_;   // (tail slot, source slot) copies, stream order (A addition)

public:
	// tail slot copies (slot j = copy of slot src), stream order; selftest compares with the JSON
	const std::vector<std::pair<int, int>> &tail_copies() const { return tail_; }
};

// ---- CRC-14 (poly 0x2E57, MSB-first, 14-bit) --------------------------------------

uint16_t crc14(const uint8_t *data, size_t len, uint16_t init);    // OTA4_CRC_CALC_LEN = 7 bytes for OTA4
const uint16_t *crc14_table();                                      // 256 entries (selftest compares)

// ---- CRSF value mapping -------------------------------------------------------------

uint32_t fmap(uint32_t x, uint32_t imin, uint32_t imax, uint32_t omin, uint32_t omax);
uint16_t crsf_to_uint10(uint16_t v);       // clamp 172..1811 then fmap -> 0..1023
uint16_t uint10_to_crsf(uint16_t v10);     // fmap(v10, 0, 1023, 172, 1811)
uint16_t crsf_to_n(uint16_t v, uint16_t cnt);   // v<=191 -> 0; v>=1792 -> cnt-1; else (v-191)*cnt/1602
uint16_t n_to_crsf(uint16_t val, uint16_t bins); // val*1601/bins + 191 (bins 127 or 63)
inline uint16_t bit_to_crsf(bool b) { return b ? 1792 : 191; }

// ---- OTA4 packets (8 bytes) ----------------------------------------------------------

enum class ota_type : uint8_t { RC = 0, MSP = 1, SYNC = 2, TLM = 3 };

// Hybrid-Wide switch slot for a nonce: ((nonce & 7) + ((nonce >> 3) & 1)) % 8
int hybrid_wide_switch_index(uint8_t nonce);

struct rc_packet {
	std::array<uint16_t, 16> ch{};   // CRSF 172..1811 (ch[0..3] analog, ch[4] armed bit, ch[5..12] aux)
	bool ch4_high = false;           // ch[4] > 992
	int switch_index = 0;            // 0..7 (7 = telemetry/power slot)
	int aux_channel = -1;            // 5 + switch_index (0-based channel index), -1 for idx 7
	uint16_t aux_value_raw = 0;      // 7-bit (tlm >= 8) or 6-bit + telemBit
	int uplink_tx_power = -1;        // CRSF power enum when switch_index == 7 (Ranger Nano = 2)
	bool telem_bit = false;
};

struct sync_packet {
	uint8_t fhss_index = 0;    // FHSSptr at TX time (0..239)
	uint8_t nonce = 0;         // OtaNonce of this packet
	bool switch_enc_mode = 0;  // bit0 of byte3 (0 = Wide)
	uint8_t new_tlm_ratio = 0; // 3-bit field; tlm_denom = 1 << (8 - field) for field >= 1, 1 for 0
	int rate_index = 0;        // 4..9
	uint8_t uid3 = 0, uid4 = 0, uid5 = 0;
	int tlm_denom() const { return new_tlm_ratio ? (1 << (8 - new_tlm_ratio)) : 1; }
	bool uid_match(const uid_t6 &uid) const { return uid3 == uid[3] && uid4 == uid[4] && (uid5 & ~0x3F) == (uid[5] & ~0x3F); }
};

struct ota4_decoded {
	ota_type type = ota_type::RC;
	bool crc_ok = false;
	uint8_t crc_high = 0, crc_low = 0;
	// for RC: the residues r = nonce % hop for which the CRC matches (0..hop-1); empty = bad
	std::vector<int> nonce_residues;
	rc_packet rc;      // valid if type == RC && crc_ok
	sync_packet sync;  // valid if type == SYNC && crc_ok
};

// Build an RC packet exactly like 3.3.1 tx_main (Wide mode). tlm_denom selects the
// 6/7-bit aux encoding, hop_interval the crcHigh injection (nonce % hop + 1).
std::array<uint8_t, 8> ota4_build_rc(const uid_t6 &uid, const std::array<uint16_t, 16> &ch_crsf, uint8_t nonce,
                                     int hop_interval, int tlm_denom = 64, int tx_power_crsf = 2, bool telem_bit = false);
// Build a SYNC packet (3.3.1 layout: fhssIndex, nonce, flags, UID3, UID4, UID5, crcLow; crcHigh = 0)
std::array<uint8_t, 8> ota4_build_sync(const uid_t6 &uid, uint8_t fhss_index, uint8_t nonce, int rate_index,
                                       uint8_t new_tlm_ratio_field, bool switch_enc_mode = false, uint8_t model_id_xor = 0);
// Decode any OTA4 packet. hop_interval is needed to try the crcHigh residues of RC packets.
// nonce (addition, defaulted): the packet's absolute OtaNonce if the caller knows it
// (FULL lock) - then rc.switch_index / aux_channel / ch[aux] / uplink_tx_power / telem_bit are
// resolved; with nonce = -1 (MOD_HOP / no lock) they stay -1 and rc.aux_value_raw holds the raw
// 7-bit switch field (rc.ch[0..3] and ch4_high are always valid when crc_ok). Non-RC types are
// validated with crcHigh = 0 (3.3.1 ValidatePacketCrcStd).
ota4_decoded ota4_decode(const uint8_t pkt[8], const uid_t6 &uid, int hop_interval, int tlm_denom = 64, int nonce = -1);
// Resolve the switch fields of an already decoded RC packet once the nonce becomes known
// (addition; same effect as passing nonce to ota4_decode). rc.aux_value_raw must still
// hold the raw 7-bit field, i.e. call it at most once per packet.
void ota4_resolve_switch(rc_packet &rc, uint8_t nonce, int tlm_denom = 64);

// ---- freerun ground truth (replay tests on the 2026-09-14 captures) ------------------
// The freerun sender (elrs-ranger-nano freerun firmware) sets channel i of tick t to
// 172 + lowbias32(t*16 + i) % 1640. tick = k + 583878 for the 25 MSps/250 Hz capture.
uint32_t lowbias32(uint32_t x);
std::array<uint16_t, 16> freerun_channels(uint32_t tick);
static const int64_t FREERUN_TICK_OFFSET_250HZ = 583878;   // tick = k - 2 + 4*50 + 960*608

// ---- selftest -----------------------------------------------------------------
// Runs every codec check against the embedded vectors (src/elrs_vectors_gen.h, generated
// from tools/elrsd_vectors.json by tools/vectors_to_h.py; the generated header is checked
// in). Must also re-derive RATE_TABLE from the formulas and compare every literal.
// Returns the number of failed checks.
int run_selftest_codec();

} // namespace elrsd

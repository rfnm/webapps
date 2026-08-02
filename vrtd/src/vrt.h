// vrtd: ANSI/VITA 49.2 wire layer. EVERY bit position and fixed-point convention lives
// in this pair of files (the lte_seq.cpp pattern: an interop fix is a one-liner here,
// nothing else moves). No I/O - pack/unpack against 32-bit big-endian word buffers.
//
// Layout cross-checked 2026-07-05 against two independent implementations:
// voyager-tech-inc/vita49-rs (49.2-2017: header, CIF0, CAM, command order, trailer) and
// ember91/libvrt (49.0: data payload format, trailer). Single source of truth below.
//
//   header word: type[31:28] C[27] ind[26:24] TSI[23:22] TSF[21:20] count[19:16] size[15:0]
//     indicators: data     - trailer[26]  notV49.0[25] spectrum[24]
//                 context  - reserved[26] notV49.0[25] TSM[24]
//                 command  - ack[26]      reserved[25] cancel[24]
//   class id: pad[31:27]|0|OUI[23:0] ; info_class[31:16]|packet_class[15:0]
//   CIF0 bits: change[31] refpt[30] bw[29] if_freq[28] rf_freq[27] rf_off[26] if_off[25]
//              ref_level[24] gain[23] overrange[22] samp_rate[21] ts_adj[20] ts_cal[19]
//              temp[18] dev_id[17] state_ev[16] payload_fmt[15] ... cif1_en[1]
//   CIF0 field encodings: bw/samp_rate U64 Q20 Hz; if/rf freq I64 Q20 Hz; ref_level
//              S16 Q7 dB (low half-word); gain word = stage2 S16 Q7 [31:16] | stage1 [15:0];
//              dev_id = OUI word + device_code[15:0]; payload_fmt = 2 words (below)
//   payload fmt word0: pack_method[31] real_cplx[30:29] item_fmt[28:24] rpt[23]
//              event_tag[22:20] chan_tag[19:16] fract_sz[15:12] pack_sz-1[11:6] item_sz-1[5:0]
//              (16-bit complex cartesian signed fixed, processing-efficient = 0x200003CF)
//   payload fmt word1: repeat_count-1[31:16] | vector_size-1[15:0]
//   trailer: enables[31:24]+[23:20] indicators[19:12]+[11:8] e[7] ctx_count[6:0]
//            pairs: caltime 31/19, valid 30/18, reflock 29/17, agc 28/16, sig 27/15,
//            inv 26/14, overrange 25/13, samploss 24/12
//   CAM word: cee_en[31] cee_fmt[30] cer_en[29] cer_fmt[28] partial[27] warn_ok[26]
//             err_ok[25] action[24:23] (0 none,1 dry,2 execute) nack_only[22]
//             req_v[20] req_x[19] req_s[18] req_warn[17] req_err[16] timing[14:12]
//             partial_taken[11] scheduled[10]
//   command packet order: header, stream_id, (class_id), (ts), CAM, message_id,
//             (controllee id/uuid per CAM), (controller id/uuid per CAM), payload:
//             control/cancel = CIF words + fields; ack = WIF/EIF words + fields
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace vrt {

// ---- word-level helpers (wire is big-endian 32-bit words) ---------------------------
inline uint32_t rd32(const uint8_t *p) {
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
inline void wr32(uint8_t *p, uint32_t v) {
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)v;
}

enum packet_type {
	PT_DATA_NOSID = 0,
	PT_DATA = 1,
	PT_EXT_DATA_NOSID = 2,
	PT_EXT_DATA = 3,
	PT_CONTEXT = 4,
	PT_EXT_CONTEXT = 5,
	PT_COMMAND = 6,
	PT_EXT_COMMAND = 7,
};

enum tsi_mode { TSI_NONE = 0, TSI_UTC = 1, TSI_GPS = 2, TSI_OTHER = 3 };
enum tsf_mode { TSF_NONE = 0, TSF_SAMPLES = 1, TSF_PS = 2, TSF_FREERUN = 3 };

// CIF0 field bits (only the ones vrtd speaks; the codec rejects others explicitly)
enum cif0_bit {
	CIF0_CHANGE = 1u << 31,
	CIF0_BANDWIDTH = 1u << 29,
	CIF0_IF_FREQ = 1u << 28,
	CIF0_RF_FREQ = 1u << 27,
	CIF0_REF_LEVEL = 1u << 24,
	CIF0_GAIN = 1u << 23,
	CIF0_SAMP_RATE = 1u << 21,
	CIF0_DEVICE_ID = 1u << 17,
	CIF0_STATE_EVENT = 1u << 16,
	CIF0_PAYLOAD_FMT = 1u << 15,
	CIF0_KNOWN_MASK = CIF0_CHANGE | CIF0_BANDWIDTH | CIF0_IF_FREQ | CIF0_RF_FREQ | CIF0_REF_LEVEL |
	                  CIF0_GAIN | CIF0_SAMP_RATE | CIF0_DEVICE_ID | CIF0_STATE_EVENT | CIF0_PAYLOAD_FMT,
};

// 16-bit complex cartesian signed fixed point, processing-efficient, no tags
constexpr uint32_t PAYLOAD_FMT_CS16_W0 = 0x200003CF;
constexpr uint32_t PAYLOAD_FMT_CS16_W1 = 0x00000000;

// ---- fixed-point converters ----------------------------------------------------------
inline uint64_t hz_to_q20(double hz) { return (uint64_t)(int64_t)(hz * 1048576.0 + (hz >= 0 ? 0.5 : -0.5)); }
inline double q20_to_hz(uint64_t q) { return (double)(int64_t)q / 1048576.0; }
inline uint16_t db_to_q7(double db) { return (uint16_t)(int16_t)(db * 128.0 + (db >= 0 ? 0.5 : -0.5)); }
inline double q7_to_db(uint16_t q) { return (double)(int16_t)q / 128.0; }

// ---- context / control field set ------------------------------------------------------
// one struct serves context packets, control commands and ack echoes: `cif0` says which
// members are meaningful
struct fields {
	uint32_t cif0 = 0;
	double bandwidth_hz = 0;        // CIF0_BANDWIDTH
	double if_freq_hz = 0;          // CIF0_IF_FREQ
	double rf_freq_hz = 0;          // CIF0_RF_FREQ
	double ref_level_dbm = 0;       // CIF0_REF_LEVEL
	double gain1_db = 0;            // CIF0_GAIN (stage 1)
	double gain2_db = 0;            // CIF0_GAIN (stage 2)
	double samp_rate_hz = 0;        // CIF0_SAMP_RATE
	uint32_t device_oui = 0;        // CIF0_DEVICE_ID
	uint16_t device_code = 0;
	uint32_t state_event = 0;       // CIF0_STATE_EVENT (raw word)
	uint32_t payload_fmt_w0 = 0;    // CIF0_PAYLOAD_FMT
	uint32_t payload_fmt_w1 = 0;
};

struct class_id {
	bool present = false;
	uint32_t oui = 0;               // 24 bits
	uint16_t info_class = 0;
	uint16_t packet_class = 0;
};

struct timestamp {
	tsi_mode tsi = TSI_NONE;
	tsf_mode tsf = TSF_NONE;
	uint32_t tsi_sec = 0;
	uint64_t tsf_frac = 0;
};

// ---- data packet ----------------------------------------------------------------------
struct data_packet {
	uint32_t stream_id = 0;
	class_id cls;
	timestamp ts;
	uint8_t count = 0;              // mod-16, caller maintains
	bool trailer = false;
	uint32_t trailer_word = 0;
	const uint8_t *payload = nullptr;   // big-endian sample words, 32-bit aligned
	size_t payload_words = 0;
};

// ---- context packet -------------------------------------------------------------------
struct context_packet {
	uint32_t stream_id = 0;
	class_id cls;
	timestamp ts;
	uint8_t count = 0;
	bool tsm_coarse = false;        // header TSM bit (indicator bit 24)
	fields f;
};

// ---- command / ack packets (49.2) ----------------------------------------------------
struct cam_word {
	bool controllee = false;
	bool controllee_uuid = false;   // id format: false = 32-bit
	bool controller = false;
	bool controller_uuid = false;
	bool partial_ok = false;
	bool warnings_ok = false;
	bool errors_ok = false;
	uint8_t action = 0;             // 0 none, 1 dry-run, 2 execute
	bool nack_only = false;
	bool req_v = false;             // request validation ack
	bool req_x = false;             // request execution ack
	bool req_s = false;             // request state query ack
	bool req_warn = false;
	bool req_err = false;
	uint8_t timing = 0;             // 0 ignore timestamp .. 4 permitted early/late
	bool partial_taken = false;     // ack echo
	bool scheduled = false;         // ack echo
	uint32_t pack() const;
	static cam_word unpack(uint32_t w);
};

struct command_packet {
	bool is_ack = false;            // header indicator bit 26
	bool is_cancel = false;         // header indicator bit 24
	uint32_t stream_id = 0;
	class_id cls;
	timestamp ts;
	uint8_t count = 0;
	cam_word cam;
	uint32_t message_id = 0;
	uint32_t controllee_id = 0;     // 32-bit form only (UUIDs rejected in v1)
	uint32_t controller_id = 0;
	fields f;                       // control payload (CIF0 + fields)
	// ack payload: warning/error indicator words + per-field error words. v1 speaks
	// WIF0/EIF0 only, and each flagged field carries one status word.
	uint32_t wif0 = 0;
	uint32_t eif0 = 0;
	std::vector<uint32_t> wif0_words;
	std::vector<uint32_t> eif0_words;
};

// ---- pack / unpack --------------------------------------------------------------------
// all packers append to `out` (bytes) and return the packet size in words;
// unpackers parse one datagram and return false on malformed/unsupported input,
// with a reason string for diagnostics
size_t pack_data(const data_packet &p, std::vector<uint8_t> &out);
size_t pack_context(const context_packet &p, std::vector<uint8_t> &out);
size_t pack_command(const command_packet &p, std::vector<uint8_t> &out);

struct parsed {
	int type = -1;                  // packet_type of the parsed packet
	data_packet data;
	context_packet ctx;
	command_packet cmd;
	// data payload view into the input buffer
	const uint8_t *payload = nullptr;
	size_t payload_words = 0;
};

bool unpack(const uint8_t *buf, size_t len, parsed &out, const char **why);

// fields sub-codec, shared by context and command payloads (exposed for tests)
size_t pack_fields(const fields &f, std::vector<uint8_t> &out);
bool unpack_fields(const uint8_t *w, size_t nwords, fields &f, const char **why);

} // namespace vrt

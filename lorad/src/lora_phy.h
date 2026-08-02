// lorad: LoRa PHY primitives - chirp tables, symbol (de)modulation, Gray mapping,
// diagonal interleaver, Hamming FEC, whitening, payload CRC, explicit header.
//
// Bit/byte conventions follow the public reverse-engineered LoRa PHY as implemented
// by the EPFL gr-lora-sdr project (Tapparel et al.); every convention that could
// differ between implementations (whitening LFSR, Gray direction, the +1 symbol
// offset, interleaver diagonal, CRC finalization, header checksum) is isolated in
// this module so on-air interop fixes stay one-line changes.
#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include "fft.h"

namespace lorad {

struct phy_params {
	int sf = 7;           // spreading factor 7..12
	int cr = 1;           // coding rate 1..4 (4/5 .. 4/8)
	bool ldro = false;    // low data rate optimization (payload blocks at SF-2 bits/symbol)
	bool has_crc = true;  // payload CRC16 present
	uint8_t sync = 0x34;  // sync word (0x34 LoRaWAN public, 0x2B Meshtastic, 0x12 private)
	int preamble_len = 8; // upchirps before sync
};

// ---- small bit helpers ----------------------------------------------------------

static inline uint32_t gray_encode(uint32_t v) {
	return v ^ (v >> 1);
}

static inline uint32_t gray_decode(uint32_t g) {
	g ^= g >> 16;
	g ^= g >> 8;
	g ^= g >> 4;
	g ^= g >> 2;
	g ^= g >> 1;
	return g;
}

// whitening LFSR over payload bytes: state seeds 0xFF, next = (s << 1) | parity(s & 0xB8)
// (x^8 + x^6 + x^5 + x^4 + 1). CRC bytes and header nibbles are not whitened.
void whiten(uint8_t *data, int len);

// payload CRC16 (poly 0x1021, init 0x0000) with the Semtech finalization quirk:
// the last two payload bytes are XORed into the CRC instead of being fed through.
uint16_t payload_crc(const uint8_t *data, int len);

// Hamming encode of one nibble (bits d3..d0) to a (4+cr)-bit codeword, MSB-first
// layout [d3 d2 d1 d0 p0 p1 p2 p3] truncated to 4+cr bits; cr==1 uses the single
// even-parity bit p = d3^d2^d1^d0.
uint8_t hamming_encode(uint8_t nibble, int cr);

// min-distance decode; cr>=3 corrects one bit, cr<=2 requires an exact match.
// returns nibble 0..15, or -1 if uncorrectable.
int hamming_decode(uint8_t codeword, int cr);

// ---- explicit PHY header --------------------------------------------------------

struct phy_header {
	int payload_len = 0;
	int cr = 1;
	bool has_crc = false;
	bool valid = false;
};

// header nibbles: [len>>4, len&0xF, (cr<<1)|crc, c4, c3..c0]; checksum per the
// published RE formulas.
void header_build(int payload_len, int cr, bool has_crc, uint8_t nib[5]);
phy_header header_parse(const uint8_t nib[5]);

// ---- symbol-level encode (used by the selftest TX and the meshtastic synth) ------

// full frame payload -> symbol values (header block + payload blocks), explicit header
std::vector<uint16_t> encode_frame(const phy_params &p, const uint8_t *payload, int len);

// number of payload symbols following the 8-symbol header block
int n_payload_symbols(const phy_params &p, int payload_len);

// ---- symbol-level decode --------------------------------------------------------

// incremental block decoder: feed raw demodulated symbol values (already CFO/timing
// corrected), get the payload out. Block 0 (8 symbols) is always reduced-rate CR4/8
// and carries the explicit header.
class frame_decoder {
public:
	explicit frame_decoder(int sf, bool ldro);

	// feed one raw symbol value (0..2^sf-1). Returns true while more symbols are needed.
	bool feed(uint16_t raw_symbol);

	bool header_ready() const { return hdr_.valid; }
	const phy_header &header() const { return hdr_; }
	bool done() const { return done_; }

	// results, valid once done()
	const std::vector<uint8_t> &payload() const { return payload_; }
	bool crc_ok() const { return crc_ok_; }
	bool fec_clean() const { return fec_clean_; }
	int symbols_needed() const; // total payload-part symbols incl. header block

private:
	void process_block();
	void finish();

	int sf_;
	bool ldro_;
	phy_header hdr_;
	std::vector<uint16_t> block_;   // raw symbols of the current interleaving block
	std::vector<uint8_t> nibbles_;  // decoded nibble stream (header nibbles stripped)
	std::vector<uint8_t> payload_;
	int block_idx_ = 0;
	int nibbles_wanted_ = 0;
	bool done_ = false;
	bool crc_ok_ = false;
	bool fec_clean_ = true;
	uint16_t rx_crc_ = 0;
};

// ---- chirp modulation (selftest TX) ----------------------------------------------

// N = 2^sf samples per symbol at fs == BW. Symbol s shifts the base upchirp by s bins.
// TX applies the +1 offset the RX removes (see frame RX in receiver.cpp).
class chirp_mod {
public:
	explicit chirp_mod(int sf);
	void symbol(uint16_t s, std::vector<cf> &out) const;   // append one upchirp
	void downchirp(std::vector<cf> &out, int quarter) const; // append N (or N/4) downchirp samples
	// full frame IQ at fs == BW: preamble, sync, SFD, data symbols
	std::vector<cf> frame(const phy_params &p, const std::vector<uint16_t> &symbols) const;
	int n() const { return n_; }

private:
	int sf_;
	int n_;
	std::vector<cf> up_;
};

} // namespace lorad

#include "lora_phy.h"

#include <cmath>
#include <cstring>

namespace lorad {

// ---- whitening / crc / hamming ---------------------------------------------------

void whiten(uint8_t *data, int len) {
	uint8_t s = 0xFF;
	for(int i = 0; i < len; i++) {
		data[i] ^= s;
		uint8_t t = s & 0xB8; // taps b7 b5 b4 b3  (x^8 + x^6 + x^5 + x^4 + 1)
		t ^= t >> 4;
		t ^= t >> 2;
		t ^= t >> 1;
		s = (uint8_t)((s << 1) | (t & 1));
	}
}

static uint16_t crc16_ccitt(const uint8_t *data, int len, uint16_t crc) {
	for(int i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for(int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

uint16_t payload_crc(const uint8_t *data, int len) {
	if(len < 2) {
		return crc16_ccitt(data, len, 0x0000);
	}
	uint16_t crc = crc16_ccitt(data, len - 2, 0x0000);
	crc ^= (uint16_t)data[len - 1];
	crc ^= (uint16_t)data[len - 2] << 8;
	return crc;
}

uint8_t hamming_encode(uint8_t nibble, int cr) {
	int d3 = (nibble >> 3) & 1, d2 = (nibble >> 2) & 1, d1 = (nibble >> 1) & 1, d0 = nibble & 1;
	uint8_t cw;
	if(cr == 1) {
		cw = (uint8_t)((nibble << 1) | (d3 ^ d2 ^ d1 ^ d0));
	} else {
		int p0 = d3 ^ d2 ^ d1;
		int p1 = d3 ^ d2 ^ d0;
		int p2 = d3 ^ d1 ^ d0;
		int p3 = d2 ^ d1 ^ d0;
		int parity = (p0 << 3) | (p1 << 2) | (p2 << 1) | p3;
		cw = (uint8_t)((nibble << cr) | (parity >> (4 - cr)));
	}
	return cw;
}

int hamming_decode(uint8_t codeword, int cr) {
	int bits = 4 + cr;
	int best = -1, best_dist = 8, second = 8;
	for(int n = 0; n < 16; n++) {
		uint8_t cand = hamming_encode((uint8_t)n, cr);
		int d = __builtin_popcount((cand ^ codeword) & ((1 << bits) - 1));
		if(d < best_dist) {
			second = best_dist;
			best_dist = d;
			best = n;
		} else if(d < second) {
			second = d;
		}
	}
	if(best_dist == 0) {
		return best;
	}
	if(cr >= 3 && best_dist == 1 && second > 1) {
		return best; // single-bit correction, unambiguous
	}
	return -1;
}

// ---- explicit header --------------------------------------------------------------

void header_build(int payload_len, int cr, bool has_crc, uint8_t nib[5]) {
	nib[0] = (uint8_t)((payload_len >> 4) & 0xF);
	nib[1] = (uint8_t)(payload_len & 0xF);
	nib[2] = (uint8_t)(((cr & 7) << 1) | (has_crc ? 1 : 0));
	int c4 = ((nib[0] >> 3) ^ (nib[0] >> 2) ^ (nib[0] >> 1) ^ nib[0]) & 1;
	int c3 = ((nib[0] >> 3) ^ (nib[1] >> 3) ^ (nib[1] >> 2) ^ (nib[1] >> 1) ^ nib[2]) & 1;
	int c2 = ((nib[0] >> 2) ^ (nib[1] >> 3) ^ nib[1] ^ (nib[2] >> 3) ^ (nib[2] >> 1)) & 1;
	int c1 = ((nib[0] >> 1) ^ (nib[1] >> 2) ^ nib[1] ^ (nib[2] >> 2) ^ (nib[2] >> 1) ^ nib[2]) & 1;
	int c0 = (nib[0] ^ (nib[1] >> 1) ^ (nib[2] >> 3) ^ (nib[2] >> 2) ^ (nib[2] >> 1) ^ nib[2]) & 1;
	nib[3] = (uint8_t)c4;
	nib[4] = (uint8_t)((c3 << 3) | (c2 << 2) | (c1 << 1) | c0);
}

phy_header header_parse(const uint8_t nib[5]) {
	phy_header h;
	uint8_t chk[5];
	header_build((int)((nib[0] << 4) | nib[1]), (nib[2] >> 1) & 7, nib[2] & 1, chk);
	h.payload_len = (nib[0] << 4) | nib[1];
	h.cr = (nib[2] >> 1) & 7;
	h.has_crc = nib[2] & 1;
	h.valid = ((chk[3] & 1) == (nib[3] & 1)) && ((chk[4] & 0xF) == (nib[4] & 0xF)) &&
	          h.cr >= 1 && h.cr <= 4 && h.payload_len > 0;
	return h;
}

// ---- interleaver -------------------------------------------------------------------
// One block: sf_app codewords of (4+cr) bits <-> (4+cr) symbols of sf_app bits.
// Diagonal mapping (EPFL convention): symbol i bit j (j==0 is the symbol MSB) takes
// codeword ((i - j - 1) mod sf_app) bit i (bit 0 is the codeword MSB).

static void interleave_block(const uint8_t *cw, int sf_app, int cr, uint16_t *symbols) {
	int nbits = 4 + cr;
	for(int i = 0; i < nbits; i++) {
		uint16_t s = 0;
		for(int j = 0; j < sf_app; j++) {
			int k = ((i - j - 1) % sf_app + sf_app) % sf_app;
			int bit = (cw[k] >> (nbits - 1 - i)) & 1;
			s = (uint16_t)((s << 1) | bit);
		}
		symbols[i] = s;
	}
}

static void deinterleave_block(const uint16_t *symbols, int sf_app, int cr, uint8_t *cw) {
	int nbits = 4 + cr;
	memset(cw, 0, (size_t)sf_app);
	for(int i = 0; i < nbits; i++) {
		for(int j = 0; j < sf_app; j++) {
			int k = ((i - j - 1) % sf_app + sf_app) % sf_app;
			int bit = (symbols[i] >> (sf_app - 1 - j)) & 1;
			cw[k] = (uint8_t)(cw[k] | (bit << (nbits - 1 - i)));
		}
	}
}

// ---- frame encode ------------------------------------------------------------------

int n_payload_symbols(const phy_params &p, int payload_len) {
	int de = p.ldro ? 1 : 0;
	int num = 8 * payload_len - 4 * p.sf + 28 + 16 * (p.has_crc ? 1 : 0);
	int den = 4 * (p.sf - 2 * de);
	int blocks = num > 0 ? (num + den - 1) / den : 0;
	return blocks * (p.cr + 4);
}

// map codeword bits onto the wire symbol value: inverse-Gray, shift for reduced
// rate, +1 offset (removed again by the RX demodulator)
static uint16_t to_wire_symbol(uint16_t bits, int sf, bool reduced) {
	uint32_t v = gray_decode(bits);
	if(reduced) {
		v <<= 2;
	}
	return (uint16_t)((v + 1) & ((1u << sf) - 1));
}

std::vector<uint16_t> encode_frame(const phy_params &p, const uint8_t *payload, int len) {
	std::vector<uint8_t> nibbles;
	uint8_t hdr[5];
	header_build(len, p.cr, p.has_crc, hdr);
	nibbles.insert(nibbles.end(), hdr, hdr + 5);

	std::vector<uint8_t> white(payload, payload + len);
	uint16_t crc = payload_crc(payload, len);
	whiten(white.data(), len);
	for(int i = 0; i < len; i++) {
		nibbles.push_back(white[i] & 0xF);
		nibbles.push_back(white[i] >> 4);
	}
	if(p.has_crc) {
		nibbles.push_back(crc & 0xF);
		nibbles.push_back((crc >> 4) & 0xF);
		nibbles.push_back((crc >> 8) & 0xF);
		nibbles.push_back((crc >> 12) & 0xF);
	}

	std::vector<uint16_t> out;
	size_t pos = 0;

	// block 0: always reduced rate, always CR4/8, sf-2 codewords, 8 symbols
	{
		int sf_app = p.sf - 2;
		std::vector<uint8_t> cw(sf_app, 0);
		for(int k = 0; k < sf_app; k++) {
			uint8_t nib = pos < nibbles.size() ? nibbles[pos++] : 0;
			cw[k] = hamming_encode(nib, 4);
		}
		uint16_t sym[8];
		interleave_block(cw.data(), sf_app, 4, sym);
		for(int i = 0; i < 8; i++) {
			out.push_back(to_wire_symbol(sym[i], p.sf, true));
		}
	}

	// payload blocks
	int sf_app = p.ldro ? p.sf - 2 : p.sf;
	while(pos < nibbles.size()) {
		std::vector<uint8_t> cw(sf_app, 0);
		for(int k = 0; k < sf_app; k++) {
			uint8_t nib = pos < nibbles.size() ? nibbles[pos++] : 0;
			cw[k] = hamming_encode(nib, p.cr);
		}
		std::vector<uint16_t> sym(4 + p.cr);
		interleave_block(cw.data(), sf_app, p.cr, sym.data());
		for(int i = 0; i < 4 + p.cr; i++) {
			out.push_back(to_wire_symbol(sym[i], p.sf, p.ldro));
		}
	}
	return out;
}

// ---- frame decode ------------------------------------------------------------------

frame_decoder::frame_decoder(int sf, bool ldro) : sf_(sf), ldro_(ldro) {
}

int frame_decoder::symbols_needed() const {
	if(!hdr_.valid) {
		return 8;
	}
	phy_params p;
	p.sf = sf_;
	p.cr = hdr_.cr;
	p.ldro = ldro_;
	p.has_crc = hdr_.has_crc;
	return 8 + n_payload_symbols(p, hdr_.payload_len);
}

// undo to_wire_symbol(): -1 offset, reduced-rate shift, Gray encode back to bits
static uint16_t from_wire_symbol(uint16_t raw, int sf, bool reduced) {
	uint32_t v = ((uint32_t)raw + (1u << sf) - 1) & ((1u << sf) - 1);
	if(reduced) {
		v >>= 2;
	}
	return (uint16_t)gray_encode(v);
}

bool frame_decoder::feed(uint16_t raw_symbol) {
	if(done_) {
		return false;
	}
	block_.push_back(raw_symbol);
	int nbits = block_idx_ == 0 ? 8 : 4 + hdr_.cr;
	if((int)block_.size() == nbits) {
		process_block();
		block_.clear();
		block_idx_++;
	}
	if(hdr_.valid && (int)nibbles_.size() >= nibbles_wanted_ && block_.empty()) {
		finish();
		return false;
	}
	if(block_idx_ > 0 && !hdr_.valid) {
		done_ = true; // header failed, nothing more to do
		return false;
	}
	return !done_;
}

void frame_decoder::process_block() {
	bool reduced = block_idx_ == 0 ? true : ldro_;
	int cr = block_idx_ == 0 ? 4 : hdr_.cr;
	int sf_app = reduced ? sf_ - 2 : sf_;

	std::vector<uint16_t> bits(block_.size());
	for(size_t i = 0; i < block_.size(); i++) {
		bits[i] = from_wire_symbol(block_[i], sf_, reduced);
	}
	std::vector<uint8_t> cw(sf_app, 0);
	deinterleave_block(bits.data(), sf_app, cr, cw.data());

	std::vector<uint8_t> nibs;
	for(int k = 0; k < sf_app; k++) {
		int n = hamming_decode(cw[k], cr);
		if(n < 0) {
			fec_clean_ = false;
			n = gray_decode(cw[k] >> cr) & 0xF; // best-effort: strip parity
		}
		nibs.push_back((uint8_t)n);
	}

	if(block_idx_ == 0) {
		hdr_ = header_parse(nibs.data());
		if(!hdr_.valid) {
			return;
		}
		nibbles_wanted_ = 2 * hdr_.payload_len + (hdr_.has_crc ? 4 : 0);
		nibbles_.insert(nibbles_.end(), nibs.begin() + 5, nibs.end());
	} else {
		nibbles_.insert(nibbles_.end(), nibs.begin(), nibs.end());
	}
}

void frame_decoder::finish() {
	done_ = true;
	payload_.resize(hdr_.payload_len);
	for(int i = 0; i < hdr_.payload_len; i++) {
		payload_[i] = (uint8_t)(nibbles_[2 * i] | (nibbles_[2 * i + 1] << 4));
	}
	whiten(payload_.data(), hdr_.payload_len); // XOR sequence, self-inverse
	if(hdr_.has_crc && (int)nibbles_.size() >= nibbles_wanted_) {
		int b = 2 * hdr_.payload_len;
		rx_crc_ = (uint16_t)(nibbles_[b] | (nibbles_[b + 1] << 4) | (nibbles_[b + 2] << 8) | (nibbles_[b + 3] << 12));
		crc_ok_ = payload_crc(payload_.data(), hdr_.payload_len) == rx_crc_;
	} else {
		crc_ok_ = !hdr_.has_crc;
	}
}

// ---- chirp modulator ---------------------------------------------------------------

chirp_mod::chirp_mod(int sf) : sf_(sf), n_(1 << sf) {
	up_.resize(n_);
	for(int i = 0; i < n_; i++) {
		double ph = 2.0 * M_PI * ((double)i * i / (2.0 * n_) - (double)i / 2.0);
		up_[i] = cf((float)cos(ph), (float)sin(ph));
	}
}

void chirp_mod::symbol(uint16_t s, std::vector<cf> &out) const {
	// shift-by-s == base chirp advanced s samples with the accumulated phase removed;
	// generate directly from the quadratic phase to keep symbol boundaries phase-clean
	for(int i = 0; i < n_; i++) {
		double fr = (double)s / n_ - 0.5;
		double ph = 2.0 * M_PI * ((double)i * i / (2.0 * n_) + fr * i);
		out.push_back(cf((float)cos(ph), (float)sin(ph)));
	}
}

void chirp_mod::downchirp(std::vector<cf> &out, int quarter) const {
	int len = quarter ? n_ / 4 : n_;
	for(int i = 0; i < len; i++) {
		out.push_back(std::conj(up_[i]));
	}
}

std::vector<cf> chirp_mod::frame(const phy_params &p, const std::vector<uint16_t> &symbols) const {
	std::vector<cf> iq;
	iq.reserve((size_t)(p.preamble_len + 2 + 3 + (int)symbols.size() + 1) * n_);
	for(int i = 0; i < p.preamble_len; i++) {
		symbol(0, iq);
	}
	symbol((uint16_t)(((p.sync >> 4) & 0xF) << 3), iq);
	symbol((uint16_t)((p.sync & 0xF) << 3), iq);
	downchirp(iq, 0);
	downchirp(iq, 0);
	downchirp(iq, 1);
	for(uint16_t s : symbols) {
		symbol(s, iq);
	}
	return iq;
}

} // namespace lorad

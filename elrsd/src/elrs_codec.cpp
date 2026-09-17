// elrsd: ExpressLRS 3.3.1 bit-level codec - implementation.
// Pure, deterministic, no I/O, no threads.
// the selftest (src/elrs_codec_selftest.cpp) checks it bit-exactly against the embedded
// vectors (tools/elrsd_vectors.json, TEST-UID only).
#include "elrs_codec.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>

namespace elrsd {

// ---- MD5 (RFC 1321) --------------------------------------------------------------------

namespace {

struct md5_ctx {
	uint32_t a = 0x67452301u, b = 0xefcdab89u, c = 0x98badcfeu, d = 0x10325476u;
	uint64_t len = 0;
	uint8_t buf[64];
	size_t buf_len = 0;
};

static const uint32_t MD5_K[64] = {
	0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu, 0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
	0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu, 0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
	0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau, 0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
	0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu, 0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
	0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu, 0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
	0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u, 0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
	0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u, 0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
	0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u, 0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u,
};
static const int MD5_S[64] = {
	7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
	5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
	4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
	6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
};

static inline uint32_t rotl32(uint32_t x, int s) { return (x << s) | (x >> (32 - s)); }

static void md5_block(md5_ctx &c, const uint8_t *p) {
	uint32_t m[16];
	for(int i = 0; i < 16; i++) {
		m[i] = (uint32_t)p[4 * i] | ((uint32_t)p[4 * i + 1] << 8) | ((uint32_t)p[4 * i + 2] << 16) | ((uint32_t)p[4 * i + 3] << 24);
	}
	uint32_t a = c.a, b = c.b, cc = c.c, d = c.d;
	for(int i = 0; i < 64; i++) {
		uint32_t f;
		int g;
		if(i < 16) { f = (b & cc) | (~b & d); g = i; }
		else if(i < 32) { f = (d & b) | (~d & cc); g = (5 * i + 1) & 15; }
		else if(i < 48) { f = b ^ cc ^ d; g = (3 * i + 5) & 15; }
		else { f = cc ^ (b | ~d); g = (7 * i) & 15; }
		uint32_t tmp = d;
		d = cc;
		cc = b;
		b = b + rotl32(a + f + MD5_K[i] + m[g], MD5_S[i]);
		a = tmp;
	}
	c.a += a; c.b += b; c.c += cc; c.d += d;
}

static void md5_update(md5_ctx &c, const uint8_t *p, size_t n) {
	c.len += n;
	while(n) {
		size_t take = std::min(n, (size_t)64 - c.buf_len);
		memcpy(c.buf + c.buf_len, p, take);
		c.buf_len += take; p += take; n -= take;
		if(c.buf_len == 64) { md5_block(c, c.buf); c.buf_len = 0; }
	}
}

static void md5_final(md5_ctx &c, uint8_t out[16]) {
	uint64_t bits = c.len * 8;
	uint8_t pad = 0x80;
	md5_update(c, &pad, 1);
	uint8_t z = 0;
	while(c.buf_len != 56) md5_update(c, &z, 1);
	uint8_t lb[8];
	for(int i = 0; i < 8; i++) lb[i] = (uint8_t)(bits >> (8 * i));
	md5_update(c, lb, 8);
	uint32_t v[4] = {c.a, c.b, c.c, c.d};
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++) out[4 * i + j] = (uint8_t)(v[i] >> (8 * j));
	}
}

} // namespace

uid_t6 uid_from_binding_phrase(const std::string &phrase) {
	std::string in = "-DMY_BINDING_PHRASE=\"" + phrase + "\"";
	md5_ctx c;
	md5_update(c, (const uint8_t *)in.data(), in.size());
	uint8_t dig[16];
	md5_final(c, dig);
	uid_t6 u;
	for(int i = 0; i < 6; i++) u[i] = dig[i];
	return u;
}

std::string uid_to_string(const uid_t6 &uid) {
	char buf[24];
	snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
	return buf;
}

bool uid_from_string(const std::string &s, uid_t6 &out) {
	int hex[12];
	int n = 0;
	for(size_t i = 0; i < s.size(); i++) {
		char ch = s[i];
		if(ch == ':' || ch == '-' || ch == ' ') continue;
		int v;
		if(ch >= '0' && ch <= '9') v = ch - '0';
		else if(ch >= 'a' && ch <= 'f') v = ch - 'a' + 10;
		else if(ch >= 'A' && ch <= 'F') v = ch - 'A' + 10;
		else return false;
		if(n >= 12) return false;
		hex[n++] = v;
	}
	if(n != 12) return false;
	for(int i = 0; i < 6; i++) out[i] = (uint8_t)((hex[2 * i] << 4) | hex[2 * i + 1]);
	return true;
}

// ---- FHSS ----------------------------------------------------------------------------------

double fhss_channel_freq_hz(int ch) { return FHSS_FREQ_START_HZ + FHSS_FREQ_STEP_HZ * (double)ch; }

double fhss_channel_freq_hz_sx1280(int ch) {
	// SX1280 register: FREQ_STEP = 52e6 / 2^18; start/stop registers of 2400.4 / 2479.4 MHz,
	// spread = (stop - start) * 256 / (n - 1) (integer, x256), freq_reg = start + spread * ch / 256
	const double step = 52.0e6 / 262144.0;
	const int64_t start_reg = (int64_t)(2400400000.0 / step);
	const int64_t stop_reg = (int64_t)(2479400000.0 / step);
	const int64_t spread = (stop_reg - start_reg) * 256 / (FHSS_NUM_CHANNELS - 1);
	const int64_t reg = start_reg + spread * (int64_t)ch / 256;
	return (double)reg * step;
}

int fhss_channel_from_freq(double f_hz) {
	double x = (f_hz - FHSS_FREQ_START_HZ) / FHSS_FREQ_STEP_HZ;
	long ch = lround(x);
	if(ch < 0 || ch >= FHSS_NUM_CHANNELS) return -1;
	if(fabs(f_hz - fhss_channel_freq_hz((int)ch)) > 0.5 * FHSS_FREQ_STEP_HZ) return -1;
	return (int)ch;
}

uint32_t elrs_rng::next() {
	// ELRS random.cpp: seed = (214013 * seed + 2531011) % 2147483648 with uint32 arithmetic
	state = (214013u * state + 2531011u) % 2147483648u;
	return state >> 16;
}

std::array<uint8_t, FHSS_SEQ_LEN> fhss_sequence(const uid_t6 &uid) {
	std::array<uint8_t, FHSS_SEQ_LEN> seq{};
	const int n = FHSS_NUM_CHANNELS;
	for(int i = 0; i < FHSS_SEQ_LEN; i++) {
		int m = i % n;
		seq[i] = (uint8_t)(m == 0 ? FHSS_SYNC_CHANNEL : (m == FHSS_SYNC_CHANNEL ? 0 : m));
	}
	elrs_rng rng(fhss_seed_from_uid(uid));
	for(int i = 0; i < FHSS_SEQ_LEN; i++) {
		if(i % n == 0) continue;
		int off = (i / n) * n;
		int r = (int)rng.next_n((uint32_t)(n - 1)) + 1;
		std::swap(seq[i], seq[off + r]);
	}
	return seq;
}

// ---- gray / whitening ------------------------------------------------------------------------

uint32_t gray_decode(uint32_t g) {
	uint32_t out = g;
	for(uint32_t mask = g >> 1; mask; mask >>= 1) out ^= mask;
	return out;
}

const uint8_t *whitening_seq() {
	// LoRa whitening sequence (gr-lora_sdr tables.h: low 8 bits of a 9-bit Fibonacci LFSR, one
	// bit shift per byte). Literal table = tools/elrsd_vectors.json whitening_seq (selftest checks).
	static const uint8_t seq[255] = {
		0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78, 0xF1, 0xE3,
		0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47,
		0x8E, 0x1C, 0x38, 0x71, 0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C, 0xB8, 0x70, 0xE0,
		0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49, 0x93, 0x26, 0x4D, 0x9B,
		0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90, 0x20, 0x41, 0x82, 0x05, 0x0A, 0x15, 0x2B, 0x56,
		0xAD, 0x5B, 0xB6, 0x6D, 0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59, 0xB2, 0x65, 0xCB, 0x96, 0x2C, 0x58,
		0xB0, 0x61, 0xC3, 0x87, 0x0F, 0x1F, 0x3E, 0x7D, 0xFB, 0xF6, 0xED, 0xDB, 0xB7, 0x6F, 0xDE, 0xBD,
		0x7A, 0xF5, 0xEB, 0xD7, 0xAE, 0x5D, 0xBA, 0x74, 0xE8, 0xD1, 0xA2, 0x44, 0x88, 0x10, 0x21, 0x43,
		0x86, 0x0D, 0x1B, 0x36, 0x6C, 0xD8, 0xB1, 0x63, 0xC7, 0x8F, 0x1E, 0x3C, 0x79, 0xF3, 0xE7, 0xCE,
		0x9C, 0x39, 0x73, 0xE6, 0xCC, 0x98, 0x31, 0x62, 0xC5, 0x8B, 0x16, 0x2D, 0x5A, 0xB4, 0x69, 0xD2,
		0xA4, 0x48, 0x91, 0x22, 0x45, 0x8A, 0x14, 0x29, 0x52, 0xA5, 0x4A, 0x95, 0x2A, 0x54, 0xA9, 0x53,
		0xA7, 0x4E, 0x9D, 0x3B, 0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xEC, 0xD9, 0xB3, 0x67, 0xCF, 0x9E, 0x3D,
		0x7B, 0xF7, 0xEF, 0xDF, 0xBF, 0x7E, 0xFD, 0xFA, 0xF4, 0xE9, 0xD3, 0xA6, 0x4C, 0x99, 0x33, 0x66,
		0xCD, 0x9A, 0x35, 0x6A, 0xD4, 0xA8, 0x51, 0xA3, 0x46, 0x8C, 0x18, 0x30, 0x60, 0xC1, 0x83, 0x07,
		0x0E, 0x1D, 0x3A, 0x75, 0xEA, 0xD5, 0xAA, 0x55, 0xAB, 0x57, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2,
		0xE5, 0xCA, 0x94, 0x28, 0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F, 0x9F, 0x3F, 0x7F,
	};
	return seq;
}

void whiten(uint8_t *data, size_t len) {
	const uint8_t *w = whitening_seq();
	for(size_t i = 0; i < len; i++) data[i] ^= w[i % 255];
}

// ---- LI codec ---------------------------------------------------------------------------------

namespace {
// bit name masks over (d3 d2 d1 d0): data bits and parities p0..p4
enum bitname { D3, D2, D1, D0, P0, P1, P2, P3, P4 };
static const uint8_t BIT_MASK[9] = {0x8, 0x4, 0x2, 0x1, 0x7, 0xB, 0xD, 0xE, 0xF};

static int parity4(uint32_t x) { return __builtin_popcount(x) & 1; }
}

li_codec::li_codec(const rate_params &rp) : rp_(rp) {
	const int sf = rp_.sf, cr = rp_.cr_bits;
	const int n_cw = 2 * rp_.payload_bytes;
	const int n_coded = n_cw * cr;
	rp_.n_cw = n_cw;
	rp_.n_coded = n_coded;
	// symbol widths
	widths_.clear();
	if(sf >= 7) {
		int head = 8 * (sf - 2);
		int rest = std::max(n_coded - head, 0);
		for(int i = 0; i < 8; i++) widths_.push_back(sf - 2);
		for(int i = 0; i < (rest + sf - 1) / sf; i++) widths_.push_back(sf);
	} else {
		for(int i = 0; i < (n_coded + sf - 1) / sf; i++) widths_.push_back(sf);
	}
	off_.assign(1, 0);
	for(int w : widths_) off_.push_back(off_.back() + w);
	rp_.n_sym = (int)widths_.size();
	rp_.n_slots = off_.back();
	rp_.n_tail = rp_.n_slots - n_coded;
	// bit table order (MSB first) and stream column order (gr-lora: d0 d1 d2 d3 P0 P1 P2 P3)
	std::vector<int> bits, cols;   // bitname enums
	bits = {D3, D2, D1, D0};
	cols = {D0, D1, D2, D3};
	std::vector<int> par;
	if(cr == 8) par = {P0, P3, P1, P2};
	else if(cr == 6) par = {P0, P3};
	else par = {P4};                // CR4/5
	for(int p : par) cols.push_back(p);
	std::vector<int> par_sorted = par;
	std::sort(par_sorted.begin(), par_sorted.end(), [](int a, int b) { return a > b; }); // descending name: p3 p2 p1 p0
	for(int p : par_sorted) bits.push_back(p);
	std::vector<int> K(cr);
	for(int b = 0; b < cr; b++) {
		K[b] = (int)(std::find(cols.begin(), cols.end(), bits[b]) - cols.begin());
	}
	// hamming tables
	for(int nib = 0; nib < 16; nib++) {
		uint32_t cw = 0;
		for(int b = 0; b < cr; b++) cw = (cw << 1) | (uint32_t)parity4((uint32_t)nib & BIT_MASK[bits[b]]);
		henc_[nib] = (uint8_t)cw;
	}
	const int ncw_all = 1 << cr;
	hdec_.assign(ncw_all, -1);
	for(int nib = 0; nib < 16; nib++) hdec_[henc_[nib]] = (int16_t)nib;
	int dmin = cr;
	for(int nib = 1; nib < 16; nib++) dmin = std::min(dmin, __builtin_popcount(henc_[nib]));
	int t = (dmin - 1) / 2;
	for(int r = 1; r <= t; r++) {
		for(int x = 0; x < ncw_all; x++) {
			if(hdec_[x] >= 0) continue;
			int hit = -1, nhit = 0;
			for(int nib = 0; nib < 16; nib++) {
				if(__builtin_popcount((uint32_t)(henc_[nib] ^ x)) == r) { hit = nib; nhit++; }
			}
			if(nhit == 1) hdec_[x] = (int16_t)hit;
		}
	}
	// interleaver: coded bit i = cr*c + b -> stream q = n_cw*K[b] + c -> slot
	auto slot_of_q = [&](int q) {
		int s = 0;
		while(q >= off_[s + 1]) s++;
		int r = q - off_[s];
		return off_[s] + (widths_[s] - 1) - r;
	};
	cb2slot_.resize(n_coded);
	for(int i = 0; i < n_coded; i++) {
		int c = i / cr, b = i % cr;
		cb2slot_[i] = slot_of_q(n_cw * K[b] + c);
	}
	tail_.clear();
	for(int q = n_coded; q < rp_.n_slots; q++) {
		tail_.push_back({slot_of_q(q), slot_of_q(q - (n_coded - 1))});
	}
	// whitening pushed through the (linear) hamming code
	wcoded_.resize(n_coded);
	const uint8_t *W = whitening_seq();
	for(int c = 0; c < n_cw; c++) {
		uint8_t wb = W[(c / 2) % 255];
		uint8_t nib = (uint8_t)((wb >> (4 * (c % 2))) & 0xF);
		uint32_t cw = henc_[nib];
		for(int b = 0; b < cr; b++) wcoded_[cr * c + b] = (uint8_t)((cw >> (cr - 1 - b)) & 1);
	}
}

int li_codec::hamming_decode(uint32_t codeword, bool correct) const {
	if(codeword >= hdec_.size()) return -1;
	if(correct) return hdec_[codeword];
	for(int nib = 0; nib < 16; nib++) {
		if(henc_[nib] == codeword) return nib;
	}
	return -1;
}

uint32_t li_codec::symbol_to_bits(uint16_t s, int sym_index, int ref) const {
	const uint32_t M = 1u << rp_.sf;
	int w = widths_[sym_index];
	uint32_t x = (uint32_t)(((int)s - ref - 1) % (int)M + (int)M) % M;
	return gray_encode(x >> (rp_.sf - w));
}

uint16_t li_codec::bits_to_symbol(uint32_t v, int sym_index, int ref) const {
	const uint32_t M = 1u << rp_.sf;
	int w = widths_[sym_index];
	uint32_t x = (gray_decode(v) << (rp_.sf - w)) + (uint32_t)(((ref % (int)M) + (int)M) % (int)M) + 1;
	return (uint16_t)(x % M);
}

std::vector<uint16_t> li_codec::encode(const uint8_t *payload, size_t len, int ref) const {
	const int cr = rp_.cr_bits, n_cw = rp_.n_cw, n_coded = rp_.n_coded;
	std::vector<uint8_t> wb((size_t)rp_.payload_bytes, 0);
	for(int i = 0; i < rp_.payload_bytes; i++) wb[i] = (size_t)i < len ? payload[i] : 0;
	whiten(wb.data(), wb.size());
	std::vector<uint8_t> slots((size_t)rp_.n_slots, 0);
	for(int c = 0; c < n_cw; c++) {
		uint8_t nib = (uint8_t)((wb[c / 2] >> (4 * (c % 2))) & 0xF);
		uint32_t cw = henc_[nib];
		for(int b = 0; b < cr; b++) slots[cb2slot_[cr * c + b]] = (uint8_t)((cw >> (cr - 1 - b)) & 1);
	}
	(void)n_coded;
	for(const auto &t : tail_) slots[t.first] = slots[t.second];
	std::vector<uint16_t> out((size_t)rp_.n_sym);
	for(int s = 0; s < rp_.n_sym; s++) {
		uint32_t v = 0;
		for(int j = off_[s]; j < off_[s + 1]; j++) v = (v << 1) | slots[j];
		out[s] = bits_to_symbol(v, s, ref);
	}
	return out;
}

bool li_codec::decode(const uint16_t *symbols, size_t n_symbols, int ref, std::vector<uint8_t> &payload,
                      bool correct, int *fec_corrected) const {
	if(fec_corrected) *fec_corrected = 0;
	if((int)n_symbols < rp_.n_sym) return false;
	const int cr = rp_.cr_bits, n_cw = rp_.n_cw;
	std::vector<uint8_t> slots((size_t)rp_.n_slots, 0);
	for(int s = 0; s < rp_.n_sym; s++) {
		int w = widths_[s];
		uint32_t v = symbol_to_bits(symbols[s], s, ref);
		for(int r = 0; r < w; r++) slots[off_[s] + r] = (uint8_t)((v >> (w - 1 - r)) & 1);
	}
	std::vector<uint8_t> nibs((size_t)n_cw);
	int corrected = 0;
	for(int c = 0; c < n_cw; c++) {
		uint32_t cw = 0;
		for(int b = 0; b < cr; b++) {
			int i = cr * c + b;
			cw = (cw << 1) | (uint32_t)(slots[cb2slot_[i]] ^ wcoded_[i]);
		}
		int nib = hamming_decode(cw, correct);
		if(nib < 0) return false;
		if(henc_[nib] != cw) corrected++;
		nibs[c] = (uint8_t)nib;
	}
	payload.resize((size_t)rp_.payload_bytes);
	for(int i = 0; i < rp_.payload_bytes; i++) payload[i] = (uint8_t)(nibs[2 * i] | (nibs[2 * i + 1] << 4));
	if(fec_corrected) *fec_corrected = corrected;
	return true;
}

// ---- CRC-14 -----------------------------------------------------------------------------------

const uint16_t *crc14_table() {
	static uint16_t tab[256];
	static bool init = false;
	if(!init) {
		const uint16_t poly = 0x2E57;
		for(int i = 0; i < 256; i++) {
			uint32_t crc = (uint32_t)i << 6;
			for(int b = 0; b < 8; b++) crc = (crc << 1) ^ ((crc & 0x2000) ? poly : 0);
			tab[i] = (uint16_t)(crc & 0x3FFF);
		}
		init = true;
	}
	return tab;
}

uint16_t crc14(const uint8_t *data, size_t len, uint16_t init) {
	const uint16_t *tab = crc14_table();
	uint32_t crc = init & 0x3FFF;
	for(size_t i = 0; i < len; i++) {
		crc = ((crc << 8) ^ tab[((crc >> 6) ^ data[i]) & 0xFF]) & 0x3FFF;
	}
	return (uint16_t)crc;
}

// ---- CRSF mapping -------------------------------------------------------------------------------

static const uint16_t CRSF_MIN = 172, CRSF_MAX = 1811, CRSF_MID = 992, CRSF_1000 = 191, CRSF_2000 = 1792;

uint32_t fmap(uint32_t x, uint32_t imin, uint32_t imax, uint32_t omin, uint32_t omax) {
	// ExpressLRS fmap(): ((x - in_min) * (out_max - out_min) * 2 / (in_max - in_min) + out_min * 2 + 1) / 2
	return ((x - imin) * (omax - omin) * 2 / (imax - imin) + omin * 2 + 1) / 2;
}

uint16_t crsf_to_uint10(uint16_t v) {
	uint16_t c = std::min(std::max(v, CRSF_MIN), CRSF_MAX);
	return (uint16_t)fmap(c, CRSF_MIN, CRSF_MAX, 0, 1023);
}

uint16_t uint10_to_crsf(uint16_t v10) { return (uint16_t)fmap(v10, 0, 1023, CRSF_MIN, CRSF_MAX); }

uint16_t crsf_to_n(uint16_t v, uint16_t cnt) {
	if(v <= CRSF_1000) return 0;
	if(v >= CRSF_2000) return (uint16_t)(cnt - 1);
	return (uint16_t)((uint32_t)(v - CRSF_1000) * cnt / (uint32_t)(CRSF_2000 - CRSF_1000 + 1));
}

uint16_t n_to_crsf(uint16_t val, uint16_t bins) {
	return (uint16_t)((uint32_t)val * (CRSF_2000 - CRSF_1000) / bins + CRSF_1000);
}

// ---- OTA4 ---------------------------------------------------------------------------------------

int hybrid_wide_switch_index(uint8_t nonce) { return ((nonce & 7) + ((nonce >> 3) & 1)) % 8; }

std::array<uint8_t, 8> ota4_build_rc(const uid_t6 &uid, const std::array<uint16_t, 16> &ch, uint8_t nonce,
                                     int hop_interval, int tlm_denom, int tx_power_crsf, bool telem_bit) {
	std::array<uint8_t, 8> pkt{};
	uint64_t word = 0;
	for(int i = 0; i < 4; i++) word |= (uint64_t)(crsf_to_uint10(ch[i]) & 0x3FF) << (10 * i);
	for(int i = 0; i < 5; i++) pkt[1 + i] = (uint8_t)(word >> (8 * i));
	int idx = hybrid_wide_switch_index(nonce);
	uint8_t val;
	if(idx == 7) {
		val = (uint8_t)(((telem_bit ? 1 : 0) << 6) | (tx_power_crsf & 0x3F));
	} else {
		bool low = tlm_denom > 1 && tlm_denom < 8;
		val = (uint8_t)(crsf_to_n(ch[idx + 5], low ? 64 : 128) & (low ? 0x3F : 0x7F));
		if(low && telem_bit) val |= 0x40;
	}
	uint8_t armed = ch[4] > CRSF_MID ? 1 : 0;
	pkt[6] = (uint8_t)((armed << 7) | val);
	int hop = hop_interval > 0 ? hop_interval : 4;
	pkt[0] = (uint8_t)(((int)ota_type::RC & 3) | (((nonce % hop) + 1) << 2));   // crcHigh := nonce % hop + 1
	uint16_t crc = crc14(pkt.data(), 7, crc14_init_from_uid(uid));
	pkt[0] = (uint8_t)(((int)ota_type::RC & 3) | (((crc >> 8) & 0x3F) << 2));
	pkt[7] = (uint8_t)(crc & 0xFF);
	return pkt;
}

std::array<uint8_t, 8> ota4_build_sync(const uid_t6 &uid, uint8_t fhss_index, uint8_t nonce, int rate_index,
                                       uint8_t new_tlm_ratio_field, bool switch_enc_mode, uint8_t model_id_xor) {
	std::array<uint8_t, 8> pkt{};
	pkt[0] = (uint8_t)ota_type::SYNC;       // crcHigh = 0 before the CRC
	pkt[1] = fhss_index;
	pkt[2] = nonce;
	pkt[3] = (uint8_t)((switch_enc_mode ? 1 : 0) | ((new_tlm_ratio_field & 7) << 1) | ((rate_index & 0xF) << 4));
	pkt[4] = uid[3];
	pkt[5] = uid[4];
	pkt[6] = (uint8_t)(uid[5] ^ model_id_xor);
	uint16_t crc = crc14(pkt.data(), 7, crc14_init_from_uid(uid));
	pkt[0] = (uint8_t)(((int)ota_type::SYNC & 3) | (((crc >> 8) & 0x3F) << 2));
	pkt[7] = (uint8_t)(crc & 0xFF);
	return pkt;
}

void ota4_resolve_switch(rc_packet &rc, uint8_t nonce, int tlm_denom) {
	int idx = hybrid_wide_switch_index(nonce);
	rc.switch_index = idx;
	uint8_t val = (uint8_t)(rc.aux_value_raw & 0x7F);
	rc.aux_channel = -1;
	rc.uplink_tx_power = -1;
	if(idx == 7) {
		rc.uplink_tx_power = val & 0x3F;
		rc.telem_bit = (val >> 6) & 1;
		rc.aux_value_raw = val;
	} else {
		bool low = tlm_denom > 1 && tlm_denom < 8;
		rc.aux_channel = 5 + idx;
		if(low) {
			rc.aux_value_raw = val & 0x3F;
			rc.telem_bit = (val >> 6) & 1;
			rc.ch[rc.aux_channel] = n_to_crsf(rc.aux_value_raw, 63);
		} else {
			rc.aux_value_raw = val & 0x7F;
			rc.telem_bit = false;
			rc.ch[rc.aux_channel] = n_to_crsf(rc.aux_value_raw, 127);
		}
	}
}

ota4_decoded ota4_decode(const uint8_t pkt[8], const uid_t6 &uid, int hop_interval, int tlm_denom, int nonce) {
	ota4_decoded d;
	d.type = (ota_type)(pkt[0] & 3);
	d.crc_high = (uint8_t)(pkt[0] >> 2);
	d.crc_low = pkt[7];
	const uint16_t init = crc14_init_from_uid(uid);
	const uint16_t want = (uint16_t)(((uint16_t)d.crc_high << 8) | d.crc_low);
	uint8_t tmp[7];
	memcpy(tmp, pkt, 7);
	if(d.type == ota_type::RC) {
		int hop = hop_interval > 0 ? hop_interval : 4;
		for(int r = 0; r < hop; r++) {
			tmp[0] = (uint8_t)((pkt[0] & 3) | ((r + 1) << 2));
			if(crc14(tmp, 7, init) == want) d.nonce_residues.push_back(r);
		}
		d.crc_ok = !d.nonce_residues.empty();
		if(d.crc_ok) {
			uint64_t word = 0;
			for(int i = 0; i < 5; i++) word |= (uint64_t)pkt[1 + i] << (8 * i);
			rc_packet &rc = d.rc;
			rc.ch.fill(0);
			for(int i = 0; i < 4; i++) rc.ch[i] = uint10_to_crsf((uint16_t)((word >> (10 * i)) & 0x3FF));
			rc.ch4_high = (pkt[6] >> 7) & 1;
			rc.ch[4] = bit_to_crsf(rc.ch4_high);
			rc.aux_value_raw = (uint16_t)(pkt[6] & 0x7F);
			rc.switch_index = -1;
			rc.aux_channel = -1;
			rc.uplink_tx_power = -1;
			rc.telem_bit = false;
			if(nonce >= 0) ota4_resolve_switch(rc, (uint8_t)nonce, tlm_denom);
		}
	} else {
		tmp[0] = (uint8_t)(pkt[0] & 3);     // crcHigh = 0 for every non-RC type
		d.crc_ok = crc14(tmp, 7, init) == want;
		if(d.crc_ok && d.type == ota_type::SYNC) {
			sync_packet &sp = d.sync;
			sp.fhss_index = pkt[1];
			sp.nonce = pkt[2];
			sp.switch_enc_mode = pkt[3] & 1;
			sp.new_tlm_ratio = (uint8_t)((pkt[3] >> 1) & 7);
			sp.rate_index = pkt[3] >> 4;
			sp.uid3 = pkt[4];
			sp.uid4 = pkt[5];
			sp.uid5 = pkt[6];
		}
	}
	return d;
}

// ---- freerun ground truth ---------------------------------------------------------------------------

uint32_t lowbias32(uint32_t x) {
	x ^= x >> 16;
	x *= 0x7FEB352Du;
	x ^= x >> 15;
	x *= 0x846CA68Bu;
	x ^= x >> 16;
	return x;
}

std::array<uint16_t, 16> freerun_channels(uint32_t tick) {
	std::array<uint16_t, 16> ch{};
	uint32_t base = tick * 16u;
	for(uint32_t i = 0; i < 16; i++) ch[i] = (uint16_t)(CRSF_MIN + lowbias32(base + i) % 1640u);
	return ch;
}

} // namespace elrsd

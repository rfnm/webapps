#include "aes128.h"

namespace lorad {

static const uint8_t SBOX[256] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
};

static inline uint8_t xtime(uint8_t x) {
	return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0x00));
}

void aes128::expand(const uint8_t key[16]) {
	memcpy(rk_, key, 16);
	uint8_t rcon = 1;
	for(int i = 16; i < 176; i += 4) {
		uint8_t t[4];
		memcpy(t, rk_ + i - 4, 4);
		if(i % 16 == 0) {
			uint8_t tmp = t[0];
			t[0] = (uint8_t)(SBOX[t[1]] ^ rcon);
			t[1] = SBOX[t[2]];
			t[2] = SBOX[t[3]];
			t[3] = SBOX[tmp];
			rcon = xtime(rcon);
		}
		for(int j = 0; j < 4; j++) {
			rk_[i + j] = (uint8_t)(rk_[i - 16 + j] ^ t[j]);
		}
	}
}

void aes128::encrypt_block(const uint8_t in[16], uint8_t out[16]) const {
	uint8_t s[16];
	for(int i = 0; i < 16; i++) {
		s[i] = (uint8_t)(in[i] ^ rk_[i]);
	}
	for(int round = 1; round <= 10; round++) {
		// SubBytes + ShiftRows
		uint8_t t[16];
		static const int SHIFT[16] = { 0, 5, 10, 15, 4, 9, 14, 3, 8, 13, 2, 7, 12, 1, 6, 11 };
		for(int i = 0; i < 16; i++) {
			t[i] = SBOX[s[SHIFT[i]]];
		}
		if(round < 10) {
			// MixColumns
			for(int c = 0; c < 4; c++) {
				uint8_t a0 = t[4 * c], a1 = t[4 * c + 1], a2 = t[4 * c + 2], a3 = t[4 * c + 3];
				uint8_t all = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
				s[4 * c] = (uint8_t)(a0 ^ all ^ xtime((uint8_t)(a0 ^ a1)));
				s[4 * c + 1] = (uint8_t)(a1 ^ all ^ xtime((uint8_t)(a1 ^ a2)));
				s[4 * c + 2] = (uint8_t)(a2 ^ all ^ xtime((uint8_t)(a2 ^ a3)));
				s[4 * c + 3] = (uint8_t)(a3 ^ all ^ xtime((uint8_t)(a3 ^ a0)));
			}
		} else {
			memcpy(s, t, 16);
		}
		for(int i = 0; i < 16; i++) {
			s[i] ^= rk_[16 * round + i];
		}
	}
	memcpy(out, s, 16);
}

// FIPS-197 InvCipher. Used only for MeshCore group traffic, which is AES-128-ECB. The
// inverse S-box is derived from SBOX once, so there is a single table to trust.
static inline uint8_t gmul(uint8_t a, int b) {
	uint8_t r = 0;
	for(; b; b >>= 1) {
		if(b & 1) {
			r ^= a;
		}
		a = xtime(a);
	}
	return r;
}

void aes128::decrypt_block(const uint8_t in[16], uint8_t out[16]) const {
	static const struct inv_sbox {
		uint8_t t[256];
		inv_sbox() {
			for(int i = 0; i < 256; i++) {
				t[SBOX[i]] = (uint8_t)i;
			}
		}
	} INV;
	static const int INV_SHIFT[16] = { 0, 13, 10, 7, 4, 1, 14, 11, 8, 5, 2, 15, 12, 9, 6, 3 };
	uint8_t s[16], t[16];
	for(int i = 0; i < 16; i++) {
		s[i] = (uint8_t)(in[i] ^ rk_[160 + i]);
	}
	for(int round = 9; round >= 0; round--) {
		for(int i = 0; i < 16; i++) {
			t[i] = (uint8_t)(INV.t[s[INV_SHIFT[i]]] ^ rk_[16 * round + i]);
		}
		if(round == 0) {
			memcpy(s, t, 16);
			break;
		}
		for(int c = 0; c < 4; c++) {
			const uint8_t *a = t + 4 * c;
			for(int r = 0; r < 4; r++) {
				s[4 * c + r] = (uint8_t)(gmul(a[r], 14) ^ gmul(a[(r + 1) & 3], 11) ^ gmul(a[(r + 2) & 3], 13) ^ gmul(a[(r + 3) & 3], 9));
			}
		}
	}
	memcpy(out, s, 16);
}

void aes128::ctr_crypt(const uint8_t nonce[16], uint8_t *data, size_t len) const {
	uint8_t ctr[16], ks[16];
	memcpy(ctr, nonce, 16);
	for(size_t off = 0; off < len; off += 16) {
		encrypt_block(ctr, ks);
		size_t n = len - off < 16 ? len - off : 16;
		for(size_t i = 0; i < n; i++) {
			data[off + i] ^= ks[i];
		}
		for(int i = 15; i >= 12; i--) {
			if(++ctr[i]) {
				break;
			}
		}
	}
}

aes256::aes256(const uint8_t key[32]) {
	memcpy(rk_, key, 32);
	uint8_t rcon = 1;
	for(int i = 8; i < 60; i++) {
		uint8_t t[4];
		memcpy(t, rk_ + 4 * (i - 1), 4);
		if(i % 8 == 0) {
			uint8_t f = t[0];
			t[0] = (uint8_t)(SBOX[t[1]] ^ rcon);
			t[1] = SBOX[t[2]];
			t[2] = SBOX[t[3]];
			t[3] = SBOX[f];
			rcon = xtime(rcon);
		} else if(i % 8 == 4) {
			for(int k = 0; k < 4; k++) {
				t[k] = SBOX[t[k]];
			}
		}
		for(int k = 0; k < 4; k++) {
			rk_[4 * i + k] = (uint8_t)(rk_[4 * (i - 8) + k] ^ t[k]);
		}
	}
}

void aes256::encrypt_block(const uint8_t in[16], uint8_t out[16]) const {
	static const int ROW[16] = { 0, 5, 10, 15, 4, 9, 14, 3, 8, 13, 2, 7, 12, 1, 6, 11 };
	uint8_t s[16], t[16];
	for(int i = 0; i < 16; i++) {
		s[i] = (uint8_t)(in[i] ^ rk_[i]);
	}
	for(int r = 1; r <= 14; r++) {
		for(int i = 0; i < 16; i++) {
			t[i] = SBOX[s[ROW[i]]];
		}
		if(r < 14) {
			for(int c = 0; c < 16; c += 4) {
				uint8_t a = t[c], b = t[c + 1], d = t[c + 2], e = t[c + 3], x = (uint8_t)(a ^ b ^ d ^ e);
				t[c] = (uint8_t)(a ^ x ^ xtime((uint8_t)(a ^ b)));
				t[c + 1] = (uint8_t)(b ^ x ^ xtime((uint8_t)(b ^ d)));
				t[c + 2] = (uint8_t)(d ^ x ^ xtime((uint8_t)(d ^ e)));
				t[c + 3] = (uint8_t)(e ^ x ^ xtime((uint8_t)(e ^ a)));
			}
		}
		for(int i = 0; i < 16; i++) {
			s[i] = (uint8_t)(t[i] ^ rk_[16 * r + i]);
		}
	}
	memcpy(out, s, 16);
}

} // namespace lorad

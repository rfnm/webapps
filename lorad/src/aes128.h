// lorad: minimal AES-128 implementation. Self-contained, no external crypto dependency;
// used solely for the best-effort Meshtastic default-PSK decode (CTR, forward cipher)
// and the MeshCore public-channel decode (ECB, inverse cipher).
#pragma once

#include <cstdint>
#include <cstring>

namespace lorad {

class aes128 {
public:
	explicit aes128(const uint8_t key[16]) {
		expand(key);
	}

	void encrypt_block(const uint8_t in[16], uint8_t out[16]) const;
	void decrypt_block(const uint8_t in[16], uint8_t out[16]) const;

	// CTR keystream XOR: nonce is the 16-byte initial counter block, incremented
	// big-endian in the last 4 bytes per block (matches Meshtastic/ESP32 usage)
	void ctr_crypt(const uint8_t nonce[16], uint8_t *data, size_t len) const;

private:
	void expand(const uint8_t key[16]);
	uint8_t rk_[176];
};

// forward cipher only: Meshtastic's public key DMs are AES-256-CCM
class aes256 {
public:
	explicit aes256(const uint8_t key[32]);
	void encrypt_block(const uint8_t in[16], uint8_t out[16]) const;

private:
	uint8_t rk_[240];
};

} // namespace lorad

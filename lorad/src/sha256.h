// lorad: minimal SHA-256 (FIPS 180-4) and HMAC-SHA-256 (RFC 2104). Self-contained, no
// external crypto dependency; used solely for the MeshCore packet hash and channel MAC.
#pragma once

#include <cstddef>
#include <cstdint>

namespace lorad {

class sha256 {
public:
	sha256();
	void update(const uint8_t *data, size_t len);
	void finish(uint8_t out[32]);

private:
	void block(const uint8_t *p);
	uint32_t h_[8];
	uint8_t buf_[64];
	size_t fill_ = 0;
	uint64_t total_ = 0;
};

void hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len, uint8_t out[32]);

} // namespace lorad

// adsld: chip-layer PHY helpers - see adsl_phy.h for the on-air conventions.
#include "adsl_phy.h"

namespace adsld {

bits bytes_to_bits_msb(const uint8_t *data, size_t len) {
	bits out;
	out.reserve(len * 8);
	for(size_t i = 0; i < len; i++) {
		for(int b = 7; b >= 0; b--) {
			out.push_back((uint8_t)((data[i] >> b) & 1));
		}
	}
	return out;
}

std::vector<uint8_t> bits_to_bytes_msb(const bits &b) {
	std::vector<uint8_t> out;
	out.reserve(b.size() / 8);
	for(size_t i = 0; i + 8 <= b.size(); i += 8) {
		uint8_t v = 0;
		for(int j = 0; j < 8; j++) {
			v = (uint8_t)((v << 1) | (b[i + j] & 1));
		}
		out.push_back(v);
	}
	return out;
}

bits manchester_encode(const bits &data) {
	bits out;
	out.reserve(data.size() * 2);
	for(uint8_t d : data) {
		// 1 -> 01, 0 -> 10: first chip is the bit's complement, second the bit itself
		out.push_back((uint8_t)(d ? 0 : 1));
		out.push_back((uint8_t)(d ? 1 : 0));
	}
	return out;
}

bits manchester_decode(const bits &chips, int *errors) {
	bits out;
	out.reserve(chips.size() / 2);
	int errs = 0;
	for(size_t i = 0; i + 2 <= chips.size(); i += 2) {
		uint8_t c0 = (uint8_t)(chips[i] & 1);
		uint8_t c1 = (uint8_t)(chips[i + 1] & 1);
		if(c0 == c1) {
			errs++;                              // invalid pair 00/11
			out.push_back((uint8_t)(c1 ^ 1));    // decode to the second chip's complement
		} else {
			out.push_back(c1);                   // 01 -> 1, 10 -> 0
		}
	}
	if(errors) {
		*errors = errs;
	}
	return out;
}

bits frame_build(const uint8_t *pkt, size_t len) {
	bits frame;
	frame.reserve((size_t)ADSL_PREAMBLE_CHIPS + (2 + len) * 16);
	// preamble, raw chips (NOT Manchester-coded): '01' x8 then '1001' x2
	for(int i = 0; i < 8; i++) {
		frame.push_back(0);
		frame.push_back(1);
	}
	static const uint8_t TAIL[4] = { 1, 0, 0, 1 };
	for(int r = 0; r < 2; r++) {
		for(int i = 0; i < 4; i++) {
			frame.push_back(TAIL[i]);
		}
	}
	// Manchester(sync word + packet bytes); pkt[0] is the LEN byte (0x18 for Traffic)
	bits payload = bytes_to_bits_msb(ADSL_SYNCWORD, sizeof(ADSL_SYNCWORD));
	bits pkt_bits = bytes_to_bits_msb(pkt, len);
	payload.insert(payload.end(), pkt_bits.begin(), pkt_bits.end());
	bits coded = manchester_encode(payload);
	frame.insert(frame.end(), coded.begin(), coded.end());
	return frame;
}

} // namespace adsld

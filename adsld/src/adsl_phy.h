// adsld: ADS-L M-band chip-layer PHY (EASA ADS-L 4 SRD-860 Issue 2). Every on-air
// chip-level convention lives here.
//
// Conventions:
//   - 2-GFSK, deviation +-50 kHz (h=1.0), BT=0.5, 100 kchip/s; channels 868.2 and
//     868.4 MHz, alternating every transmission.
//   - Manchester coding 1->01, 0->10 applied to the sync word AND the entire packet,
//     NOT to the preamble.
//   - Preamble: '01' x8 then '1001' x2 = 24 chips. Sync word 0x72 0x4B, MSB-first,
//     Manchester-encoded = 32 chips. Packet 25 bytes = 200 bits = 400 chips.
//     Total frame = 456 chips (~4.56 ms).
//   - Practical 64-chip sync detection pattern = Manchester(0xF5, 0x72, 0x4B, 0x18):
//     the preamble tail 0xF5, the sync word, and the fixed Traffic LEN byte 0x18.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dsp.h"

namespace adsld {

// A bit/chip stream is one bit (0/1) per byte - clearest for Manchester / correlation
// work; ADS-L frames are short (456 chips) so this is not hot.
typedef std::vector<uint8_t> bits;

// ---- radio constants -----------------------------------------------------------------

constexpr double ADSL_CHIP_RATE_HZ = 100e3;   // 100 kchip/s
constexpr double ADSL_DEV_HZ = 50e3;          // 2-GFSK deviation (h = 1.0)
constexpr double ADSL_BT = 0.5;               // Gaussian bandwidth-time product
constexpr int ADSL_NUM_CHANNELS = 2;
constexpr double ADSL_CH_HZ[ADSL_NUM_CHANNELS] = { 868.2e6, 868.4e6 };

// ---- frame geometry (chips) ----------------------------------------------------------

constexpr int ADSL_PREAMBLE_CHIPS = 24;       // '01'x8 + '1001'x2, NOT Manchester-coded
constexpr int ADSL_SYNC_CHIPS = 32;           // Manchester(0x72, 0x4B)
constexpr int ADSL_PACKET_CHIPS = 400;        // Manchester(25 packet bytes)
constexpr int ADSL_FRAME_CHIPS = ADSL_PREAMBLE_CHIPS + ADSL_SYNC_CHIPS + ADSL_PACKET_CHIPS; // 456

constexpr uint8_t ADSL_SYNCWORD[2] = { 0x72, 0x4B };

// 64-chip detection pattern, packed MSB-first: Manchester(0xF5, 0x72, 0x4B, 0x18).
// Receivers correlate the chip stream against these 64 chips (accepting a few chip
// errors); a match aligns the frame at the LEN byte's end.
constexpr uint8_t ADSL_SYNC64[8] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A };
constexpr int ADSL_SYNC64_CHIPS = 64;

// ---- helpers -------------------------------------------------------------------------

// bytes -> bit stream, MSB-first per byte (the on-air bit order)
bits bytes_to_bits_msb(const uint8_t *data, size_t len);
// bit stream (MSB-first per byte, len multiple of 8) -> bytes
std::vector<uint8_t> bits_to_bytes_msb(const bits &b);

// Manchester encode: each input bit becomes two chips, 1 -> 01, 0 -> 10.
bits manchester_encode(const bits &data);
// Hard-decision Manchester decode: chip pairs -> bits (01 -> 1, 10 -> 0). Invalid pairs
// (00/11) decode to the second chip's complement and are counted in *errors if non-null.
// chips.size() must be even. (The receiver decides bits differentially on SOFT chip
// integrals instead - this hard decoder is for the selftest/loopback paths.)
bits manchester_decode(const bits &chips, int *errors = nullptr);

// Build the full transmit chip stream for one packet: preamble chips + Manchester(sync
// word + pkt[0..len)). len is normally 25 (ADSL_PKT_LEN); result has
// ADSL_PREAMBLE_CHIPS + (2 + len) * 16 chips (= ADSL_FRAME_CHIPS for len 25).
bits frame_build(const uint8_t *pkt, size_t len);

} // namespace adsld

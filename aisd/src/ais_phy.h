// aisd: AIS link-layer PHY primitives (ITU-R M.1371 / IEC 62320), isolated so every
// on-air convention that could differ between implementations lives in one place and
// interop fixes stay one-line changes.
//
// Conventions (must match real AIS for on-air interop; proven self-consistent + against
// standard KATs by the selftest, real-signal decode is the pending bench step):
//   - Bit rate 9600 bps, GMSK BT=0.4, modulation index h=0.5.
//   - HDLC framing: 24-bit training preamble, 0x7E start flag, bit-stuffed
//     (data+FCS), 0x7E end flag. NRZI line coding (a data 0 = level transition).
//   - FCS = CRC-16/X.25 (reflected poly 0x8408, init 0xFFFF, xorout 0xFFFF), appended
//     LSB-first. Standard check value CRC("123456789") = 0x906E.
//   - The on-wire data field carries each byte LSB-first; the AIS "logical" message
//     (big-endian fields, 6-bit-ASCII armored in AIVDM) is that stream with the bits of
//     every byte reversed. logical_to_wire() / wire_to_logical() are that (self-inverse)
//     transform.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "dsp.h"

namespace aisd {

// A bit stream is one bit (0/1) per byte -- clearest representation for HDLC bit-level
// stuffing / NRZI / CRC; AIS frames are short (<= ~256 bits) so this is not hot.
typedef std::vector<uint8_t> bits;

constexpr double AIS_BAUD = 9600.0;
constexpr double AIS_BT = 0.4;   // Gaussian filter bandwidth-time product
constexpr double AIS_H = 0.5;    // modulation index
constexpr double AIS_CH_A_HZ = 161.975e6;  // channel 87B / 2087
constexpr double AIS_CH_B_HZ = 162.025e6;  // channel 88B / 2088

// ---- CRC / coding ----------------------------------------------------------------

// CRC-16/X.25 over a bit stream fed in order (each caller supplies bits LSB-first per
// byte where a byte convention applies). Returns the 16-bit FCS.
uint16_t crc16_x25(const bits &b);

bits nrzi_encode(const bits &data, int init_level = 1);   // data 0 -> transition
bits nrzi_decode(const bits &line, int init_level = 1);

bits bit_stuff(const bits &in);                            // insert 0 after five 1s

// reverse the bits within every 8-bit group (len must be a multiple of 8); self-inverse
bits logical_to_wire(const bits &logical);
inline bits wire_to_logical(const bits &wire) { return logical_to_wire(wire); }

// build the full pre-NRZI transmit bit stream: preamble + start flag + bit-stuffed
// (wire_msg + FCS) + end flag. wire_msg is the on-wire data field (see logical_to_wire).
bits build_frame(const bits &wire_msg, int preamble_bits = 24);

// ---- GMSK modulator (selftest / bench only; the daemon is RX) ---------------------

// Modulate NRZI line bits to complex baseband at sample rate fs (Hz). Handles a
// fractional fs/baud via a continuous Gaussian phase pulse. Amplitude 1.0.
std::vector<cf> gmsk_modulate(const bits &line, double fs, double baud = AIS_BAUD);

// ---- HDLC deframer (streaming) ----------------------------------------------------

// Consumes NRZI-decoded data bits one at a time; on a flag-delimited region whose
// destuffed content has a valid FCS, calls on_frame(wire_msg) with the data field
// (FCS stripped). Robust to the discriminator's absolute polarity (NRZI is differential).
class hdlc_rx {
public:
	explicit hdlc_rx(std::function<void(const bits &)> on_frame) : on_frame_(std::move(on_frame)) {}
	void push_bit(int b);
	void reset();

private:
	std::function<void(const bits &)> on_frame_;
	bits buf_;
	long last_flag_end_ = -1;   // index just after the last detected flag, or -1
};

// ---- AIVDM 6-bit ASCII armor ------------------------------------------------------

// logical message bits -> AIVDM payload string + fill bits (pad to a multiple of 6).
std::string aivdm_armor(const bits &logical, int &fill_out);
// AIVDM payload string + fill -> logical message bits.
bits aivdm_dearmor(const std::string &payload, int fill);
// NMEA 0183 checksum (XOR of the sentence body between '!'/'$' and '*').
uint8_t nmea_checksum(const std::string &body);
// Build the !AIVDM sentence(s) for a logical message, fragmenting long messages.
// channel is 'A' or 'B'; seq is the multi-sentence message id (0..9, "" if single).
std::vector<std::string> ais_aivdm(const bits &logical, char channel, int seq = 0);

// ---- logical-field accessors (MSB-first big-endian fields) ------------------------

uint64_t bits_get_uint(const bits &b, size_t start, size_t len);
int64_t bits_get_int(const bits &b, size_t start, size_t len);
std::string bits_get_str(const bits &b, size_t start, size_t nchars);   // AIS 6-bit ASCII
// pack an unsigned value MSB-first into b[start..start+len) (b must be pre-sized)
void bits_put_uint(bits &b, size_t start, size_t len, uint64_t v);

} // namespace aisd

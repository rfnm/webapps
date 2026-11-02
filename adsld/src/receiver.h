// adsld: per-channel ADS-L receiver. Consumes complex baseband at the demod rate
// (400 kHz = 4 samples/chip) and emits CRC-validated Traffic (25-byte) and Status
// (17-byte) packets. Non-coherent 2-GFSK: FM discriminator arg(x * conj(prev)) -> a bank
// of K fixed-phase feed-forward integrate-and-dump CHIP decoders (one per sub-chip timing
// hypothesis, same architecture as aisd: cum-ring fractional strobes); a sliding 64-chip
// correlator against the preamble tail + sync word + LEN byte (ADSL_SYNC64 for Traffic,
// accepting <= SYNC_MAX_CHIP_ERRORS chip errors) finds the frame, then the rest of the
// packet is block-captured. Manchester bit decisions are made DIFFERENTIALLY on the soft
// chip integrals: bit = sign(int_chip0 - int_chip1), which cancels carrier frequency
// offset. The link-layer CRC24 selects the winning phase; adjacent-phase duplicates are
// suppressed. One instance per ADS-L channel.
#pragma once

#include <cstdint>
#include <functional>

#include "adsl_codec.h"
#include "adsl_phy.h"
#include "dsp.h"

namespace adsld {

// metadata delivered with every decoded packet
struct adsl_rx_meta {
	int channel = 0;         // channel index (0 = 868.2 MHz, 1 = 868.4 MHz)
	double freq_hz = 0;      // channel center frequency
	int phase = 0;           // winning timing-hypothesis index (0..PHASES-1)
	float rssi_db = 0;       // burst power at frame completion, dBFS
	bool corrected = false;  // true if the CRC24 single-bit syndrome correction fired
};

class adsl_receiver {
public:
	// cb receives the on-air packet in an ADSL_PKT_LEN buffer (LEN byte first, data still
	// XXTEA-scrambled, CRC verified - and patched if `corrected`). pkt[0] selects the codec:
	// ADSL_LEN_TRAFFIC -> 25 bytes for adsl_decode(), ADSL_LEN_STATUS ->
	// 17 bytes for adsl_status_decode(); bytes past adsl_onair_len(pkt[0]) are zero.
	// Called synchronously from feed().
	using packet_cb = std::function<void(const uint8_t pkt[ADSL_PKT_LEN], const adsl_rx_meta &)>;

	static constexpr int PHASES = 16;              // sub-chip timing hypotheses
	static constexpr int SYNC_MAX_CHIP_ERRORS = 4; // sync64 correlator acceptance

	// channel: index into ADSL_CH_HZ; out_rate: the DDC output rate (400 kHz nominal,
	// must be an integer number of samples per chip).
	adsl_receiver(int channel, double freq_hz, double out_rate, packet_cb cb);
	~adsl_receiver();
	adsl_receiver(const adsl_receiver &) = delete;
	adsl_receiver &operator=(const adsl_receiver &) = delete;

	// feed decimated baseband; may invoke the callback zero or more times
	void feed(const cf *samples, size_t count);

	// LBT observables, safe to read from another thread than the one calling feed():
	// short-window in-band power and a slow-tracking noise-floor estimate, both dBFS.
	float rssi_db() const;
	float noise_floor_db() const;

private:
	struct impl;
	impl *im_;
};

} // namespace adsld

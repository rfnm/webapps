// aisd: per-channel AIS receiver. Consumes complex baseband at the demod rate and emits
// decoded messages. Non-coherent GMSK: FM discriminator -> a bank of K fixed-phase
// feed-forward integrate-and-dump decoders (one per sub-symbol timing hypothesis) whose
// output the link-layer CRC selects between. AIS bursts are short (~256 symbols, 27 ms),
// so TX/RX clock drift over a burst is far under one symbol -- a fixed phase per burst
// suffices and no closed timing loop is needed, which sidesteps the acquisition and
// half-symbol-lock pathologies of a tracking loop. NRZI decode + HDLC deframe (destuff +
// CRC-16/X.25) per hypothesis. One instance per AIS channel.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "ais_phy.h"
#include "dsp.h"

namespace aisd {

struct ais_packet {
	double freq_hz = 0;
	char channel = '?';
	float level_db = 0;    // rough burst level (EMA of |x|) at frame completion
	int type = 0;          // AIS message type (first 6 logical bits)
	uint32_t mmsi = 0;
	bits logical;          // logical message bits (big-endian fields)
};

class channel_receiver {
public:
	channel_receiver(double freq_hz, char channel, double out_rate,
	                 std::function<void(const ais_packet &)> on_packet);

	void feed(const cf *samples, size_t count);

private:
	void on_wire_msg(const bits &wire_msg);

	static constexpr int PHASES = 16;   // sub-symbol timing hypotheses
	static constexpr int RING = 128;    // cum history for fractional interpolation (>= 2*sps)

	double freq_hz_;
	char channel_;
	double sps_;
	std::function<void(const ais_packet &)> on_packet_;

	// FM discriminator + running integral of the discriminator (for integrate-and-dump)
	cf prev_{0, 0};
	bool have_prev_ = false;
	float mag_ema_ = 0;
	long n_ = 0;
	double cum_ = 0;
	double cum_ring_[RING] = {0};

	// one decode hypothesis per fixed timing phase
	struct phase_dec {
		double t_strobe = 0;     // absolute time of this hypothesis's next symbol strobe
		int prev_line = 1;       // NRZI reference
		hdlc_rx hdlc;
		phase_dec(std::function<void(const bits &)> cb) : hdlc(std::move(cb)) {}
	};
	std::vector<phase_dec> phases_;

	// duplicate suppression: adjacent phases decode the same burst, so a message can arrive
	// from several hypotheses within a few symbols
	bits last_msgs_[4];
	long last_n_[4] = {0};
	int last_pos_ = 0;
};

} // namespace aisd

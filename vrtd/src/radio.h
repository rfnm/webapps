// vrtd: radio backend interface. The daemon and the selftest speak only this surface;
// rfnm_radio.cpp (WITH_RFNM) maps it onto librfnm, mock_radio (selftest) onto a
// deterministic sample generator. All rates in Hz, gains in dB, power in dBm.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

namespace vrtd {

struct radio_state {
	double rx_freq_hz = 751e6;
	double tx_freq_hz = 751e6;
	double samp_rate_hz = 2.4e6;
	double rx_gain_db = 0;
	double tx_power_dbm = -10;
	bool rx_on = false;
	bool tx_on = false;
};

class radio {
public:
	virtual ~radio() = default;

	// RX: cb delivers native-endian int16 interleaved I/Q; runs on the backend's thread
	// until stop_rx(). Backends must deliver in whole samples.
	using rx_cb = std::function<void(const int16_t *iq, size_t nsamples)>;
	virtual bool start_rx(const rx_cb &cb) = 0;
	virtual void stop_rx() = 0;

	// TX: native-endian int16 interleaved I/Q; backend buffers to its own granularity.
	// Returns false when the TX path is saturated or down (caller drops and counts).
	virtual bool submit_tx(const int16_t *iq, size_t nsamples) = 0;
	virtual bool start_tx() = 0;
	virtual void stop_tx() = 0;

	// control surface: return false when the hardware rejects the value - the caller
	// answers with a VRT error ack. Values are applied immediately (49.2 execute).
	virtual bool set_rx_freq(double hz) = 0;
	virtual bool set_tx_freq(double hz) = 0;
	virtual bool set_samp_rate(double hz) = 0;
	virtual bool set_rx_gain(double db) = 0;
	virtual bool set_tx_power(double dbm) = 0;

	virtual radio_state state() = 0;
	virtual std::string describe() = 0;
};

radio *make_mock_radio();
#ifdef WITH_RFNM
// antenna: fleet convention name ("sma_a".."sma_h" | "a".."h" | "embed"; anything else =
// the channel's preferred path). tx_policy: 0 = none (TX hard-refused, fail-safe),
// 1 = embed (TX forced onto the embedded antenna), 2 = all. agc: device AGC on channel 0.
radio *make_rfnm_radio(const std::string &antenna, int tx_policy, bool agc);
#endif

} // namespace vrtd

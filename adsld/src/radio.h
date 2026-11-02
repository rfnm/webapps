// adsld: radio backend interface (same shape as vrtd). The daemon, scheduler and selftests
// speak only this surface; radio_rfnm.cpp (WITH_RFNM) maps it onto librfnm, radio_mock.cpp onto
// a deterministic in-memory device for the hardware-free selftests. All rates in Hz,
// gains in dB. Samples are native-endian int16 interleaved I/Q (CS16).
//
// TX gating is 3-layered, fail-safe (common/rfnm_boardconf.h): the board's
// /rfnm/board.conf TX_ALLOWED policy (default absent -> none -> TX hard-refused), the
// daemon's --tx flag, and the per-burst LBT decision in the scheduler. The rfnm backend
// must refuse start_tx()/submit_tx() unless the policy allows the configured antenna.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace adsld {

// mirror of common/rfnm_boardconf.h's vsg_tx_policy (kept as our own enum class so the
// pure-logic code never includes the C header)
enum class tx_allowed {
	none = 0,   // TX hard-refused (fail-safe default)
	embed,      // TX only on the embedded antenna
	all,        // no restriction
};

inline const char *tx_allowed_str(tx_allowed p) {
	return p == tx_allowed::all ? "all" : (p == tx_allowed::embed ? "embed" : "none");
}

class radio {
public:
	virtual ~radio() = default;

	// RX: cb delivers native-endian CS16 interleaved I/Q in whole samples; runs on the
	// backend's thread until stop_rx(). antenna: name ("sma_a".."sma_h" |
	// "a".."h" | "embed"; empty/unknown = the channel's preferred path).
	using rx_cb = std::function<void(const int16_t *iq, size_t nsamples)>;
	virtual bool start_rx(double rate_hz, double freq_hz, int gain_db,
	                      const std::string &antenna, const rx_cb &cb) = 0;
	virtual void stop_rx() = 0;

	// TX: bring the TX chain up at rate_hz/freq_hz (the rfnm backend applies RX and TX
	// configs separately, and the driver may refuse TX while RX streams, see main.cpp --rx). Returns false when TX is refused (policy,
	// hardware, or not built). power_dbm is best-effort.
	virtual bool start_tx(double rate_hz, double freq_hz, double power_dbm,
	                      const std::string &antenna) = 0;
	virtual void stop_tx() = 0;

	// Queue one burst of CS16 interleaved I/Q for transmission as soon as possible.
	// Returns false when the TX path is down or saturated (caller drops and counts).
	virtual bool submit_tx(const int16_t *iq, size_t nsamples) = 0;

	// effective TX policy after all static gates (boardconf x --tx flag)
	virtual tx_allowed tx_policy() const = 0;

	// monotonic milliseconds - the ONE clock the scheduler ticks on (fake in the mock)
	virtual uint64_t now_ms() = 0;

	virtual std::string describe() = 0;
};

// Deterministic mock for selftests: samples in, samples out, fake clock.
class mock_radio : public radio {
public:
	// queue samples to be delivered to the RX callback (synchronously, on the caller's
	// thread, in one cb invocation per inject_rx call)
	virtual void inject_rx(const int16_t *iq, size_t nsamples) = 0;
	// everything submitted via submit_tx() since the last call; returns and clears
	virtual std::vector<int16_t> tx_captured() = 0;
	// fake clock control (now_ms() starts at 0)
	virtual void set_now_ms(uint64_t ms) = 0;
	virtual void advance_ms(uint64_t ms) = 0;
	// the policy tx_policy() reports (default tx_allowed::all so TX paths are testable)
	virtual void set_tx_policy(tx_allowed p) = 0;
};

mock_radio *make_mock_radio();

#ifdef WITH_RFNM
// policy: the already-resolved static TX gate (boardconf x --tx flag; pass
// tx_allowed::none to run RX-only). agc: device AGC on the RX channel.
radio *make_rfnm_radio(tx_allowed policy, bool agc);
#endif

} // namespace adsld

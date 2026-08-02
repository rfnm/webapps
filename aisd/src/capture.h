// aisd: RFNM capture front-end. Compiled only WITH_RFNM; the host selftest build never
// references it. A single wideband capture centred between the two AIS channels feeds
// both per-channel DDCs.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include "dsp.h"

namespace aisd {

// Opens the local-transport device, streams RX channel 0 at rate_hz centered on
// center_hz and calls cb with float IQ until stop is set. gain_db is clamped to the
// channel's advertised gain_range; agc engages the device AGC with that gain as the
// starting point (off = manual gain). antenna selects the RF path by name (empty = the
// channel's preferred path). Increments *samples for the /status counter. Returns false
// on any setup failure (already logged to stderr).
bool run_capture(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna,
                 std::atomic<bool> &stop, std::atomic<uint64_t> &samples,
                 const std::function<void(const cf *, size_t)> &cb);

} // namespace aisd

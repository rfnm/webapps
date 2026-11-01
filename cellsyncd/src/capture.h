// cellsync: RFNM capture front-end. Compiled only WITH_RFNM; the host selftest build
// never references it.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include "fft.h"

namespace cellsync {

// cb(x, n, gap): gap = input samples MISSING immediately before x, derived from the
// stream's per-read nanosecond stamps (0 = contiguous). librfnm reads finish short on
// a chain break and realign truthfully, so a stamp jump is a real air-time hole - a
// consumer that ignores it is gluing the stream (the pre-2026-07-21 acquire poisoner).
typedef std::function<void(const cf *, size_t, uint64_t)> capture_cb;

// One RX session that survives retunes. open() is the expensive part (device open,
// stale-channel sweep, full apply - seconds); retune() is a freq-only apply on the
// running stream plus a sample-counted settle discard (~0.35 s at the capture rate,
// covering LO/FE settling and IQ already in flight at the old frequency). The survey
// and --scan visit many frequencies; per-visit open/close dominated their wall time
// (~5 s/candidate) before this class existed. run() delivers samples until *stop and
// returns false only when the stream dies (setup failure or 30 s of dry reads).
// retune()'s optional `antenna` changes the RF path in the SAME apply (empty = keep the
// current one), so a frequency-dependent antenna never costs a session reopen.
class rx_session {
public:
	rx_session();
	~rx_session();
	bool open(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna);
	bool retune(double center_hz, const std::string &antenna = "");
	bool run(std::atomic<bool> &stop, std::atomic<uint64_t> &samples, const capture_cb &cb);

private:
	struct impl;
	std::unique_ptr<impl> p_;
};

// One-shot wrapper (the live loop's shape): open the device, stream RX channel 0 at
// rate_hz centered on center_hz and call cb until stop is set. gain_db is clamped to
// the channel's advertised gain_range; agc engages the device AGC with that gain as
// the starting point (off = manual gain). antenna selects the RF path by name (empty =
// the channel's preferred path, "embed" = the internal antenna). Increments *samples
// for the /status counter. Returns false on any setup failure (already logged).
bool run_capture(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna,
                 std::atomic<bool> &stop, std::atomic<uint64_t> &samples, const capture_cb &cb);

} // namespace cellsync

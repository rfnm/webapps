// cellsync: the one sample path from a capture callback into the sync engine -
// NCO(-offset) -> L/M resampler -> engine - with honest stream-gap handling.
// Shared by the live tracker, the auto-acquire survey and --scan so every path
// gets the same gap policy; librfnm-free so the selftest can exercise it.
//
// Gap policy (gap = missing input samples reported by the capture layer, which
// derives it from the per-read nanosecond stamps): a SMALL gap is zero-filled at
// its true length, so the engine's absolute sample timeline stays aligned with
// the air - trackers just miss a beat and acquire folds an empty window. A BIG
// gap would mean megabytes of zeros and seconds of poisoned windows; instead the
// engine's acquisition state is flushed (every absolute-position anchor is false
// after an unfilled gap). Before this policy the capture layer glued drops
// silently and the acquire accumulators poisoned themselves permanently - the
// "committed to a band, never locks" failure.
#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <vector>

#include "resampler.h"
#include "tracker.h"

namespace cellsync {

struct feed_chain {
	static constexpr double GAP_ZFILL_S = 0.25;   // fill up to this, flush beyond

	nco mix;
	resampler rs;
	sync_engine &eng;
	double in_rate;
	std::atomic<uint64_t> *discont;               // optional /status counters
	std::atomic<uint64_t> *gap_max_us;
	std::vector<cf> tmp, res, zeros;

	feed_chain(double offset_hz, double rate_hz, int L, int M, sync_engine &e,
	           std::atomic<uint64_t> *dc = nullptr, std::atomic<uint64_t> *gm = nullptr)
		: mix(-offset_hz, rate_hz), rs(L, M), eng(e), in_rate(rate_hz), discont(dc), gap_max_us(gm) {
		zeros.assign(8192, cf(0, 0));
	}

	void run(const cf *x, size_t n) {
		tmp.assign(x, x + n);                     // capacity reused: no steady-state allocs
		mix.mix(tmp.data(), tmp.size());
		res.clear();
		rs.process(tmp.data(), tmp.size(), res);
		eng.feed(res.data(), res.size());
	}

	void operator()(const cf *x, size_t n, uint64_t gap) {
		if(gap) {
			if(discont) {
				discont->fetch_add(1, std::memory_order_relaxed);
			}
			if(gap_max_us) {
				uint64_t us = (uint64_t)(gap * 1e6 / in_rate);
				if(us > gap_max_us->load(std::memory_order_relaxed)) {
					gap_max_us->store(us, std::memory_order_relaxed);
				}
			}
			if(gap <= (uint64_t)(GAP_ZFILL_S * in_rate)) {
				for(uint64_t left = gap; left; ) {
					size_t c = std::min<uint64_t>(left, zeros.size());
					run(zeros.data(), c);
					left -= c;
				}
			} else {
				eng.flush_acquire();
			}
		}
		run(x, n);
	}
};

} // namespace cellsync

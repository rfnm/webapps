// cellscan: the wideband burst recorder. A scan is capture-then-decode: grab a dwell of
// 15.36 MSps into RAM with NO DSP running (the capture discipline - a gain change or a dropped
// sample mid-decode invalidates the offline decode), then hand the buffer to cellfind/decode.
// Two sources behind one interface: a cf32 file replay (drives --file and the selftest, and
// the two reference captures are exactly this) and the board rx_session (WITH_RFNM only), the
// same session class cellsyncd uses so open/retune costs are paid once per job, not per carrier.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "fft.h"

namespace cellscan {

using cellsync::cf;

class burst_source {
public:
	static constexpr double RATE = 15.36e6;   // the 50-PRB numerology

	virtual ~burst_source() {}
	// Open the device/file at a centre frequency. Called once per job.
	virtual bool open(double center_hz, int gain_db, bool agc, const std::string &antenna) = 0;
	// Move to another carrier on the already-open session (cheap; open is the expensive part).
	// `antenna` is the RF path this carrier wants (bandplan.h's policy): an unchanged name
	// costs nothing, a changed one rides the same apply - the session is never reopened.
	virtual bool retune(double center_hz, const std::string &antenna) = 0;
	// Capture `dwell_s` seconds into `out` (cf, DC-centred as the decode expects). Returns
	// false on capture failure. *gap_samples (optional) gets the worst timestamp gap seen
	// (0 = contiguous); the caller retries once on a large gap, then reports capture_error.
	// Aborts and returns false if `cancel` is set.
	virtual bool capture(double dwell_s, std::vector<cf> &out, std::atomic<bool> &cancel,
	                     uint64_t *gap_samples) = 0;
};

// ---- what a capture costs, and whether the machine can pay it -------------------------------
// A capture is ONE contiguous allocation of dwell x 15.36e6 x sizeof(cf) bytes, and cf is 8
// bytes: 245.76 MB for the 2 s dwell, 983.04 MB for the 8 s deep one. On a board with 3.27 GB
// of RAM and no swap that is not a detail - an over-commit is an OOM kill or a wedged kernel,
// not a slowdown, and the board hung twice in an hour once the escalation gate opened wide
// enough that nearly every stage-2 carrier asked for the 983 MB one.
inline size_t capture_samples(double dwell_s) {
	return dwell_s > 0.0 ? (size_t)(dwell_s * burst_source::RATE) : 0;
}
inline uint64_t capture_bytes(double dwell_s) {
	return (uint64_t)capture_samples(dwell_s) * (uint64_t)sizeof(cf);
}

// What ELSE scales with the dwell, checked when this budget was added - a bound on the one
// allocation that leaves an unbounded neighbour is not a bound:
//   - the settle discard (0.05 s) and the gap threshold (2 ms) are constants, not fractions
//   - cellfind's PSS search is bounded by the first two frames, and sweep_probe_level by 384
//     FFT windows, whatever the buffer length
//   - decode_cell's PBCH observation cache is bounded PER WINDOW (MIB_MAX_FRAMES = 200 frames,
//     ~1.9 MB) and each window is decoded and dropped before the next is built
//   - the SIB1 soft buffer is bounded by the transport block, not by the occasion count
// So the burst itself is the only thing that grows with the dwell in MEMORY. In TIME the deep
// pass costs ~4x: remove_cfo derotates every sample (a sin/cos each, 122.9 M of them at 8 s),
// the MIB pass walks 4 windows instead of 1, and the SIB1 loop walks 4x the subframes. That
// ~4x is the second reason the escalation is capped per job (job.h) and not only per byte.

// Empty `out` and give it storage for exactly `n` samples, RELEASING what it held FIRST.
// This is the whole of (d): out.reserve() on a vector that still holds the 2 s capture makes
// the allocator hold both blocks for the length of the copy - a transient ~1.23 GB that no
// budget check ever sees, because it exists between two allocations. Freeing first costs one
// extra munmap and makes the peak of a capture exactly capture_bytes(dwell), one buffer.
// Returns n.
size_t capture_reset(std::vector<cf> &out, size_t n);

// MemAvailable in bytes, 0 when it cannot be read (non-Linux host builds, or a hardened
// /proc). 0 means "unknown", and unknown never blocks a capture - a check that cannot measure
// must not invent a verdict.
uint64_t mem_available_bytes();
// --selftest seam: pin what the next mem_available_bytes() returns (0 = read the real one).
// A simulated low-memory board is the only way to prove the refusal path without a board.
void set_mem_available_override(uint64_t bytes);

// The share of MemAvailable one capture may claim. Half, and not more, because the capture is
// never the only thing live: the decode reads the burst while holding ~1.9 MB of PBCH
// observations per cell per window, the FFT plans, the scan document and the httpd sit beside
// it, and MemAvailable is the kernel's ESTIMATE of what could be freed - reclaiming page cache
// takes time a 983 MB mmap does not wait for. Half of 3.13 GB idle = 1.56 GB, so the deep
// capture still fits on an idle board (the feature is not weakened); it is refused once
// something else already holds ~1.2 GB, which is exactly the case that used to hang the board.
constexpr double CAPTURE_MEM_FRACTION = 0.5;

// Would a capture of `dwell_s` fit the budget? Fills *need (bytes) and *avail (MemAvailable,
// 0 = unknown). True when unknown.
bool capture_fits(double dwell_s, uint64_t *need, uint64_t *avail);

// cf32 file replay. The file IS the burst; retune is a no-op. `conj_iq` conjugates on read
// (the host-TCP convention of the reference captures - ; board-side is non-inverted).
class file_burst_source : public burst_source {
public:
	file_burst_source(std::string path, bool conj_iq) : path_(std::move(path)), conj_(conj_iq) {}
	bool open(double, int, bool, const std::string &) override;
	bool retune(double, const std::string &) override { return true; }
	bool capture(double dwell_s, std::vector<cf> &out, std::atomic<bool> &cancel,
	             uint64_t *gap_samples) override;

private:
	std::string path_;
	bool conj_;
	// whole file, read once. This IS a second buffer beside `out` - a replay peaks at file
	// size + the served dwell - but it is a developer path (--file, the selftest) fed by
	// reference captures of a few hundred MB, never the board's 8 s escalation.
	std::vector<cf> cache_;
	bool loaded_ = false;
};

#ifdef WITH_RFNM
// Board recorder over cellsyncd's rx_session. Streams cf into a pre-sized buffer; the capture
// callback does only memcpy + gap bookkeeping (no DSP), then decode runs offline.
class rfnm_burst_source : public burst_source {
public:
	rfnm_burst_source();
	~rfnm_burst_source() override;
	bool open(double center_hz, int gain_db, bool agc, const std::string &antenna) override;
	bool retune(double center_hz, const std::string &antenna) override;
	bool capture(double dwell_s, std::vector<cf> &out, std::atomic<bool> &cancel,
	             uint64_t *gap_samples) override;

private:
	struct impl;
	impl *p_;
};
#endif

} // namespace cellscan

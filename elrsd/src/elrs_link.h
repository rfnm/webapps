// elrsd: the ELRS link timeline model - slots, nonce, FHSS pointer, lock acquisition.
// (pure logic, shares the FHSS knowledge of elrs_codec). CONSUMED by the
// daemon (RX tracking) and the tx chain (hop planning). No I/O, no threads.
//
// Time model (3.3.1 tx_main, on the freerun captures):
//   * one slot per interval_us; every slot the TX does nonce = (nonce + 1) & 0xFF and
//     sends a packet carrying that nonce
//   * after the packet: if (nonce + 1) % hop == 0 -> fhss_ptr = (fhss_ptr + 1) % 240
//     -> a dwell is hop consecutive slots, the FIRST packet of a dwell has nonce % hop == 0,
//        all packets of a dwell air on seq[fhss_ptr]
//   * hop divides 256, so the dwell phase survives the nonce wrap
//   * SYNC packets (type 2) carry (fhss_index, nonce) of the very packet they are in and
//     are sent only while the current channel is FHSS_SYNC_CHANNEL (41), i.e. at
//     fhss_ptr in {0, 80, 160}
//   * the slot raster drifts 2..100 ppm: never count slots from a fixed t0, integrate
//     slot indices from neighbour distances (round(dt / T)) and re-anchor on every burst
//
// "slot" below is the daemon's own monotonically increasing slot counter (int64), derived
// by the caller from burst timestamps (see slot_clock). Nonce/ptr are mapped onto it once
// a lock exists.
#pragma once

#include <cstdint>
#include <vector>
#include "elrs_codec.h"

namespace elrsd {

// ---- slot clock: burst start times -> slot indices (drift tolerant) ---------------------
class slot_clock {
public:
	explicit slot_clock(double interval_s = 4e-3, double tolerance = 0.25);
	void set_interval(double interval_s);
	double interval_s() const { return t_; }
	// feed the start time (seconds on the source clock) of a detected burst; returns the slot
	// index assigned to it (first burst = slot 0). Slots are integrated cumulatively from
	// the previous burst (round(dt / T)); a burst that lands too far from the raster
	// (|residual| > tolerance * T) still gets a slot but flags `off_raster`.
	int64_t observe(double t_start_s, bool *off_raster = nullptr);
	// predicted start time of a slot (extrapolated from the last observed burst and the
	// running interval estimate); valid once >= 1 burst was observed
	double predict_start_s(int64_t slot) const;
	// current interval estimate (updated with a slow IIR from the observed distances)
	double interval_estimate_s() const { return t_est_; }
	bool valid() const { return n_ > 0; }
	int64_t last_slot() const { return last_slot_; }
	double last_start_s() const { return last_t_; }
	void reset();

private:
	double t_, t_est_, tol_, last_t_ = 0;
	int64_t last_slot_ = 0;
	uint64_t n_ = 0;
};

// ---- link model: nonce + FHSS pointer as a function of the slot ---------------------------
enum class lock_state : uint8_t { NONE = 0, MOD_HOP = 1, FULL = 2 };
// NONE:    nothing known
// MOD_HOP: dwell phase known (nonce % hop and fhss_ptr for every slot) - from a sequence
//          fit on RC packets (lock B); switch_index still ambiguous (needs nonce mod 16)
// FULL:    absolute nonce known - from a SYNC packet (lock A)

struct link_lock {
	lock_state state = lock_state::NONE;
	int64_t slot0 = 0;      // reference slot
	uint8_t nonce0 = 0;     // nonce at slot0 (FULL) / nonce0 % hop meaningful (MOD_HOP)
	int fhss_ptr0 = 0;      // fhss pointer at slot0 (0..239)
	int score = 0;          // dwells matched by the fit (lock B) / 1 (lock A)
	int candidates = 0;     // competing (phase, ptr) hypotheses with the same score; 1 = unique
};

class link_model {
public:
	link_model(const uid_t6 &uid, const rate_params &rp);
	void set_rate(const rate_params &rp);          // resets the lock
	void set_uid(const uid_t6 &uid);               // resets the lock
	const rate_params &rate() const { return rp_; }
	const std::array<uint8_t, FHSS_SEQ_LEN> &sequence() const { return seq_; }

	// --- queries (valid for state >= MOD_HOP) ---
	const link_lock &lock() const { return lock_; }
	int fhss_ptr_at(int64_t slot) const;           // 0..239
	int channel_at(int64_t slot) const;            // 0..79 (seq[fhss_ptr_at])
	double freq_hz_at(int64_t slot) const;         // nominal grid
	uint8_t nonce_at(int64_t slot) const;          // FULL: exact; MOD_HOP: only nonce % hop is meaningful
	int nonce_residue_at(int64_t slot) const;      // nonce % hop
	bool is_dwell_start(int64_t slot) const;       // nonce % hop == 0
	bool is_sync_channel_at(int64_t slot) const;   // channel_at == 41
	// next slot >= from whose channel is 41 (where SYNC packets can appear); -1 without lock
	int64_t next_sync_slot(int64_t from) const;

	// --- lock A: a decoded SYNC packet observed in `slot` ---
	void lock_from_sync(int64_t slot, const sync_packet &sp);

	// --- lock B: sequence fit on (slot, channel) observations (RC packets or bursts) ---
	struct observation { int64_t slot; int channel; std::vector<int> nonce_residues; }; // residues optional (CRC)
	void add_observation(const observation &o);    // keeps the last max_obs (default 512)
	size_t observations() const { return obs_.size(); }
	void clear_observations();
	// score all (phase in 0..hop-1, ptr0 in 0..239) hypotheses; sets lock_ to MOD_HOP if the
	// best score >= min_dwells and is unique; returns true on a (new or confirmed) lock.
	bool fit(int min_dwells = 8);
	// consistency of a new observation with the current lock (for tracking / re-fit trigger)
	bool consistent(const observation &o) const;
	int mismatches() const { return mismatch_; }   // consecutive inconsistent observations
	void drop_lock();

private:
	uid_t6 uid_;
	rate_params rp_;
	std::array<uint8_t, FHSS_SEQ_LEN> seq_{};
	link_lock lock_;
	std::vector<observation> obs_;
	size_t max_obs_ = 512;
	int mismatch_ = 0;
};

// selftest: slot_clock drift cases, lock A from the JSON sync vectors, lock B on a
// synthetic (slot, channel) trace with dropouts and a wrapped nonce, next_sync_slot.
int run_selftest_link();

} // namespace elrsd

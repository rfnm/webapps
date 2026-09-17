// NEEDS: src/elrs_codec.cpp
// elrsd: ELRS link timeline model - slot clock, nonce / FHSS pointer, lock A (SYNC) and
// lock B (sequence fit). Pure logic, no I/O, no threads.
//
// Slot/nonce/pointer model (elrs_link.h): nonce(slot) = nonce0 + (slot - slot0)
// (uint8 wrap), a dwell = hop consecutive nonces starting at nonce % hop == 0, fhss_ptr(slot) =
// ptr0 + (dwell(nonce(slot)) - dwell(nonce0)) mod 240 with dwell(n) = floor(n / hop) on the
// UNWRAPPED nonce (hop divides 256, so the wrap never shifts the dwell phase).
#include "elrs_link.h"

#include <algorithm>
#include <cmath>

namespace elrsd {

namespace {
inline int64_t floor_div(int64_t a, int64_t b) {
	int64_t q = a / b, r = a % b;
	if(r != 0 && ((r < 0) != (b < 0))) q--;
	return q;
}
inline int64_t pos_mod(int64_t a, int64_t b) {
	int64_t r = a % b;
	return r < 0 ? r + b : r;
}
} // namespace

// ---- slot_clock ---------------------------------------------------------------------------

slot_clock::slot_clock(double interval_s, double tolerance) : t_(interval_s), t_est_(interval_s), tol_(tolerance) {}

void slot_clock::set_interval(double interval_s) {
	t_ = interval_s;
	t_est_ = interval_s;
}

int64_t slot_clock::observe(double t_start_s, bool *off_raster) {
	if(off_raster) *off_raster = false;
	if(n_ == 0) {
		n_ = 1;
		last_t_ = t_start_s;
		last_slot_ = 0;
		return 0;
	}
	double dt = t_start_s - last_t_;
	int64_t k = (int64_t)llround(dt / t_est_);
	double resid = dt - (double)k * t_est_;
	bool off = fabs(resid) > tol_ * t_est_;
	if(off_raster) *off_raster = off;
	if(k <= 0) {
		// same slot (or earlier): do not move the anchor; a burst before the last one is off-raster
		if(off_raster && dt < 0) *off_raster = true;
		return last_slot_ + (k < 0 ? k : 0);
	}
	if(!off) {
		// slow IIR on the per-slot distance; bounded to +-2 % of the nominal interval so a burst
		// train on a wrong raster cannot walk the estimate away
		double per = dt / (double)k;
		t_est_ += 0.02 * (per - t_est_);
		t_est_ = std::min(std::max(t_est_, 0.98 * t_), 1.02 * t_);
	}
	n_++;
	last_t_ = t_start_s;
	last_slot_ += k;
	return last_slot_;
}

double slot_clock::predict_start_s(int64_t slot) const { return last_t_ + (double)(slot - last_slot_) * t_est_; }

void slot_clock::reset() {
	t_est_ = t_;
	last_t_ = 0;
	last_slot_ = 0;
	n_ = 0;
}

// ---- link_model ----------------------------------------------------------------------------

link_model::link_model(const uid_t6 &uid, const rate_params &rp) : uid_(uid), rp_(rp) { seq_ = fhss_sequence(uid_); }

void link_model::set_rate(const rate_params &rp) {
	rp_ = rp;
	drop_lock();
	clear_observations();
}

void link_model::set_uid(const uid_t6 &uid) {
	uid_ = uid;
	seq_ = fhss_sequence(uid_);
	drop_lock();
	clear_observations();
}

int link_model::fhss_ptr_at(int64_t slot) const {
	const int64_t hop = rp_.hop_interval > 0 ? rp_.hop_interval : 4;
	int64_t n = (int64_t)lock_.nonce0 + (slot - lock_.slot0);
	int64_t d = floor_div(n, hop) - floor_div((int64_t)lock_.nonce0, hop);
	return (int)pos_mod((int64_t)lock_.fhss_ptr0 + d, FHSS_SEQ_LEN);
}

int link_model::channel_at(int64_t slot) const { return seq_[fhss_ptr_at(slot)]; }

double link_model::freq_hz_at(int64_t slot) const { return fhss_channel_freq_hz(channel_at(slot)); }

uint8_t link_model::nonce_at(int64_t slot) const { return (uint8_t)pos_mod((int64_t)lock_.nonce0 + (slot - lock_.slot0), 256); }

int link_model::nonce_residue_at(int64_t slot) const {
	const int64_t hop = rp_.hop_interval > 0 ? rp_.hop_interval : 4;
	return (int)pos_mod((int64_t)lock_.nonce0 + (slot - lock_.slot0), hop);
}

bool link_model::is_dwell_start(int64_t slot) const { return nonce_residue_at(slot) == 0; }

bool link_model::is_sync_channel_at(int64_t slot) const { return channel_at(slot) == FHSS_SYNC_CHANNEL; }

int64_t link_model::next_sync_slot(int64_t from) const {
	if(lock_.state == lock_state::NONE) return -1;
	const int64_t hop = rp_.hop_interval > 0 ? rp_.hop_interval : 4;
	// the sequence visits channel 41 every 80 dwells; scan at most one full sequence period
	for(int64_t s = from; s < from + (int64_t)FHSS_SEQ_LEN * hop + hop; s++) {
		if(channel_at(s) == FHSS_SYNC_CHANNEL) return s;
	}
	return -1;
}

void link_model::lock_from_sync(int64_t slot, const sync_packet &sp) {
	lock_.state = lock_state::FULL;
	lock_.slot0 = slot;
	lock_.nonce0 = sp.nonce;
	lock_.fhss_ptr0 = (int)(sp.fhss_index % FHSS_SEQ_LEN);
	lock_.score = 1;
	lock_.candidates = 1;
	mismatch_ = 0;
}

void link_model::add_observation(const observation &o) {
	if(lock_.state != lock_state::NONE) {
		if(consistent(o)) mismatch_ = 0;
		else mismatch_++;
	}
	obs_.push_back(o);
	if(obs_.size() > max_obs_) obs_.erase(obs_.begin(), obs_.begin() + (ptrdiff_t)(obs_.size() - max_obs_));
}

void link_model::clear_observations() {
	obs_.clear();
	mismatch_ = 0;
}

bool link_model::fit(int min_dwells) {
	// Hypotheses: phase in 0..hop-1 (slot `phase` is a dwell start, nonce residue 0) x ptr0 in
	// 0..239. A dwell of a hypothesis counts as MATCHED only if every observation inside it
	// agrees (same channel == seq[(ptr0 + dwell) mod 240], and a known CRC residue == the slot
	// residue of that phase); a dwell with disagreeing channels or a wrong residue is a
	// CONTRADICTION for every ptr0 of that phase. Counting "any channel in the dwell matches"
	// would be phase-blind: with a wrong phase every dwell straddles two true dwells and one of
	// the two channels always matches (found by the selftest).
	const int hop = rp_.hop_interval > 0 ? rp_.hop_interval : 4;
	if(obs_.empty()) return false;
	std::vector<std::vector<int>> where(FHSS_NUM_CHANNELS);   // positions of every channel (3 each)
	for(int i = 0; i < FHSS_SEQ_LEN; i++) where[seq_[i]].push_back(i);
	int best_score = -1, best_phase = 0, best_p0 = 0, best_cands = 0, best_ndwell = 0;
	std::vector<int> score((size_t)FHSS_SEQ_LEN);
	struct ent { int64_t dwell; int channel; bool res_ok; };
	std::vector<ent> ents;
	for(int phase = 0; phase < hop; phase++) {
		std::fill(score.begin(), score.end(), 0);
		ents.clear();
		for(const observation &o : obs_) {
			if(o.channel < 0 || o.channel >= FHSS_NUM_CHANNELS) continue;
			bool res_ok = true;
			if(!o.nonce_residues.empty()) {
				int res = (int)pos_mod(o.slot - phase, hop);
				res_ok = std::find(o.nonce_residues.begin(), o.nonce_residues.end(), res) != o.nonce_residues.end();
			}
			ents.push_back({floor_div(o.slot - phase, hop), o.channel, res_ok});
		}
		std::sort(ents.begin(), ents.end(), [](const ent &a, const ent &b) { return a.dwell < b.dwell; });
		int n_dwells = 0;
		for(size_t i = 0; i < ents.size();) {
			size_t j = i;
			int ch = ents[i].channel;
			bool clean = true;
			while(j < ents.size() && ents[j].dwell == ents[i].dwell) {
				if(!ents[j].res_ok || ents[j].channel != ch) clean = false;
				j++;
			}
			n_dwells++;
			if(clean) {
				for(int x : where[ch]) score[(size_t)pos_mod((int64_t)x - ents[i].dwell, FHSS_SEQ_LEN)]++;
			}
			i = j;
		}
		for(int p0 = 0; p0 < FHSS_SEQ_LEN; p0++) {
			if(score[p0] > best_score) {
				best_score = score[p0];
				best_phase = phase;
				best_p0 = p0;
				best_cands = 1;
				best_ndwell = n_dwells;
			} else if(score[p0] == best_score) {
				best_cands++;
			}
		}
	}
	// accept: enough matched dwells, a unique maximum, and at least 2/3 of the observed dwells explained
	if(best_score < min_dwells || best_cands != 1 || 3 * best_score < 2 * best_ndwell) return false;
	if(lock_.state == lock_state::FULL) {
		// keep the absolute nonce if the fit confirms it
		bool same = nonce_residue_at(best_phase) == 0 && fhss_ptr_at(best_phase) == best_p0;
		if(same) {
			lock_.score = best_score;
			lock_.candidates = 1;
			mismatch_ = 0;
			return true;
		}
	}
	lock_.state = lock_state::MOD_HOP;
	lock_.slot0 = best_phase;
	lock_.nonce0 = 0;
	lock_.fhss_ptr0 = best_p0;
	lock_.score = best_score;
	lock_.candidates = 1;
	mismatch_ = 0;
	return true;
}

bool link_model::consistent(const observation &o) const {
	if(lock_.state == lock_state::NONE) return false;
	if(channel_at(o.slot) != o.channel) return false;
	if(!o.nonce_residues.empty()) {
		int res = nonce_residue_at(o.slot);
		if(std::find(o.nonce_residues.begin(), o.nonce_residues.end(), res) == o.nonce_residues.end()) return false;
	}
	return true;
}

void link_model::drop_lock() {
	lock_ = link_lock{};
	mismatch_ = 0;
}

} // namespace elrsd

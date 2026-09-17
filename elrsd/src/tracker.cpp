// elrsd: rx_tracker - rx_burst -> li_codec::decode + ota4_decode (A) -> link_model lock
// A/B (A) -> packet_entry ring + link stats + channel hints back to the chain (
//). Runs on the DSP thread; every query takes the tracker mutex and returns a copy.
// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp
// The ONLY symbol of B referenced here is rx_chain::set_channel_hint; it is declared WEAK
// (asm .weak on the mangled name) so this unit links without B's rx_chain.cpp (the daemon
// selftest drives on_burst with synthetic bursts and chain == nullptr). A non-null chain
// implies someone constructed an rx_chain, i.e. rx_chain.cpp IS linked and the symbol is
// defined - the weak reference is never called while null.
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include "daemon.h"
#include "daemon_internal.h"

asm(".weak _ZN5elrsd8rx_chain16set_channel_hintEddi");

namespace elrsd {

struct rx_tracker::impl {
	packet_ring &ring;
	rx_chain *chain;
	mutable std::mutex mtx;

	uid_t6 uid{};
	bool uid_set = false;
	rate_params rp = RATE_TABLE[2];              // 250 Hz default until set_rate
	bool auto_rate = true;                       // until set_rate() is called
	std::deque<int> rate_votes;
	std::unique_ptr<link_model> lm;
	slot_clock clock;
	std::map<int, std::unique_ptr<li_codec>> codecs;
	int tlm_denom;
	link_stats st;
	int window_lo = -1, window_hi = -1;
	bool wallclock = false;

	std::function<bool(const rx_burst &, std::array<uint8_t, 8> &)> truth;
	std::function<void(const packet_entry &)> sink;

	packet_entry last_rc;
	bool last_rc_valid = false;
	std::array<uint16_t, 16> rc_state{};        // last known CRSF values (aux slots persist)

	std::deque<double> rate_window;              // t_s of crc_ok packets (1 s)
	std::deque<int> lq_window;                   // per expected slot in window: 1 = crc_ok packet seen
	int64_t last_counted_slot = 0;
	bool counting = false;
	int mismatch = 0;
	int64_t mismatch_dwell = INT64_MIN;
	size_t obs_since_fit = 0;
	int last_channel = -1;
	int last_rate_idx = 0;

	impl(packet_ring &r, rx_chain *c) : ring(r), chain(c), clock(4e-3), tlm_denom(RATE_TABLE[2].tlm_denom_default) {
		for(auto &v : rc_state) v = 992;
		rc_state[4] = 172;
	}

	li_codec &codec_for(int idx) {
		auto it = codecs.find(idx);
		if(it != codecs.end()) return *it->second;
		const rate_params *r = rate_by_index(idx);
		if(!r) r = &rp;
		auto c = std::make_unique<li_codec>(*r);
		li_codec &ref = *c;
		codecs[idx] = std::move(c);
		return ref;
	}

	void switch_rate(const rate_params &nr) {
		rp = nr;
		clock.set_interval(rp.interval_us * 1e-6);
		tlm_denom = rp.tlm_denom_default;
		if(lm) lm->set_rate(rp);
		lq_window.clear();
		counting = false;
		mismatch = 0;
		obs_since_fit = 0;
	}

	bool locked() const { return lm && lm->lock().state != lock_state::NONE; }

	// slots_expected / slots_missed / LQ bookkeeping for every slot up to `slot`
	void count_slots(int64_t slot, bool ok_this_slot) {
		if(!locked()) { counting = false; return; }
		if(!counting) { counting = true; last_counted_slot = slot - 1; }
		if(slot - last_counted_slot > 4096) last_counted_slot = slot - 4096;   // runaway guard (clock jump)
		for(int64_t s = last_counted_slot + 1; s <= slot; s++) {
			int ch = lm->channel_at(s);
			bool in_win = window_lo < 0 || (ch >= window_lo && ch <= window_hi);
			if(!in_win) continue;
			st.slots_expected++;
			bool ok = (s == slot) && ok_this_slot;
			if(!ok) st.slots_missed++;
			lq_window.push_back(ok ? 1 : 0);
			while(lq_window.size() > 100) lq_window.pop_front();
		}
		last_counted_slot = slot;
		int sum = 0;
		for(int v : lq_window) sum += v;
		st.lq_pct = lq_window.empty() ? 0 : 100.0 * sum / (double)lq_window.size();
	}

	fhss_view view() const {
		fhss_view v;
		v.slot = clock.valid() ? clock.last_slot() : 0;
		v.interval_est_us = clock.valid() ? clock.interval_estimate_s() * 1e6 : rp.interval_us;
		if(lm) {
			v.lock = lm->lock();
			if(v.lock.state != lock_state::NONE) {
				v.nonce = lm->nonce_at(v.slot);
				v.nonce_conf = lock_state_name(v.lock.state);
				v.fhss_index = lm->fhss_ptr_at(v.slot);
				v.channel = lm->channel_at(v.slot);
				v.freq_hz = lm->freq_hz_at(v.slot);
				v.next_sync_slot = lm->next_sync_slot(v.slot);
			}
		}
		if(v.channel < 0) v.channel = last_channel;
		return v;
	}
};

rx_tracker::rx_tracker(packet_ring &ring, rx_chain *chain) : p_(new impl(ring, chain)) {}

void rx_tracker::set_uid(const uid_t6 &uid) {
	std::lock_guard<std::mutex> lk(p_->mtx);
	p_->uid = uid;
	p_->uid_set = true;
	p_->lm = std::make_unique<link_model>(uid, p_->rp);
	p_->mismatch = 0;
	p_->obs_since_fit = 0;
	p_->counting = false;
}

bool rx_tracker::uid_set() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->uid_set;
}

void rx_tracker::set_rate(const rate_params &rp) {
	std::lock_guard<std::mutex> lk(p_->mtx);
	p_->auto_rate = false;
	p_->rate_votes.clear();
	if(rp.index != p_->rp.index) p_->switch_rate(rp);
	else p_->rp = rp;
}

const rate_params &rx_tracker::rate() const { return p_->rp; }

void rx_tracker::set_truth_provider(std::function<bool(const rx_burst &, std::array<uint8_t, 8> &)> f) {
	std::lock_guard<std::mutex> lk(p_->mtx);
	p_->truth = std::move(f);
}

void rx_tracker_hooks::set_window(rx_tracker &t, int ch_lo, int ch_hi) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	t.p_->window_lo = ch_lo;
	t.p_->window_hi = ch_hi;
	t.p_->st.window_ch_lo = ch_lo;
	t.p_->st.window_ch_hi = ch_hi;
	t.p_->st.channels_in_window = (ch_lo >= 0 && ch_hi >= ch_lo) ? ch_hi - ch_lo + 1 : 0;
}
void rx_tracker_hooks::set_packet_sink(rx_tracker &t, std::function<void(const packet_entry &)> f) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	t.p_->sink = std::move(f);
}
void rx_tracker_hooks::set_tlm_denom(rx_tracker &t, int tlm_denom) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	if(tlm_denom >= 1) t.p_->tlm_denom = tlm_denom;
}
void rx_tracker_hooks::set_auto_rate(rx_tracker &t, bool on) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	t.p_->auto_rate = on;
}
void rx_tracker_hooks::set_wallclock(rx_tracker &t, bool on) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	t.p_->wallclock = on;
}
const packet_entry *rx_tracker_hooks::last_rc_entry(rx_tracker &t, packet_entry &copy_out) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	if(!t.p_->last_rc_valid) return nullptr;
	copy_out = t.p_->last_rc;
	return &copy_out;
}
int rx_tracker_hooks::rate_index_now(const rx_tracker &t) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	return t.p_->rp.index;
}

void rx_tracker::on_burst(const rx_burst &b) {
	impl &I = *p_;
	packet_entry e;
	std::function<void(const packet_entry &)> sink;
	{
		std::lock_guard<std::mutex> lk(I.mtx);
		I.st.bursts++;
		if(b.channel >= 0 && b.channel < FHSS_NUM_CHANNELS) {
			I.st.per_channel[b.channel]++;
			I.st.last_seen_s[b.channel] = b.t_start_s;
			I.last_channel = b.channel;
		}
		// auto rate: majority of the last 8 bursts among the verified rates
		if(I.auto_rate && rate_by_index(b.rate_index) && rate_by_index(b.rate_index)->verified) {
			I.rate_votes.push_back(b.rate_index);
			while(I.rate_votes.size() > 8) I.rate_votes.pop_front();
			std::map<int, int> cnt;
			int best = I.rp.index, bestn = 0;
			for(int v : I.rate_votes) if(++cnt[v] > bestn) { bestn = cnt[v]; best = v; }
			if(best != I.rp.index && bestn >= 3) I.switch_rate(*rate_by_index(best));
		}
		bool off_raster = false;
		int64_t slot = I.clock.observe(b.t_start_s, &off_raster);

		const rate_params *brp = rate_by_index(b.rate_index);
		if(!brp) brp = &I.rp;
		e.t_s = b.t_start_s;
		if(I.wallclock) {
			e.epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch()).count();
		}
		e.slot = slot;
		e.channel = b.channel;
		e.channel_predicted = b.channel_predicted;
		e.freq_hz = b.channel >= 0 ? fhss_channel_freq_hz(b.channel) : 0;
		e.rate_index = brp->index;
		e.sf = brp->sf;
		e.snr_db = b.snr_db;
		e.rssi_db = b.rssi_db;
		e.ref = b.sync.ref;
		e.kind = "undecodable";
		e.nonce_conf = "none";
		e.ch = I.rc_state;
		const float a = 0.05f;
		I.st.snr_avg_db = I.st.bursts == 1 ? b.snr_db : I.st.snr_avg_db + a * (b.snr_db - I.st.snr_avg_db);
		I.st.rssi_avg_db = I.st.bursts == 1 ? b.rssi_db : I.st.rssi_avg_db + a * (b.rssi_db - I.st.rssi_avg_db);

		// ---- decode ----
		ota4_decoded d;
		bool have_payload = false;
		std::vector<uint8_t> payload;
		if(I.uid_set && b.sync.found && b.symbols.size() >= (size_t)brp->n_sym) {
			li_codec &c = I.codec_for(brp->index);
			int fec = 0;
			if(c.decode(b.symbols.data(), b.symbols.size(), b.sync.ref, payload, true, &fec) && payload.size() >= 8) {
				have_payload = true;
				I.st.decoded++;
				e.fec_corrected = fec;
				e.hex = hex_bytes(payload.data(), 8);
				d = ota4_decode(payload.data(), I.uid, brp->hop_interval, I.tlm_denom);
				e.crc_ok = d.crc_ok;
				const char *kinds[4] = { "rc", "msp", "sync", "tlm" };
				e.kind = kinds[(int)d.type & 3];
				if(d.crc_ok) {
					I.st.crc_ok++;
					if(d.type == ota_type::RC) I.st.rc++;
					else if(d.type == ota_type::SYNC) I.st.sync++;
					else I.st.other++;
				} else {
					I.st.crc_fail++;
				}
			}
		}

		// ---- link model ----
		if(I.lm && e.crc_ok) {
			int ch_obs = b.channel >= 0 ? b.channel : b.channel_predicted;
			if(d.type == ota_type::SYNC && d.sync.uid_match(I.uid)) {
				e.sync = d.sync;
				I.lm->lock_from_sync(slot, d.sync);
				I.mismatch = 0;
				I.st.lock_acquired++;
				I.tlm_denom = d.sync.tlm_denom();
				const rate_params *srp = rate_by_index(d.sync.rate_index);
				if(I.auto_rate && srp && srp->verified && srp->index != I.rp.index) {
					I.switch_rate(*srp);
					I.lm->lock_from_sync(slot, d.sync);   // switch_rate reset the lock
				}
			} else if(d.type == ota_type::RC && ch_obs >= 0) {
				link_model::observation o{ slot, ch_obs, d.nonce_residues };
				if(I.lm->lock().state != lock_state::NONE) {
					// Consistency = CRC residue agrees with the predicted dwell phase AND the channel
					// estimate is within one channel of the prediction (a dwell on the window's edge
					// channel is estimated one channel inward for all its packets replay: every
					// lock drop sat on channel 3 of the 3..26 window). Inconsistencies are counted per
					// DISTINCT dwell, not per packet - the packets of one dwell are one event.
					int pred_ch = I.lm->channel_at(slot);
					int pred_res = I.lm->nonce_residue_at(slot);
					bool res_ok = d.nonce_residues.empty() ||
					              std::find(d.nonce_residues.begin(), d.nonce_residues.end(), pred_res) != d.nonce_residues.end();
					int dch = ch_obs - pred_ch;
					bool ch_ok = dch == 0 || ((dch == 1 || dch == -1) && res_ok);
					if(res_ok && ch_ok) {
						I.mismatch = 0;
						if(dch != 0) { o.channel = pred_ch; e.channel_predicted = pred_ch; }
					} else {
						int64_t dwell = slot >= 0 ? slot / I.rp.hop_interval : -((-slot + I.rp.hop_interval - 1) / I.rp.hop_interval);
						if(dwell != I.mismatch_dwell) {
							I.mismatch_dwell = dwell;
							if(++I.mismatch >= 4) {
								I.lm->drop_lock();
								I.lm->clear_observations();
								I.mismatch = 0;
								I.counting = false;
								I.st.lock_drops++;
							}
						}
					}
				}
				I.lm->add_observation(o);
				if(I.lm->lock().state == lock_state::NONE) {
					I.obs_since_fit++;
					if(I.lm->observations() >= 8 && I.obs_since_fit >= 4) {
						if(I.lm->fit(8)) I.st.lock_acquired++;
						I.obs_since_fit = 0;
					}
				}
			}
		}
		bool locked = I.locked();
		if(locked) {
			lock_state ls = I.lm->lock().state;
			e.nonce = ls == lock_state::FULL ? I.lm->nonce_at(slot) : (uint8_t)I.lm->nonce_residue_at(slot);
			e.nonce_conf = lock_state_name(ls);
			if(e.channel_predicted < 0) e.channel_predicted = I.lm->channel_at(slot);
		}

		// ---- RC values ----
		if(e.crc_ok && d.type == ota_type::RC) {
			for(int i = 0; i < 5; i++) I.rc_state[i] = d.rc.ch[i];
			e.ch4_high = d.rc.ch4_high;
			e.aux_raw = d.rc.aux_value_raw;
			if(locked && I.lm->lock().state == lock_state::FULL) {
				int idx = hybrid_wide_switch_index(e.nonce);
				e.switch_index = idx;
				if(idx != 7) {
					e.aux_channel = 5 + idx;
					I.rc_state[5 + idx] = n_to_crsf(d.rc.aux_value_raw, I.tlm_denom >= 8 ? 127 : 63);
				}
			}
			e.ch = I.rc_state;
			I.last_rc = e;
			I.last_rc_valid = true;
			I.rate_window.push_back(b.t_start_s);
		}
		while(!I.rate_window.empty() && I.rate_window.front() < b.t_start_s - 1.0) I.rate_window.pop_front();
		I.st.packet_rate_hz = (double)I.rate_window.size();
		I.count_slots(slot, e.crc_ok);

		// ---- truth ----
		if(I.truth && e.crc_ok && have_payload) {
			std::array<uint8_t, 8> tp{};
			if(I.truth(b, tp)) {
				e.truth_known = true;
				e.truth_match = std::equal(tp.begin(), tp.end(), payload.begin());
			}
		}

		// ---- hint for the next slot ----
		if(locked && I.chain) {
			double T = I.clock.interval_estimate_s();
			double t0 = I.clock.predict_start_s(slot + 1) - 0.3 * T;
			I.chain->set_channel_hint(t0, t0 + T, I.lm->channel_at(slot + 1));
		}
		I.ring.push(e);
		sink = I.sink;
	}
	if(sink) sink(e);
}

link_stats rx_tracker::stats() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->st;
}

link_lock rx_tracker::lock() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->lm ? p_->lm->lock() : link_lock{};
}

int64_t rx_tracker::current_slot() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->clock.valid() ? p_->clock.last_slot() : 0;
}

uint8_t rx_tracker::nonce_now() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->view().nonce;
}

int rx_tracker::channel_now() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return p_->view().channel;
}

std::string rx_tracker::status_json_fragment() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	return fhss_counters_json(p_->st, p_->view());
}

std::vector<uint16_t> rx_tracker::last_rc() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	if(!p_->last_rc_valid) return {};
	return std::vector<uint16_t>(p_->last_rc.ch.begin(), p_->last_rc.ch.end());
}

fhss_view rx_tracker_hooks::view(const rx_tracker &t) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	return t.p_->view();
}
void rx_tracker_hooks::set_chain(rx_tracker &t, rx_chain *chain) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	t.p_->chain = chain;
}
std::vector<int> rx_tracker_hooks::next_channels(const rx_tracker &t, int n) {
	std::lock_guard<std::mutex> lk(t.p_->mtx);
	std::vector<int> out;
	if(!t.p_->locked()) return out;
	int64_t slot = t.p_->clock.valid() ? t.p_->clock.last_slot() : 0;
	for(int i = 1; i <= n; i++) out.push_back(t.p_->lm->channel_at(slot + i));
	return out;
}

} // namespace elrsd

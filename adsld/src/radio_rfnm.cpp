// adsld: librfnm radio backend (WITH_RFNM builds only) - see radio.h. Device behaviour this
// file has to work around is encoded (and re-explained) inline:
//
//   1. FRESH-BOOT THROWAWAY SESSION: the first RX session after device power-up can come
//      up "healthy" (every apply OK, stream started) yet deliver zero samples - the DCS
//      pipeline settles only after a full session cycle. start_rx() therefore probes for
//      samples synchronously and, if none arrive, tears the stream down and retries the
//      whole bring-up; the throwaway session is the fix, not an error.
//   2. RX BEFORE TX: the TX chain refuses to come up on a handle that has
//      never run an RX session. start_tx() on a fresh handle first cycles a short
//      throwaway RX session (start, probe, stop) before touching the TX channel.
//   3. set_tx_channel_rfic_lpf_bw() is REQUIRED before the TX apply - without it the
//      device answers PROBE_FAIL and the whole TX apply is rejected.
//   4. RX and TX applies must be issued SEPARATELY: the current driver rejects a
//      combined RX+TX apply mask.
//   5. tx_work_start(TX_LATENCY_POLICY_RELAXED): ADS-L bursts are scheduled hundreds of
//      milliseconds apart, so we trade TX latency for underrun immunity.
//   6. tx_qbuf granularity is 256-sample multiples; each submitted burst is zero-padded
//      to the boundary so the whole burst flushes instead of leaving a tail staged.
//   7. Gain is clamped to the channel's advertised gain_range: device-side validation
//      rejects an out-of-range gain and fails the whole apply.
//   8. Antenna names resolve via string_to_rf_path against the channel's path_possible
//      list, falling back to path_preferred - a bad --antenna must never crash-loop the
//      daemon.
//
// TX gating (layer 3 of 3, holds even if the upper gates are ever bypassed): the policy
// handed to make_rfnm_radio() is the already-resolved boardconf x --tx gate
// (common/rfnm_boardconf.h vsg_boardconf_parse in main.cpp, default NONE = refuse).
// policy none hard-refuses start_tx()/submit_tx() and counts the refusals; policy embed
// forces the embedded antenna UNCONDITIONALLY - on RX-only boards the preferred TX path
// is an SMA that feeds receive-side hardware.
#include "radio.h"

#ifdef WITH_RFNM

#include <librfnm/constants.h>
#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace adsld {

namespace {

class rfnm_radio : public radio {
public:
	rfnm_radio(tx_allowed policy, bool agc) : policy_(policy), agc_(agc),
	                                          t0_(std::chrono::steady_clock::now()) {}

	~rfnm_radio() override {
		stop_rx();
		stop_tx();
	}

	bool open() {
		try {
			dev_.reset(new rfnm::device(rfnm::TRANSPORT_LOCAL, ""));
		} catch(const std::exception &e) {
			fprintf(stderr, "adsld: device open failed: %s\n", e.what());
			return false;
		}
		rfnm_api_failcode err = dev_->set_stream_format(rfnm::STREAM_FORMAT_CS16);
		if(err) {
			fprintf(stderr, "adsld: set_stream_format: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		return true;
	}

	// ---- RX --------------------------------------------------------------------------

	bool start_rx(double rate_hz, double freq_hz, int gain_db,
	              const std::string &antenna, const rx_cb &cb) override {
		stop_rx();
		if(!ensure_rate(rate_hz)) {
			return false;
		}
		// lesson 1: retry the full bring-up once - the first session on a fresh boot can
		// stream nothing despite every apply succeeding; a teardown/retry cycle fixes it
		for(int attempt = 0; attempt < 2; attempt++) {
			if(!rx_bringup(freq_hz, gain_db, antenna)) {
				return false;
			}
			if(rx_probe(3000)) {
				rx_launch(cb);
				return true;
			}
			fprintf(stderr, "adsld: rx session came up but streamed nothing (fresh-boot "
			                "throwaway session), tearing down and retrying\n");
			rx_teardown();
		}
		fprintf(stderr, "adsld: rx produced no samples after retry, giving up\n");
		return false;
	}

	void stop_rx() override {
		rx_run_ = false;
		if(rx_thr_.joinable()) {
			rx_thr_.join();
		}
		rx_teardown();
	}

	// ---- TX --------------------------------------------------------------------------

	bool start_tx(double rate_hz, double freq_hz, double power_dbm,
	              const std::string &antenna) override {
		// TX SAFETY (layer 3): policy none = hard refusal, counted
		if(policy_ == tx_allowed::none) {
			tx_forbidden_++;
			fprintf(stderr, "adsld: TX refused by board policy (TX_ALLOWED none)\n");
			return false;
		}
		if(!ensure_rate(rate_hz)) {
			return false;
		}
		// lesson 2: one RX session must have cycled on this handle
		// before the first TX apply, or the TX chain refuses to come up. Run a throwaway
		// RX session if the daemon is TX-first (normal operation starts RX earlier).
		if(!rx_cycled_) {
			fprintf(stderr, "adsld: no rx session has run yet - cycling a throwaway rx "
			                "session before first TX\n");
			if(!rx_bringup(freq_hz, 0, "")) {
				return false;
			}
			rx_probe(3000);
			rx_teardown();
		}
		std::lock_guard<std::mutex> lk(mtx_);
		const struct rfnm_api_tx_ch *txc = dev_->get_tx_channel(0);
		dev_->set_tx_channel_freq(0, (int64_t)freq_hz);
		dev_->set_tx_channel_power(0, (int8_t)power_dbm);
		// policy embed locks the embedded antenna unconditionally (see file header)
		enum rfnm_rf_path txpath =
		    policy_ == tx_allowed::embed
		        ? RFNM_PATH_EMBED_ANT
		        : resolve_path(antenna, copy_paths(txc), txc->path_preferred);
		dev_->set_tx_channel_path(0, txpath);
		// Name the path that was actually selected, and every path this channel offers:
		// "antenna=preferred" in the banner would not reveal a transmitter on the wrong SMA.
		{
			std::string avail;
			std::array<enum rfnm_rf_path, 10> pp = copy_paths(txc);
			for(int i = 0; i < 10 && pp[i] != RFNM_PATH_NULL; i++) {
				avail += (i ? " " : "") + rfnm::device::rf_path_to_string(pp[i]);
			}
			fprintf(stderr, "adsld: tx path %s (channel offers: %s)\n",
			        rfnm::device::rf_path_to_string(txpath).c_str(), avail.c_str());
			// And the level, against the range the channel actually accepts. --power is in
			// dBm and is NOT --gain (that is the receiver): asking for a level outside the
			// range is silently clamped device-side, so print both and let the operator see
			// where the request landed.
			fprintf(stderr, "adsld: tx power %d dBm requested, channel range %d..%d dBm\n",
			        (int)(int8_t)power_dbm, (int)txc->power_range.min,
			        (int)txc->power_range.max);
		}
		// lesson 3: REQUIRED, or the TX apply returns PROBE_FAIL
		int16_t lpf_mhz = (int16_t)(rate_hz / 1e6);
		if(lpf_mhz < 1) {
			lpf_mhz = 1;
		}
		dev_->set_tx_channel_rfic_lpf_bw(0, lpf_mhz);
		// STREAM_ON, not STREAM_AUTO. AUTO lets the driver park the channel when it judges
		// the radio idle - and a sparse burst transmitter (one 4.6 ms ADS-L burst a second)
		// looks exactly like that, and every later burst would go into a parked channel. The
		// reference TX-only utility rfnm_tx_sine in librfnm uses STREAM_ON for the same reason.
		dev_->set_tx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);
		// lesson 4: TX apply issued on its own mask, never combined with the RX apply
		rfnm_api_failcode err = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		if(err) {
			fprintf(stderr, "adsld: tx apply: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		// Learn the packet size and allocate OUR OWN buffers before arming the workers -
		// see the TX buffer pool note above. set_stream_format is called again here with its
		// out-parameters; the format itself is unchanged from the RX bring-up.
		tx_pool_free();
		err = dev_->set_stream_format(rfnm::STREAM_FORMAT_CS16, &tx_bufsize_, &tx_bpe_);
		if(err || tx_bufsize_ == 0) {
			fprintf(stderr, "adsld: tx set_stream_format: %s (bufsize %zu)\n",
			        rfnm::device::failcode_to_string(err), tx_bufsize_);
			return false;
		}
		txmem_.resize(NBUF);
		txbufs_.assign(NBUF, rfnm::tx_buf{});   // zeroed: junk tx_flags would flip the
		txfree_.reserve(NBUF);                  // device into timed mode and park the pump
		for(int i = 0; i < NBUF; i++) {
			txmem_[i].assign(tx_bufsize_, 0);
			txbufs_[i].buf = txmem_[i].data();
			txfree_.push_back(&txbufs_[i]);
		}

		// lesson 5: arm the librfnm TX workers (tx_qbuf without tx_work_start queues
		// buffers nothing drains); RELAXED trades latency for underrun immunity
		err = dev_->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED);
		if(err) {
			fprintf(stderr, "adsld: tx_work_start: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		// TIMED TX. Sparse burst transmitters cannot free-run on this device: the drain
		// consumes at line rate, so bursts written ~1/s fall further and further behind and
		// the kernel discards every one of them as late packets. librfnm's
		// answer is tx_buf_schedule(): stamp each burst with an absolute phytimer tick a
		// comfortable distance in the future, and the kernel zero-fills the silence between
		// bursts for us. Read the tick rate once here; clock_now() gives the "now" and is
		// explicitly safe for TX-only clients (device.h: "TX-only clients must NOT open an
		// RX stream just to read the clock").
		struct rfnm::timing t {};
		if(dev_->get_timing(rfnm::DIR_TX, &t) == RFNM_API_OK && t.r_den) {
			tx_r_num_ = t.r_num;
			tx_r_den_ = t.r_den;
			tx_t0_ = t.t0_ext;
			// Ring slots are 256 samples: a burst stamped off that grid is refused as
			// SCHED_MISALIGNED unless the buffer has slack for the pad, and ours is a full
			// packet. Aligning the stamp to the slot grid avoids the whole question.
			tx_slot_ticks_ = (uint64_t)256 * t.r_num / (t.r_den ? t.r_den : 1);
			fprintf(stderr, "adsld: tx timing: %u/%u ticks per sample, tick_hz %llu, "
			                "epoch %u, anchored %d\n",
			        t.r_num, t.r_den, (unsigned long long)t.tick_hz, t.epoch,
			        (int)t.anchored);
		} else {
			fprintf(stderr, "adsld: tx timing unavailable - scheduling with the default "
			                "ticks-per-sample %u/%u\n", tx_r_num_, tx_r_den_);
		}
		tx_on_ = true;
		return true;
	}

	void stop_tx() override {
		std::lock_guard<std::mutex> lk(mtx_);
		if(!tx_on_) {
			return;
		}
		dev_->tx_work_stop();
		dev_->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		dev_->apply(rfnm::tx_channel_apply_flags[0], true, 2000000);
		tx_on_ = false;
		// The pool is ours to free, and only after the workers have stopped: a buffer still
		// in flight would otherwise be drained out of memory we just released.
		txstage_.clear();
		tx_pool_free();
	}

	// ---- TX buffer pool -------------------------------------------------------------
	// librfnm does NOT hand out transmit buffers: tx_dqbuf() pops from a queue of buffers the
	// CALLER previously queued with tx_qbuf() and the device has since drained (an empty queue
	// returns DQBUF_NO_DATA, for ever). So the caller owns the memory. The reference TX-only
	// utility rfnm_tx_sine in librfnm allocates its own pool, and so does this file.
	//
	// NBUF: each buffer holds one USB TX packet. 16 is ~4 x more than one 4.6 ms ADS-L burst
	// needs at 2 MSps, so a burst never blocks on the drain of the one before it.
	static constexpr int NBUF = 16;
	std::vector<rfnm::tx_buf> txbufs_;
	std::vector<std::vector<uint8_t>> txmem_;
	std::vector<rfnm::tx_buf *> txfree_;
	size_t tx_bufsize_ = 0;      // bytes per buffer, from set_stream_format()
	// ticks per sample = r_num/r_den, from get_timing(DIR_TX). The fallback of 1/1 only ever
	// applies if the timing surface is unavailable, and then the lead below still dominates.
	uint32_t tx_r_num_ = 1;
	uint32_t tx_r_den_ = 1;
	uint64_t tx_t0_ = 0;            // this direction's epoch anchor, from get_timing()
	uint64_t tx_slot_ticks_ = 0;    // ticks per 256-sample ring slot; 0 = grid unknown
	// How far ahead of "now" a burst is scheduled. The kernel's floor is ~64 ring slots
	// (524288 ticks, seen in its own reject message); 4x that is ~10 ms of margin at the
	// observed tick rate - enough to absorb a scheduling hiccup on a loaded board, far below
	// the 500 ms GNSS-to-air budget the ADS-L MAC has to hold.
	static constexpr uint64_t TX_LEAD_TICKS = 4 * 524288ULL;
	uint8_t tx_bpe_ = 4;         // bytes per element (CS16 = 4)

	// Move every buffer the device has finished with back into the free list.
	void tx_reap() {
		struct rfnm::tx_buf *b = nullptr;
		while(dev_->tx_dqbuf(&b) == RFNM_API_OK && b) {
			txfree_.push_back(b);
			b = nullptr;
		}
	}

	void tx_pool_free() {
		txfree_.clear();
		txbufs_.clear();
		txmem_.clear();
		tx_bufsize_ = 0;
	}

	// Rate-limited one-liner for the failure paths of submit_tx(). Returning a bare false
	// from a transmit call is how a radio goes quiet without anyone noticing; every branch
	// that drops a burst says so at least once, and then once per 1000 further drops.
	void warn_once(uint64_t &counter, const char *what) {
		if(counter == 0 || counter % 1000 == 0) {
			fprintf(stderr, "adsld: %s (drop #%llu)\n", what,
			        (unsigned long long)(counter + 1));
		}
		counter++;
	}
	uint64_t w_off_ = 0;
	uint64_t w_dq_ = 0;
	uint64_t w_qb_ = 0;
	uint64_t w_clk_ = 0;
	uint64_t w_sch_ = 0;

	bool submit_tx(const int16_t *iq, size_t nsamples) override {
		std::lock_guard<std::mutex> lk(mtx_);
		if(policy_ == tx_allowed::none) {
			tx_forbidden_++;
			return false;
		}
		if(!tx_on_) {
			warn_once(w_off_, "submit_tx: TX path is not up (start_tx never succeeded)");
			return false;
		}
		txstage_.insert(txstage_.end(), iq, iq + 2 * nsamples);
		// lesson 6: pad the burst to the 256-sample tx_buf granularity so it flushes
		// completely - ADS-L bursts are self-contained, a staged tail would air 15-250 ms
		// late glued to the front of the next burst
		while((txstage_.size() / 2) % 256) {
			txstage_.push_back(0);
			txstage_.push_back(0);
		}
		size_t off = 0;
		uint64_t sched_base = 0;      // absolute tick of this submit's first sample
		size_t sched_samples = 0;     // samples already scheduled within this submit
		while(txstage_.size() - off >= 2 * 256) {
			size_t chunk = (txstage_.size() - off) / 2;
			if(chunk > (size_t)RFNM_USB_TX_PACKET_ELEM_CNT) {
				chunk = (size_t)RFNM_USB_TX_PACKET_ELEM_CNT;
			}
			chunk = (chunk / 256) * 256;
			// take a buffer from OUR pool, reaping drained ones first. A transiently
			// saturated queue recovers within a few packet times; bound the wait so a
			// wedged TX path fails the submit instead of hanging the scheduler.
			tx_reap();
			int spins = 0;
			while(txfree_.empty()) {
				if(++spins >= 200) {
					txstage_.clear();
					// Rate-limited so a genuinely wedged path does not flood the journal.
					warn_once(w_dq_, "submit_tx: all TX buffers still in flight after 20 ms "
					                 "- the device is not draining them");
					return false;   // saturated/wedged: caller drops and counts
				}
				std::this_thread::sleep_for(std::chrono::microseconds(100));
				tx_reap();
			}
			struct rfnm::tx_buf *b = txfree_.back();
			txfree_.pop_back();
			if(chunk * tx_bpe_ > tx_bufsize_) {           // cannot happen with the 256-grid
				chunk = tx_bufsize_ / tx_bpe_;            // clamp above, but never overrun
				chunk = (chunk / 256) * 256;
			}
			memcpy(b->buf, &txstage_[off], chunk * tx_bpe_);
			b->elem_cnt = (uint32_t)chunk;
			b->tx_flags = 0;
			// Stamp this chunk. The first chunk of a submit lands TX_LEAD_TICKS ahead of the
			// board clock; the rest follow contiguously at their sample offset, so one ADS-L
			// burst airs as one unbroken waveform.
			if(sched_base == 0) {
				uint64_t now_tick = 0;
				if(dev_->clock_now(&now_tick) != RFNM_API_OK) {
					txfree_.push_back(b);
					txstage_.clear();
					warn_once(w_clk_, "submit_tx: clock_now() failed - cannot time a burst");
					return false;
				}
				sched_base = now_tick + TX_LEAD_TICKS;
				// Round UP onto the ring-slot grid anchored at t0. Rounding up only:
				// tx_buf_schedule is forward-only and a backward stamp is SCHED_ORDER.
				if(tx_slot_ticks_ && sched_base > tx_t0_) {
					uint64_t rel = sched_base - tx_t0_;
					uint64_t slots = (rel + tx_slot_ticks_ - 1) / tx_slot_ticks_;
					sched_base = tx_t0_ + slots * tx_slot_ticks_;
				}
			}
			uint64_t tick = sched_base + (uint64_t)sched_samples * tx_r_num_ / tx_r_den_;
			rfnm_api_failcode serr = dev_->tx_buf_schedule(b, tick);
			if(serr != RFNM_API_OK) {
				txfree_.push_back(b);
				txstage_.clear();
				char m[192];
				snprintf(m, sizeof(m),
				         "submit_tx: tx_buf_schedule refused the burst: %s "
				         "(tick %llu, base %llu, slot %llu ticks, %u samples)",
				         rfnm::device::failcode_to_string(serr),
				         (unsigned long long)tick, (unsigned long long)sched_base,
				         (unsigned long long)tx_slot_ticks_, (unsigned)chunk);
				warn_once(w_sch_, m);
				return false;
			}
			sched_samples += chunk;
			if(dev_->tx_qbuf(b) != RFNM_API_OK) {
				txfree_.push_back(b);   // still ours: the device never took it
				txstage_.clear();
				warn_once(w_qb_, "submit_tx: tx_qbuf refused the buffer");
				return false;
			}
			off += 2 * chunk;
		}
		txstage_.erase(txstage_.begin(), txstage_.begin() + off);
		return true;
	}

	// ---- misc ------------------------------------------------------------------------

	tx_allowed tx_policy() const override { return policy_; }

	uint64_t now_ms() override {
		return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
		           std::chrono::steady_clock::now() - t0_)
		    .count();
	}

	std::string describe() override {
		char b[96];
		snprintf(b, sizeof(b), "rfnm ch0 (tx policy %s, %llu tx refusals)",
		         tx_allowed_str(policy_), (unsigned long long)tx_forbidden_.load());
		return b;
	}

private:
	// stage the device sample rate; a live rate move under a running stream is refused
	// (known mid-reclock hazards - the caller must stop_rx() first, matching vrtd)
	bool ensure_rate(double rate_hz) {
		if(rate_hz == cur_rate_hz_) {
			return true;
		}
		if(rx_run_) {
			fprintf(stderr, "adsld: refusing live sample-rate change (stop_rx first)\n");
			return false;
		}
		rfnm_api_failcode err = dev_->set_samp_rate((uint64_t)rate_hz, 2000000);
		if(err) {
			fprintf(stderr, "adsld: set_samp_rate(%.0f): %s\n", rate_hz,
			        rfnm::device::failcode_to_string(err));
			return false;
		}
		cur_rate_hz_ = rate_hz;
		return true;
	}

	// element-wise copy of a channel's packed path_possible list (taking its address
	// directly trips -Waddress-of-packed-member; per-element reads are safe)
	template <typename CH>
	static std::array<enum rfnm_rf_path, 10> copy_paths(const CH *c) {
		std::array<enum rfnm_rf_path, 10> a;
		for(int i = 0; i < 10; i++) {
			a[i] = c->path_possible[i];
		}
		return a;
	}

	// lesson 8: --antenna name -> RF path, preferred on anything
	// unresolvable or not advertised by the channel (never crash-loop on config)
	enum rfnm_rf_path resolve_path(const std::string &antenna,
	                               const std::array<enum rfnm_rf_path, 10> &possible,
	                               enum rfnm_rf_path preferred) const {
		if(antenna.empty()) {
			return preferred;
		}
		enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(antenna);
		if(wanted == RFNM_PATH_NULL) {
			fprintf(stderr, "adsld: antenna '%s' not understood, using preferred %s\n",
			        antenna.c_str(), rfnm::device::rf_path_to_string(preferred).c_str());
			return preferred;
		}
		for(int i = 0; i < 10; i++) {
			if(possible[i] == wanted) {
				return wanted;
			}
		}
		fprintf(stderr, "adsld: antenna '%s' (%s) not offered by this channel, using preferred %s\n",
		        antenna.c_str(), rfnm::device::rf_path_to_string(wanted).c_str(),
		        rfnm::device::rf_path_to_string(preferred).c_str());
		return preferred;
	}

	// configure channel 0 and start the stream. Touches ONLY channel 0: device-side
	// validation rejects apply masks naming never-configured channels.
	bool rx_bringup(double freq_hz, int gain_db, const std::string &antenna) {
		// recover channels a killed session left enabled - they block stream creation
		rfnm_api_failcode err = dev_->rx_disable_stale_channels(20000000);
		if(err) {
			fprintf(stderr, "adsld: rx_disable_stale_channels: %s\n",
			        rfnm::device::failcode_to_string(err));
			return false;
		}
		const struct rfnm_api_rx_ch *rxc = dev_->get_rx_channel(0);
		// lesson 7: clamp to the advertised range or the whole apply fails
		int gain = gain_db;
		if(gain < rxc->gain_range.min) {
			fprintf(stderr, "adsld: gain %d below advertised min %d, clamping up\n", gain_db,
			        rxc->gain_range.min);
			gain = rxc->gain_range.min;
		}
		if(gain > rxc->gain_range.max) {
			fprintf(stderr, "adsld: gain %d clamped to advertised max %d\n", gain_db,
			        rxc->gain_range.max);
			gain = rxc->gain_range.max;
		}
		dev_->set_rx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_AUTO);
		dev_->set_rx_channel_freq(0, (int64_t)freq_hz);
		dev_->set_rx_channel_gain(0, (int8_t)gain);
		// gain above stays the AGC starting point / the manual value
		dev_->set_rx_channel_agc(0, agc_ ? RFNM_AGC_DEFAULT : RFNM_AGC_OFF);
		dev_->set_rx_channel_path(0, resolve_path(antenna, copy_paths(rxc), rxc->path_preferred));
		// lesson 4: RX apply issued on its own mask
		err = dev_->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
		if(err) {
			fprintf(stderr, "adsld: rx apply: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		stream_.reset(dev_->rx_stream_create(rfnm::channel_flags[0]));
		err = stream_->start();
		if(err) {
			fprintf(stderr, "adsld: rx stream start: %s\n", rfnm::device::failcode_to_string(err));
			stream_.reset();
			return false;
		}
		return true;
	}

	// synchronous probe for first samples (lesson 1): true once the stream delivers
	// anything within timeout_ms. The few probe milliseconds are discarded - this runs
	// before the steady-state thread, at bring-up, when nothing is listening yet.
	bool rx_probe(int timeout_ms) {
		std::vector<int16_t> raw(1 << 14);
		void *buffs[1] = { raw.data() };
		for(int waited = 0; waited < timeout_ms; waited += 100) {
			size_t n = 0;
			uint64_t ts = 0;
			if(stream_->read(buffs, raw.size() / 2, n, ts, 100000) == RFNM_API_OK && n) {
				rx_cycled_ = true;   // a session has demonstrably streamed (lesson 2 satisfied)
				return true;
			}
		}
		return false;
	}

	void rx_launch(const rx_cb &cb) {
		rx_run_ = true;
		rx_thr_ = std::thread([this, cb] {
			std::vector<int16_t> raw(1 << 17);
			void *buffs[1] = { raw.data() };
			int dry = 0;
			while(rx_run_) {
				size_t n = 0;
				uint64_t ts = 0;
				rfnm_api_failcode r = stream_->read(buffs, raw.size() / 2, n, ts, 100000);
				if(r != RFNM_API_OK || !n) {
					// occasional timeouts are normal; a stream returning nothing for
					// ~30 s (unplug/reset/wedge) must exit instead of spinning silently,
					// so systemd's Restart=on-failure can bring the daemon back up
					if(++dry >= 300) {
						fprintf(stderr, "adsld: rx stream dry for 30 s, giving up\n");
						break;
					}
					continue;
				}
				dry = 0;
				cb(raw.data(), n);
			}
		});
	}

	void rx_teardown() {
		if(stream_) {
			stream_->stop();
			stream_.reset();
		}
	}

	std::unique_ptr<rfnm::device> dev_;
	std::unique_ptr<rfnm::rx_stream> stream_;
	std::thread rx_thr_;
	std::atomic<bool> rx_run_{ false };
	bool rx_cycled_ = false;   // a session has streamed samples on this handle (lesson 2)
	bool tx_on_ = false;
	std::vector<int16_t> txstage_;
	std::mutex mtx_;           // serializes TX-side device calls (RX thread only reads the stream)
	double cur_rate_hz_ = 0;
	tx_allowed policy_;
	bool agc_;
	std::atomic<uint64_t> tx_forbidden_{ 0 };
	std::chrono::steady_clock::time_point t0_;
};

} // namespace

radio *make_rfnm_radio(tx_allowed policy, bool agc) {
	rfnm_radio *r = new rfnm_radio(policy, agc);
	if(!r->open()) {
		delete r;
		return nullptr;
	}
	return r;
}

} // namespace adsld

#endif // WITH_RFNM

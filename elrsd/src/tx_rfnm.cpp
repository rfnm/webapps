// elrsd: librfnm TX adapter - CONTINUOUS free-run feed over rfnm::tx_stream on the ONE
// process device (rfnm_device_open() in sample_source.h / capture_rfnm.cpp, owned by the daemon).
// Compiled only WITH_RFNM=1 (the Makefile keeps it out of host builds). Never opens a device.
//
// *** Written against librfnm 0.2.x (include/librfnm/{device,tx_stream,constants}.h); the board's
// *** librfnm.so.0.2.0 (sysroot blue126, 2026-08-05) is the same source as the checkout.
//
// Feed shape (changed 2026-09-14 after the first board run): the sparse positional shape
// (write_at -> tx_feed_seek over the gap + EOB short packet per burst) put 2770 bursts through the
// host side (`sent`) while the RX heard only the LO leak. This adapter now feeds the way every
// measured TX in the bundle feeds (vrtd/lorad/adsld): ONE gapless sample stream in full packets,
// zeros (or the --selfloop-cw tone) explicitly written between the bursts, no seeks, no EOB. The
// bursts are placed sample-exact into that stream by tx_feed (tx_chain.h, host-tested) and a
// feeder thread pushes it through tx_stream::write(); tx_stream's pacer holds the feed frontier
// at now + pace_lead (device contract, >= 2 ms) so the stream neither runs late (the device drops
// POS packets below its 16-slot guard) nor floods the transport.
//
// Feed axis: librfnm stamps every packet POS_VALID as anchor_t0 + feed_pos*R against the anchor it
// latched at start_earliest(); position p airs at t0 + p*R. timing() reads get_timing(DIR_TX)
// every call and reports anchor_moved when t0/epoch differ from the session's - the scheduler
// then calls session_restart() (a session latched on a dead anchor would feed a grid the device
// judges on a different t0: late drops, silence). The self-loop starts the session only after
// the RX apply on the same device for exactly that reason (tx_start_params.start_session=false).
//
// LO strategy unchanged ("hybrid"): chirps sit digitally at f_ch - f_lo inside the
// window, the LO moves only when the next dwell leaves it; a retune whose apply reports a TX
// timeline break (get_apply_timing bit1) is answered with session_restart() by the scheduler.
//
// Bring-up (Wiki where marked): get_tx_channel(0) -> power_range/path_possible;
// freq; power clamped to [range.min, min(range.max, cap)]; path per policy (EMBED only with
// EMBED_ANT in path_possible - never fall back); set_tx_channel_rfic_lpf_bw MANDATORY (else
// PROBE_FAIL); RF_ON + STREAM_ON (AUTO parks after ~3 s); apply(tx_channel_apply_flags[0]) -
// the TX mask alone, never combined with RX. Session: tx_stream(dev, 64, RELAXED);
// start_earliest(&pos0); set_pace_lead_us(contract); feeder thread from pos0.
#ifdef WITH_RFNM

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <librfnm/constants.h>
#include <librfnm/device.h>
#include <librfnm/tx_stream.h>
#include "tx_chain.h"

// Compile-time ceiling that no configuration, CLI flag or API call can raise (bench rule: minimal power).
#ifndef ELRSD_TX_HARD_CAP_DBM
#define ELRSD_TX_HARD_CAP_DBM 0
#endif

namespace elrsd {

namespace {

class radio_tx_rfnm : public radio_tx {
public:
	radio_tx_rfnm(rfnm::device *dev, tx_policy pol) : dev_(dev), pol_(pol) {}
	~radio_tx_rfnm() override { stop(); }

	tx_policy policy() const override { return pol_; }

	bool start(const tx_start_params &p, uint64_t *pos0) override {
		std::lock_guard<std::mutex> l(mtx_);
		// ---- the adapter's own gates (K6): never trust the caller ----
		if(pol_ == tx_policy::NONE) return refuse("policy none: TX refused by the adapter");
		if(!dev_) return refuse("no device");
		if(p.power_dbm > p.power_cap_dbm) return refuse("power above cap: refused by the adapter");
		if(p.power_dbm > ELRSD_TX_HARD_CAP_DBM) return refuse("power above the compiled hard cap: refused by the adapter");
		if(rf_up_) return refuse("already running");
		const struct rfnm_api_tx_ch *tx = dev_->get_tx_channel(0);
		if(!tx || !tx->avail) return refuse("tx channel 0 not available");
		// path resolution against path_possible
		enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(p.antenna);
		bool have_embed = false, have_wanted = false;
		std::string possible;
		for(int i = 0; i < 10; i++) {
			enum rfnm_rf_path pp = tx->path_possible[i];
			if(pp == RFNM_PATH_NULL) break;
			possible += rfnm::device::rf_path_to_string(pp) + " ";
			if(pp == RFNM_PATH_EMBED_ANT) have_embed = true;
			if(pp == wanted) have_wanted = true;
		}
		enum rfnm_rf_path path;
		if(pol_ == tx_policy::EMBED) {
			if(!have_embed) return refuse("policy embed but no embed_ant in path_possible (" + possible + "): refused");
			path = RFNM_PATH_EMBED_ANT;
		} else {
			path = have_wanted ? wanted : tx->path_preferred;
		}
		pmin_ = tx->power_range.min;
		pmax_ = (int)tx->power_range.max;
		// the clamp below never raises the power above the caps: a board whose minimum lies above them is refused
		if(pmin_ > std::min(p.power_cap_dbm, ELRSD_TX_HARD_CAP_DBM)) return refuse("power_range.min " + std::to_string(pmin_) + " dBm above the cap: refused by the adapter");
		int pw = std::max(pmin_, std::min(std::min(std::min(pmax_, p.power_cap_dbm), ELRSD_TX_HARD_CAP_DBM), p.power_dbm));
		fprintf(stderr, "elrsd tx: ch0 freq %lld..%lld power_range %d..%d paths [%s] preferred %s -> path %s power %d dBm (cap %d) lo %.1f MHz\n",
		        (long long)tx->freq_min, (long long)tx->freq_max, pmin_, pmax_, possible.c_str(),
		        rfnm::device::rf_path_to_string(tx->path_preferred).c_str(), rfnm::device::rf_path_to_string(path).c_str(), pw, p.power_cap_dbm, p.freq_lo_hz / 1e6);
		if((int64_t)p.freq_lo_hz < tx->freq_min || (int64_t)p.freq_lo_hz > tx->freq_max) return refuse("LO outside the TX range");
		rfnm_api_failcode r;
		// Set the device sample rate FIRST: this reclocks the DCS and configures the DAC
		// upsampling (tx_interp_log2) for the feed. Without it the samples reach the ring
		// (pos_placed rises) but the DAC never emits them - measured on one board (DCS 200 MHz,
		// R 4/2): elrsd TX radiated nothing while VSG, which reclocks, was strong. (RX does
		// this in capture_rfnm.cpp; the TX path was missing it.)
		if((r = dev_->set_samp_rate((uint64_t)std::llround(p.fs_hz), 2000000)) != RFNM_API_OK) return refuse("set_samp_rate: " + std::to_string((int)r) + " " + dev_->get_apply_error(0, true));
		if((r = dev_->set_tx_channel_freq(0, (int64_t)std::llround(p.freq_lo_hz))) != RFNM_API_OK) return refuse("set_tx_channel_freq: " + std::to_string((int)r));
		if((r = dev_->set_tx_channel_power(0, (int8_t)pw)) != RFNM_API_OK) return refuse("set_tx_channel_power: " + std::to_string((int)r));
		if((r = dev_->set_tx_channel_path(0, path)) != RFNM_API_OK) return refuse("set_tx_channel_path: " + std::to_string((int)r));
		if((r = dev_->set_tx_channel_rfic_lpf_bw(0, (int16_t)std::max(1, p.lpf_bw_mhz))) != RFNM_API_OK) return refuse("set_tx_channel_rfic_lpf_bw: " + std::to_string((int)r));
		if((r = dev_->set_tx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON)) != RFNM_API_OK) return refuse("set_tx_channel_status: " + std::to_string((int)r));
		if((r = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000)) != RFNM_API_OK) {
			std::string e = dev_->get_apply_error(0, true);
			park();
			return refuse("apply(tx): " + std::to_string((int)r) + " " + e);
		}
		tx = dev_->get_tx_channel(0);
		power_applied_ = tx ? tx->power : pw;
		path_applied_ = rfnm::device::rf_path_to_string(tx ? tx->path : path);
		freq_lo_ = p.freq_lo_hz;
		fs_ = p.fs_hz;
		rf_up_ = true;
		last_error_.clear();
		if(pos0) *pos0 = 0;
		if(!p.start_session) {
			fprintf(stderr, "elrsd tx: RF up, feed session deferred (session_start() later)\n");
			return true;
		}
		if(!session_start_locked(pos0)) { park(); rf_up_ = false; return false; }
		return true;
	}

	bool session_start(uint64_t *pos0) override {
		std::lock_guard<std::mutex> l(mtx_);
		if(pol_ == tx_policy::NONE) return refuse("policy none: session refused by the adapter");
		if(!rf_up_) return refuse("session_start: RF not up");
		if(ts_) return refuse("session already running");
		return session_start_locked(pos0);
	}

	bool session_restart(uint64_t *pos0) override {
		std::lock_guard<std::mutex> l(mtx_);
		if(pol_ == tx_policy::NONE) return refuse("policy none: session refused by the adapter");
		if(!rf_up_) return refuse("session_restart: RF not up");
		session_stop_locked();
		rebases_++;
		return session_start_locked(pos0);
	}

	void stop() override {
		std::lock_guard<std::mutex> l(mtx_);
		session_stop_locked();
		if(rf_up_) park();
		rf_up_ = false;
	}

	bool running() const override { std::lock_guard<std::mutex> l(mtx_); return rf_up_; }
	bool session_running() const override { std::lock_guard<std::mutex> l(mtx_); return ts_ != nullptr; }

	bool write_at(uint64_t pos, const int16_t *iq, size_t samples) override {
		std::lock_guard<std::mutex> l(mtx_);
		if(pol_ == tx_policy::NONE) return refuse("policy none: write refused by the adapter");
		if(!ts_) return refuse("no feed session");
		if(feed_broken_.load()) return refuse("feed session broken (write failed on the device) - restart");
		std::string err;
		if(!feed_.queue_burst(pos, iq, samples, &err)) {
			last_error_ = "queue_burst: " + err;
			return false;
		}
		return true;
	}

	bool flush(bool end_of_burst) override {
		std::lock_guard<std::mutex> l(mtx_);
		(void)end_of_burst;   // nothing to flush: the feeder writes the stream continuously
		return ts_ != nullptr;
	}

	uint64_t pos() override { return feed_.frontier(); }

	void set_idle_tone(double offset_hz, int16_t amplitude) override {
		std::lock_guard<std::mutex> l(mtx_);
		tone_offset_hz_ = offset_hz;
		tone_amp_ = amplitude;
		feed_.set_idle_tone(fs_ > 0 ? offset_hz / fs_ : 0.0, amplitude);
		if(amplitude) fprintf(stderr, "elrsd tx: idle fill = tone at LO %+.3f MHz amplitude %d\n", offset_hz / 1e6, (int)amplitude);
	}

	long retune(double freq_lo_hz, bool *timeline_broken) override {
		std::lock_guard<std::mutex> l(mtx_);
		if(timeline_broken) *timeline_broken = false;
		if(pol_ == tx_policy::NONE || !rf_up_) { refuse("retune while not running"); return -1; }
		auto t0 = std::chrono::steady_clock::now();
		rfnm_api_failcode r = dev_->set_tx_channel_freq(0, (int64_t)std::llround(freq_lo_hz));
		if(r == RFNM_API_OK) r = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		long us = (long)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
		if(r != RFNM_API_OK) {
			last_error_ = "retune apply: " + std::to_string((int)r) + " " + dev_->get_apply_error(0, true);
			return -1;
		}
		freq_lo_ = freq_lo_hz;
		uint8_t brk = 0; uint32_t rxe = 0, txe = 0;
		if(dev_->get_apply_timing(&brk, &rxe, &txe) == RFNM_API_OK && (brk & 0x2)) {
			// librfnm reset the feed axis (feed_pos = 0, anchor invalid): the feeder's next write
			// fails loudly; the scheduler restarts the session
			if(timeline_broken) *timeline_broken = true;
		}
		return us;
	}

	bool timing(tx_timing &out) override {
		std::lock_guard<std::mutex> l(mtx_);
		if(!ts_) return false;
		if(!refresh_timing_locked()) return false;
		out = tm_;
		uint64_t now = 0;
		if(dev_->clock_now(&now) == RFNM_API_OK) out.now_tick = now;
		out.anchor_moved = (tm_.t0_ext != session_t0_ || tm_.epoch != session_epoch_);
		if(out.anchor_moved && !anchor_moved_reported_) {
			anchor_moved_reported_ = true;
			anchor_moves_++;
			fprintf(stderr, "elrsd tx: device TX anchor moved (t0 %llu -> %llu, epoch %u -> %u) - session must restart\n",
			        (unsigned long long)session_t0_, (unsigned long long)tm_.t0_ext, session_epoch_, tm_.epoch);
		}
		// the frontier's air time vs now (diagnostic: must stay near the pace lead). pos_now is taken on
		// the feed-sample axis (air-now + axis_skew_) so a re-minted t0_ext cannot inflate the lead to
		// the ~85 s phantom seen on one board; on a healthy device axis_skew_ ~ 0 and this is unchanged.
		if(tm_.r_num && now > tm_.t0_ext) {
			double pos_now = (double)(now - tm_.t0_ext) * (double)tm_.r_den / (double)tm_.r_num + axis_skew_;
			frontier_lead_us_ = (int64_t)(((double)feed_.frontier() - pos_now) * 1e6 / fs_);
			// Measured on the board: an RX apply on the same device can shift the TX position axis by seconds
			// while t0/epoch stay put -> the frontier falls far behind the front, every packet is judged
			// late. Treat a persistently negative lead as a moved anchor so the scheduler restarts the session.
			if(frontier_lead_us_ < -(int64_t)pace_lead_us_) neg_lead_hits_++; else neg_lead_hits_ = 0;
			// one-shot with a 2 s hold-off: on the board the re-anchored session immediately fell behind again
			// (343 restarts in 5 s) -> the device front is unreachable after an RX apply, restarting forever
			// only hides that. Report once, then leave it to the operator / a second board.
			if(neg_lead_hits_ >= 2 && !anchor_moved_reported_ && neg_lead_restarts_ < 1) {
				anchor_moved_reported_ = true; anchor_moves_++; out.anchor_moved = true; neg_lead_restarts_++;
				fprintf(stderr, "elrsd tx: frontier lead %lld us behind the device front - treating as a moved anchor once, session must restart\n", (long long)frontier_lead_us_);
			}
		}
		if(feed_broken_.load()) {
			// the feeder died on the device (write refused after a timeline break, or a loud
			// drop): let the scheduler re-anchor instead of disarming
			last_error_ = "feed session broken: " + feed_error_;
			if(!anchor_moved_reported_) { anchor_moved_reported_ = true; anchor_moves_++; }
			out.anchor_moved = true;
		}
		return true;
	}

	void health(tx_health &out) override {
		std::lock_guard<std::mutex> l(mtx_);
		out = tx_health();
		out.refusals = refusals_;
		out.last_error = last_error_;
		out.rebases = rebases_;
		out.anchor_moves = anchor_moves_;
		out.frontier_lead_us = frontier_lead_us_;
		tx_feed::stats fs = feed_.get_stats();
		out.bursts_queued = fs.queued; out.bursts_fed = fs.fed; out.bursts_late = fs.refused_late;
		out.zero_samples = fs.zero_samples; out.burst_samples = fs.burst_samples;
		if(ts_) {
			rfnm::tx_stream::stats s{};
			ts_->get_stats(&s);
			out.writes = s.writes; out.packets = s.packets; out.short_flushes = s.short_flushes;
			out.seeks = s.seeks; out.drops = s.drops; out.stalls = s.stalls;
			out.rebases += s.rebases;
		}
		rfnm::health h{};
		if(dev_ && dev_->get_health(&h) == RFNM_API_OK) {
			// device counters are cumulative since module load: report the delta since session start
			out.pos_late = h.tx_pos_late - h0_.tx_pos_late;
			out.pos_placed = h.tx_pos_placed - h0_.tx_pos_placed;
			out.pos_misaligned = (h.tx_pos_misaligned - h0_.tx_pos_misaligned) + (h.tx_pos_stale - h0_.tx_pos_stale);
			out.underruns = h.tx_underruns - h0_.tx_underruns;
			out.timed_rejects = h.tx_timed_rejects - h0_.tx_timed_rejects;
			out.headroom_peak_abs = h.tx_headroom_peak_abs;
		}
	}

	int power_applied_dbm() const override { std::lock_guard<std::mutex> l(mtx_); return power_applied_; }
	std::string path_applied() const override { std::lock_guard<std::mutex> l(mtx_); return path_applied_; }
	bool power_range(int *min_dbm, int *max_dbm) const override {
		std::lock_guard<std::mutex> l(mtx_);
		if(min_dbm) *min_dbm = pmin_;
		if(max_dbm) *max_dbm = pmax_;
		return true;
	}
	std::string describe() const override {
		std::lock_guard<std::mutex> l(mtx_);
		return std::string("rfnm tx_stream continuous(policy=") + tx_policy_str(pol_) + ", lo " + std::to_string((long long)freq_lo_) + " Hz, path " + path_applied_ +
		       ", pace " + std::to_string(pace_lead_us_) + " us)";
	}

private:
	bool refuse(const std::string &why) {
		refusals_++;
		last_error_ = why;
		fprintf(stderr, "elrsd tx: refused: %s\n", why.c_str());
		return false;
	}
	// RF_OFF + STREAM_AUTO on the TX mask (mtx_ held). Errors are logged, never thrown.
	void park() {
		if(!dev_) return;
		dev_->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		rfnm_api_failcode r = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		if(r != RFNM_API_OK) fprintf(stderr, "elrsd tx: park apply failed: %d %s\n", (int)r, dev_->get_apply_error(0, true).c_str());
	}
	bool refresh_timing_locked() {
		rfnm::timing t{};
		if(dev_->get_timing(rfnm::DIR_TX, &t) != RFNM_API_OK || !t.anchored || t.r_num == 0) return false;
		rfnm::feed_contract c{};
		if(dev_->get_contract(&c) != RFNM_API_OK) return false;
		tm_.tick_hz = t.tick_hz;
		tm_.r_num = t.r_num;
		tm_.r_den = t.r_den;
		tm_.t0_ext = t.t0_ext;
		tm_.epoch = t.epoch;
		// the scheduler's lead = the feeder's pace lead + 1 ms of feeder jitter (the frontier
		// already sits at now + pace: a burst queued closer than that lands behind it = late)
		uint64_t jitter_ticks = t.tick_hz / 1000;
		tm_.feed_lead_ticks = (uint32_t)(pace_lead_ticks_ + jitter_ticks);
		contract_lead_ticks_ = c.tx_feed_lead_ticks;
		return true;
	}

	// ---- the feed session (mtx_ held) ----
	bool session_start_locked(uint64_t *pos0) {
		if(ts_) return refuse("session already running");
		feed_broken_ = false;
		feed_error_.clear();
		anchor_moved_reported_ = false; neg_lead_hits_ = 0;
		ts_ = new rfnm::tx_stream(*dev_, 64, rfnm::TX_LATENCY_POLICY_RELAXED);
		uint64_t pos = 0;
		rfnm_api_failcode r;
		if((r = ts_->start_earliest(&pos, 0, 3000)) != RFNM_API_OK) {
			delete ts_; ts_ = nullptr;
			return refuse("tx_stream start_earliest: " + std::to_string((int)r));
		}
		// pace lead: the device contract, never below 2 ms (the receipted transit + scheduling allowance)
		rfnm::timing t{};
		rfnm::feed_contract c{};
		if(dev_->get_timing(rfnm::DIR_TX, &t) != RFNM_API_OK || !t.anchored || !t.tick_hz || dev_->get_contract(&c) != RFNM_API_OK) {
			ts_->stop(); delete ts_; ts_ = nullptr;
			return refuse("get_timing(DIR_TX)/get_contract not available after start_earliest");
		}
		uint64_t two_ms = t.tick_hz / 500;
		pace_lead_ticks_ = std::max<uint64_t>(c.tx_feed_lead_ticks, two_ms);
		pace_lead_us_ = (uint32_t)(pace_lead_ticks_ * 1000000ULL / t.tick_hz);
		ts_->set_pace_lead_us(pace_lead_us_);
		if(!refresh_timing_locked()) {
			ts_->stop(); delete ts_; ts_ = nullptr;
			return refuse("get_timing(DIR_TX) not anchored");
		}
		session_t0_ = tm_.t0_ext;
		session_epoch_ = tm_.epoch;
		// Latch the feed-axis skew: the healthy device puts the frontier `pace` ahead of the tick-
		// derived air-now, so axis_skew_ ~ 0. After a set_samp_rate reclock / RX-apply re-mint the
		// returned pos0 no longer matches (now - t0_ext)*R (the observed anchor re-mint, frontier ~85 s "ahead"):
		// keeping the skew here lets frontier_lead_us report the true pace instead of the phantom 85 s.
		{
			uint64_t now0 = 0;
			double air_now0 = 0.0;
			if(dev_->clock_now(&now0) == RFNM_API_OK && now0 > session_t0_ && tm_.r_num)
				air_now0 = (double)(now0 - session_t0_) * (double)tm_.r_den / (double)tm_.r_num;
			double pace_samples = tm_.r_num ? (double)pace_lead_ticks_ * (double)tm_.r_den / (double)tm_.r_num : 0.0;
			axis_skew_ = (double)pos - (air_now0 + pace_samples);
		}
		dev_->get_health(&h0_);
		feed_.reset(pos);
		feed_.set_idle_tone(fs_ > 0 ? tone_offset_hz_ / fs_ : 0.0, tone_amp_);
		feeder_stop_ = false;
		feeder_ = std::thread([this]() { feeder_main(); });
		fprintf(stderr, "elrsd tx: feed session: pos0 %llu, t0 %llu epoch %u, R %u/%u, tick %llu Hz, pace %u us (contract %llu ticks), packet %u samples, idle %s\n",
		        (unsigned long long)pos, (unsigned long long)session_t0_, session_epoch_, tm_.r_num, tm_.r_den, (unsigned long long)tm_.tick_hz,
		        pace_lead_us_, (unsigned long long)contract_lead_ticks_, (unsigned)RFNM_USB_TX_PACKET_ELEM_CNT, tone_amp_ ? "tone" : "zeros");
		if(pos0) *pos0 = pos;
		last_error_.clear();
		return true;
	}
	void session_stop_locked() {
		if(!ts_) return;
		feeder_stop_ = true;
		if(feeder_.joinable()) feeder_.join();
		ts_->stop();
		delete ts_;      // reclaims the pool while the workers drain (<= 500 ms), then tx_work_stop
		ts_ = nullptr;
		feed_.reset(0);
	}
	// the feeder thread: pull the continuous stream from tx_feed at the frontier, push it through
	// tx_stream::write() one full packet at a time; the pacer inside write() sleeps whenever the
	// frontier runs ahead of now + pace_lead. No lock held while writing (write blocks).
	void feeder_main() {
		const size_t pkt = (size_t)RFNM_USB_TX_PACKET_ELEM_CNT;
		std::vector<int16_t> buf(2 * pkt);
		while(!feeder_stop_.load()) {
			size_t n = feed_.next(buf.data(), pkt);
			if(!n) { std::this_thread::sleep_for(std::chrono::microseconds(100)); continue; }
			rfnm_api_failcode r = ts_->write(buf.data(), n);
			if(r != RFNM_API_OK) {
				// TX_NOT_ANCHORED after a timeline break (retune) or a loud drop past the retry budget:
				// stop feeding, report through timing()/health(), the scheduler restarts the session
				feed_error_ = "tx_stream write: " + std::to_string((int)r);
				feed_broken_ = true;
				fprintf(stderr, "elrsd tx: feeder stopped: %s\n", feed_error_.c_str());
				break;
			}
		}
	}

	rfnm::device *dev_;
	tx_policy pol_;
	mutable std::mutex mtx_;
	rfnm::tx_stream *ts_ = nullptr;
	bool rf_up_ = false;
	tx_timing tm_;
	uint64_t session_t0_ = 0;
	uint32_t session_epoch_ = 0;
	bool anchor_moved_reported_ = false;
	uint64_t pace_lead_ticks_ = 0, contract_lead_ticks_ = 0;
	uint32_t pace_lead_us_ = 0;
	double axis_skew_ = 0.0;             // feed frontier pos0 - (air-now + pace) at session start (0 = healthy)
	int64_t frontier_lead_us_ = 0;
	int neg_lead_hits_ = 0;
	int neg_lead_restarts_ = 0;
	rfnm::health h0_{};
	tx_feed feed_{0};
	std::thread feeder_;
	std::atomic<bool> feeder_stop_{false};
	std::atomic<bool> feed_broken_{false};
	std::string feed_error_;
	double fs_ = 25e6, freq_lo_ = 0, tone_offset_hz_ = 0;
	int16_t tone_amp_ = 0;
	int pmin_ = -60, pmax_ = 0, power_applied_ = 0;
	std::string path_applied_, last_error_;
	uint64_t refusals_ = 0, rebases_ = 0, anchor_moves_ = 0;
};

} // namespace

radio_tx *make_rfnm_tx(void *dev_handle, tx_policy policy) {
	if(policy == tx_policy::NONE) {
		fprintf(stderr, "elrsd tx: board TX policy is none - no TX radio object\n");
		return nullptr;
	}
	if(!dev_handle) {
		fprintf(stderr, "elrsd tx: no device handle - no TX radio object\n");
		return nullptr;
	}
	return new radio_tx_rfnm(static_cast<rfnm::device *>(dev_handle), policy);
}

} // namespace elrsd

#endif // WITH_RFNM

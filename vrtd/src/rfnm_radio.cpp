// vrtd: librfnm backend. RX uses the proven touch-ONLY-channel-0 setup sequence
// (the device validates every channel in an apply mask since the 2026-07-05 driver
// work - an all-channel mask fails on the untouched defaults of channels 1-3).
// TX fills librfnm tx_buf packets in 256-sample multiples via tx_dqbuf/tx_qbuf.
#include "radio.h"

#ifdef WITH_RFNM

#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace vrtd {

class rfnm_radio : public radio {
public:
	rfnm_radio(const std::string &antenna, int tx_policy, bool agc) : antenna_(antenna), tx_policy_(tx_policy), agc_(agc) {}

	~rfnm_radio() override {
		stop_rx();
		stop_tx();
	}

	bool open() {
		try {
			dev_.reset(new rfnm::device(rfnm::TRANSPORT_LOCAL, ""));
		} catch(const std::exception &e) {
			fprintf(stderr, "vrtd: device open failed: %s\n", e.what());
			return false;
		}
		rfnm_api_failcode err = dev_->set_stream_format(rfnm::STREAM_FORMAT_CS16);
		if(err) {
			fprintf(stderr, "vrtd: set_stream_format: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		err = dev_->set_samp_rate((uint64_t)st_.samp_rate_hz, 20000000);
		if(err) {
			fprintf(stderr, "vrtd: set_samp_rate: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		return true;
	}

	bool start_rx(const rx_cb &cb) override {
		stop_rx();
		// recover channels a killed session left enabled - they block stream creation
		rfnm_api_failcode err = dev_->rx_disable_stale_channels(20000000);
		if(err) {
			fprintf(stderr, "vrtd: rx_disable_stale_channels: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		const struct rfnm_api_rx_ch *rx = dev_->get_rx_channel(0);
		dev_->set_rx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		dev_->set_rx_channel_freq(0, (uint64_t)st_.rx_freq_hz);
		// clamp to the device's advertised gain range (fleet rule: never crash apply on a
		// config number - the aisd first-on-air lesson)
		int g = (int)st_.rx_gain_db;
		g = g < rx->gain_range.min ? rx->gain_range.min : (g > rx->gain_range.max ? rx->gain_range.max : g);
		dev_->set_rx_channel_gain(0, (int8_t)g);
		dev_->set_rx_channel_agc(0, agc_ ? RFNM_AGC_DEFAULT : RFNM_AGC_OFF);   // gain above = AGC start / manual value
		dev_->set_rx_channel_path(0, resolve_rx_path(rx));
		err = dev_->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
		if(err) {
			fprintf(stderr, "vrtd: rx apply: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		stream_.reset(dev_->rx_stream_create(rfnm::channel_flags[0]));
		err = stream_->start();
		if(err) {
			fprintf(stderr, "vrtd: rx stream start: %s\n", rfnm::device::failcode_to_string(err));
			stream_.reset();
			return false;
		}
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
					if(++dry >= 300) {
						fprintf(stderr, "vrtd: rx stream dry for 30 s, giving up\n");
						break;
					}
					continue;
				}
				dry = 0;
				cb(raw.data(), n);
			}
		});
		st_.rx_on = true;
		return true;
	}

	void stop_rx() override {
		rx_run_ = false;
		if(rx_thr_.joinable()) {
			rx_thr_.join();
		}
		if(stream_) {
			stream_->stop();
			stream_.reset();
		}
		st_.rx_on = false;
	}

	bool start_tx() override {
		// TX SAFETY layer 3 (holds even if the engine gates are ever bypassed): policy none
		// = hard refusal; policy embed = the embedded antenna UNCONDITIONALLY - on RX-only
		// boards the preferred TX path is an SMA that feeds receive-side hardware
		if(tx_policy_ <= 0) {
			fprintf(stderr, "vrtd: TX refused by board policy (TX_ALLOWED none)\n");
			return false;
		}
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->set_tx_channel_freq(0, (int64_t)st_.tx_freq_hz);
		dev_->set_tx_channel_power(0, (int8_t)st_.tx_power_dbm);
		dev_->set_tx_channel_path(0, tx_policy_ == 1 ? RFNM_PATH_EMBED_ANT : dev_->get_tx_channel(0)->path_preferred);
		dev_->set_tx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_AUTO);
		rfnm_api_failcode err = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		if(err) {
			fprintf(stderr, "vrtd: tx apply: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		// arm the librfnm TX worker threads - tx_qbuf without tx_work_start queues
		// buffers that nothing drains (found during the RyFi backend work)
		err = dev_->tx_work_start();
		if(err) {
			fprintf(stderr, "vrtd: tx_work_start: %s\n", rfnm::device::failcode_to_string(err));
			return false;
		}
		st_.tx_on = true;
		return true;
	}

	void stop_tx() override {
		if(!st_.tx_on) {
			return;
		}
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->tx_work_stop();
		dev_->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		st_.tx_on = false;
	}

	bool submit_tx(const int16_t *iq, size_t nsamples) override {
		if(!st_.tx_on && !start_tx()) {
			return false;
		}
		txstage_.insert(txstage_.end(), iq, iq + 2 * nsamples);
		// drain in 256-sample multiples (librfnm tx_buf granularity)
		size_t off = 0;
		while(txstage_.size() - off >= 2 * 256) {
			size_t chunk = ((txstage_.size() - off) / (2 * 256)) * 256;
			if(chunk > (size_t)RFNM_USB_TX_PACKET_ELEM_CNT) {
				chunk = RFNM_USB_TX_PACKET_ELEM_CNT;
			}
			struct rfnm::tx_buf *b = nullptr;
			if(dev_->tx_dqbuf(&b) != RFNM_API_OK || !b) {
				return false;                    // saturated: caller counts a drop
			}
			memcpy(b->buf, &txstage_[off], chunk * 4);
			b->elem_cnt = (uint32_t)chunk;
			b->tx_flags = 0;
			if(dev_->tx_qbuf(b) != RFNM_API_OK) {
				return false;
			}
			off += 2 * chunk;
		}
		txstage_.erase(txstage_.begin(), txstage_.begin() + off);
		return true;
	}

	bool set_rx_freq(double hz) override {
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->set_rx_channel_freq(0, (uint64_t)hz);
		if(dev_->apply(rfnm::rx_channel_apply_flags[0], true, 20000000)) {
			return false;
		}
		st_.rx_freq_hz = hz;
		return true;
	}
	bool set_tx_freq(double hz) override {
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->set_tx_channel_freq(0, (int64_t)hz);
		if(dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000)) {
			return false;
		}
		st_.tx_freq_hz = hz;
		return true;
	}
	bool set_samp_rate(double hz) override {
		std::lock_guard<std::mutex> lk(mtx_);
		// rate changes re-plan the DCS chain; the stream must be re-established by the
		// caller (stop_rx/start_rx) - v1 rejects live rate moves to avoid the known
		// mid-reclock hazards, matching the driver's own conservatism
		(void)hz;
		return false;
	}
	bool set_rx_gain(double db) override {
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->set_rx_channel_gain(0, (int8_t)db);
		if(dev_->apply(rfnm::rx_channel_apply_flags[0], true, 20000000)) {
			return false;
		}
		st_.rx_gain_db = db;
		return true;
	}
	bool set_tx_power(double dbm) override {
		std::lock_guard<std::mutex> lk(mtx_);
		dev_->set_tx_channel_power(0, (int8_t)dbm);
		if(dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000)) {
			return false;
		}
		st_.tx_power_dbm = dbm;
		return true;
	}

	radio_state state() override {
		std::lock_guard<std::mutex> lk(mtx_);
		return st_;
	}
	std::string describe() override { return "rfnm ch0"; }

private:
	std::unique_ptr<rfnm::device> dev_;
	// fleet --antenna convention: name -> RF path, preferred on anything unresolvable
	// (never crash-loop over a config value)
	enum rfnm_rf_path resolve_rx_path(const struct rfnm_api_rx_ch *rx) const {
		std::string a = antenna_;
		if(a.rfind("sma_", 0) == 0) {
			a = a.substr(4);
		}
		if(a == "embed" || a == "embedded") {
			return RFNM_PATH_EMBED_ANT;
		}
		if(a.size() == 1 && a[0] >= 'a' && a[0] <= 'h') {
			return (enum rfnm_rf_path)(RFNM_PATH_SMA_A + (a[0] - 'a'));
		}
		if(!a.empty()) {
			fprintf(stderr, "vrtd: unknown antenna '%s' - using the channel's preferred path\n", antenna_.c_str());
		}
		return rx->path_preferred;
	}

	std::unique_ptr<rfnm::rx_stream> stream_;
	std::thread rx_thr_;
	std::atomic<bool> rx_run_{ false };
	std::vector<int16_t> txstage_;
	std::mutex mtx_;
	radio_state st_;
	std::string antenna_;
	int tx_policy_ = 0;
	bool agc_ = false;
};

radio *make_rfnm_radio(const std::string &antenna, int tx_policy, bool agc) {
	rfnm_radio *r = new rfnm_radio(antenna, tx_policy, agc);
	if(!r->open()) {
		delete r;
		return nullptr;
	}
	return r;
}

} // namespace vrtd

#endif // WITH_RFNM

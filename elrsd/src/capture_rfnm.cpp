// elrsd: librfnm RX source + the process-wide device open/close. Compiled only
// WITH_RFNM=1 (Makefile RFNM_SRCS); the host build gets the stubs in source_buffer.cpp.
// Sequence = lorad capture_rfnm.cpp (in the bundle) + rfnm_tcp_capture's TX park (duplex
// trap) as fixed in The only file besides tx_rfnm.cpp that includes librfnm.
//
// *** Written in this workflow but never executed against a device (RF rules: no deploy). ***
#include "sample_source.h"

#include <librfnm/constants.h>
#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace elrsd {

namespace {

const char *fc_str(rfnm_api_failcode c) { return rfnm::device::failcode_to_string(c); }

// antenna name -> RF path advertised by the channel; unknown/unoffered -> preferred (lorad).
// strict (self-loop): the name is taken literally; unknown -> RFNM_PATH_NULL (caller fails),
// unadvertised -> warning only, the apply decides.
enum rfnm_rf_path resolve_rx_path(const struct rfnm_api_rx_ch *rx, const std::string &antenna, bool strict) {
	std::string avail;
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == RFNM_PATH_NULL) continue;
		avail += " " + rfnm::device::rf_path_to_string(rx->path_possible[i]);
	}
	fprintf(stderr, "elrsd: rx antenna paths available:%s; preferred %s\n", avail.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
	if(antenna.empty()) return rx->path_preferred;
	enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(antenna);
	if(wanted == RFNM_PATH_NULL) {
		if(strict) {
			fprintf(stderr, "elrsd: rx path '%s' not understood (strict): refusing\n", antenna.c_str());
			return RFNM_PATH_NULL;
		}
		fprintf(stderr, "elrsd: antenna '%s' not understood, using preferred\n", antenna.c_str());
		return rx->path_preferred;
	}
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == wanted) return wanted;
	}
	if(strict) {
		fprintf(stderr, "elrsd: rx path '%s' not advertised by path_possible (strict): using it anyway, the apply decides\n", antenna.c_str());
		return wanted;
	}
	fprintf(stderr, "elrsd: antenna '%s' not offered by this channel, using preferred\n", antenna.c_str());
	return rx->path_preferred;
}

class rfnm_rx_source : public sample_source {
public:
	rfnm_rx_source(const rfnm_rx_config &cfg, rfnm::device *dev) : cfg_(cfg), dev_(dev) {}
	double fs_hz() const override { return cfg_.fs_hz; }
	double fc_hz() const override { return cfg_.fc_hz; }
	std::string describe() const override {
		char b[160];
		snprintf(b, sizeof b, "rfnm rx%d fs=%.0f fc=%.0f gain=%d agc=%d antenna=%s", cfg_.rx_channel, cfg_.fs_hz, cfg_.fc_hz, cfg_.gain_db, (int)cfg_.agc, cfg_.antenna.c_str());
		return b;
	}
	void stats(source_stats &out) const override {
		out.samples = samples_.load();
		out.drops = drops_.load();
		out.reads = reads_.load();
		out.empty_reads = empty_.load();
		out.eof = false;
	}

	bool run(std::atomic<bool> &stop, const std::function<void(const source_block &)> &cb) override {
		if(!dev_) {
			fprintf(stderr, "elrsd: rfnm rx: no device handle\n");
			return false;
		}
		rfnm_api_failcode err;
		if(cfg_.park_tx) {
			// TX ch0 parked with an in-range frequency, RF and stream OFF (a freshly booted TX
			// chain at 0 MHz gets the whole apply rejected; an enabled TX chain from a dead
			// session puts the device into duplex) - own apply mask, never combined with RX
			const struct rfnm_api_tx_ch *tx = dev_->get_tx_channel(0);
			int64_t f = (int64_t)cfg_.fc_hz;
			if(tx && (f < tx->freq_min || f > tx->freq_max)) f = tx->freq_min;
			dev_->set_tx_channel_freq(0, f);
			dev_->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_OFF);
			err = dev_->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
			if(err) fprintf(stderr, "elrsd: rfnm rx: TX park apply: %s (continuing)\n", fc_str(err));
		}
		const uint32_t ch = (uint32_t)std::max(0, cfg_.rx_channel);
		if(ch >= rfnm::MAX_RX_CHANNELS) {
			fprintf(stderr, "elrsd: rfnm rx: channel %u out of range\n", ch);
			return false;
		}
		const struct rfnm_api_rx_ch *rx = dev_->get_rx_channel(ch);
		if(!rx || !rx->avail) {
			fprintf(stderr, "elrsd: rfnm rx: get_rx_channel(%u) failed / not available\n", ch);
			return false;
		}
		int gain = cfg_.gain_db;
		if(gain < rx->gain_range.min) {
			fprintf(stderr, "elrsd: gain %d below min %d, clamping\n", gain, rx->gain_range.min);
			gain = rx->gain_range.min;
		}
		if(gain > rx->gain_range.max) {
			fprintf(stderr, "elrsd: gain %d above max %d, clamping\n", gain, rx->gain_range.max);
			gain = rx->gain_range.max;
		}
		enum rfnm_rf_path path = resolve_rx_path(rx, cfg_.antenna, cfg_.strict_path);
		if(path == RFNM_PATH_NULL) return false;
		dev_->set_rx_channel_status(ch, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		dev_->set_rx_channel_freq(ch, (int64_t)cfg_.fc_hz);
		dev_->set_rx_channel_gain(ch, (int8_t)gain);
		dev_->set_rx_channel_agc(ch, cfg_.agc ? RFNM_AGC_DEFAULT : RFNM_AGC_OFF);
		dev_->set_rx_channel_path(ch, path);
		dev_->set_rx_channel_rfic_lpf_bw(ch, rfnm::device::suggested_lpf_bw((uint64_t)cfg_.fs_hz, dev_->get_hwinfo() ? &dev_->get_hwinfo()->clock : nullptr));
		err = dev_->apply(rfnm::rx_channel_apply_flags[ch], true, 20000000);   // RX mask only
		if(err) {
			fprintf(stderr, "elrsd: rfnm rx%u: apply: %s %s\n", ch, fc_str(err), dev_->get_apply_error(ch, false).c_str());
			return false;
		}
		fprintf(stderr, "elrsd: rfnm rx%u: path %s gain %d fc %.1f MHz fs %.1f MSps\n", ch,
		        rfnm::device::rf_path_to_string(dev_->get_rx_channel(ch)->path).c_str(), gain, cfg_.fc_hz / 1e6, cfg_.fs_hz / 1e6);

		std::unique_ptr<rfnm::rx_stream> stream(dev_->rx_stream_create(rfnm::channel_flags[ch]));
		if(!stream) {
			fprintf(stderr, "elrsd: rfnm rx: rx_stream_create failed\n");
			park_rx();
			return false;
		}
		err = stream->start();
		if(err) {
			fprintf(stderr, "elrsd: rfnm rx: stream start: %s\n", fc_str(err));
			park_rx();
			return false;
		}
		std::vector<int16_t> raw((size_t)65536 * 2);
		void *buffs[1] = {raw.data()};
		int dry = 0;
		uint64_t index = 0;
		uint64_t expect_ts = 0;
		bool have_ts = false;
		const double ns_per_sample = 1e9 / cfg_.fs_hz;
		bool ok = true;
		while(!stop) {
			size_t n = 0;
			uint64_t ts = 0;
			rfnm_api_failcode r = stream->read(buffs, 65536, n, ts, 100000);
			reads_++;
			if(r != RFNM_API_OK || !n) {
				empty_++;
				// timeouts are normal; a stream that stays silent ~30 s (unplug/reset/wedge) ends
				// run() with false so systemd Restart=on-failure brings the daemon back
				if(++dry >= cfg_.empty_read_limit) {
					fprintf(stderr, "elrsd: rfnm rx: no samples for %d reads (last: %s), giving up\n", dry, fc_str(r));
					ok = false;
					break;
				}
				continue;
			}
			dry = 0;
			// discontinuity = device timestamp jumps by more than half a block vs the sample
			// count (stream health from the counters, never from log lines)
			if(have_ts) {
				double gap_ns = (double)ts - (double)expect_ts;
				if(gap_ns > 0.5 * n * ns_per_sample || gap_ns < -0.5 * n * ns_per_sample) {
					drops_++;
					uint64_t skipped = gap_ns > 0 ? (uint64_t)(gap_ns / ns_per_sample) : 0;
					index += skipped;   // keep the sample clock in step with the device time
				}
			}
			have_ts = true;
			expect_ts = ts + (uint64_t)(n * ns_per_sample);
			source_block b;
			b.iq = raw.data();
			b.count = n;
			b.sample_index = index;
			b.ts_ns = (int64_t)ts;
			cb(b);
			index += n;
			samples_ += n;
		}
		stream->stop();
		park_rx();
		return ok;
	}

private:
	// RX RF_OFF + STREAM_AUTO on the shared device so the TX adapter can take the radio
	// (v1 TX mode = RX stopped); run() may be called again afterwards
	void park_rx() {
		const uint32_t ch = (uint32_t)std::max(0, cfg_.rx_channel);
		dev_->set_rx_channel_status(ch, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
		rfnm_api_failcode err = dev_->apply(rfnm::rx_channel_apply_flags[ch], true, 20000000);
		if(err) fprintf(stderr, "elrsd: rfnm rx: RF_OFF apply: %s\n", fc_str(err));
	}

	rfnm_rx_config cfg_;
	rfnm::device *dev_;
	std::atomic<uint64_t> samples_{0}, drops_{0}, reads_{0}, empty_{0};
};

} // namespace

// the one rfnm::device per process: TRANSPORT_LOCAL, CS16, rate at rest, stale
// RX channels recovered, TX ch0 parked - shared by the RX source and the TX adapter (D)
void *rfnm_device_open(double fs_hz, std::string &error) {
	rfnm::device *dev = nullptr;
	try {
		dev = new rfnm::device(rfnm::TRANSPORT_LOCAL, "");
	} catch(const std::exception &e) {
		error = std::string("device open failed: ") + e.what();
		return nullptr;
	}
	rfnm_api_failcode err = dev->set_stream_format(rfnm::STREAM_FORMAT_CS16);
	if(err) {
		error = std::string("set_stream_format: ") + fc_str(err);
		delete dev;
		return nullptr;
	}
	err = dev->set_samp_rate((uint64_t)fs_hz, 2000000);   // 2nd arg = timeout_us; rate only at rest
	if(err) {
		error = std::string("set_samp_rate: ") + fc_str(err);
		delete dev;
		return nullptr;
	}
	// channels a killed session left enabled block stream creation; the driver's sweep applies
	// only those (an all-channel mask names never-configured channels and fails the apply)
	err = dev->rx_disable_stale_channels(20000000);
	if(err) {
		error = std::string("rx_disable_stale_channels: ") + fc_str(err);
		delete dev;
		return nullptr;
	}
	// park TX ch0: in-range freq, RF_OFF, STREAM_OFF, own mask (rfnm_tcp_capture)
	const struct rfnm_api_tx_ch *tx = dev->get_tx_channel(0);
	if(tx) {
		int64_t f = 2441400000LL;
		if(f < tx->freq_min || f > tx->freq_max) f = tx->freq_min;
		dev->set_tx_channel_freq(0, f);
		dev->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_OFF);
		err = dev->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
		if(err) fprintf(stderr, "elrsd: rfnm_device_open: TX park apply: %s (continuing)\n", fc_str(err));
	}
	return dev;
}

void rfnm_device_close(void *handle) {
	rfnm::device *dev = static_cast<rfnm::device *>(handle);
	if(!dev) return;
	dev->set_rx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
	dev->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
	dev->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_OFF);
	dev->apply(rfnm::tx_channel_apply_flags[0], true, 20000000);
	delete dev;
}

sample_source *make_rfnm_rx_source(const rfnm_rx_config &cfg, void *dev) {
	if(!dev) {
		fprintf(stderr, "elrsd: make_rfnm_rx_source: no device handle\n");
		return nullptr;
	}
	return new rfnm_rx_source(cfg, static_cast<rfnm::device *>(dev));
}

} // namespace elrsd

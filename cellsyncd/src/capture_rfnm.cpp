#include "capture.h"

#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace cellsync {

// Resolve the requested antenna name to an RF path on the rx channel and log the paths
// the channel advertises so operators see the options. An empty name keeps the channel's
// preferred path; a name librfnm can't parse, or one the channel doesn't advertise, is
// warned about and falls back to preferred so a bad --antenna never crash-loops the daemon.
static enum rfnm_rf_path resolve_rx_path(const struct rfnm_api_rx_ch *rx, const std::string &antenna) {
	std::string avail;
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == RFNM_PATH_NULL) {
			continue;
		}
		avail += " " + rfnm::device::rf_path_to_string(rx->path_possible[i]);
	}
	fprintf(stderr, "cellsync: antenna paths available:%s; preferred %s\n",
	        avail.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());

	if(antenna.empty()) {
		return rx->path_preferred;
	}
	enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(antenna);
	if(wanted == RFNM_PATH_NULL) {
		fprintf(stderr, "cellsync: antenna '%s' not understood, using preferred %s\n",
		        antenna.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
		return rx->path_preferred;
	}
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == wanted) {
			return wanted;
		}
	}
	fprintf(stderr, "cellsync: antenna '%s' (%s) not offered by this channel, using preferred %s\n",
	        antenna.c_str(), rfnm::device::rf_path_to_string(wanted).c_str(),
	        rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
	return rx->path_preferred;
}

struct rx_session::impl {
	std::unique_ptr<rfnm::device> dev;
	std::unique_ptr<rfnm::rx_stream> stream;
	double rate_hz = 0;
	std::string antenna;             // RF path name as last requested (open/retune)
	uint64_t drop = 0;               // settle discard armed by retune(), in samples
	uint64_t exp_ts = 0;             // stamp the next read should carry if contiguous
	bool have_ts = false;
	int dry_reads = 0;
	std::vector<int16_t> raw;
	std::vector<cf> iq;
};

rx_session::rx_session() : p_(new impl) {
}

rx_session::~rx_session() {
	if(p_->stream) {
		p_->stream->stop();
	}
}

// same setup sequence as the verified readsb driver / rate_probe: stage the rate,
// force every channel off, configure channel 0, one apply, then stream
bool rx_session::open(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna) {
	try {
		p_->dev.reset(new rfnm::device(rfnm::TRANSPORT_LOCAL, ""));
	} catch(const std::exception &e) {
		fprintf(stderr, "cellsync: device open failed: %s\n", e.what());
		return false;
	}

	rfnm_api_failcode err = p_->dev->set_stream_format(rfnm::STREAM_FORMAT_CS16);
	if(err) {
		fprintf(stderr, "cellsync: set_stream_format: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	err = p_->dev->set_samp_rate((uint64_t)rate_hz, 20000000);   // 2nd arg is timeout_us, not a rate
	if(err) {
		fprintf(stderr, "cellsync: set_samp_rate(%.0f): %s\n", rate_hz, rfnm::device::failcode_to_string(err));
		return false;
	}

	// recover channels a killed session left enabled (they block stream creation);
	// the driver sweep names only those channels, so the apply validates
	err = p_->dev->rx_disable_stale_channels(20000000);
	if(err) {
		fprintf(stderr, "cellsync: rx_disable_stale_channels: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	// touch ONLY channel 0 and apply ONLY channel 0 (rate_probe's proven sequence):
	// the device now validates every channel in the apply mask, and the untouched
	// defaults of channels 1-3 can fail it
	const struct rfnm_api_rx_ch *rx = p_->dev->get_rx_channel(0);

	// clamp the requested gain to the advertised range: 0.3.x device-side validation
	// rejects an out-of-range gain and fails the whole apply
	int gain = gain_db;
	if(gain < rx->gain_range.min) {
		fprintf(stderr, "cellsync: gain %d below advertised min %d, clamping up\n", gain_db, rx->gain_range.min);
		gain = rx->gain_range.min;
	}
	if(gain > rx->gain_range.max) {
		fprintf(stderr, "cellsync: gain %d clamped to advertised max %d\n", gain_db, rx->gain_range.max);
		gain = rx->gain_range.max;
	}
	enum rfnm_rf_path path = resolve_rx_path(rx, antenna);

	p_->dev->set_rx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
	p_->dev->set_rx_channel_freq(0, (uint64_t)center_hz);
	p_->dev->set_rx_channel_gain(0, (int8_t)gain);
	p_->dev->set_rx_channel_agc(0, agc ? RFNM_AGC_DEFAULT : RFNM_AGC_OFF);   // gain above stays the AGC starting point / manual value
	p_->dev->set_rx_channel_path(0, path);
	err = p_->dev->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
	if(err) {
		fprintf(stderr, "cellsync: apply: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	p_->stream.reset(p_->dev->rx_stream_create(rfnm::channel_flags[0]));
	err = p_->stream->start();
	if(err) {
		fprintf(stderr, "cellsync: stream start: %s\n", rfnm::device::failcode_to_string(err));
		p_->stream.reset();
		return false;
	}

	p_->rate_hz = rate_hz;
	p_->antenna = antenna;
	p_->raw.resize(1 << 17);
	p_->iq.resize(p_->raw.size() / 2);
	return true;
}

bool rx_session::retune(double center_hz, const std::string &antenna) {
	if(!p_->stream) {
		return false;
	}
	p_->dev->set_rx_channel_freq(0, (uint64_t)center_hz);
	// The RF path is a channel field like the frequency, so a changed antenna rides the SAME
	// apply on the running stream - no stream restart, no device reopen, and the settle discard
	// below (already paid by every retune) covers the FE switch. Only touch it when the name
	// actually changed: resolve_rx_path logs the path list, which is per-job news, not per-carrier.
	if(!antenna.empty() && antenna != p_->antenna) {
		p_->dev->set_rx_channel_path(0, resolve_rx_path(p_->dev->get_rx_channel(0), antenna));
		p_->antenna = antenna;
	}
	rfnm_api_failcode err = p_->dev->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
	if(err) {
		fprintf(stderr, "cellsync: retune apply: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	p_->drop = (uint64_t)(0.35 * p_->rate_hz);
	p_->have_ts = false;
	return true;
}

bool rx_session::run(std::atomic<bool> &stop, std::atomic<uint64_t> &samples, const capture_cb &cb) {
	if(!p_->stream) {
		return false;
	}
	void *buffs[1] = { p_->raw.data() };
	const float scale = 1.0f / 32768.0f;
	while(!stop) {
		size_t n = 0;
		uint64_t ts = 0;
		rfnm_api_failcode r = p_->stream->read(buffs, p_->raw.size() / 2, n, ts, 100000);
		if(r != RFNM_API_OK || !n) {
			// occasional timeouts are normal; a stream that returns nothing for ~30 s
			// (device unplug/reset/wedge) must exit instead of spinning silently forever,
			// so systemd's Restart=on-failure can bring the daemon back up cleanly
			if(++p_->dry_reads >= 300) {
				fprintf(stderr, "cellsync: stream read produced no samples for 30 s (last: %s), giving up\n",
				        rfnm::device::failcode_to_string(r));
				p_->stream->stop();
				p_->stream.reset();
				return false;
			}
			continue;
		}
		p_->dry_reads = 0;
		// post-retune settle: consume without delivering (stamp chain restarts after)
		size_t off = 0;
		if(p_->drop) {
			off = p_->drop < n ? (size_t)p_->drop : n;
			p_->drop -= off;
			if(off == n) {
				continue;
			}
		}
		// stamp continuity -> gap length in samples. The stamps are exact (librfnm
		// chains per-channel contiguity and jumps them truthfully on a break), so any
		// full-sample deviation is a real hole in air time, never rounding.
		uint64_t gap = 0;
		if(p_->have_ts) {
			int64_t d_samp = (int64_t)llround(((double)ts - (double)p_->exp_ts) * p_->rate_hz / 1e9);
			if(d_samp > 0) {
				gap = (uint64_t)d_samp;
			}
		}
		p_->exp_ts = ts + (uint64_t)llround((double)n * 1e9 / p_->rate_hz);
		p_->have_ts = true;
		for(size_t i = off; i < n; i++) {
			p_->iq[i - off] = cf(p_->raw[2 * i] * scale, p_->raw[2 * i + 1] * scale);
		}
		samples += n - off;
		cb(p_->iq.data(), n - off, gap);
	}
	// the stream stays live across run() calls - retune() continues the session; the
	// destructor owns the final stop (one-shot users just let the session go out of scope)
	return true;
}

bool run_capture(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna,
                 std::atomic<bool> &stop, std::atomic<uint64_t> &samples, const capture_cb &cb) {
	rx_session s;
	if(!s.open(center_hz, rate_hz, gain_db, agc, antenna)) {
		return false;
	}
	return s.run(stop, samples, cb);
}

} // namespace cellsync

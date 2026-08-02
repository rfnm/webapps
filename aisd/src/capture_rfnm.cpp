#include "capture.h"

#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace aisd {

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
	fprintf(stderr, "aisd: antenna paths available:%s; preferred %s\n",
	        avail.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());

	if(antenna.empty()) {
		return rx->path_preferred;
	}
	enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(antenna);
	if(wanted == RFNM_PATH_NULL) {
		fprintf(stderr, "aisd: antenna '%s' not understood, using preferred %s\n",
		        antenna.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
		return rx->path_preferred;
	}
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == wanted) {
			return wanted;
		}
	}
	fprintf(stderr, "aisd: antenna '%s' (%s) not offered by this channel, using preferred %s\n",
	        antenna.c_str(), rfnm::device::rf_path_to_string(wanted).c_str(),
	        rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
	return rx->path_preferred;
}

// setup sequence: stage the rate, recover stale channels via the driver's canonical
// sweep, configure channel 0 in an apply naming only channel 0, then stream. This is
// the same as-flown sequence lorad uses; it encodes the 2026-07-05 driver rule that an
// apply mask may name only fully-configured channels.
bool run_capture(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna,
                 std::atomic<bool> &stop, std::atomic<uint64_t> &samples,
                 const std::function<void(const cf *, size_t)> &cb) {
	std::unique_ptr<rfnm::device> dev;
	try {
		dev.reset(new rfnm::device(rfnm::TRANSPORT_LOCAL, ""));
	} catch(const std::exception &e) {
		fprintf(stderr, "aisd: device open failed: %s\n", e.what());
		return false;
	}

	rfnm_api_failcode err = dev->set_stream_format(rfnm::STREAM_FORMAT_CS16);
	if(err) {
		fprintf(stderr, "aisd: set_stream_format: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	err = dev->set_samp_rate((uint64_t)rate_hz, 2000000);   // 2nd arg is timeout_us, not a rate
	if(err) {
		fprintf(stderr, "aisd: set_samp_rate(%.0f): %s\n", rate_hz, rfnm::device::failcode_to_string(err));
		return false;
	}

	// recover channels a killed session left enabled (they block stream creation); the
	// driver sweep applies only those, since device-side validation rejects apply masks
	// naming never-configured channels - the old all-channel mask failed the whole apply
	err = dev->rx_disable_stale_channels(20000000);
	if(err) {
		fprintf(stderr, "aisd: rx_disable_stale_channels: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	const struct rfnm_api_rx_ch *rx = dev->get_rx_channel(0);

	// clamp the requested gain to the advertised range: 0.3.x device-side validation
	// rejects an out-of-range gain and fails the whole apply (was the on-air crash)
	int gain = gain_db;
	if(gain < rx->gain_range.min) {
		fprintf(stderr, "aisd: gain %d below advertised min %d, clamping up\n", gain_db, rx->gain_range.min);
		gain = rx->gain_range.min;
	}
	if(gain > rx->gain_range.max) {
		fprintf(stderr, "aisd: gain %d clamped to advertised max %d\n", gain_db, rx->gain_range.max);
		gain = rx->gain_range.max;
	}
	enum rfnm_rf_path path = resolve_rx_path(rx, antenna);

	dev->set_rx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
	dev->set_rx_channel_freq(0, (uint64_t)center_hz);
	dev->set_rx_channel_gain(0, (int8_t)gain);
	dev->set_rx_channel_agc(0, agc ? RFNM_AGC_DEFAULT : RFNM_AGC_OFF);   // gain above stays the AGC starting point / manual value
	dev->set_rx_channel_path(0, path);
	err = dev->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
	if(err) {
		fprintf(stderr, "aisd: apply: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	std::unique_ptr<rfnm::rx_stream> stream(dev->rx_stream_create(rfnm::channel_flags[0]));
	err = stream->start();
	if(err) {
		fprintf(stderr, "aisd: stream start: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	std::vector<int16_t> raw(1 << 17);
	std::vector<cf> iq(raw.size() / 2);
	void *buffs[1] = { raw.data() };
	const float scale = 1.0f / 32768.0f;
	int dry_reads = 0;
	while(!stop) {
		size_t n = 0;
		uint64_t ts = 0;
		rfnm_api_failcode r = stream->read(buffs, raw.size() / 2, n, ts, 100000);
		if(r != RFNM_API_OK || !n) {
			// occasional timeouts are normal; a stream that returns nothing for ~30 s
			// (device unplug/reset/wedge) must exit instead of spinning silently forever,
			// so systemd's Restart=on-failure can bring the daemon back up cleanly
			if(++dry_reads >= 300) {
				fprintf(stderr, "aisd: stream read produced no samples for 30 s (last: %s), giving up\n",
				        rfnm::device::failcode_to_string(r));
				stream->stop();
				return false;
			}
			continue;
		}
		dry_reads = 0;
		for(size_t i = 0; i < n; i++) {
			iq[i] = cf(raw[2 * i] * scale, raw[2 * i + 1] * scale);
		}
		samples += n;
		cb(iq.data(), n);
	}
	stream->stop();
	return true;
}

} // namespace aisd

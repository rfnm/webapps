#include "capture.h"

#include <librfnm/device.h>
#include <librfnm/rx_stream.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace lorad {

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
	fprintf(stderr, "lorad: antenna paths available:%s; preferred %s\n",
	        avail.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());

	if(antenna.empty()) {
		return rx->path_preferred;
	}
	enum rfnm_rf_path wanted = rfnm::device::string_to_rf_path(antenna);
	if(wanted == RFNM_PATH_NULL) {
		fprintf(stderr, "lorad: antenna '%s' not understood, using preferred %s\n",
		        antenna.c_str(), rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
		return rx->path_preferred;
	}
	for(int i = 0; i < 10; i++) {
		if(rx->path_possible[i] == wanted) {
			return wanted;
		}
	}
	fprintf(stderr, "lorad: antenna '%s' (%s) not offered by this channel, using preferred %s\n",
	        antenna.c_str(), rfnm::device::rf_path_to_string(wanted).c_str(),
	        rfnm::device::rf_path_to_string(rx->path_preferred).c_str());
	return rx->path_preferred;
}

// One timed burst on channel 0. RX must be closed: with RX off the plan at 2 MSps is TX-only on
// the same 128 MHz DCS as RX (2e6 << 6), so no reclock and no LA9310 reset is involved. The buffer
// pool, STREAM_ON and the timed stamps follow adsld's radio_rfnm.cpp, measured on silicon there.
static bool tx_burst(rfnm::device *dev, double center_hz, double rate_hz, const tx_request &r, std::string &err) {
	const struct rfnm_api_tx_ch *txc = dev->get_tx_channel(0);
	if(!txc || !txc->avail) {
		err = "tx channel 0 not available";
		return false;
	}
	if(txc->power_range.min > r.power_dbm) {
		err = "power_range.min is above the requested power";
		return false;
	}
	bool sma = false;
	for(int i = 0; i < 10 && txc->path_possible[i] != RFNM_PATH_NULL; i++) {
		sma = sma || txc->path_possible[i] == RFNM_PATH_SMA_A;
	}
	if(!sma) {
		err = "no sma_a TX path on this board";
		return false;
	}
	rfnm_api_failcode e;
	if((e = dev->set_samp_rate((uint64_t)rate_hz, 2000000)) != RFNM_API_OK) {
		err = std::string("tx set_samp_rate: ") + rfnm::device::failcode_to_string(e);
		return false;
	}
	dev->set_tx_channel_freq(0, (int64_t)center_hz);
	dev->set_tx_channel_power(0, (int8_t)r.power_dbm);
	dev->set_tx_channel_path(0, RFNM_PATH_SMA_A);
	int16_t lpf = (int16_t)(rate_hz / 1e6);
	dev->set_tx_channel_rfic_lpf_bw(0, lpf < 1 ? 1 : lpf);   // PROBE_FAIL without it
	dev->set_tx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);   // AUTO parks a sparse sender
	if((e = dev->apply(rfnm::tx_channel_apply_flags[0], true, 20000000)) != RFNM_API_OK) {
		err = std::string("tx apply: ") + rfnm::device::failcode_to_string(e);
		return false;
	}
	bool ok = false;
	size_t bufsize = 0;
	uint8_t bpe = 4;
	// The pool outlives the burst: librfnm keeps finished buffers in its return queue, so a pool
	// freed here hands the next burst dangling pointers through tx_dqbuf.
	static std::vector<std::vector<uint8_t>> mem;
	static std::vector<rfnm::tx_buf> bufs;
	std::vector<rfnm::tx_buf *> freeb;
	bool worker = false;
	do {
		if((e = dev->set_stream_format(rfnm::STREAM_FORMAT_CS16, &bufsize, &bpe)) != RFNM_API_OK || !bufsize) {
			err = "tx set_stream_format";
			break;
		}
		// All of a frame is queued up front: refilling during a burst left a gap in anything longer
		// than the pool. 256 buffers hold the longest frame (255 bytes, about 2.2 s at SF8/62.5 kHz).
		const int NBUF = 256;
		if(mem.empty()) {
			mem.resize(NBUF);
			bufs.assign(NBUF, rfnm::tx_buf{});
			for(int i = 0; i < NBUF; i++) {
				mem[i].assign(bufsize, 0);
				bufs[i].buf = mem[i].data();
			}
		}
		for(struct rfnm::tx_buf *old = nullptr; dev->tx_dqbuf(&old) == RFNM_API_OK && old; old = nullptr) {
		}
		for(int i = 0; i < NBUF; i++) {
			freeb.push_back(&bufs[i]);
		}
		if((e = dev->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED)) != RFNM_API_OK) {
			err = "tx_work_start";
			break;
		}
		worker = true;
		struct rfnm::timing t {};
		if(dev->get_timing(rfnm::DIR_TX, &t) != RFNM_API_OK || !t.r_den) {
			err = "tx timing unavailable";
			break;
		}
		const uint64_t slot = (uint64_t)256 * t.r_num / t.r_den;
		uint64_t now = 0;
		if(dev->clock_now(&now) != RFNM_API_OK) {
			err = "clock_now";
			break;
		}
		uint64_t base = now + 4 * 524288ULL;     // adsld's TX_LEAD_TICKS
		if(slot && base > t.t0_ext) {
			base = t.t0_ext + (base - t.t0_ext + slot - 1) / slot * slot;
		}
		const size_t total = r.iq.size() / 2;
		size_t off = 0;
		size_t maxchunk = bufsize / bpe / 256 * 256;
		bool fail = false;
		while(off < total && !fail) {
			size_t chunk = std::min(maxchunk, total - off);
			struct rfnm::tx_buf *b = nullptr;
			while(dev->tx_dqbuf(&b) == RFNM_API_OK && b) {
				freeb.push_back(b);
				b = nullptr;
			}
			if(freeb.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
				static_assert(sizeof(int16_t) == 2, "cs16");
				continue;
			}
			b = freeb.back();
			freeb.pop_back();
			memcpy(b->buf, &r.iq[off * 2], chunk * bpe);
			b->elem_cnt = (uint32_t)chunk;
			b->tx_flags = 0;
			uint64_t tick = base + (uint64_t)off * t.r_num / t.r_den;
			// Feed at the air rate: a burst queued whole is stamped more than a second ahead, and
			// frames over about 1 s then fail at the far end while the driver reports no error.
			const uint64_t horizon = 25 * 524288ULL;      // about 200 ms at 64 MHz
			while(!fail) {
				uint64_t cur = 0;
				if(dev->clock_now(&cur) != RFNM_API_OK || tick <= cur + horizon) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
			}
			if((e = dev->tx_buf_schedule(b, tick)) != RFNM_API_OK || dev->tx_qbuf(b) != RFNM_API_OK) {
				err = std::string("tx schedule: ") + rfnm::device::failcode_to_string(e);
				fail = true;
				break;
			}
			off += chunk;
		}
		if(fail) {
			break;
		}
		// the burst ends at a known tick: wait for it on the board clock, bounded by wall time
		const uint64_t end = base + (uint64_t)total * t.r_num / t.r_den;
		auto limit = std::chrono::steady_clock::now() + std::chrono::duration<double>(total / rate_hz + 2.0);
		while(std::chrono::steady_clock::now() < limit) {
			if(dev->clock_now(&now) == RFNM_API_OK && now > end + slot) {
				ok = true;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		if(!ok) {
			err = "burst did not finish on the board clock";
		}
	} while(false);
	if(worker) {
		dev->tx_work_stop();
	}
	dev->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
	if(dev->apply(rfnm::tx_channel_apply_flags[0], true, 2000000) != RFNM_API_OK && ok) {
		err = "tx off apply failed";
		ok = false;
	}
	return ok;
}

// setup sequence: stage the rate, recover stale channels via the driver's canonical
// sweep, configure channel 0 in an apply naming only channel 0, then stream
bool run_capture(double center_hz, double rate_hz, int gain_db, bool agc, const std::string &antenna,
                 std::atomic<bool> &stop, std::atomic<uint64_t> &samples,
                 const std::function<void(const cf *, size_t)> &cb, tx_request *tx) {
	std::unique_ptr<rfnm::device> dev;
	try {
		dev.reset(new rfnm::device(rfnm::TRANSPORT_LOCAL, ""));
	} catch(const std::exception &e) {
		fprintf(stderr, "lorad: device open failed: %s\n", e.what());
		return false;
	}

	rfnm_api_failcode err = dev->set_stream_format(rfnm::STREAM_FORMAT_CS16);
	if(err) {
		fprintf(stderr, "lorad: set_stream_format: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	err = dev->set_samp_rate((uint64_t)rate_hz, 2000000);   // 2nd arg is timeout_us, not a rate
	if(err) {
		fprintf(stderr, "lorad: set_samp_rate(%.0f): %s\n", rate_hz, rfnm::device::failcode_to_string(err));
		return false;
	}

	// recover channels a killed session left enabled (they block stream creation); the
	// driver sweep applies only those, since device-side validation rejects apply masks
	// naming never-configured channels - the old all-channel mask failed the whole apply
	err = dev->rx_disable_stale_channels(20000000);
	if(err) {
		fprintf(stderr, "lorad: rx_disable_stale_channels: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}
	const struct rfnm_api_rx_ch *rx = dev->get_rx_channel(0);

	// clamp the requested gain to the advertised range: 0.3.x device-side validation
	// rejects an out-of-range gain and fails the whole apply
	int gain = gain_db;
	if(gain < rx->gain_range.min) {
		fprintf(stderr, "lorad: gain %d below advertised min %d, clamping up\n", gain_db, rx->gain_range.min);
		gain = rx->gain_range.min;
	}
	if(gain > rx->gain_range.max) {
		fprintf(stderr, "lorad: gain %d clamped to advertised max %d\n", gain_db, rx->gain_range.max);
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
		fprintf(stderr, "lorad: apply: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	std::unique_ptr<rfnm::rx_stream> stream(dev->rx_stream_create(rfnm::channel_flags[0]));
	err = stream->start();
	if(err) {
		fprintf(stderr, "lorad: stream start: %s\n", rfnm::device::failcode_to_string(err));
		return false;
	}

	std::vector<int16_t> raw(1 << 17);
	std::vector<cf> iq(raw.size() / 2);
	void *buffs[1] = { raw.data() };
	const float scale = 1.0f / 32768.0f;
	int dry_reads = 0;
	while(!stop) {
		if(tx) {
			std::unique_lock<std::mutex> lk(tx->m);
			if(tx->pending) {
				tx->pending = false;
				stream->stop();
				stream.reset();
				dev->rx_disable_stale_channels(20000000);
				std::string terr;
				bool ok = tx_burst(dev.get(), center_hz, rate_hz, *tx, terr);
				dev->set_rx_channel_freq(0, (uint64_t)center_hz);
				dev->set_rx_channel_gain(0, (int8_t)gain);
				dev->set_rx_channel_path(0, path);
				dev->set_rx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
				err = dev->apply(rfnm::rx_channel_apply_flags[0], true, 20000000);
				if(!err) {
					stream.reset(dev->rx_stream_create(rfnm::channel_flags[0]));
					err = stream->start();
				}
				tx->ok = ok;
				tx->err = terr;
				tx->done = true;
				tx->cv.notify_all();
				if(err) {
					fprintf(stderr, "lorad: rx restart after tx: %s\n", rfnm::device::failcode_to_string(err));
					return false;   // systemd restarts the daemon, the gates start closed again
				}
			}
		}
		size_t n = 0;
		uint64_t ts = 0;
		rfnm_api_failcode r = stream->read(buffs, raw.size() / 2, n, ts, 100000);
		if(r != RFNM_API_OK || !n) {
			// occasional timeouts are normal; a stream that returns nothing for ~30 s
			// (device unplug/reset/wedge) must exit instead of spinning silently forever,
			// so systemd's Restart=on-failure can bring the daemon back up cleanly
			if(++dry_reads >= 300) {
				fprintf(stderr, "lorad: stream read produced no samples for 30 s (last: %s), giving up\n",
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

} // namespace lorad

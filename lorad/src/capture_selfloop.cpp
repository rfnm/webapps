// LoRa TX->RX self-loopback over the RFNM radio. Synthesizes a Meshtastic LongFast frame,
// transmits it on the RFNM TX while the RX (same device handle, board-coupled) captures and
// decodes it. Proves the receiver end-to-end THROUGH the radio without any ambient traffic.
// The LoRa carrier is placed at +offset from the LO so it avoids the DC/LO spike.
#include "lora_phy.h"
#include "receiver.h"
#include "channelizer.h"
#include "fft.h"

#include <librfnm/device.h>
#include <librfnm/rx_stream.h>
#include <librfnm/constants.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <vector>

namespace lorad {

// oversampled frame synth at fs = os*BW with a frequency offset (cycles per output sample).
// Copied from selftest.cpp (the RX-verified generator) so the decoder sees the exact wire form.
static std::vector<cf> synth_frame_os(const phy_params &p, const std::vector<uint16_t> &symbols, int os, double f_off_cps) {
	int n = 1 << p.sf;
	std::vector<cf> iq;
	double ph = 0;
	auto emit_up = [&](int s) {
		for(int k = 0; k < n * os; k++) {
			double pos = fmod((double)s + (double)k / os, (double)n);
			double f = (pos + 0.5) / n - 0.5;
			ph += 2.0 * M_PI * (f / os + f_off_cps);
			iq.push_back(cf((float)cos(ph), (float)sin(ph)));
		}
	};
	auto emit_down = [&](int quarter) {
		int len = quarter ? n * os / 4 : n * os;
		for(int k = 0; k < len; k++) {
			double f = 0.5 - ((double)k / os + 0.5) / n;
			ph += 2.0 * M_PI * (f / os + f_off_cps);
			iq.push_back(cf((float)cos(ph), (float)sin(ph)));
		}
	};
	for(int i = 0; i < p.preamble_len; i++) { emit_up(0); }
	emit_up(((p.sync >> 4) & 0xF) << 3);
	emit_up((p.sync & 0xF) << 3);
	emit_down(0);
	emit_down(0);
	emit_down(1);
	for(uint16_t s : symbols) { emit_up(s); }
	return iq;
}

using namespace rfnm;

// returns the number of packets decoded from the loopback (>0 = success)
int run_selfloop(double center_hz, double offset_hz, double rate_hz, double out_rate_hz,
                 int gain_db, int tx_power_dbm, int timeout_s) {
	phy_params p;
	p.sf = 11; p.cr = 1; p.ldro = false; p.has_crc = true; p.sync = 0x2B; p.preamble_len = 8;   // Meshtastic LongFast
	const bool bw250 = true;
	const std::vector<uint8_t> payload = { 'R','F','N','M','-','L','O','R','A','-','L','O','O','P' };

	int os = (int)llround(rate_hz / out_rate_hz);
	std::vector<uint16_t> syms = encode_frame(p, payload.data(), (int)payload.size());
	std::vector<cf> frame = synth_frame_os(p, syms, os, offset_hz / rate_hz);

	// one TX period = frame at ~0.43 full-scale + a silence gap of equal length, 256-padded
	const float amp = 14000.0f;
	std::vector<int16_t> period;
	period.reserve(frame.size() * 4 + 1024);
	for(const cf &c : frame) { period.push_back((int16_t)lrintf(c.real() * amp)); period.push_back((int16_t)lrintf(c.imag() * amp)); }
	for(size_t i = 0; i < frame.size(); i++) { period.push_back(0); period.push_back(0); }
	while((period.size() / 2) % 256) { period.push_back(0); period.push_back(0); }

	std::unique_ptr<device> dev;
	try { dev.reset(new device(TRANSPORT_LOCAL, "")); }
	catch(const std::exception &e) { fprintf(stderr, "selfloop: device open: %s\n", e.what()); return -1; }

	if(dev->set_stream_format(STREAM_FORMAT_CS16)) { fprintf(stderr, "selfloop: stream_format\n"); return -1; }
	if(dev->set_samp_rate((uint64_t)rate_hz, 2000000)) { fprintf(stderr, "selfloop: samp_rate\n"); return -1; }
	dev->rx_disable_stale_channels(20000000);

	ch_helper rx = dev->rx_ch_helper(0);
	ch_helper tx = dev->tx_ch_helper(0);
	const struct rfnm_api_rx_ch *rxc = dev->get_rx_channel(0);
	const struct rfnm_api_tx_ch *txc = dev->get_tx_channel(0);

	dev->set_rx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_AUTO);
	dev->set_rx_channel_freq(0, (int64_t)center_hz);
	dev->set_rx_channel_gain(0, (int8_t)gain_db);
	dev->set_rx_channel_path(0, rxc->path_preferred);

	dev->set_tx_channel_status(0, RFNM_CH_RF_ON, RFNM_CH_STREAM_AUTO);
	dev->set_tx_channel_freq(0, (int64_t)center_hz);
	dev->set_tx_channel_power(0, (int8_t)tx_power_dbm);
	dev->set_tx_channel_path(0, txc->path_preferred);
	int16_t txlpf = (int16_t)(rate_hz / 1e6); if(txlpf < 1) { txlpf = 1; }
	dev->set_tx_channel_rfic_lpf_bw(0, txlpf);   // REQUIRED: without it the TX apply returns PROBE_FAIL

	// apply RX and TX separately: the current driver rejects a combined RX+TX apply mask
	rfnm_api_failcode ar = dev->apply(rx.apply, true, 20000000);
	rfnm_api_failcode at = dev->apply(tx.apply, true, 20000000);
	if(ar || at) { fprintf(stderr, "selfloop: apply rx=%d tx=%d\n", (int)ar, (int)at); return -1; }
	printf("selfloop: LO %.3f MHz | LoRa +%.0f kHz | SF%d BW250 sync 0x%02X | tx_pwr %d dBm | os %d | %zu-sample period\n",
	       center_hz / 1e6, offset_hz / 1e3, p.sf, p.sync, tx_power_dbm, os, period.size() / 2);

	dev->tx_work_start(TX_LATENCY_POLICY_RELAXED);
	std::unique_ptr<rx_stream> stream(dev->rx_stream_create(channel_flags[0]));
	if(stream->start()) { fprintf(stderr, "selfloop: rx start\n"); return -1; }

	std::atomic<bool> stop{ false };
	std::atomic<int> got{ 0 };

	std::thread txth([&]() {
		while(!stop) {
			size_t off = 0;
			while(off < period.size() && !stop) {
				size_t rem = (period.size() - off) / 2;
				size_t chunk = rem > (size_t)RFNM_USB_TX_PACKET_ELEM_CNT ? (size_t)RFNM_USB_TX_PACKET_ELEM_CNT : rem;
				chunk = (chunk / 256) * 256;
				if(!chunk) { break; }
				struct tx_buf *b = nullptr;
				if(dev->tx_dqbuf(&b) != RFNM_API_OK || !b) { std::this_thread::sleep_for(std::chrono::microseconds(150)); continue; }
				memcpy(b->buf, &period[off], chunk * 4);
				b->elem_cnt = (uint32_t)chunk;
				b->tx_flags = 0;
				dev->tx_qbuf(b);
				off += chunk * 2;
			}
		}
	});

	ddc dc(rate_hz, out_rate_hz, offset_hz);
	channel_receiver crx(center_hz + offset_hz, p.sync, (1u << p.sf), bw250, [&](const rx_packet &pk) {
		got++;
		printf("selfloop: *** DECODED LoRa packet #%d  SF%d CR4/%d crc_present=%d crc_ok=%d len=%zu payload=\"%.*s\" ***\n",
		       got.load(), pk.sf, pk.cr + 4, pk.crc_present, pk.crc_ok, pk.payload.size(),
		       (int)pk.payload.size(), (const char *)pk.payload.data());
	});

	std::vector<int16_t> raw(1 << 17);
	std::vector<cf> iq(raw.size() / 2), base;
	void *bufs[1] = { raw.data() };
	const float scale = 1.0f / 32768.0f;
	auto t0 = std::chrono::steady_clock::now();
	while(got < 3 && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
		size_t n = 0; uint64_t ts = 0;
		if(stream->read(bufs, raw.size() / 2, n, ts, 100000) != RFNM_API_OK || !n) { continue; }
		for(size_t i = 0; i < n; i++) { iq[i] = cf(raw[2 * i] * scale, raw[2 * i + 1] * scale); }
		base.clear();
		dc.process(iq.data(), n, base);
		crx.feed(base.data(), base.size());
	}

	stop = true;
	txth.join();
	stream->stop();
	dev->tx_work_stop();
	dev->set_tx_channel_status(0, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO);
	dev->apply(tx.apply, true, 2000000);
	printf("selfloop: DONE — %d packet(s) decoded over the radio loopback\n", got.load());
	return got.load();
}

} // namespace lorad

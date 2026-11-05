// Offline TX->RX loopback proof. The Lime is half-duplex (TDD) so it cannot receive its own
// transmission over the air; this exercises the SAME code both sides would run: the RFNM-rate
// LoRa waveform the self-loop TX emits (synth_frame_os, identical to capture_selfloop.cpp) is
// pushed through a realistic radio hop (CS16 quantize + carrier offset + AWGN) and then decoded
// by the EXACT live RX chain (ddc -> channel_receiver). Writes the TX loopback IQ and the RX
// baseband as artifacts. No radio / no librfnm needed, so it builds and runs anywhere.
#include "lora_phy.h"
#include "receiver.h"
#include "channelizer.h"
#include "fft.h"

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>

namespace lorad {

// identical generator to capture_selfloop.cpp (the RX-verified wire form)
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

int run_selfloop_offline(double snr_db, double cfo_hz) {
	phy_params p;
	p.sf = 11; p.cr = 1; p.ldro = false; p.has_crc = true; p.sync = 0x2B; p.preamble_len = 8;   // Meshtastic LongFast
	const double bw_hz = 250e3;
	const std::vector<uint8_t> payload = { 'R','F','N','M','-','L','O','R','A','-','L','O','O','P' };
	const double rate = 2e6, out_rate = 250e3, offset = 400e3;   // RFNM capture rate, channel BW, +offset off DC
	const int os = (int)llround(rate / out_rate);

	// ---- TX: the exact IQ the self-loop would emit at the RFNM rate, LoRa at +offset ----
	std::vector<uint16_t> syms = encode_frame(p, payload.data(), (int)payload.size());
	std::vector<cf> tx = synth_frame_os(p, syms, os, offset / rate);
	std::vector<cf> air;
	air.reserve(tx.size() + 8192);
	for(int i = 0; i < 4096; i++) { air.push_back(cf(0, 0)); }   // leading silence
	for(const cf &c : tx) { air.push_back(c); }
	for(int i = 0; i < 4096; i++) { air.push_back(cf(0, 0)); }   // trailing silence

	// ---- radio hop model: carrier/CFO rotation + AWGN at snr_db + CS16 (12-bit-ish) quantize ----
	std::mt19937 rng(0xC0FFEE);
	double sigp = 0; for(const cf &c : tx) { sigp += std::norm(c); } sigp /= tx.size();
	double npow = sigp / pow(10.0, snr_db / 10.0);
	std::normal_distribution<float> nd(0.0f, (float)sqrt(npow / 2.0));
	double ph = 0, dph = 2.0 * M_PI * cfo_hz / rate;
	const float amp = 14000.0f;
	std::vector<int16_t> tx_cs16; tx_cs16.reserve(air.size() * 2);
	std::vector<cf> rx(air.size());
	for(size_t i = 0; i < air.size(); i++) {
		cf x = air[i] * cf((float)cos(ph), (float)sin(ph)); ph += dph;   // carrier offset
		x += cf(nd(rng), nd(rng));                                        // AWGN
		float I = fmaxf(-32768.0f, fminf(32767.0f, x.real() * amp));
		float Q = fmaxf(-32768.0f, fminf(32767.0f, x.imag() * amp));
		int16_t Ii = (int16_t)lrintf(I), Qi = (int16_t)lrintf(Q);         // CS16 quantize (the wire)
		tx_cs16.push_back(Ii); tx_cs16.push_back(Qi);
		rx[i] = cf(Ii / 32768.0f, Qi / 32768.0f);                         // dequantized = the RX sees this
	}
	FILE *ft = fopen("/tmp/tx_loopback.cs16", "wb"); if(ft) { fwrite(tx_cs16.data(), 2, tx_cs16.size(), ft); fclose(ft); }

	// ---- RX: the exact live decode chain (ddc down-convert+decimate -> channel_receiver) ----
	int got = 0; std::vector<uint8_t> rxpl; bool rx_crc = false; int rx_sf = 0, rx_cr = 0;
	ddc dc(rate, out_rate, offset);
	channel_receiver crx(906.875e6, p.sync, (1u << p.sf), bw_hz, [&](const rx_packet &pk) {
		got++; rxpl = pk.payload; rx_crc = pk.crc_ok; rx_sf = pk.sf; rx_cr = pk.cr;
	});
	std::vector<cf> base;
	dc.process(rx.data(), rx.size(), base);
	crx.feed(base.data(), base.size());
	FILE *fr = fopen("/tmp/rx_data.cf32", "wb"); if(fr) { fwrite(base.data(), sizeof(cf), base.size(), fr); fclose(fr); }

	bool match = (got > 0) && (rxpl == payload) && rx_crc;
	printf("offline-loop: Meshtastic LongFast SF%d BW250 sync 0x%02X, CFO %.0f Hz, SNR %.1f dB, CS16 wire\n",
	       p.sf, p.sync, cfo_hz, snr_db);
	printf("offline-loop: TX %zu samples @ %.0f MSps (+%.0f kHz)  ->  /tmp/tx_loopback.cs16 (%zu bytes)\n",
	       air.size(), rate / 1e6, offset / 1e3, tx_cs16.size() * 2);
	printf("offline-loop: RX ddc -> %zu baseband samples @ %.0f kSps  ->  /tmp/rx_data.cf32\n", base.size(), out_rate / 1e3);
	if(got) {
		printf("offline-loop: DECODED SF%d CR4/%d crc_ok=%d len=%zu payload=\"%.*s\"\n",
		       rx_sf, rx_cr + 4, rx_crc, rxpl.size(), (int)rxpl.size(), (const char *)rxpl.data());
	}
	printf("offline-loop: %s — %d packet decoded, payload %s (TX \"%.*s\")\n",
	       match ? "PASS" : "FAIL", got, match ? "BYTE-EXACT + CRC-OK" : "mismatch",
	       (int)payload.size(), (const char *)payload.data());
	return match ? 0 : 1;
}

} // namespace lorad

// adsld selftest: chip-layer cases - Manchester round trips, SYNC64 pattern,
// frame geometry, GFSK modulator spectrum and frequency path sanity.
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "adsl_phy.h"
#include "gfsk_mod.h"
#include "selftest.h"

namespace adsld {

// fixed 25-byte Traffic-shaped wire packet: LEN byte + arbitrary but fixed content
static std::vector<uint8_t> test_packet() {
	std::vector<uint8_t> pkt(25);
	pkt[0] = 0x18;
	std::mt19937 rng(0xAD51);
	for(size_t i = 1; i < pkt.size(); i++) {
		pkt[i] = (uint8_t)(rng() & 0xFF);
	}
	return pkt;
}

// instantaneous frequency between samples n-1 and n, Hz
static double inst_freq(const std::vector<cf> &x, size_t n, double fs) {
	cf d = x[n] * std::conj(x[n - 1]);
	return std::atan2((double)d.imag(), (double)d.real()) * fs / (2.0 * M_PI);
}

int selftest_phy() {
	int fails = 0;
	std::mt19937 rng(20260826);

	// ---- bit/byte order ----------------------------------------------------------------
	{
		const uint8_t b = 0x72;
		bits got = bytes_to_bits_msb(&b, 1);
		bits want = { 0, 1, 1, 1, 0, 0, 1, 0 };
		check(fails, got == want, "bytes_to_bits_msb 0x72 KAT");
		std::vector<uint8_t> data(64);
		for(auto &v : data) {
			v = (uint8_t)(rng() & 0xFF);
		}
		check(fails, bits_to_bytes_msb(bytes_to_bits_msb(data.data(), data.size())) == data,
		      "bytes<->bits round trip");
	}

	// ---- Manchester --------------------------------------------------------------------
	{
		check(fails, manchester_encode({ 1 }) == bits({ 0, 1 }), "manchester 1 -> 01");
		check(fails, manchester_encode({ 0 }) == bits({ 1, 0 }), "manchester 0 -> 10");
		bits data(200);
		for(auto &v : data) {
			v = (uint8_t)(rng() & 1);
		}
		int errs = -1;
		bits back = manchester_decode(manchester_encode(data), &errs);
		check(fails, back == data && errs == 0, "manchester round trip, 0 errors");
		bits chips = manchester_encode(data);
		chips[10] ^= 1;   // makes chip pair 5 invalid (00 or 11)
		errs = -1;
		manchester_decode(chips, &errs);
		check(fails, errs == 1, "manchester invalid pair counted", std::to_string(errs));
	}

	// ---- frame geometry + SYNC64 pattern -----------------------------------------------
	{
		std::vector<uint8_t> pkt = test_packet();
		bits frame = frame_build(pkt.data(), pkt.size());
		check(fails, (int)frame.size() == ADSL_FRAME_CHIPS, "frame is 456 chips",
		      std::to_string(frame.size()));

		bool pre_ok = frame.size() >= 24;
		for(int i = 0; pre_ok && i < 16; i++) {
			pre_ok = frame[i] == (i & 1);                      // '01' x8
		}
		static const uint8_t TAIL[8] = { 1, 0, 0, 1, 1, 0, 0, 1 };
		for(int i = 0; pre_ok && i < 8; i++) {
			pre_ok = frame[16 + i] == TAIL[i];                 // '1001' x2
		}
		check(fails, pre_ok, "preamble chips");

		// the 64-chip detection pattern = Manchester(0xF5, 0x72, 0x4B, 0x18) must appear
		// verbatim at chip offset 8 (preamble tail + sync word + LEN byte)
		bits sync64 = bytes_to_bits_msb(ADSL_SYNC64, sizeof(ADSL_SYNC64));
		bool sync_ok = frame.size() >= 8 + sync64.size();
		for(size_t i = 0; sync_ok && i < sync64.size(); i++) {
			sync_ok = frame[8 + i] == sync64[i];
		}
		check(fails, sync_ok, "SYNC64 pattern at chip offset 8");

		// decoding everything after the preamble recovers sync word + packet
		int errs = -1;
		bits post(frame.begin() + ADSL_PREAMBLE_CHIPS, frame.end());
		std::vector<uint8_t> bytes = bits_to_bytes_msb(manchester_decode(post, &errs));
		bool ok = errs == 0 && bytes.size() == 27 && bytes[0] == ADSL_SYNCWORD[0] &&
		          bytes[1] == ADSL_SYNCWORD[1];
		for(size_t i = 0; ok && i < pkt.size(); i++) {
			ok = bytes[2 + i] == pkt[i];
		}
		check(fails, ok, "frame decodes to sync word + packet");
	}

	// ---- modulator: settled deviation and offset ---------------------------------------
	{
		bits ones(20, 1);
		std::vector<cf> burst = gfsk_render_burst(ones, 400e3, 0.0, 1.0f);
		size_t mid = burst.size() / 2;
		double f = inst_freq(burst, mid, 400e3);
		check(fails, std::fabs(f - ADSL_DEV_HZ) < 500.0, "settled deviation +50 kHz",
		      std::to_string(f));

		std::vector<cf> off = gfsk_render_burst(ones, 2e6, 300e3, 1.0f);
		double fo = 0;
		size_t m0 = off.size() / 2;
		for(size_t n = m0; n < m0 + 20; n++) {
			fo += inst_freq(off, n, 2e6);
		}
		fo /= 20;
		check(fails, std::fabs(fo - (300e3 + ADSL_DEV_HZ)) < 1e3, "offset mixer +300 kHz",
		      std::to_string(fo));

		check(fails, gfsk_render_burst(ones, 250e3, 0.0, 1.0f).empty(),
		      "non-multiple fs refused");
	}

	// ---- modulator: chip frequency recovery at 4 samples/chip -------------------------
	{
		std::vector<uint8_t> pkt = test_packet();
		bits chips = frame_build(pkt.data(), pkt.size());
		const double fs = 400e3;
		const int S = 4;
		std::vector<cf> burst = gfsk_render_burst(chips, fs, 0.0, 1.0f);
		check(fails, burst.size() == (chips.size() + 2 * GFSK_RAMP_CHIPS) * (size_t)S,
		      "burst length", std::to_string(burst.size()));
		int sign_errs = 0;
		double devsum = 0;
		for(size_t k = 0; k < chips.size(); k++) {
			size_t n = (GFSK_RAMP_CHIPS + k) * (size_t)S + 2;   // chip-center discriminator
			double f = inst_freq(burst, n, fs);
			if((f > 0) != (chips[k] != 0)) {
				sign_errs++;
			}
			devsum += std::fabs(f);
		}
		double devavg = devsum / chips.size();
		check(fails, sign_errs == 0, "discriminator recovers all 456 chips",
		      std::to_string(sign_errs) + " sign errors");
		check(fails, devavg > 40e3 && devavg < 52e3, "mean |deviation| plausible for BT=0.5",
		      std::to_string(devavg));
	}

	// ---- modulator: 99% occupied bandwidth < 200 kHz -----------------------------------
	{
		std::vector<uint8_t> pkt = test_packet();
		bits chips = frame_build(pkt.data(), pkt.size());
		const double fs = 400e3;
		std::vector<cf> burst = gfsk_render_burst(chips, fs, 0.0, 1.0f);
		// coarse direct DFT, 1 kHz spacing across the whole fs span
		const int NBINS = 400;
		std::vector<double> pwr(NBINS);
		double total = 0;
		for(int b = 0; b < NBINS; b++) {
			double f = -fs / 2 + (double)b * fs / NBINS;
			double w = -2.0 * M_PI * f / fs;
			double re = 0, im = 0;
			for(size_t n = 0; n < burst.size(); n++) {
				double c = std::cos(w * (double)n), s = std::sin(w * (double)n);
				re += burst[n].real() * c - burst[n].imag() * s;
				im += burst[n].real() * s + burst[n].imag() * c;
			}
			pwr[b] = re * re + im * im;
			total += pwr[b];
		}
		// grow a window from DC outward until it holds 99% of the power
		const int dc = NBINS / 2;
		double acc = pwr[dc];
		int w = 0;
		while(acc < 0.99 * total && w < dc - 1) {
			w++;
			acc += pwr[dc - w] + pwr[dc + w];
		}
		double obw_hz = (2 * w + 1) * fs / NBINS;
		check(fails, obw_hz < 200e3, "99% occupied bandwidth < 200 kHz",
		      std::to_string(obw_hz));
	}

	// ---- CS16 render -------------------------------------------------------------------
	{
		bits chips = manchester_encode(bytes_to_bits_msb(test_packet().data(), 25));
		std::vector<cf> iq = gfsk_render_burst(chips, 400e3, 0.0, 14000.0f);
		std::vector<int16_t> iq16 = gfsk_render_burst_cs16(chips, 400e3, 0.0, 14000);
		bool ok = iq16.size() == 2 * iq.size();
		int16_t peak = 0;
		for(size_t i = 0; ok && i < iq.size(); i++) {
			ok = std::lround((double)iq[i].real()) == iq16[2 * i] &&
			     std::lround((double)iq[i].imag()) == iq16[2 * i + 1];
			if(std::abs(iq16[2 * i]) > peak) {
				peak = (int16_t)std::abs(iq16[2 * i]);
			}
		}
		check(fails, ok, "CS16 matches rounded float render");
		check(fails, peak <= 14000 && peak > 13000, "CS16 peak amplitude ~14000",
		      std::to_string(peak));
		size_t mid = iq.size() / 2;
		double mag = std::hypot((double)iq16[2 * mid], (double)iq16[2 * mid + 1]);
		check(fails, std::fabs(mag - 14000.0) < 3.0, "settled envelope magnitude",
		      std::to_string(mag));
	}

	return fails;
}

} // namespace adsld

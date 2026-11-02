// adsld selftest: full-RF receiver cases. A LOCAL 2-GFSK synthesizer (chips -> Gaussian
// pulse shaping -> phase accumulator, built here so the suite does not depend on the
// gfsk_mod/adsl_phy translation units) renders frames at the 2 MSps capture rate on the
// +300 kHz channel offset; they run through the real ddc (/5, 130 kHz cutoff) into the
// real receiver. Fixed-seed sweeps: clean, CFO +-15 kHz, sub-chip timing offsets, AWGN,
// and a 5 s pure-noise false-decode soak. (The calibrated SNR/CFO sweep matrix against
// the production modulator lands in the integration stage.)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "adsl_codec.h"
#include "channelizer.h"
#include "receiver.h"
#include "selftest.h"

namespace adsld {

namespace {

constexpr double FS_IN = 2e6;    // capture rate
constexpr double FS_OUT = 400e3; // demod rate (4 samples/chip)
constexpr double CH_OFF = 300e3; // 868.2 MHz at the 867.9 MHz LO
constexpr double DDC_CUTOFF = 130e3;

// deterministic test packet: LEN + pseudo-random NetHdr/data + a CRC24 consistent with
// the linked codec, so the receiver's CRC gate is exercised for real
std::vector<uint8_t> test_packet(uint32_t seed) {
	std::mt19937 rng(seed);
	std::vector<uint8_t> pkt(ADSL_PKT_LEN);
	pkt[0] = ADSL_LEN_TRAFFIC;
	for(size_t i = 1; i <= ADSL_CRC_SPAN; i++) {
		pkt[i] = (uint8_t)(rng() & 0xff);
	}
	uint32_t c = crc24(&pkt[1], ADSL_CRC_SPAN);
	pkt[ADSL_CRC_SPAN + 1] = (uint8_t)(c >> 16);
	pkt[ADSL_CRC_SPAN + 2] = (uint8_t)(c >> 8);
	pkt[ADSL_CRC_SPAN + 3] = (uint8_t)c;
	return pkt;
}

// frame chips straight from the documented layout (independent of adsl_phy.cpp):
// '01'x8 + '1001'x2 preamble, then Manchester(sync word + packet), 1 -> 01, 0 -> 10
std::vector<uint8_t> frame_chips(const std::vector<uint8_t> &pkt) {
	std::vector<uint8_t> ch;
	ch.reserve(ADSL_FRAME_CHIPS);
	for(int i = 0; i < 8; i++) {
		ch.push_back(0);
		ch.push_back(1);
	}
	for(int i = 0; i < 2; i++) {
		ch.push_back(1);
		ch.push_back(0);
		ch.push_back(0);
		ch.push_back(1);
	}
	auto manchester_byte = [&ch](uint8_t b) {
		for(int k = 7; k >= 0; k--) {
			if((b >> k) & 1) {
				ch.push_back(0);
				ch.push_back(1);
			} else {
				ch.push_back(1);
				ch.push_back(0);
			}
		}
	};
	manchester_byte(ADSL_SYNCWORD[0]);
	manchester_byte(ADSL_SYNCWORD[1]);
	for(uint8_t b : pkt) {
		manchester_byte(b);
	}
	return ch;
}

// local 2-GFSK synthesizer: chip k (centered at t = k + 0.5 chips) contributes
// nrz * h(t - center) to the instantaneous frequency, where h is a rect(1 chip)
// convolved with the BT = 0.5 Gaussian (closed form via erf); a phase accumulator and a
// complex exponential mixed to offset_hz produce the burst. delay_chips shifts the burst
// by a fractional chip for the timing sweep.
std::vector<cf> synth_burst(const std::vector<uint8_t> &chips, double offset_hz,
                            double delay_chips, float amp) {
	const double spc = FS_IN / ADSL_CHIP_RATE_HZ; // samples per chip (20)
	const double kk = M_PI * ADSL_BT * std::sqrt(2.0 / std::log(2.0));
	const size_t nsamp = (size_t)llround(((double)chips.size() + 8.0) * spc);
	const double woff = 2.0 * M_PI * offset_hz / FS_IN;
	std::vector<cf> out(nsamp);
	double phi = 0;
	for(size_t i = 0; i < nsamp; i++) {
		double t = (double)i / spc - 4.0 - delay_chips; // chip time; chip 0 spans [0,1)
		int k0 = (int)std::floor(t - 4.0);
		int k1 = (int)std::ceil(t + 4.0);
		if(k0 < 0) {
			k0 = 0;
		}
		if(k1 > (int)chips.size() - 1) {
			k1 = (int)chips.size() - 1;
		}
		double f = 0;
		for(int k = k0; k <= k1; k++) {
			double d = t - ((double)k + 0.5);
			f += (chips[(size_t)k] ? 1.0 : -1.0) * 0.5 *
			     (std::erf(kk * (d + 0.5)) - std::erf(kk * (d - 0.5)));
		}
		phi += 2.0 * M_PI * ADSL_DEV_HZ * f / FS_IN;
		double w = phi + woff * (double)i;
		out[i] = cf((float)(amp * std::cos(w)), (float)(amp * std::sin(w)));
	}
	return out;
}

// The receiver relies on the linked crc24() to reject near-miss timing hypotheses.
// While adsl_codec.cpp is still the scaffold stub (crc24 constant), that gate is
// inoperative and a noisy trial can deliver an extra corrupted packet alongside the good
// one - so the "exactly one callback" assertions only arm once the real codec is linked.
// Byte-exact delivery of the transmitted packet is required unconditionally.
bool codec_live() {
	uint8_t a = 0x00, b = 0x01;
	return crc24(&a, 1) != crc24(&b, 1);
}

struct rx_result {
	int count = 0;        // total callbacks
	int matched = 0;      // callbacks whose 25 bytes equal the transmitted packet
	adsl_rx_meta meta;    // metadata of the first matching callback
};

// run a 2 MSps stimulus (plus optional complex AWGN of the given total rms) through
// ddc + receiver; pad_samples of silence lead and tail surround the signal
rx_result run_rx(const std::vector<cf> &sig, const uint8_t *expect, double noise_rms,
                 uint32_t noise_seed, size_t pad_samples = 4000) {
	rx_result res;
	ddc chan(FS_IN, FS_OUT, CH_OFF, DDC_CUTOFF);
	adsl_receiver rx(0, ADSL_CH_HZ[0], FS_OUT,
	                 [&res, expect](const uint8_t *p, const adsl_rx_meta &m) {
		                 res.count++;
		                 if(memcmp(p, expect, ADSL_PKT_LEN) == 0 && res.matched++ == 0) {
			                 res.meta = m;
		                 }
	                 });
	std::mt19937 rng(noise_seed);
	std::normal_distribution<float> gauss(0.0f, (float)(noise_rms / std::sqrt(2.0)));
	const size_t total = sig.size() + 2 * pad_samples;
	std::vector<cf> chunk, out;
	for(size_t i = 0; i < total;) {
		chunk.clear();
		for(; i < total && chunk.size() < 4096; i++) {
			cf x = (i >= pad_samples && i - pad_samples < sig.size()) ? sig[i - pad_samples]
			                                                         : cf(0, 0);
			if(noise_rms > 0) {
				x += cf(gauss(rng), gauss(rng));
			}
			chunk.push_back(x);
		}
		out.clear();
		chan.process(chunk.data(), chunk.size(), out);
		rx.feed(out.data(), out.size());
	}
	return res;
}

} // namespace

int selftest_rx() {
	int fails = 0;
	char detail[256];

	const std::vector<uint8_t> pkt = test_packet(0xad51);
	const std::vector<uint8_t> chips = frame_chips(pkt);
	const bool strict = codec_live();   // see codec_live() - stub codec relaxes counts
	auto good = [strict](const rx_result &r) {
		return strict ? (r.count == 1 && r.matched == 1) : r.matched >= 1;
	};

	// frame geometry + the documented sync64 pattern must fall out of the layout: chips
	// [8, 72) are the preamble tail + sync word + LEN byte = ADSL_SYNC64 (MSB-first)
	{
		bool ok = chips.size() == (size_t)ADSL_FRAME_CHIPS;
		for(int i = 0; i < ADSL_SYNC64_CHIPS && ok; i++) {
			int want = (ADSL_SYNC64[i / 8] >> (7 - i % 8)) & 1;
			ok = chips[(size_t)(8 + i)] == (uint8_t)want;
		}
		snprintf(detail, sizeof(detail), "%zu chips", chips.size());
		check(fails, ok, "rx frame geometry + sync64 pattern", detail);
	}

	// clean decode at ideal conditions: exactly one callback (adjacent-phase duplicates
	// suppressed), byte-exact packet, no correction, sane metadata
	{
		rx_result r = run_rx(synth_burst(chips, CH_OFF, 0.0, 0.5f), pkt.data(), 0.0, 1);
		bool ok = good(r) && !r.meta.corrected && r.meta.channel == 0 &&
		          r.meta.phase >= 0 && r.meta.phase < adsl_receiver::PHASES;
		snprintf(detail, sizeof(detail), "count=%d phase=%d rssi=%.1f dB", r.count,
		         r.meta.phase, (double)r.meta.rssi_db);
		check(fails, ok, "rx clean decode", detail);
	}

	// CFO: the differential Manchester decision must hold across +-15 kHz
	{
		const double cfos[] = { -15e3, -7.5e3, 7.5e3, 15e3 };
		for(double cfo : cfos) {
			rx_result r =
				run_rx(synth_burst(chips, CH_OFF + cfo, 0.0, 0.5f), pkt.data(), 0.0, 2);
			snprintf(detail, sizeof(detail), "cfo=%+.1f kHz count=%d", cfo / 1e3, r.count);
			check(fails, good(r), "rx decode with CFO", detail);
		}
	}

	// timing: the 16-phase bank must cover every sub-chip arrival alignment
	{
		int bad = -1;
		for(int j = 0; j < 16; j++) {
			double off = (double)j / 16.0;
			rx_result r = run_rx(synth_burst(chips, CH_OFF, off, 0.5f), pkt.data(), 0.0, 3);
			if(!good(r)) {
				bad = j;
				break;
			}
		}
		snprintf(detail, sizeof(detail),
		         bad < 0 ? "16/16 offsets" : "failed at offset %d/16", bad);
		check(fails, bad < 0, "rx timing offsets 0..1 chip", detail);
	}

	// AWGN at a generous 20 dB SNR (referred to the 400 kHz demod bandwidth): 100%
	// decode over 20 fixed-seed trials. White noise at FS_IN carries FS_IN/FS_OUT times
	// the in-band power, hence the sqrt(5) factor on the rms.
	{
		const float amp = 0.5f;
		const double snr_db = 20.0;
		const double rms =
			amp * std::sqrt(FS_IN / FS_OUT) * std::pow(10.0, -snr_db / 20.0);
		const std::vector<cf> burst = synth_burst(chips, CH_OFF, 0.0, amp);
		int ngood = 0;
		for(uint32_t trial = 0; trial < 20; trial++) {
			rx_result r = run_rx(burst, pkt.data(), rms, 100 + trial);
			if(good(r)) {
				ngood++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/20 trials", ngood);
		check(fails, ngood == 20, "rx AWGN 20 dB decode", detail);
	}

	// 5 s of pure noise: no false decodes, and the min-statistics noise floor must have
	// converged to just under the noise RSSI
	{
		int false_decodes = 0;
		ddc chan(FS_IN, FS_OUT, CH_OFF, DDC_CUTOFF);
		adsl_receiver rx(0, ADSL_CH_HZ[0], FS_OUT,
		                 [&false_decodes](const uint8_t *, const adsl_rx_meta &) {
			                 false_decodes++;
		                 });
		std::mt19937 rng(42);
		std::normal_distribution<float> gauss(0.0f, 0.1f);
		std::vector<cf> chunk(4096), out;
		const size_t total = (size_t)(5.0 * FS_IN);
		for(size_t i = 0; i < total; i += chunk.size()) {
			for(auto &x : chunk) {
				x = cf(gauss(rng), gauss(rng));
			}
			out.clear();
			chan.process(chunk.data(), chunk.size(), out);
			rx.feed(out.data(), out.size());
		}
		snprintf(detail, sizeof(detail), "%d false decodes", false_decodes);
		check(fails, false_decodes == 0, "rx noise-only 5 s soak", detail);

		float rssi = rx.rssi_db(), nf = rx.noise_floor_db();
		bool ok = nf <= rssi + 0.5f && rssi - nf < 6.0f;
		snprintf(detail, sizeof(detail), "rssi=%.1f dB floor=%.1f dB", (double)rssi,
		         (double)nf);
		check(fails, ok, "rx noise floor tracks noise rssi", detail);
	}

	return fails;
}

} // namespace adsld

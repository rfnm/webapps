// adsld selftest: definitive offline end-to-end loopback gates. The PRODUCTION chain on
// both sides: adsl_encode -> frame_build -> gfsk_render_burst (the real TX modulator) ->
// channel impairments (AWGN at calibrated in-channel SNR, CFO, sub-chip timing offsets,
// sample-clock ppm, adjacent-channel bursts) -> ddc -> adsl_receiver -> adsl_decode.
// Packet success rate is measured over fixed-seed trial sweeps; every gate below is a
// hard [FAIL] when unmet:
//   - 100%   @ >= 12 dB in-channel SNR (200 trials)
//   - >= 99% @ 9 dB (200 trials)
//   - no degradation at CFO up to +-15 kHz (200 trials each at 12 dB)
//   - timing 0..1 chip sweep clean
//   - sample-clock offset +-50 ppm ok
//   - both channels decoded simultaneously from one 2 MSps stream
//   - adjacent-channel interferer +10 dB: no cross-decode, < 1 dB desense
//   - zero false decodes on 30 s equivalent of noise (both channels)
// In-channel SNR convention: signal power over the noise power falling in the 400 kHz
// demod bandwidth (white noise at the 2 MSps capture rate carries 5x that power, hence
// the sqrt(FS_IN/FS_OUT) factor on the generator rms).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "adsl_codec.h"
#include "adsl_phy.h"
#include "channelizer.h"
#include "gfsk_mod.h"
#include "receiver.h"
#include "selftest.h"

namespace adsld {

namespace {

constexpr double FS_IN = 2e6;      // capture rate
constexpr double FS_OUT = 400e3;   // demod rate (4 samples/chip)
constexpr double LO_HZ = 867.9e6;  // channels at +300 kHz / +500 kHz
constexpr double DDC_CUTOFF = 130e3;
constexpr float SIG_AMP = 0.25f;   // burst amplitude into the float chain

inline double ch_off(int ch) {
	return ADSL_CH_HZ[ch] - LO_HZ;
}

// a plausible, deterministic Traffic report per seed (fields the sweep round-trips)
adsl_traffic make_traffic(uint32_t seed) {
	std::mt19937 rng(seed);
	adsl_traffic t;
	t.amt = (uint8_t)(rng() & 0x3F);
	t.addr = rng() & 0xFFFFFF;
	t.ts_qs = (uint8_t)(rng() % 60);
	t.flightstate = 2;
	t.category = (uint8_t)(rng() & 0x1F);
	t.pos_valid = true;
	t.lat_deg = -60.0 + (double)(rng() % 120000) / 1000.0;
	t.lon_deg = -170.0 + (double)(rng() % 340000) / 1000.0;
	t.alt_valid = true;
	t.alt_m = (float)(rng() % 3000);
	t.speed_valid = true;
	t.speed_mps = (float)(rng() % 60);
	t.track_valid = true;
	t.track_deg = (float)(rng() % 360);
	t.sil = 1;
	t.nacp = 5;
	return t;
}

// production TX side: report -> packet -> chips -> GFSK baseband at the channel offset
std::vector<cf> render(const adsl_traffic &t, uint8_t pkt_out[ADSL_PKT_LEN], int ch,
                       double cfo_hz) {
	adsl_encode(t, pkt_out);
	bits chips = frame_build(pkt_out, ADSL_PKT_LEN);
	return gfsk_render_burst(chips, FS_IN, ch_off(ch) + cfo_hz, SIG_AMP);
}

// linear-interpolation resampler: models a TX sample clock off by (ratio-1), e.g.
// ratio = 1 + 50e-6 for +50 ppm (chip rate and burst duration scale together)
std::vector<cf> resample(const std::vector<cf> &in, double ratio) {
	std::vector<cf> out;
	out.reserve(in.size() + 16);
	for(double t = 0;; t += ratio) {
		size_t i = (size_t)t;
		if(i + 1 >= in.size()) {
			break;
		}
		float f = (float)(t - (double)i);
		out.push_back(in[i] * (1.0f - f) + in[i + 1] * f);
	}
	return out;
}

// composite of two bursts, b delayed by b_off samples against a
std::vector<cf> mix(const std::vector<cf> &a, const std::vector<cf> &b, size_t b_off) {
	std::vector<cf> out(std::max(a.size(), b_off + b.size()), cf(0, 0));
	for(size_t i = 0; i < a.size(); i++) {
		out[i] = a[i];
	}
	for(size_t i = 0; i < b.size(); i++) {
		out[b_off + i] += b[i];
	}
	return out;
}

struct rx_out {
	int count = 0;     // total callbacks
	int matched = 0;   // callbacks byte-identical to the expected packet
	adsl_rx_meta meta; // metadata of the first match
};

// one channel chain: lead silence + stimulus + tail silence (+ AWGN of the given total
// rms over everything) -> ddc -> receiver
rx_out run_chain(int ch, const std::vector<cf> &sig, const uint8_t *expect,
                 double noise_rms, uint32_t noise_seed, size_t lead = 4000) {
	rx_out res;
	ddc chan(FS_IN, FS_OUT, ch_off(ch), DDC_CUTOFF);
	adsl_receiver rx(ch, ADSL_CH_HZ[ch], FS_OUT,
	                 [&res, expect](const uint8_t *p, const adsl_rx_meta &m) {
		                 res.count++;
		                 if(expect && memcmp(p, expect, ADSL_PKT_LEN) == 0 &&
		                    res.matched++ == 0) {
			                 res.meta = m;
		                 }
	                 });
	std::mt19937 rng(noise_seed);
	std::normal_distribution<float> gauss(0.0f, (float)(noise_rms / std::sqrt(2.0)));
	const size_t total = sig.size() + lead + 4000;
	std::vector<cf> chunk, out;
	for(size_t i = 0; i < total;) {
		chunk.clear();
		for(; i < total && chunk.size() < 8192; i++) {
			cf x = (i >= lead && i - lead < sig.size()) ? sig[i - lead] : cf(0, 0);
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

inline bool clean_success(const rx_out &r) {
	// the expected packet arrived, and nothing else (no corrupted extra deliveries)
	return r.matched >= 1 && r.count == r.matched;
}

inline double snr_to_rms(double snr_db) {
	return (double)SIG_AMP * std::sqrt(FS_IN / FS_OUT) * std::pow(10.0, -snr_db / 20.0);
}

// success rate over `trials` fixed-seed runs at one (SNR, CFO) point; NPKT distinct
// packets rendered once, sub-sample timing dithered per trial via the lead pad
constexpr int NPKT = 8;
int sweep(double snr_db, double cfo_hz, int trials, uint32_t seed_base) {
	std::vector<std::vector<cf>> bursts(NPKT);
	std::vector<std::vector<uint8_t>> pkts(NPKT, std::vector<uint8_t>(ADSL_PKT_LEN));
	for(int p = 0; p < NPKT; p++) {
		bursts[p] = render(make_traffic(seed_base + (uint32_t)p), pkts[p].data(), 0, cfo_hz);
	}
	const double rms = snr_to_rms(snr_db);
	int good = 0;
	for(int t = 0; t < trials; t++) {
		int p = t % NPKT;
		rx_out r = run_chain(0, bursts[p], pkts[p].data(), rms,
		                     seed_base + 1000 + (uint32_t)t, 4000 + (size_t)(t % 20));
		if(clean_success(r)) {
			good++;
		}
	}
	return good;
}

} // namespace

int selftest_loopback() {
	int fails = 0;
	char detail[256];
	const int TRIALS = 200;

	// ---- clean field-level round trip (production TX -> production RX -> decode) -------
	{
		adsl_traffic tx = make_traffic(0x10c);
		uint8_t pkt[ADSL_PKT_LEN];
		std::vector<cf> burst = render(tx, pkt, 0, 0);
		rx_out r = run_chain(0, burst, pkt, 0, 1);
		adsl_traffic d;
		bool ok = clean_success(r) && !r.meta.corrected &&
		          adsl_decode(pkt, d) == adsl_decode_result::ok && d.amt == tx.amt &&
		          d.addr == tx.addr && d.ts_qs == tx.ts_qs && d.category == tx.category &&
		          d.pos_valid && std::fabs(d.lat_deg - tx.lat_deg) < 2e-5 &&
		          std::fabs(d.lon_deg - tx.lon_deg) < 4e-5 && d.alt_valid &&
		          std::fabs(d.alt_m - tx.alt_m) < 2.0f;
		snprintf(detail, sizeof(detail), "count=%d phase=%d rssi=%.1f dB", r.count,
		         r.meta.phase, (double)r.meta.rssi_db);
		check(fails, ok, "loopback field round-trip", detail);
	}

	// ---- GATE: 100% at >= 12 dB in-channel SNR -----------------------------------------
	{
		int good = sweep(12.0, 0, TRIALS, 0x1200);
		snprintf(detail, sizeof(detail), "%d/%d @ 12 dB", good, TRIALS);
		check(fails, good == TRIALS, "loopback gate 100% @ 12 dB SNR", detail);
	}

	// ---- GATE: >= 99% at 9 dB ----------------------------------------------------------
	{
		int good = sweep(9.0, 0, TRIALS, 0x900);
		snprintf(detail, sizeof(detail), "%d/%d @ 9 dB (need >= %d)", good, TRIALS,
		         (TRIALS * 99 + 99) / 100);
		check(fails, good * 100 >= TRIALS * 99, "loopback gate >= 99% @ 9 dB SNR", detail);
	}

	// ---- GATE: no degradation at CFO up to +-15 kHz (still 100% at 12 dB) --------------
	{
		const double cfos[] = { -15e3, 15e3 };
		for(double cfo : cfos) {
			int good = sweep(12.0, cfo, TRIALS, 0xCF0 + (cfo > 0 ? 1 : 0));
			snprintf(detail, sizeof(detail), "cfo=%+.0f kHz: %d/%d @ 12 dB", cfo / 1e3,
			         good, TRIALS);
			check(fails, good == TRIALS, "loopback gate CFO +-15 kHz undegraded", detail);
		}
	}

	// ---- GATE: timing 0..1 chip sweep clean --------------------------------------------
	// 20 sub-chip arrival phases in 1/20-chip steps (one capture-rate sample each)
	{
		adsl_traffic tx = make_traffic(0x71);
		uint8_t pkt[ADSL_PKT_LEN];
		std::vector<cf> burst = render(tx, pkt, 0, 0);
		int bad = -1;
		for(int j = 0; j < 20 && bad < 0; j++) {
			rx_out r = run_chain(0, burst, pkt, 0, (uint32_t)j, 4000 + (size_t)j);
			if(!clean_success(r)) {
				bad = j;
			}
		}
		snprintf(detail, sizeof(detail),
		         bad < 0 ? "20/20 offsets" : "failed at offset %d/20 chip", bad);
		check(fails, bad < 0, "loopback gate timing 0..1 chip sweep", detail);
	}

	// ---- GATE: sample-clock offset +-50 ppm --------------------------------------------
	{
		for(double ppm : { -50.0, 50.0 }) {
			adsl_traffic tx = make_traffic(0x50 + (uint32_t)(ppm > 0));
			uint8_t pkt[ADSL_PKT_LEN];
			std::vector<cf> burst = resample(render(tx, pkt, 0, 0), 1.0 + ppm * 1e-6);
			const double rms = snr_to_rms(12.0);
			int good = 0;
			const int N = 50;
			for(int t = 0; t < N; t++) {
				rx_out r = run_chain(0, burst, pkt, rms, 0x33000 + (uint32_t)t,
				                     4000 + (size_t)(t % 20));
				if(clean_success(r)) {
					good++;
				}
			}
			snprintf(detail, sizeof(detail), "%+.0f ppm: %d/%d @ 12 dB", ppm, good, N);
			check(fails, good == N, "loopback gate sample clock +-50 ppm", detail);
		}
	}

	// ---- GATE: both channels decoded simultaneously from one 2 MSps stream -------------
	{
		int good = 0;
		const int N = 10;
		for(int t = 0; t < N; t++) {
			adsl_traffic ta = make_traffic(0x2000 + (uint32_t)t);
			adsl_traffic tb = make_traffic(0x3000 + (uint32_t)t);
			uint8_t pa[ADSL_PKT_LEN], pb[ADSL_PKT_LEN];
			std::vector<cf> stream =
			    mix(render(ta, pa, 0, 0), render(tb, pb, 1, 0), (size_t)(t * 731) % 3000);
			rx_out r0 = run_chain(0, stream, pa, snr_to_rms(12.0), 0x44000 + (uint32_t)t);
			rx_out r1 = run_chain(1, stream, pb, snr_to_rms(12.0), 0x45000 + (uint32_t)t);
			if(clean_success(r0) && clean_success(r1)) {
				good++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/%d dual-burst streams", good, N);
		check(fails, good == N, "loopback gate both channels from one stream", detail);
	}

	// ---- GATE: adjacent-channel interferer +10 dB: no cross-decode, < 1 dB desense -----
	{
		const float iamp_rel = std::pow(10.0f, 10.0f / 20.0f);   // +10 dB over the desired
		// (a) cross-decode: a strong valid ADS-L burst on 868.4 alone must produce nothing
		// from the 868.2 receiver
		int cross = 0;
		for(int t = 0; t < 5; t++) {
			adsl_traffic ti = make_traffic(0x6000 + (uint32_t)t);
			uint8_t pi[ADSL_PKT_LEN];
			std::vector<cf> ib = render(ti, pi, 1, 0);
			for(auto &x : ib) {
				x *= iamp_rel;
			}
			rx_out r = run_chain(0, ib, nullptr, snr_to_rms(12.0), 0x55000 + (uint32_t)t);
			cross += r.count;
		}
		snprintf(detail, sizeof(detail), "%d decodes from interferer-only streams", cross);
		check(fails, cross == 0, "loopback gate no adjacent-channel cross-decode", detail);

		// (b) desense: with the >= 99% @ 9 dB gate above, >= 99% at 10 dB under a +10 dB
		// adjacent interferer bounds the desense to < 1 dB
		int good = 0;
		const double rms = snr_to_rms(10.0);
		for(int t = 0; t < TRIALS; t++) {
			adsl_traffic td = make_traffic(0x7000 + (uint32_t)(t % NPKT));
			adsl_traffic ti = make_traffic(0x8000 + (uint32_t)(t % NPKT));
			uint8_t pd[ADSL_PKT_LEN], pi[ADSL_PKT_LEN];
			std::vector<cf> db = render(td, pd, 0, 0);
			std::vector<cf> ib = render(ti, pi, 1, 0);
			for(auto &x : ib) {
				x *= iamp_rel;
			}
			std::vector<cf> stream = mix(db, ib, (size_t)(t * 137) % 2000);
			rx_out r = run_chain(0, stream, pd, rms, 0x66000 + (uint32_t)t,
			                     4000 + (size_t)(t % 20));
			if(clean_success(r)) {
				good++;
			}
		}
		snprintf(detail, sizeof(detail), "%d/%d @ 10 dB + interferer (need >= %d)", good,
		         TRIALS, (TRIALS * 99 + 99) / 100);
		check(fails, good * 100 >= TRIALS * 99,
		      "loopback gate < 1 dB adjacent-channel desense", detail);
	}

	// ---- GATE: zero false decodes on 30 s equivalent of noise (both channels) ----------
	{
		int false_decodes = 0;
		auto cb = [&false_decodes](const uint8_t *, const adsl_rx_meta &) {
			false_decodes++;
		};
		ddc c0(FS_IN, FS_OUT, ch_off(0), DDC_CUTOFF), c1(FS_IN, FS_OUT, ch_off(1), DDC_CUTOFF);
		adsl_receiver r0(0, ADSL_CH_HZ[0], FS_OUT, cb), r1(1, ADSL_CH_HZ[1], FS_OUT, cb);
		std::mt19937 rng(0x5017);
		std::normal_distribution<float> gauss(0.0f, 0.1f);
		std::vector<cf> chunk(8192), out;
		const size_t total = (size_t)(30.0 * FS_IN);
		for(size_t i = 0; i < total; i += chunk.size()) {
			for(auto &x : chunk) {
				x = cf(gauss(rng), gauss(rng));
			}
			out.clear();
			c0.process(chunk.data(), chunk.size(), out);
			r0.feed(out.data(), out.size());
			out.clear();
			c1.process(chunk.data(), chunk.size(), out);
			r1.feed(out.data(), out.size());
		}
		snprintf(detail, sizeof(detail), "%d false decodes in 2 x 30 s", false_decodes);
		check(fails, false_decodes == 0, "loopback gate 30 s noise, zero false decodes",
		      detail);
	}

	return fails;
}

} // namespace adsld

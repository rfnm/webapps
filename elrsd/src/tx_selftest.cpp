// elrsd: TX loopback selftest - burst_synth -> wide cs16 stream with AWGN ->
// rx_chain (B) -> raw symbols + ref -> li_codec::decode -> payload bytes must be identical.
// NEEDS: src/rx_chain.cpp src/dsp.cpp src/elrs_codec.cpp src/elrs_link.cpp src/tx_chain.cpp
//
// Hooked into run_selftest_tx() through the weak symbol run_selftest_tx_loopback(); until B's
// rx_chain.cpp exists this file is pruned by the Makefile and tx_chain.cpp reports [SKIP].
// No hardware, deterministic seed. Independent of this file, the synthesis was cross-checked
// against the Python-verified path (resample_poly 13/100 + the Python reference demod and decode):
// all four rates byte-exact at +3.7 MHz (2026-09-14, a local probe script).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "elrs_codec.h"
#include "rx_chain.h"
#include "selftest.h"
#include "tx_chain.h"

namespace elrsd {

namespace {

// wide-band (per-sample) SNR of the noisy cases. B's envelope gate needs env > floor * 6 (+15.6 dB,
// the threshold the real captures pass), so the noisy loopback runs at 20 dB wide-band = ~35 dB in
// the 812.5 kHz chirp band (real captures at 10 cm / gain -24 sit well above that).
#ifndef LOOP_SNR_DB
#define LOOP_SNR_DB 20.0
#endif
#ifndef LOOP_N_BURSTS
#define LOOP_N_BURSTS 2
#endif

struct lcg {
	uint64_t s;
	explicit lcg(uint64_t seed) : s(seed) {}
	double uniform() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return (double)(s >> 11) / 9007199254740992.0; }
	// Box-Muller
	double gauss() {
		double u1 = std::max(1e-12, uniform()), u2 = uniform();
		return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
	}
};

static int g_loop_total = 0, g_loop_matched = 0;

struct loop_case {
	int rate_index;
	double offset_hz;     // channel offset from the window centre (on the 1 MHz grid)
	bool iq_invert;
	double snr_db;        // wide-band (per-sample) SNR of the burst vs the AWGN floor
};

// one wide-band stream: lead-in noise, `n_bursts` bursts on the slot raster, tail noise
static int run_case(selftest_counter &c, const loop_case &lc, int n_bursts) {
	const rate_params *rp = rate_by_index(lc.rate_index);
	const double fs = 25e6;
	// RX window centre ON the channel grid (channel 41): the offsets +4/+1/-6 MHz then land on channels
	// 45/42/35, and their mirrors (see below) on 37/40/47 - also on the grid.
	const double fc = 2441.4e6;
	const uid_t6 uid{0xd2, 0x4c, 0x3e, 0x2e, 0xfb, 0x4d};
	li_codec codec(*rp);
	burst_synth_config sc;
	sc.fs_hz = fs;
	sc.iq_invert = lc.iq_invert;
	burst_synth bs(*rp, sc);
	const size_t slot_n = (size_t)std::llround((double)rp->interval_us * fs / 1e6);
	const size_t lead_n = (size_t)(0.020 * fs), tail_n = (size_t)(0.020 * fs);
	const size_t total = lead_n + (size_t)n_bursts * slot_n + tail_n;
	std::vector<int16_t> wide(2 * total);
	lcg rng(0x5eed0000ULL + (uint64_t)lc.rate_index * 97 + (uint64_t)(lc.iq_invert ? 7 : 0) + (uint64_t)std::llround(std::fabs(lc.offset_hz) / 1e5));
	const double A = 14000.0;
	const double sigma = A / std::sqrt(2.0 * std::pow(10.0, lc.snr_db / 10.0));
	for(size_t i = 0; i < 2 * total; i++) {
		double v = sigma * rng.gauss();
		wide[i] = (int16_t)std::max(-32767.0, std::min(32767.0, std::lround(v) + 0.0));
	}
	std::vector<std::array<uint8_t, 8>> sent;
	std::vector<int16_t> iq;
	for(int k = 0; k < n_bursts; k++) {
		std::array<uint16_t, 16> ch;
		for(size_t i = 0; i < 16; i++) ch[i] = (uint16_t)(172 + (uint32_t)(rng.uniform() * 1639.0));
		uint8_t nonce = (uint8_t)(rng.uniform() * 256.0);
		std::array<uint8_t, 8> pkt = ota4_build_rc(uid, ch, nonce, rp->hop_interval, rp->tlm_denom_default, 2, false);
		sent.push_back(pkt);
		std::vector<uint16_t> syms = codec.encode(pkt.data(), 8, 0);
		size_t n = bs.synth(syms, lc.offset_hz, iq);
		size_t at = lead_n + (size_t)k * slot_n + (size_t)(rng.uniform() * 2000.0);   // jitter the onset inside the slot
		for(size_t i = 0; i < n && at + i < total; i++) {
			int vi = wide[2 * (at + i)] + iq[2 * i], vq = wide[2 * (at + i) + 1] + iq[2 * i + 1];
			wide[2 * (at + i)] = (int16_t)std::max(-32767, std::min(32767, vi));
			wide[2 * (at + i) + 1] = (int16_t)std::max(-32767, std::min(32767, vq));
		}
	}
	// ---- receive ----
	rx_chain_config rc;
	rc.fs_hz = fs;
	rc.fc_hz = fc;
	rc.os = 4;
	rc.rate_index = lc.rate_index;
	// invertIQ (uid[5] & 1) mirrors the CHIRP only (up <-> down, carrier untouched). B's `conj` is the
	// capture-path IQ sign and mirrors the WHOLE wide stream, so it undoes the chirp mirror but also
	// mirrors the spectrum around fc: the channel estimate then lands on fc - offset. The loopback
	// expects exactly that (finding for B/C: a live invertIQ sender needs the channel estimate
	// mirrored, or a chirp-direction knob that is independent of the stream sign).
	rc.conj = lc.iq_invert;
	rc.auto_conj = false;
	rx_chain chain(rc);
	struct got { std::vector<uint16_t> symbols; int ref; bool found; int channel; int rate; };
	std::vector<got> bursts;
	chain.set_on_burst([&](const rx_burst &b) {
		got g;
		g.symbols = b.symbols;
		g.ref = b.sync.ref;
		g.found = b.sync.found;
		g.channel = b.channel;
		g.rate = b.rate_index;
		bursts.push_back(g);
	});
	const size_t blk = 65536;
	for(size_t pos = 0; pos < total; pos += blk) {
		size_t n = std::min(blk, total - pos);
		chain.feed(&wide[2 * pos], n, pos, -1);
	}
	// flush: feed a little more noise so that the last burst's end is seen
	{
		std::vector<int16_t> pad(2 * blk);
		for(auto &v : pad) v = (int16_t)std::lround(sigma * rng.gauss());
		chain.feed(pad.data(), blk, total, -1);
	}
	// ---- decode ----
	int expect_ch = fhss_channel_from_freq(fc + (lc.iq_invert ? -lc.offset_hz : lc.offset_hz));
	int found = 0, decoded = 0, matched = 0, ch_ok = 0;
	std::string detail;
	size_t si = 0;
	for(const got &g : bursts) {
		if(!g.found) continue;
		found++;
		if(g.channel == expect_ch) ch_ok++;
		std::vector<uint8_t> payload;
		if(g.symbols.size() >= (size_t)rp->n_sym && codec.decode(g.symbols.data(), g.symbols.size(), g.ref, payload)) {
			decoded++;
			// match against any sent packet (bursts may be reported out of order only if B splits them; normally in order)
			bool m = false;
			for(size_t k = si; k < sent.size(); k++) {
				if(std::memcmp(payload.data(), sent[k].data(), 8) == 0) { m = true; si = k + 1; break; }
			}
			if(m) matched++;
		}
	}
	char name[128];
	snprintf(name, sizeof(name), "loopback idx%d %+.1fMHz inv%d snr%.0fdB", lc.rate_index, lc.offset_hz / 1e6, lc.iq_invert ? 1 : 0, lc.snr_db);
	// CR4/8 rates (6/7/9): every burst must come back byte-equal. SF5 (idx 4, CR4/6 = detection
	// only): the nominal-offset receive method has a one-bin timing step at SF5/OS4 and misses a
	// fraction of placements even noise-free - the Python reference decodes 6/8 random placements
	// of the same synthesized burst (2026-09-14 check_sf5_jitter.py) and 50 % of the real SF5
	// packets on air. So idx 4 requires >= 1 byte-equal burst and NO wrong decode.
	bool wrong = decoded != matched;
	bool ok = !wrong && ch_ok == found && (lc.rate_index == 4 ? matched >= 1 : matched == n_bursts);
	c.check(ok, name, "bursts " + std::to_string(bursts.size()) + " found " + std::to_string(found) + " decoded " + std::to_string(decoded) +
	        " byte-equal " + std::to_string(matched) + "/" + std::to_string(n_bursts) + " channel " + std::to_string(expect_ch) + " ok " + std::to_string(ch_ok) +
	        (lc.rate_index == 4 && matched < n_bursts ? " (SF5 CR4/6 miss, see comment)" : ""));
	g_loop_total += n_bursts;
	g_loop_matched += matched;
	return ok ? 0 : 1;
}

} // namespace

int run_selftest_tx_loopback(selftest_counter &c) {
	int fails = 0;
	for(int idx : rate_indices_verified()) {
		for(double off : {4.0e6, 1.0e6, -6.0e6}) {
			for(int inv = 0; inv < 2; inv++) {
				loop_case lc{idx, off, inv != 0, LOOP_SNR_DB};
				fails += run_case(c, lc, LOOP_N_BURSTS);
			}
		}
	}
	// a clean (noise-free) run at the window centre must also pass: catches sign/offset regressions without noise luck
	fails += run_case(c, loop_case{6, 1.0e6, false, 40.0}, 3);
	printf("tx loopback: %d/%d bursts byte-equal through rx_chain (%d checks failed)\n", g_loop_matched, g_loop_total, fails);
	return fails;
}

} // namespace elrsd

// NEEDS: src/rx_chain.cpp src/dsp.cpp src/source_buffer.cpp
// elrsd: RX selftest + --bench driver. Own minimal LoRa frame synthesizer (preamble
// up-chirps, 2 sync-word symbols, 2.25 down-chirps, 2 fine-sync symbols for SF5/6, payload)
// at any sample rate via a phase accumulator, so B never depends on D's tx_chain.
#include "rx_chain.h"
#include "sample_source.h"
#include "selftest.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

namespace elrsd {

namespace {

// ---------------------------------------------------------------- synthesizer -------
struct chirp_piece {
	double sym;      // symbol value (cyclic shift in bins) for up-chirps
	bool down;       // down-chirp
	double len_sym;  // duration in symbols (0.25 for the SFD tail)
};

// LoRa frame of `rp` with payload symbols, sampled at fs with the carrier at f_off (Hz),
// starting at t0 seconds inside a buffer of total_s seconds (complex float, unit amplitude).
// The instantaneous frequency of an up-chirp with symbol s is
//   f(t) = f_off + bw * (frac((t - t_k) / Tsym + s / M) - 0.5)
// (a down-chirp mirrors the sweep). The phase is evaluated in closed form per sample,
// phi(u) = 2*pi*M*(u^2/2 - u/2) (+ 2*pi*f_off*t), continuous at the chirp wrap (phi(0) = phi(1) = 0);
// a rectangle-rule phase accumulator would bias the chirp by 1/(2*os) bins.
void synth_frame(const rate_params &rp, const std::vector<uint16_t> &payload, double fs, double f_off, double t0,
                 std::vector<cf> &buf, double amp = 1.0, bool iq_invert = false) {
	const int M = 1 << rp.sf;
	const double tsym = rp.t_sym_s();
	std::vector<chirp_piece> pieces;
	for(int i = 0; i < rp.preamble_len; i++) pieces.push_back({0, false, 1.0});
	pieces.push_back({(double)(M / 4), false, 1.0});     // sync word symbols (value not verified, RX ignores)
	pieces.push_back({(double)(M / 2), false, 1.0});
	pieces.push_back({0, true, 1.0});
	pieces.push_back({0, true, 1.0});
	pieces.push_back({0, true, 0.25});
	if(rp.fine_sync) {
		pieces.push_back({0, false, 1.0});
		pieces.push_back({0, false, 1.0});
	}
	for(uint16_t s : payload) pieces.push_back({(double)(s % M), false, 1.0});
	size_t n0 = (size_t)llround(t0 * fs);
	size_t pi = 0;
	double piece_start = 0;
	const double dt = 1.0 / fs;
	for(size_t n = n0; n < buf.size(); n++) {
		double t = (double)n * dt - t0;   // time inside the frame
		while(pi < pieces.size() && t >= piece_start + pieces[pi].len_sym * tsym) {
			piece_start += pieces[pi].len_sym * tsym;
			pi++;
		}
		if(pi >= pieces.size()) break;
		double u = (t - piece_start) / tsym + pieces[pi].sym / M;
		u -= floor(u);
		double phi = 2.0 * M_PI * M * (0.5 * u * u - 0.5 * u);
		if(pieces[pi].down) phi = -phi;
		double phase = 2.0 * M_PI * f_off * t + phi;
		if(iq_invert) phase = -phase;
		buf[n] += cf((float)(amp * cos(phase)), (float)(amp * sin(phase)));
	}
}

std::vector<uint16_t> random_symbols(std::mt19937 &rng, int n, int sf) {
	std::vector<uint16_t> v(n);
	for(auto &s : v) s = (uint16_t)(rng() % (1u << sf));
	return v;
}

void add_awgn(std::vector<cf> &buf, double sigma, std::mt19937 &rng) {
	std::normal_distribution<float> nd(0.0f, (float)sigma);
	for(auto &x : buf) x += cf(nd(rng), nd(rng));
}

void to_i16(const std::vector<cf> &buf, double scale, std::vector<int16_t> &out) {
	out.resize(buf.size() * 2);
	for(size_t i = 0; i < buf.size(); i++) {
		double re = buf[i].real() * scale, im = buf[i].imag() * scale;
		re = std::max(-32768.0, std::min(32767.0, re));
		im = std::max(-32768.0, std::min(32767.0, im));
		out[2 * i] = (int16_t)llround(re);
		out[2 * i + 1] = (int16_t)llround(im);
	}
}

// (rx - ref) mod M == tx for every payload symbol
int count_symbol_matches(const std::vector<uint16_t> &rx, int ref, const std::vector<uint16_t> &tx, int sf) {
	const int M = 1 << sf;
	int ok = 0;
	for(size_t i = 0; i < tx.size() && i < rx.size(); i++) {
		if((((int)rx[i] - ref) % M + M) % M == (int)tx[i]) ok++;
	}
	return ok;
}

// wide-band synthetic stream: bursts of `rp` in the 25 MSps window; returns int16 buffer
struct wide_burst { double t0; int channel; std::vector<uint16_t> syms; };

void synth_wide(const rate_params &rp, const std::vector<wide_burst> &bursts, double fs, double fc, double total_s,
                double amp_i16, double noise_i16, std::mt19937 &rng, std::vector<int16_t> &out, bool iq_invert = false) {
	std::vector<cf> buf((size_t)llround(total_s * fs), cf(0, 0));
	for(const wide_burst &b : bursts) {
		double f = FHSS_FREQ_START_HZ + b.channel * FHSS_FREQ_STEP_HZ - fc;
		synth_frame(rp, b.syms, fs, f, b.t0, buf, 1.0, iq_invert);
	}
	add_awgn(buf, noise_i16 / amp_i16, rng);
	to_i16(buf, amp_i16, out);
}

} // namespace

// ---------------------------------------------------------------- selftest -------------
int run_selftest_rx() {
	selftest_counter c;
	std::mt19937 rng(20260914);

	// 1. FFT vs naive DFT
	for(unsigned n : {32u, 256u, 1024u}) {
		fft_plan fft(n);
		std::vector<cf> in(n), out(n);
		std::uniform_real_distribution<float> ud(-1, 1);
		for(auto &x : in) x = cf(ud(rng), ud(rng));
		fft.run(in.data(), out.data());
		double err = 0;
		for(unsigned k = 0; k < n; k++) {
			cf acc(0, 0);
			for(unsigned i = 0; i < n; i++) {
				double a = -2.0 * M_PI * (double)k * i / n;
				acc += in[i] * cf((float)cos(a), (float)sin(a));
			}
			err = std::max(err, (double)std::abs(acc - out[k]));
		}
		char name[64];
		snprintf(name, sizeof name, "fft_%u_vs_dft", n);
		c.check(err < 1e-3, name, "max err " + std::to_string(err));
	}

	// 2. resampler 25e6 -> 3.25e6 (13/100): output count, tone frequency, passband gain, stopband
	{
		const double fs = 25e6;
		resampler rs(13, 100, fs, 0.0);
		c.check(std::fabs(rs.out_rate() - 3.25e6) < 1e-3, "resampler_out_rate", std::to_string(rs.out_rate()));
		const size_t n = 200000;
		std::vector<cf> in(n), out;
		const double ftone = 100e3;
		for(size_t i = 0; i < n; i++) {
			double ph = 2.0 * M_PI * ftone * i / fs;
			in[i] = cf((float)cos(ph), (float)sin(ph));
		}
		rs.process(in.data(), n, out);
		size_t expect = n * 13 / 100;
		c.check(out.size() >= expect - 1 && out.size() <= expect + 1, "resampler_count", std::to_string(out.size()) + " vs " + std::to_string(expect));
		// steady state: phase slope and amplitude over the second half
		size_t a = out.size() / 2, b = out.size() - 8;
		double amp = 0;
		for(size_t i = a; i < b; i++) amp += std::abs(out[i]);
		amp /= (double)(b - a);
		double dph = std::arg(out[b - 1] * std::conj(out[a]));
		// unwrap using the expected slope
		double slope_exp = 2.0 * M_PI * ftone / rs.out_rate() * (double)(b - 1 - a);
		double k = std::round((slope_exp - dph) / (2.0 * M_PI));
		double f_meas = (dph + k * 2.0 * M_PI) / (2.0 * M_PI * (double)(b - 1 - a)) * rs.out_rate();
		c.check(std::fabs(f_meas - ftone) < 20.0, "resampler_tone_freq", std::to_string(f_meas));
		c.check(std::fabs(20 * log10(amp)) < 0.5, "resampler_passband_gain_db", std::to_string(20 * log10(amp)));
		// the chain's extraction filter (48 taps/branch, cutoff 0.55 bw): the chirp edge at
		// 0.4 MHz passes within 1 dB, the adjacent channel (1.0 MHz and beyond) is down > 20 dB
		auto tone_db = [&](double f) {
			resampler r(13, 100, fs, 0.0, 32, 0);
			std::vector<cf> o;
			for(size_t i = 0; i < n; i++) {
				double ph = 2.0 * M_PI * f * i / fs;
				in[i] = cf((float)cos(ph), (float)sin(ph));
			}
			r.process(in.data(), n, o);
			double m = 0;
			for(size_t i = o.size() / 2; i < o.size(); i++) m = std::max(m, (double)std::abs(o[i]));
			return 20 * log10(m + 1e-12);
		};
		double e04 = tone_db(0.4e6), e29 = tone_db(2.9e6), e32 = tone_db(3.2e6);
		c.check(e04 > -0.5, "resampler_passband_edge_0p4MHz_db", std::to_string(e04));
		c.check(e29 < -40.0 && e32 < -40.0, "resampler_alias_stopband_db", "2.9 MHz " + std::to_string(e29) + " 3.2 MHz " + std::to_string(e32));
		// NCO mixing: a tone at +3.7 MHz mixed by offset 3.7 MHz lands at DC
		resampler rs3(13, 100, fs, 3.7e6);
		std::vector<cf> out3;
		for(size_t i = 0; i < n; i++) {
			double ph = 2.0 * M_PI * 3.7e6 * i / fs;
			in[i] = cf((float)cos(ph), (float)sin(ph));
		}
		rs3.process(in.data(), n, out3);
		double dph3 = std::arg(out3[out3.size() - 1] * std::conj(out3[out3.size() / 2]));
		double f3 = dph3 / (2.0 * M_PI * (double)(out3.size() - 1 - out3.size() / 2)) * rs3.out_rate();
		double amp3 = std::abs(out3[out3.size() - 1]);
		c.check(std::fabs(f3) < 20.0 && amp3 > 0.9, "resampler_nco_mix", "f " + std::to_string(f3) + " amp " + std::to_string(amp3));
		// process_i16 == process on scaled input
		resampler r4(13, 100, fs, 1e6), r5(13, 100, fs, 1e6);
		std::vector<int16_t> i16(2000 * 2);
		std::vector<cf> f4(2000), o4, o5;
		for(size_t i = 0; i < 2000; i++) {
			i16[2 * i] = (int16_t)(rng() % 20000) - 10000;
			i16[2 * i + 1] = (int16_t)(rng() % 20000) - 10000;
			f4[i] = cf(i16[2 * i] / 32768.0f, -i16[2 * i + 1] / 32768.0f);
		}
		r4.process_i16(i16.data(), 2000, o4, true);
		r5.process(f4.data(), 2000, o5);
		bool same = o4.size() == o5.size();
		for(size_t i = 0; same && i < o4.size(); i++) same = std::abs(o4[i] - o5[i]) < 1e-6f;
		c.check(same, "resampler_i16_conj_path");
	}

	// 3. envelope_i16 exact block sums, partial-block carry
	{
		envelope_i16 e(100);
		std::vector<int16_t> v(2 * 1050);
		for(auto &x : v) x = (int16_t)((int)(rng() % 65536) - 32768);
		std::vector<uint32_t> out;
		e.process(v.data(), 37, out);
		e.process(v.data() + 74, 1050 - 37, out);
		bool ok = out.size() == 10;
		for(size_t b = 0; ok && b < 10; b++) {
			uint64_t s = 0;
			for(size_t i = 0; i < 200; i++) {
				int x = v[b * 200 + i];
				s += x < 0 ? -x : x;
			}
			ok = s == out[b];
		}
		c.check(ok, "envelope_i16_exact", std::to_string(out.size()) + " blocks");
		// DC tracking: a +13000/-3200 offset on white noise (the capture's numbers) must be
		// removed - block sums after lock within 5 % of the DC-free sums
		envelope_i16 ed(1000);
		ed.set_dc_tracking(true);
		std::vector<int16_t> w(2 * 20000);
		std::normal_distribution<float> nd(0.0f, 2400.0f);
		for(size_t i = 0; i < 20000; i++) {
			w[2 * i] = (int16_t)std::max(-32768.0f, std::min(32767.0f, 13614 + nd(rng)));
			w[2 * i + 1] = (int16_t)std::max(-32768.0f, std::min(32767.0f, -3245 + nd(rng)));
		}
		std::vector<uint32_t> od;
		ed.process(w.data(), 20000, od);
		double ref = 2.0 * 1000 * 2400.0 * 0.7979;   // E|N(0,s)| = s*sqrt(2/pi), I and Q
		bool okd = od.size() == 20 && std::abs(ed.dc_i() - 13614) < 200 && std::abs(ed.dc_q() + 3245) < 200;
		for(size_t b = 2; okd && b < od.size(); b++) okd = std::fabs(od[b] / ref - 1.0) < 0.05;
		c.check(okd, "envelope_i16_dc_tracking", "dc " + std::to_string(ed.dc_i()) + "/" + std::to_string(ed.dc_q()) + " blk2 " + (od.size() > 2 ? std::to_string(od[2] / ref) : "-"));
	}

	// 4. css_modem round trip SF5..8 at os 1 and 4, with a small CFO and noise
	for(int sf = 5; sf <= 8; sf++) {
		for(int os : {1, 4}) {
			css_modem m(sf, os);
			const int M = 1 << sf, N = m.samples_per_symbol();
			std::vector<uint16_t> tx = random_symbols(rng, 40, sf);
			std::vector<cf> sig(tx.size() * N);
			for(size_t i = 0; i < tx.size(); i++) {
				for(int k = 0; k < N; k++) sig[i * N + k] = m.upchirp()[(k + tx[i] * os) % N];
			}
			// CFO of 0.2 bins shifts every symbol equally (rounding picks the nominal bin)
			css_modem::rotate_cfo(sig.data(), sig.size(), 0.2 * ELRS_BW_HZ / M, os * ELRS_BW_HZ);
			add_awgn(sig, 0.3, rng);
			std::vector<uint16_t> rx;
			m.demod(sig.data(), tx.size(), rx);
			int ok = 0;
			for(size_t i = 0; i < tx.size(); i++) ok += rx[i] == tx[i];
			char name[64];
			snprintf(name, sizeof name, "css_modem_sf%d_os%d", sf, os);
			c.check(ok == (int)tx.size(), name, std::to_string(ok) + "/" + std::to_string(tx.size()));
			// down-chirp reference sees a down-chirp
			std::vector<cf> dn(N);
			for(int k = 0; k < N; k++) dn[k] = m.downchirp()[k];
			float pu = 0, pd = 0;
			m.demod_symbol(dn.data(), &pu);
			int bd = m.demod_symbol_down(dn.data(), &pd);
			snprintf(name, sizeof name, "css_modem_sf%d_os%d_downchirp", sf, os);
			c.check(bd == 0 && pd > 4 * pu, name);
		}
	}

	// 4b. fractional-bin estimator: symbol 5 shifted by +0.3 / -0.3 bins
	for(int os : {1, 4}) {
		css_modem m(6, os);
		const int N = m.samples_per_symbol();
		bool ok = true;
		std::string det;
		for(double fb : {0.3, -0.3, 0.45}) {
			std::vector<cf> sig(N);
			for(int k = 0; k < N; k++) sig[k] = m.upchirp()[(k + 5 * os) % N];
			css_modem::rotate_cfo(sig.data(), N, fb * ELRS_BW_HZ / 64, os * ELRS_BW_HZ);
			float fr = 0;
			int b = m.demod_symbol_frac(sig.data(), &fr);
			det += " " + std::to_string(b) + "+" + std::to_string(fr);
			ok = ok && b == 5 && std::fabs(fr - fb) < 0.05;
		}
		c.check(ok, ("css_modem_frac_os" + std::to_string(os)).c_str(), det);
	}

	// 5. frame_sync on synthesized narrow-band frames (fs = os * bw): CFO, STO, noise, both
	//    os; check found, and (sym - ref) mod M == tx for all payload symbols
	for(int idx : rate_indices_verified()) {
		const rate_params &rp = *rate_by_index(idx);
		for(int os : {4, 1}) {
			const double fs = os * ELRS_BW_HZ;
			int total_ok = 0, total = 0, found = 0, cases = 0;
			for(int trial = 0; trial < 6; trial++) {
				double cfo = (trial % 3 - 1) * 0.3 * ELRS_BW_HZ / (1 << rp.sf);   // -0.3, 0, +0.3 bins
				double t0 = 0.3e-3 + (double)(rng() % 500) / fs;                 // 0.3 ms + random STO
				double snr_db = trial < 3 ? 20.0 : 8.0;
				std::vector<uint16_t> tx = random_symbols(rng, rp.n_sym, rp.sf);
				size_t len = (size_t)llround((rp.frame_offset_symbols + rp.n_sym + 4) * rp.t_sym_s() * fs) + (size_t)llround(t0 * fs);
				std::vector<cf> buf(len, cf(0, 0));
				synth_frame(rp, tx, fs, cfo, t0, buf);
				add_awgn(buf, pow(10.0, -snr_db / 20.0), rng);
				frame_sync fsy(rp, os, false);
				std::vector<uint16_t> rx;
				frame_sync_result r = fsy.run(buf.data(), buf.size(), rx);
				cases++;
				int ok = r.found ? count_symbol_matches(rx, r.ref, tx, rp.sf) : -1;
				if(getenv("ELRSD_RX_DEBUG")) {
					printf("  rate%d os%d trial%d cfo %.1f Hz t0 %.1f smp snr %.0f: found %d run %d ref %d pre_start %zu (truth %.1f) ok %d/%d",
					       idx, os, trial, cfo, t0 * fs, snr_db, r.found, r.preamble_run, r.ref, r.preamble_start, t0 * fs, ok, rp.n_sym);
					if(r.found) {
						printf(" diffs:");
						for(int i = 0; i < rp.n_sym; i++) printf(" %d", (((int)rx[i] - r.ref) % (1 << rp.sf) + (1 << rp.sf)) % (1 << rp.sf) - (int)tx[i]);
					}
					printf("\n");
				}
				if(!r.found) continue;
				found++;
				total_ok += ok;
				total += rp.n_sym;
				// preamble start must be within one symbol of the truth
				double err = ((double)r.preamble_start - t0 * fs) / (rp.t_sym_s() * fs);
				if(std::fabs(err) > 1.0) total_ok -= rp.n_sym;
			}
			char name[64];
			snprintf(name, sizeof name, "frame_sync_rate%d_sf%d_os%d", idx, rp.sf, os);
			// os 4 (the Python-verified path) must be exact; os 1 (lorad-style, unverified on
			// air) tolerates 3 % symbol slips at the 8 dB in-band stress SNR (gated bursts are
			// >= 20 dB in-band)
			bool pass = found == cases && (os == 4 ? total_ok == total : total_ok >= (int)(0.97 * total));
			c.check(pass, name,
			        "found " + std::to_string(found) + "/" + std::to_string(cases) + " symbols " + std::to_string(total_ok) + "/" + std::to_string(total));
		}
		// candidate offsets: nominal first
		frame_sync fsy(rp, 4, false);
		auto co = fsy.candidate_offsets();
		c.check(co.size() == 3 && co[0] == rp.frame_offset_symbols, "candidate_offsets_nominal_first");
	}
	// a conjugated frame (wrong IQ sign) must NOT be reported as found (auto_conj relies on it)
	{
		const rate_params &rp = *rate_by_index(6);
		const double fs = 4 * ELRS_BW_HZ;
		std::vector<uint16_t> tx = random_symbols(rng, rp.n_sym, rp.sf);
		size_t len = (size_t)llround((rp.frame_offset_symbols + rp.n_sym + 4) * rp.t_sym_s() * fs) + 300;
		std::vector<cf> buf(len, cf(0, 0));
		synth_frame(rp, tx, fs, 0.0, 300.0 / fs, buf, 1.0, true);
		add_awgn(buf, 0.05, rng);
		frame_sync fsy(rp, 4, false);
		std::vector<uint16_t> rx;
		frame_sync_result r = fsy.run(buf.data(), buf.size(), rx);
		c.check(!r.found, "frame_sync_rejects_conjugated", "run " + std::to_string(r.preamble_run) + " snr " + std::to_string(r.snr_db));
	}
	// half-bin CFO (the worst case for argmax rounding): any residual symbol error must be +-1
	// bin only - that is what Gray coding + the LI Hamming code absorb
	{
		const rate_params &rp = *rate_by_index(6);
		const double fs = 4 * ELRS_BW_HZ;
		int bad = 0, total = 0;
		for(int trial = 0; trial < 4; trial++) {
			std::vector<uint16_t> tx = random_symbols(rng, rp.n_sym, rp.sf);
			size_t len = (size_t)llround((rp.frame_offset_symbols + rp.n_sym + 4) * rp.t_sym_s() * fs) + (size_t)(0.3e-3 * fs);
			std::vector<cf> buf(len, cf(0, 0));
			synth_frame(rp, tx, fs, 0.5 * ELRS_BW_HZ / 64, 0.3e-3 + trial * 37.0 / fs, buf);
			add_awgn(buf, 0.1, rng);
			frame_sync fsy(rp, 4, false);
			std::vector<uint16_t> rx;
			frame_sync_result r = fsy.run(buf.data(), buf.size(), rx);
			if(!r.found) {
				bad += rp.n_sym;
				continue;
			}
			for(int i = 0; i < rp.n_sym; i++) {
				int d = (((int)rx[i] - r.ref) % 64 + 64) % 64 - (int)tx[i];
				if(d > 32) d -= 64;
				if(d < -32) d += 64;
				total++;
				if(d < -1 || d > 1) bad++;
			}
		}
		c.check(bad == 0 && total == 4 * rp.n_sym, "frame_sync_halfbin_errors_gray_safe", "non +-1 errors " + std::to_string(bad));
	}
	// refine path must not break a clean frame
	{
		const rate_params &rp = *rate_by_index(6);
		const double fs = 4 * ELRS_BW_HZ;
		std::vector<uint16_t> tx = random_symbols(rng, rp.n_sym, rp.sf);
		size_t len = (size_t)llround((rp.frame_offset_symbols + rp.n_sym + 4) * rp.t_sym_s() * fs) + (size_t)(0.3e-3 * fs);
		std::vector<cf> buf(len, cf(0, 0));
		synth_frame(rp, tx, fs, 2.0 * ELRS_BW_HZ / 64, 0.3e-3, buf);   // 2 bins CFO
		add_awgn(buf, 0.1, rng);
		frame_sync fsy(rp, 4, true);
		std::vector<uint16_t> rx;
		frame_sync_result r = fsy.run(buf.data(), buf.size(), rx);
		int ok = r.found ? count_symbol_matches(rx, r.ref, tx, rp.sf) : 0;
		c.check(r.found && ok == rp.n_sym, "frame_sync_refine_sf6", "ok " + std::to_string(ok) + " cfo_est " + std::to_string(r.cfo_hz));
	}

	// 6. rate from duration
	{
		bool ok = rate_index_from_duration(1.506e-3) == 4 && rate_index_from_duration(3.328e-3) == 6 &&
		          rate_index_from_duration(5.868e-3) == 7 && rate_index_from_duration(10.791e-3) == 9 &&
		          rate_index_from_duration(0.5e-3) == 0 && rate_index_from_duration(20e-3) == 0 && rate_index_from_duration(7.6e-3) == 0;
		c.check(ok, "rate_index_from_duration");
	}

	// 7. rx_chain end-to-end: 25 MSps synthetic stream, 3 bursts on 3 channels (offsets
	//    +3.7, -5, +0.6 MHz around fc = 2415 MHz), both conj signs, auto rate and forced rate,
	//    fed through make_buffer_source in device-sized blocks; channel hint path
	for(int conj_sign = 0; conj_sign < 2; conj_sign++) {
		const rate_params &rp = *rate_by_index(6);
		const double fs = 25e6, fc = 2415e6;
		std::vector<wide_burst> bursts = {
		    {2.0e-3, 18, random_symbols(rng, rp.n_sym, rp.sf)},   // 2418.4 = +3.4 MHz
		    {6.0e-3, 9, random_symbols(rng, rp.n_sym, rp.sf)},    // 2409.4 = -5.6 MHz
		    {10.0e-3, 15, random_symbols(rng, rp.n_sym, rp.sf)},  // 2415.4 = +0.4 MHz
		};
		std::vector<int16_t> stream;
		synth_wide(rp, bursts, fs, fc, 16e-3, 3000.0, 250.0, rng, stream, conj_sign == 1);
		rx_chain_config cfg;
		cfg.fs_hz = fs;
		cfg.fc_hz = fc;
		cfg.os = 4;
		cfg.rate_index = 0;
		cfg.conj = conj_sign == 1;
		rx_chain chain(cfg);
		struct got_t { int channel, predicted, rate; bool found; int ok; double t; };
		std::vector<got_t> got;
		chain.set_on_burst([&](const rx_burst &b) {
			got_t g;
			g.channel = b.channel;
			g.predicted = b.channel_predicted;
			g.rate = b.rate_index;
			g.found = b.sync.found;
			g.t = b.t_start_s;
			g.ok = 0;
			for(const wide_burst &wb : bursts) {
				if(std::fabs(wb.t0 - b.t_start_s) < 0.5e-3) g.ok = b.sync.found ? count_symbol_matches(b.symbols, b.sync.ref, wb.syms, rp.sf) : 0;
			}
			got.push_back(g);
			// hand a hint for the next burst (exercises the reentrant knob path)
			chain.set_channel_hint(b.t_start_s + 3.5e-3, b.t_start_s + 4.5e-3, 9);
		});
		sample_source *src = make_buffer_source(stream.data(), stream.size() / 2, fs, fc, 65536);
		std::atomic<bool> stop{false};
		src->run(stop, [&](const source_block &blk) { chain.feed(blk.iq, blk.count, blk.sample_index, blk.ts_ns); });
		delete src;
		rx_chain::stats st;
		chain.get_stats(st);
		bool ok = got.size() == 3;
		std::string detail = "bursts " + std::to_string(st.bursts) + " found " + std::to_string(st.demod_found) + ":";
		for(size_t i = 0; i < got.size() && i < 3; i++) {
			ok = ok && got[i].found && got[i].channel == bursts[i].channel && got[i].rate == 6 && got[i].ok == rp.n_sym;
			detail += " ch" + std::to_string(got[i].channel) + "/" + std::to_string(bursts[i].channel) + " sym " + std::to_string(got[i].ok) + "/" + std::to_string(rp.n_sym);
		}
		if(got.size() >= 2) ok = ok && got[1].predicted == 9;
		char name[64];
		snprintf(name, sizeof name, "rx_chain_e2e_conj%d", conj_sign);
		c.check(ok, name, detail);
		c.check(st.samples == stream.size() / 2 && st.ring_overruns == 0, ("rx_chain_stats_conj" + std::to_string(conj_sign)).c_str(), "samples " + std::to_string(st.samples));
	}
	// 7b. auto_conj: stream with inverted IQ, chain configured for the wrong sign -> flips
	{
		const rate_params &rp = *rate_by_index(4);
		const double fs = 25e6, fc = 2441.4e6;
		std::vector<wide_burst> bursts = {{2.0e-3, 41, random_symbols(rng, rp.n_sym, rp.sf)}, {4.0e-3, 33, random_symbols(rng, rp.n_sym, rp.sf)}};
		std::vector<int16_t> stream;
		synth_wide(rp, bursts, fs, fc, 8e-3, 3000.0, 250.0, rng, stream, true);
		rx_chain_config cfg;
		cfg.fs_hz = fs;
		cfg.fc_hz = fc;
		cfg.rate_index = 4;
		cfg.conj = false;
		cfg.auto_conj = true;
		rx_chain chain(cfg);
		int found = 0, okc = 0;
		chain.set_on_burst([&](const rx_burst &b) {
			if(b.sync.found) found++;
			int m = b.sync.found ? count_symbol_matches(b.symbols, b.sync.ref, bursts[found - 1].syms, rp.sf) : -1;
			if(getenv("ELRSD_RX_DEBUG")) printf("  auto_conj burst t %.4f found %d conj %d ch %d (pred %d) off %.0f match %d\n", b.t_start_s, b.sync.found, b.conj, b.channel, b.channel_predicted, b.freq_offset_hz, m);
			if(b.sync.found && b.conj) okc += m == rp.n_sym;
		});
		chain.feed(stream.data(), stream.size() / 2, 0, -1);
		c.check(found == 2 && okc == 2 && chain.config().conj == true, "rx_chain_auto_conj", "found " + std::to_string(found));
	}
	// 7c. SF8 (rate 9, hop 2) and SF7 through the wide chain at fs 25e6 with forced rate
	for(int idx : {7, 9}) {
		const rate_params &rp = *rate_by_index(idx);
		const double fs = 25e6, fc = 2415e6;
		std::vector<wide_burst> bursts = {{2.0e-3, 20, random_symbols(rng, rp.n_sym, rp.sf)}};
		std::vector<int16_t> stream;
		synth_wide(rp, bursts, fs, fc, 18e-3, 3000.0, 300.0, rng, stream, false);
		rx_chain_config cfg;
		cfg.fs_hz = fs;
		cfg.fc_hz = fc;
		cfg.rate_index = 0;
		rx_chain chain(cfg);
		int ok = -1, rate = 0;
		chain.set_on_burst([&](const rx_burst &b) {
			rate = b.rate_index;
			ok = b.sync.found ? count_symbol_matches(b.symbols, b.sync.ref, bursts[0].syms, rp.sf) : -1;
		});
		for(size_t off = 0; off < stream.size() / 2; off += 65536) {
			size_t n = std::min<size_t>(65536, stream.size() / 2 - off);
			chain.feed(stream.data() + 2 * off, n, off, -1);
		}
		char name[64];
		snprintf(name, sizeof name, "rx_chain_wide_rate%d_sf%d", idx, rp.sf);
		c.check(ok == rp.n_sym && rate == idx, name, "rate " + std::to_string(rate) + " sym " + std::to_string(ok) + "/" + std::to_string(rp.n_sym));
	}

	printf("rx selftest: %d passed, %d failed\n", c.passes, c.fails);
	return c.fails;
}

// ---------------------------------------------------------------- bench ----------------
int run_bench_rx(double seconds) {
	if(seconds <= 0) seconds = 2;
	std::mt19937 rng(1);
	const rate_params &rp = *rate_by_index(6);
	const double fs = 25e6, fc = 2415e6;
	// one second of signal: 250 bursts on random in-window channels 3..26
	std::vector<wide_burst> bursts;
	for(int k = 0; k < 250; k++) bursts.push_back({k * 4e-3 + 0.7e-3, 3 + (int)(rng() % 24), random_symbols(rng, rp.n_sym, rp.sf)});
	std::vector<int16_t> stream;
	printf("bench: synthesizing 1 s of 25 MSps (250 SF6 bursts)...\n");
	synth_wide(rp, bursts, fs, fc, 1.0, 3000.0, 250.0, rng, stream);
	rx_chain_config cfg;
	cfg.fs_hz = fs;
	cfg.fc_hz = fc;
	cfg.rate_index = 6;
	rx_chain chain(cfg);
	uint64_t found = 0, sym_ok = 0;
	chain.set_on_burst([&](const rx_burst &b) {
		if(!b.sync.found) return;
		found++;
		int k = (int)llround((b.t_start_s - 0.7e-3) / 4e-3);
		k %= 250;
		if(k >= 0) sym_ok += count_symbol_matches(b.symbols, b.sync.ref, bursts[k].syms, rp.sf) == rp.n_sym;
	});
	const size_t total = stream.size() / 2;
	const size_t blk = 65536;
	int reps = (int)std::ceil(seconds);
	auto t0 = std::chrono::steady_clock::now();
	uint64_t index = 0;
	for(int r = 0; r < reps; r++) {
		for(size_t off = 0; off < total; off += blk) {
			size_t n = std::min(blk, total - off);
			chain.feed(stream.data() + 2 * off, n, index, -1);
			index += n;
		}
	}
	double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	rx_chain::stats st;
	chain.get_stats(st);
	double sig_s = (double)index / fs;
	printf("bench: signal %.2f s, samples %llu, bursts %llu, found %llu, payload-exact %llu, wall %.3f s, realtime_factor %.2f, cpu_busy %.2f\n",
	       sig_s, (unsigned long long)st.samples, (unsigned long long)st.bursts, (unsigned long long)found,
	       (unsigned long long)sym_ok, wall, sig_s / wall, st.cpu_busy);
	return 0;
}

} // namespace elrsd

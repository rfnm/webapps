// cellsync: acceptance gate. Proves everything provable without hardware:
// sequences, resampler, estimator accuracy against a synthesized downlink with a
// known injected reference error (CFO + SCO tied together like the real board), the
// CFO<->timing-drift identity, multi-cell tracking, and the closed steering loop
// with the exact 0.100000039438 ppb step plant.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

#include "acquire.h"
#include "feed_chain.h"
#include "resampler.h"
#include "steer.h"
#include "synth.h"
#include "tracker.h"

namespace cellsync {

static int g_fails = 0;

static void check(bool ok, const char *name, const char *detail = "") {
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail[0] ? ": " : "", detail);
	if(!ok) {
		g_fails++;
	}
	fflush(stdout);
}

// ---- component tests --------------------------------------------------------------------

static void test_acquire() {
	auto o = acquire_order(806.0, k_acquire_mhz, k_acquire_n);
	check(o.size() == k_acquire_n && o[0] == 806.0, "acquire order: last-good first, table dup removed");
	auto o2 = acquire_order(0, k_acquire_mhz, k_acquire_n);
	check(o2.size() == k_acquire_n && o2[0] == k_acquire_mhz[0], "acquire order: no last-good = table order");
	auto o3 = acquire_order(1842.5, k_acquire_mhz, k_acquire_n);
	check(o3.size() == k_acquire_n + 1 && o3[0] == 1842.5, "acquire order: off-table last-good prepends");
	check(acquire_order(99.0, k_acquire_mhz, k_acquire_n)[0] == k_acquire_mhz[0], "acquire order: insane last-good ignored");
	const char *p = "./selftest_last_good.tmp";
	check(last_good_store(p, 796.0, 1.5), "last-good store");
	last_good lg = last_good_load(p);
	check(lg.mhz == 796.0 && lg.have_snr && std::fabs(lg.snr_db - 1.5) < 0.01, "last-good load round-trip w/ snr");
	FILE *jf = fopen(p, "w");
	fprintf(jf, "751.0\n");   // pre-snr file format
	fclose(jf);
	lg = last_good_load(p);
	check(lg.mhz == 751.0 && !lg.have_snr, "last-good legacy single-field accepted");
	jf = fopen(p, "w");
	fprintf(jf, "junk");
	fclose(jf);
	check(last_good_load(p).mhz == 0, "last-good rejects junk");
	unlink(p);
	check(last_good_load("./does_not_exist_lgf").mhz == 0, "last-good missing file = none");

	// survey fast path: early commit iff the last-good freq re-confirms near strength
	last_good st;
	st.mhz = 751.0;
	st.snr_db = 1.0;
	st.have_snr = true;
	check(survey_commit_early(st, 751.0, -4.0), "commit-early: within 6 dB of stored snr");
	check(!survey_commit_early(st, 751.0, -5.5), "commit-early: faded beyond 6 dB = full survey");
	check(!survey_commit_early(st, 763.0, 5.0), "commit-early: only at the last-good freq");
	last_good legacy;
	legacy.mhz = 751.0;
	check(survey_commit_early(legacy, 751.0, -5.0) && !survey_commit_early(legacy, 751.0, -7.0),
	      "commit-early: legacy file = absolute -6 dB floor");
	check(!survey_commit_early(last_good(), 751.0, 10.0), "commit-early: no last-good = never");
}

static void test_nco() {
	// phasor recurrence vs the exact rotation, across several 4096-sample reseeds
	nco n(300e3, 2.4e6);
	const size_t N = 20000;
	std::vector<cf> x(N, cf(1.0f, 0.0f));
	n.mix(x.data(), x.size());
	double step = -2.0 * M_PI * 300e3 / 2.4e6, worst = 0;
	for(size_t i = 0; i < N; i += 97) {
		cf ref((float)cos(step * (double)i), (float)sin(step * (double)i));
		worst = std::max(worst, (double)std::abs(x[i] - ref));
	}
	char d[48];
	snprintf(d, sizeof(d), "worst |err| %.2e", worst);
	check(worst < 2e-3, "nco_phasor_accuracy", d);
}

static void test_gap() {
	// feed_chain gap policy against a synthesized cell: a small hole zero-filled at
	// TRUE length keeps the cell (the timeline stays honest, trackers just miss a
	// beat); a flush leaves an engine that re-confirms from scratch.
	const double f_lo = 751.3e6;
	dl_synth syn(301, 25, 11);
	std::vector<cf> x = syn.frames(302);
	add_awgn(x, 10.0f, 12);
	sync_engine eng(f_lo, false);
	feed_chain chain(0.0, 1920000.0, 1, 1, eng);   // 1:1 chain at the engine rate
	size_t half = x.size() / 2, off = 0;
	while(off < half) {
		size_t c = std::min<size_t>(4096, half - off);
		chain(x.data() + off, c, 0);
		off += c;
	}
	bool found_pre = false;
	for(const auto &m : eng.cells()) {
		found_pre |= m.pci == 301;
	}
	check(found_pre, "gap_pre_confirm");
	// 50 ms hole: resume at the true post-hole position, gap reported honestly
	size_t hole = (size_t)(0.05 * 1920000.0);
	off = half + hole;
	bool first = true;
	while(off < x.size()) {
		size_t c = std::min<size_t>(4096, x.size() - off);
		chain(x.data() + off, c, first ? hole : 0);
		first = false;
		off += c;
	}
	bool found_post = false;
	for(const auto &m : eng.cells()) {
		found_post |= m.pci == 301;
	}
	check(found_post, "gap_zfill_cell_survives");
	eng.flush_acquire();
	check(eng.cells().empty() && !eng.tracking(), "gap_flush_clears");
	std::vector<cf> y = syn.frames(302);
	add_awgn(y, 10.0f, 13);
	off = 0;
	while(off < y.size()) {
		size_t c = std::min<size_t>(4096, y.size() - off);
		chain(y.data() + off, c, 0);
		off += c;
	}
	bool found_re = false;
	for(const auto &m : eng.cells()) {
		found_re |= m.pci == 301;
	}
	check(found_re, "gap_flush_reconfirms");
}

static void test_fft() {
	fft_plan f(128);
	std::vector<cf> in(128, cf(0, 0)), out(128);
	in[0] = cf(1, 0);
	f.run(in.data(), out.data());
	float err = 0;
	for(const auto &v : out) {
		err += std::abs(v - cf(1, 0));
	}
	check(err < 1e-3f, "fft_impulse");
}

static void test_resampler() {
	// 100 kHz tone at 2.4M must come out at 100 kHz at 1.92M with clean spectrum
	const double fs_in = 2.4e6, ftone = 100e3;
	std::vector<cf> in(48000);
	for(size_t i = 0; i < in.size(); i++) {
		double ph = 2 * M_PI * ftone * i / fs_in;
		in[i] = cf((float)cos(ph), (float)sin(ph));
	}
	resampler rs(4, 5);
	std::vector<cf> out;
	rs.process(in.data(), in.size(), out);
	check(std::abs((double)out.size() - in.size() * 4.0 / 5.0) < 8, "resampler_count");
	// measure output frequency by phase slope over the steady-state middle
	double acc = 0;
	int n = 0;
	for(size_t i = out.size() / 4; i < 3 * out.size() / 4; i++) {
		acc += std::arg(out[i + 1] * std::conj(out[i]));
		n++;
	}
	double fmeas = acc / n * 1.92e6 / (2 * M_PI);
	char d[64];
	snprintf(d, sizeof(d), "%.1f Hz", fmeas);
	check(std::fabs(fmeas - ftone) < 5.0, "resampler_tone", d);
	// amplitude flatness (image/aliasing would modulate the envelope)
	float amin = 1e9, amax = 0;
	for(size_t i = out.size() / 4; i < 3 * out.size() / 4; i++) {
		float a = std::abs(out[i]);
		amin = std::min(amin, a);
		amax = std::max(amax, a);
	}
	snprintf(d, sizeof(d), "ripple %.4f", amax / std::max(amin, 1e-9f));
	check(amax / amin < 1.02f, "resampler_flatness", d);
}

static void test_pss() {
	for(int nid2 = 0; nid2 < 3; nid2++) {
		std::vector<cf> t = pss_time(nid2);
		cf self(0, 0);
		for(int i = 0; i < FFT_N; i++) {
			self += t[i] * std::conj(t[i]);
		}
		check(std::fabs(std::abs(self) - 1.0f) < 1e-3f, "pss_unit_energy");
		for(int other = 0; other < 3; other++) {
			if(other == nid2) {
				continue;
			}
			std::vector<cf> o = pss_time(other);
			cf cross(0, 0);
			for(int i = 0; i < FFT_N; i++) {
				cross += t[i] * std::conj(o[i]);
			}
			if(other > nid2) {
				char d[48];
				snprintf(d, sizeof(d), "roots %d/%d |x|=%.3f", nid2, other, std::abs(cross));
				// roots 25/34 legitimately reach ~0.38 at zero lag - detection never
				// relies on cross-root suppression alone (SSS validates the PCI)
				check(std::abs(cross) < 0.45f, "pss_cross_low", d);
			}
		}
	}
}

static void test_sss() {
	// encode -> decode across a spread of NID1, both half-frames, with a random
	// per-bin phase channel (decode equalizes off PSS through the same channel)
	fft_plan f128(FFT_N);
	std::mt19937 rng(99);
	std::uniform_real_distribution<float> ud(-M_PI, M_PI);
	for(int nid1 : { 0, 17, 84, 167 }) {
		for(int half = 0; half < 2; half++) {
			int nid2 = (nid1 + half) % 3;
			cf ch[FFT_N];
			for(int i = 0; i < FFT_N; i++) {
				ch[i] = std::polar(1.0f + 0.3f * (i % 5) / 5.0f, ud(rng));
			}
			auto make_sym = [&](bool pss) {
				cf bins[FFT_N];
				std::fill(bins, bins + FFT_N, cf(0, 0));
				if(pss) {
					auto d = pss_freq(nid2);
					map62(d.data(), bins);
				} else {
					auto d = sss_seq(nid1, nid2, half);
					for(int n = 0; n < 62; n++) {
						int k = n < 31 ? n - 31 : n - 30;
						bins[(k + FFT_N) % FFT_N] = cf(d[n], 0);
					}
				}
				for(int i = 0; i < FFT_N; i++) {
					bins[i] *= ch[i];
				}
				// to time domain
				std::vector<cf> tmp(FFT_N), td(FFT_N);
				for(int i = 0; i < FFT_N; i++) {
					tmp[i] = std::conj(bins[i]);
				}
				f128.run(tmp.data(), td.data());
				for(int i = 0; i < FFT_N; i++) {
					td[i] = std::conj(td[i]) * (1.0f / FFT_N);
				}
				return td;
			};
			std::vector<cf> psym = make_sym(true), ssym = make_sym(false);
			// decode via the tracker's own path: re-run FFTs inside sss_decode-like flow
			// (sss_decode is static in tracker.cpp, so replicate the coherent decode here)
			std::vector<cf> pb(FFT_N), sb(FFT_N);
			f128.run(psym.data(), pb.data());
			f128.run(ssym.data(), sb.data());
			auto pref = pss_freq(nid2);
			float best = 0, metric_true = 0;
			int best_h = -1;
			for(int h = 0; h < 336; h++) {
				auto d = sss_seq(h / 2, nid2, h & 1);
				cf corr(0, 0);
				float pw = 0;
				for(int n = 0; n < 62; n++) {
					int k = n < 31 ? n - 31 : n - 30;
					cf eq = sb[(k + FFT_N) % FFT_N] * std::conj(pb[(k + FFT_N) % FFT_N]) * pref[n];
					corr += eq * d[n];
					pw += std::norm(eq);
				}
				float m = std::norm(corr) / (pw * 62.0f);
				if(m > best) {
					best = m;
					best_h = h;
				}
				if(h / 2 == nid1 && (h & 1) == half) {
					metric_true = m;
				}
			}
			char det[64];
			snprintf(det, sizeof(det), "nid1 %d half %d metric %.2f", nid1, half, metric_true);
			check(best_h == nid1 * 2 + half && best > 0.5f, "sss_roundtrip", det);
		}
	}
}

static void test_crs_layout() {
	// positions: 6 apart, never DC, counts stable, m' consecutive (descramble relies on it)
	for(int nrb : NRB_CANDIDATES) {
		auto p0 = crs_positions(123, nrb, 0, -36, 36);
		bool ok = p0.size() >= 11 && p0.size() <= 16;
		for(size_t i = 0; i < p0.size(); i++) {
			ok = ok && p0[i].rel != 0;
			if(i) {
				ok = ok && p0[i].mprime == p0[i - 1].mprime + 1;
			}
		}
		check(ok, "crs_layout", ("nrb " + std::to_string(nrb) + " n=" + std::to_string(p0.size())).c_str());
	}
}

// ---- end-to-end: synthesized downlink through the full estimator -------------------------

struct e2e_result {
	bool found = false;
	cell_meas m;
	int n_cells = 0;
};

static e2e_result run_e2e(const char *tag, int pci, int nrb, double eps_ppb, float snr_db, double seconds,
                          int pci2 = -1, uint32_t seed = 7) {
	const double f_lo = 751.3e6;
	int n_frames = (int)(seconds * 100) + 2;
	dl_synth syn(pci, nrb, seed);
	std::vector<cf> x = syn.frames(n_frames);
	if(pci2 >= 0) {
		dl_synth syn2(pci2, nrb, seed + 13);
		std::vector<cf> x2 = syn2.frames(n_frames);
		for(size_t i = 0; i + 3000 < x.size() && i < x2.size(); i++) {
			x[i + 3000] += 0.8f * x2[i];
		}
	}
	apply_channel(x, 0.25f, 0.7f, 2);
	apply_cfo(x, -eps_ppb * 1e-9 * f_lo, FS_SYNC);
	x = apply_sco(x, eps_ppb * 1e-9);
	add_awgn(x, snr_db, seed + 1);

	sync_engine eng(f_lo, false);
	for(size_t off = 0; off < x.size(); off += 4096) {
		eng.feed(x.data() + off, std::min<size_t>(4096, x.size() - off));
	}
	e2e_result r;
	auto cells = eng.cells();
	r.n_cells = (int)cells.size();
	for(const auto &m : cells) {
		if(m.pci == pci) {
			r.found = true;
			r.m = m;
		}
	}
	char d[160];
	if(r.found) {
		snprintf(d, sizeof(d), "pci %d %s snr %.1f dB offset %+0.3f ppb (want %+0.1f) slots %llu",
		         r.m.pci, r.m.wide ? "wide" : "1.4M", r.m.snr_db, r.m.offset_ppb, eps_ppb, (unsigned long long)r.m.slots);
	} else {
		snprintf(d, sizeof(d), "cell not found (%d tracked)", r.n_cells);
	}
	check(r.found, tag, d);
	return r;
}

static void test_e2e() {
	// clean acquisition + fine accuracy
	e2e_result r = run_e2e("e2e_clean_sf50", 301, 50, 500.0, 10.0f, 3.0);
	if(r.found) {
		check(r.m.wide, "e2e_wide_detect");
		char d[64];
		snprintf(d, sizeof(d), "err %+0.2f ppb", r.m.offset_ppb - 500.0);
		check(std::fabs(r.m.offset_ppb - 500.0) < 5.0, "e2e_offset_5ppb", d);
	}

	// negative reference error, narrowband cell, modest SNR
	r = run_e2e("e2e_nrb6", 44, 6, -730.0, 5.0f, 3.0);
	if(r.found) {
		check(!r.m.wide, "e2e_narrow_detect");
		check(std::fabs(r.m.offset_ppb + 730.0) < 8.0, "e2e_nrb6_offset");
	}

	// low SNR
	r = run_e2e("e2e_snr0", 208, 25, 250.0, 0.0f, 4.0);
	if(r.found) {
		check(std::fabs(r.m.offset_ppb - 250.0) < 15.0, "e2e_snr0_offset");
	}

	// large initial error: exercises the fgrid + integer-subcarrier path (8 ppm at
	// 751 MHz = -6.0 kHz CFO)
	r = run_e2e("e2e_8ppm", 150, 50, 8000.0, 8.0f, 4.0);
	if(r.found) {
		char d[64];
		snprintf(d, sizeof(d), "err %+0.2f ppb", r.m.offset_ppb - 8000.0);
		check(std::fabs(r.m.offset_ppb - 8000.0) < 10.0, "e2e_8ppm_offset", d);
	}

	// two cells on one carrier (same reference truth), both must track
	r = run_e2e("e2e_two_cells", 77, 25, 80.0, 12.0f, 14.0, 210);
	check(r.n_cells >= 2, "e2e_second_cell", ("tracked " + std::to_string(r.n_cells)).c_str());

	// long run: timing-drift estimator must agree with the CFO view (same epsilon)
	r = run_e2e("e2e_identity_35s", 385, 50, 150.0, 15.0f, 35.0);
	if(r.found) {
		char d[96];
		snprintf(d, sizeof(d), "cfo-view %+0.2f ppb, timing-view %+0.1f ppb (valid=%d)",
		         r.m.offset_ppb, r.m.tim_ppb, (int)r.m.tim_valid);
		check(r.m.tim_valid, "e2e_timing_valid", d);
		if(r.m.tim_valid) {
			check(std::fabs(r.m.tim_ppb - 150.0) < 40.0, "e2e_timing_identity", d);
		}
		check(std::fabs(r.m.offset_ppb - 150.0) < 3.0, "e2e_35s_offset_3ppb", d);
	}

	// daemon-path variant: synth 1.92M -> upsample 5/4 to 2.4M -> the exact live
	// pipeline (NCO 0 + 4/5) -> engine
	{
		const double f_lo = 751.3e6;
		dl_synth syn(63, 25, 21);
		std::vector<cf> x = syn.frames(302);
		apply_cfo(x, -300e-9 * f_lo, FS_SYNC);
		x = apply_sco(x, 300e-9);
		add_awgn(x, 10.0f, 5);
		resampler up(5, 4);
		std::vector<cf> x24;
		x24.reserve(x.size() * 5 / 4 + 16);
		up.process(x.data(), x.size(), x24);
		sync_engine eng(f_lo, false);
		resampler down(4, 5);
		std::vector<cf> res;
		for(size_t off = 0; off < x24.size(); off += 4096) {
			size_t n = std::min<size_t>(4096, x24.size() - off);
			res.clear();
			down.process(x24.data() + off, n, res);
			eng.feed(res.data(), res.size());
		}
		bool found = false;
		double got = 0;
		for(const auto &m : eng.cells()) {
			if(m.pci == 63) {
				found = true;
				got = m.offset_ppb;
			}
		}
		char d[64];
		snprintf(d, sizeof(d), "offset %+0.2f ppb (want +300)", got);
		check(found && std::fabs(got - 300.0) < 6.0, "e2e_resampler_path", d);
	}
}

// ---- closed-loop steering simulation -------------------------------------------------------

struct sim_out {
	double resid_at_100 = 1e9;
	double steady_rms = 1e9;
	int32_t final_steps = 0;
	int final_sign = 0;
	bool holdover_seen = false;
	double resid_after_holdover = 1e9;
};

static sim_out simulate(double eps0_ppb, double noise_ppb, int sign_true, bool sign_probe, bool print) {
	dco_ctl dco;
	dco.mock = true;
	steer_loop::config cfg;
	cfg.tau_s = 120;
	cfg.period_s = 10;
	cfg.sign_probe = sign_probe;
	steer_loop loop(&dco, cfg);
	std::mt19937 rng(4242);
	std::normal_distribution<double> nd(0.0, noise_ppb);

	sim_out out;
	double steady_acc = 0;
	int steady_n = 0;
	const int N = 260;
	for(int k = 0; k < N; k++) {
		double aging = 0.02 * k;                                     // ~0.17 ppb/day-scale drift
		double true_eps = eps0_ppb + aging + dco.steps * dco.step_ppb * sign_true;
		bool valid = true;
		if(k >= 160 && k < 175) {
			valid = false;                                           // signal outage window
		}
		double meas = true_eps + nd(rng);
		loop.update(valid ? meas : 0, valid);
		if(loop.state() == steer_loop::HOLDOVER) {
			out.holdover_seen = true;
		}
		if(k == 100) {
			out.resid_at_100 = true_eps;
		}
		if(k >= 120 && k < 160) {
			steady_acc += true_eps * true_eps;
			steady_n++;
		}
		if(k == N - 1) {
			out.resid_after_holdover = true_eps;
		}
		if(print && k % 10 == 0) {
			printf("sim k=%3d state=%-9s true=%+9.2f ppb meas=%+9.2f steps=%+7d\n",
			       k, loop.state_name(), true_eps, meas, dco.steps);
		}
	}
	out.steady_rms = steady_n ? sqrt(steady_acc / steady_n) : 1e9;
	out.final_steps = dco.steps;
	out.final_sign = dco.sign;
	return out;
}

static void test_sim() {
	sim_out s = simulate(2000.0, 2.0, 1, false, false);
	char d[96];
	snprintf(d, sizeof(d), "resid@100 %+0.2f ppb steady-rms %.2f steps %d", s.resid_at_100, s.steady_rms, s.final_steps);
	check(std::fabs(s.resid_at_100) < 5.0, "sim_converge_2ppm", d);
	check(s.steady_rms < 4.0, "sim_steady_rms", d);
	check(std::fabs(s.final_steps * 0.1 + 2000.0) < 100.0, "sim_steps_plausible", d);
	check(s.holdover_seen && std::fabs(s.resid_after_holdover) < 6.0, "sim_holdover_recovery");

	// inverted actuator: the sign probe must detect and flip
	sim_out f = simulate(1500.0, 2.0, -1, true, false);
	snprintf(d, sizeof(d), "sign %+d resid %+0.2f", f.final_sign, f.resid_at_100);
	check(f.final_sign == -1 && std::fabs(f.resid_at_100) < 10.0, "sim_sign_probe_flip", d);

	// dco clamp semantics
	dco_ctl dc;
	dc.mock = true;
	// default authority is +-2 ppm (20000 steps) - the chip's +-100000 is opt-in via
	// --dco-range-ppm; factory offsets are 1-2 ppm at worst
	dc.write(250000);
	check(dc.steps == 20000, "dco_clamp_high");
	dc.write(-250000);
	check(dc.steps == -20000, "dco_clamp_low");
	dc.range_steps = 100000;
	dc.write(250000);
	check(dc.steps == 100000, "dco_clamp_widened");
	dc.write(0);
	dc.range_steps = 20000;

	// weighted median robustness: one femtocell lying by 200 ppb must not move us
	std::vector<std::pair<double, double>> vw = { { 1.0, 900 }, { 1.4, 800 }, { 201.0, 300 } };
	double med = weighted_median(vw);
	check(med < 2.0, "median_femtocell_reject", (std::to_string(med) + " ppb").c_str());
}

int run_selftest() {
	test_acquire();
	test_nco();
	test_fft();
	test_resampler();
	test_pss();
	test_sss();
	test_crs_layout();
	test_e2e();
	test_sim();
	test_gap();
	printf(g_fails ? "SELFTEST FAILED (%d)\n" : "SELFTEST OK\n", g_fails);
	return g_fails ? 1 : 0;
}

int run_sim(bool quiet) {
	simulate(2000.0, 2.0, 1, true, !quiet);
	return 0;
}

} // namespace cellsync

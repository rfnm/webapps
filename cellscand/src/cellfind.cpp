#include "cellfind.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "bandplan.h"
#include "lte_seq.h"

static bool cf_debug() {
	static int d = -1;
	if(d < 0) {
		d = getenv("CELLSCAN_DEBUG") ? 1 : 0;
	}
	return d != 0;
}

namespace cellscan {

using namespace cellsync;

// Build the matched-filter template for a NID2 PSS: the same construction the pdsch.cpp
// testbench uses (freq map -> IFFT -> normalise), kept here so the search is self-contained.
static std::vector<cf> pss_template(int nid2, const wb_num &g, const fft_plan &plan) {
	std::vector<cf> tpl(g.fft, cf(0, 0));
	auto d = pss_freq(nid2);
	for(int i = 0; i < 62; i++) {
		int k = (i < 31) ? (i - 31) : (i - 30);
		tpl[(k + g.fft) % g.fft] = std::conj(d[i]);
	}
	std::vector<cf> t(g.fft);
	plan.run(tpl.data(), t.data());
	double e = 0;
	for(auto &v : t) {
		e += std::norm(v);
	}
	float inv = 1.0f / (float)std::sqrt(e);
	for(size_t i = 0; i < t.size(); i++) {
		tpl[i] = std::conj(t[i]) * inv;
	}
	return tpl;
}

int64_t pss_search(const std::vector<cf> &x, const wb_num &g, const fft_plan &plan,
                   int nid2, int64_t search_end, double *metric) {
	std::vector<cf> tpl = pss_template(nid2, g, plan);
	int64_t end = std::min<int64_t>(search_end, (int64_t)x.size() - g.fft);
	double best = 0;
	int64_t bp = -1;
	for(int64_t i = 0; i < end; i++) {
		cf c(0, 0);
		double p = 0;
		const cf *xi = &x[i];
		for(int u = 0; u < g.fft; u++) {
			c += xi[u] * std::conj(tpl[u]);
			p += std::norm(xi[u]);
		}
		double m = std::norm(c) / (p + 1e-30);
		if(m > best) {
			best = m;
			bp = i;
		}
	}
	if(metric) {
		*metric = best;
	}
	return bp;
}

double remove_cfo(std::vector<cf> &x, const wb_num &g, int64_t slot0) {
	// CP correlation over ~10 frames (the CP-correlation derotation removes even the
	// -805 Hz residual of a pre-discipline capture). Sum conj(x[t]) * x[t+fft] over every CP.
	cf acc(0, 0);
	for(int fr = 0; fr < 10; fr++) {
		for(int sl = 0; sl < 20; sl++) {
			int64_t ss = slot0 + (int64_t)fr * g.frame + (int64_t)sl * g.slot;
			if(ss < 0 || (size_t)(ss + g.slot) > x.size()) {
				continue;
			}
			for(int l = 0; l < 7; l++) {
				int64_t d0 = ss + g.sym_off(l) - g.cp_len(l);
				if(d0 < 0 || (size_t)(d0 + g.cp_len(l) + g.fft) > x.size()) {
					continue;
				}
				for(int t = 0; t < g.cp_len(l); t++) {
					acc += x[d0 + t] * std::conj(x[d0 + t + g.fft]);
				}
			}
		}
	}
	double ph = std::atan2(acc.imag(), acc.real());
	double df = -ph * g.fs / (2.0 * M_PI * g.fft);
	double w = -2.0 * M_PI * df / g.fs;
	for(size_t i = 0; i < x.size(); i++) {
		double a = w * (double)i;
		x[i] *= cf((float)std::cos(a), (float)std::sin(a));
	}
	return df;
}

double sweep_probe_level(const std::vector<cf> &x, const fft_plan &plan, double fs) {
	const unsigned N = plan.size();
	if(x.size() < N || fs <= 0) {
		return 0.0;
	}
	// Welch-style average: up to 384 windows spread evenly across the whole probe capture (not
	// only its head), so a fade at one instant does not decide the level. 384 x 1024 bins puts
	// the estimator noise well under a dB - more windows would buy nothing but CPU.
	size_t navg = std::min<size_t>(384, x.size() / N);
	size_t stride = x.size() / navg;
	if(stride < N) {
		stride = N;
	}
	std::vector<double> psd(N, 0.0);
	std::vector<cf> out(N);
	for(size_t w = 0; w < navg; w++) {
		plan.run(&x[w * stride], out.data());
		for(unsigned k = 0; k < N; k++) {
			psd[k] += std::norm(out[k]);
		}
	}
	double binw = fs / N;
	double in = 0, lo = 0, hi = 0;
	unsigned inn = 0, lon = 0, hin = 0;
	for(unsigned k = 0; k < N; k++) {
		double f = (k <= N / 2) ? k * binw : ((double)k - N) * binw;
		double af = std::fabs(f);
		if(af > SWEEP_DC_EXCLUDE_HZ && af <= SWEEP_INBAND_HZ) {
			in += psd[k];
			inn++;
		} else if(af >= SWEEP_FLOOR_LO_HZ && af <= SWEEP_FLOOR_HI_HZ) {
			if(f < 0) {
				lo += psd[k];
				lon++;
			} else {
				hi += psd[k];
				hin++;
			}
		}
	}
	double in_mean = inn ? in / inn : 0.0;
	double lo_mean = lon ? lo / lon : 0.0;
	double hi_mean = hin ? hi / hin : 0.0;
	// the quieter side is the floor; if one side is dead (synthetic vectors) use the other
	double floor_mean = std::min(lo_mean, hi_mean);
	if(floor_mean <= 0.0) {
		floor_mean = std::max(lo_mean, hi_mean);
	}
	const double eps = 1e-20;
	return 10.0 * std::log10((in_mean + eps) / (floor_mean + eps));
}

std::vector<found_cell> cellfind(std::vector<cf> &x, const wb_num &g, const fft_plan &plan,
                                 double *cfo_hz, std::atomic<bool> *cancel) {
	std::vector<found_cell> out;
	if((int64_t)x.size() < 2 * g.frame) {
		if(cfo_hz) {
			*cfo_hz = 0;
		}
		return out;
	}
	// 1) coarse anchor: strongest PSS of any NID2 in the first two frames
	int64_t search_end = std::min<int64_t>(2 * g.frame, (int64_t)x.size() - g.fft);
	int64_t anchor = -1;
	double anchor_m = 0;
	for(int nid2 = 0; nid2 < 3; nid2++) {
		if(cancel && cancel->load()) {
			if(cfo_hz) {
				*cfo_hz = 0;
			}
			return out;
		}
		double m = 0;
		int64_t p = pss_search(x, g, plan, nid2, search_end, &m);
		if(p >= 0 && m > anchor_m) {
			anchor_m = m;
			anchor = p;
		}
	}
	if(cf_debug()) {
		fprintf(stderr, "[cellfind] anchor pos %lld metric %.4f (size %zu, 2*frame %lld)\n",
		        (long long)anchor, anchor_m, x.size(), (long long)(2 * g.frame));
	}
	if(anchor < 0 || anchor_m < PSS_DETECT_METRIC) {
		if(cfo_hz) {
			*cfo_hz = 0;
		}
		return out;   // no plausible PSS at all -> no_signal
	}
	int64_t slot0_anchor = anchor - g.sym_off(6);
	if(slot0_anchor < 0) {
		slot0_anchor = anchor;   // PSS near the very start; CP sum will just skip out-of-range
	}

	// 2) remove the carrier CFO once, on the anchor's symbol grid
	double df = remove_cfo(x, g, slot0_anchor);
	if(cfo_hz) {
		*cfo_hz = df;
	}

	// 3) per NID2, take the strongest PSS on the derotated signal and decode its SSS.
	// One cell per NID2 is the honest v1 scope: the reference carriers each hold a single
	// cell, and a survey that reports the dominant cell per sync colour is truthful without
	// pretending to separate co-NID2 neighbours the sync window cannot cleanly resolve.
	for(int nid2 = 0; nid2 < 3; nid2++) {
		if(cancel && cancel->load()) {
			break;
		}
		double m = 0;
		int64_t pss = pss_search(x, g, plan, nid2, search_end, &m);
		if(pss < 0 || m < PSS_DETECT_METRIC) {
			continue;
		}
		int nid1 = -1, half = 0;
		float sss_margin = 0;
		bool sss_ok = wb_sss_decode(x.data(), x.size(), pss, nid2, g, plan, &nid1, &half, &sss_margin);
		if(cf_debug()) {
			fprintf(stderr, "[cellfind] nid2 %d: pss pos %lld metric %.4f -> sss_ok %d nid1 %d half %d margin %.3f\n",
			        nid2, (long long)pss, m, (int)sss_ok, nid1, half, sss_margin);
		}
		if(!sss_ok || nid1 < 0 || sss_margin < SSS_DETECT_MARGIN) {
			continue;   // SSS not confidently decoded (margin below the trust floor)
		}
		found_cell fc;
		fc.nid2 = nid2;
		fc.nid1 = nid1;
		fc.half = half;
		fc.pci = 3 * nid1 + nid2;
		fc.pss_pos = pss;
		fc.pss_metric = m;
		// both quality signals travel with the cell: the escalation gate (job.h) needs to know
		// HOW FAR above the floors this detection sat, not just that it cleared them
		fc.sss_margin = sss_margin;
		// rough SNR from the normalised PSS metric: m ~ signal/(signal+noise) at the peak
		fc.snr_db = 10.0 * std::log10(std::max(1e-3, m) / std::max(1e-3, 1.0 - m));
		bool dup = false;
		for(const auto &e : out) {
			if(e.pci == fc.pci) {
				dup = true;
				break;
			}
		}
		if(!dup) {
			out.push_back(fc);
		}
	}
	return out;
}

} // namespace cellscan

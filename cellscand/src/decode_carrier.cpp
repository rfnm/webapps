#include "decode_carrier.h"

#include <cmath>
#include <cstring>

#include "lte_coding.h"
#include "lte_rrc.h"
#include "lte_seq.h"
#include "plmn.h"
#include "report.h"

namespace cellscan {

using namespace cellsync;

namespace {
// One radio frame's PBCH observation: the 240 REs and the four ports' channel at each of
// them, i.e. everything wb_mib_decode_n needs, cached so the frames of a 40 ms TTI can be
// combined without demodulating any of them twice. ~9.6 kB - see MIB_MAX_FRAMES.
struct pbch_obs {
	int fr = 0;              // frame index in the window, which is what fixes its quarter
	std::vector<cf> re;
	cf h[4][240];
};
} // namespace

void mib_group_runs(const std::vector<int> &fr, int h,
                    std::vector<std::pair<size_t, size_t>> &runs) {
	runs.clear();
	size_t i = 0;
	while(i < fr.size()) {
		size_t j = i + 1;
		while(j < fr.size() && fr[j] == fr[j - 1] + 1 && mib_tti(h, fr[j]) == mib_tti(h, fr[i])) {
			j++;
		}
		runs.push_back(std::make_pair(i, j));
		i = j;
	}
}

int fill_cell_from_tb(cell_result &out, const uint8_t *tb, size_t nbytes) {
	sib1_head h;
	int rc = sib1_head_decode(tb, nbytes, h);
	if(rc != 0) {
		return rc;
	}
	out.plmns.clear();
	for(const auto &p : h.plmn) {
		plmn_entry e;
		char mcc[8], mnc[8];
		snprintf(mcc, sizeof(mcc), "%d%d%d", p.mcc[0], p.mcc[1], p.mcc[2]);
		if(p.mnc_digits == 3) {
			snprintf(mnc, sizeof(mnc), "%d%d%d", p.mnc[0], p.mnc[1], p.mnc[2]);
		} else {
			snprintf(mnc, sizeof(mnc), "%d%d", p.mnc[0], p.mnc[1]);
		}
		e.mcc = mcc;
		e.mnc = mnc;
		e.reserved = p.reserved_for_operator;
		out.plmns.push_back(e);
	}
	out.tac = h.tac;
	out.eci = h.cell_identity;
	out.band_ind = h.freq_band;
	out.q_rxlevmin = h.q_rxlevmin;           // raw ASN value (-64 for the 816 cell, )
	out.q_rxlevmin_dbm = 2 * h.q_rxlevmin;   // units of 2 dB -> actual dBm (-128)
	out.si_window_ms = h.si_window_ms;
	out.cell_barred = h.cell_barred;
	out.sibs.clear();
	out.sib16 = false;
	// entry 0 of schedulingInfoList implicitly also carries SIB2 (36.331 5.2.2.7); the
	// mapping list holds the rest. Surface the union so the row lists every scheduled SIB.
	bool has_sib2 = false;
	for(size_t i = 0; i < h.sched.size(); i++) {
		if(i == 0) {
			has_sib2 = true;
		}
		for(int s : h.sched[i].sib_types) {
			out.sibs.push_back(s);
			if(s == 16) {
				out.sib16 = true;
			}
		}
	}
	if(has_sib2) {
		out.sibs.insert(out.sibs.begin(), 2);
	}
	out.sib1_ok = true;
	derive_cell(out);
	return 0;
}

// bit vector -> bytes (MSB first), for the TB the turbo decoder produces
static void bits_to_bytes(const std::vector<uint8_t> &bits, size_t nbits, std::vector<uint8_t> &out) {
	out.clear();
	for(size_t i = 0; i + 7 < nbits; i += 8) {
		int v = 0;
		for(int j = 0; j < 8; j++) {
			v = (v << 1) | (bits[i + j] & 1);
		}
		out.push_back((uint8_t)v);
	}
}

// Note a parsed SI grant for the diagnostics, deduped on the whole tuple: a 2 s burst holds
// several 80 ms windows and the same grant repeats in each, so what a reader wants is the set
// of distinct grants that were on air, not a transcript.
static void diag_note_dci(sib1_diag &d, const dci_si &s) {
	si_dci_seen e;
	e.format = s.format;
	e.localized = s.localized;
	e.rb_start = s.rb_start;
	e.n_prb = s.n_prb;
	e.mcs = s.i_tbs;
	e.rv = s.rv;
	for(const auto &p : d.dcis) {
		if(p.format == e.format && p.localized == e.localized && p.rb_start == e.rb_start &&
		   p.n_prb == e.n_prb && p.mcs == e.mcs && p.rv == e.rv) {
			return;
		}
	}
	if(d.dcis.size() < SIB1_DIAG_MAX_DCIS) {
		d.dcis.push_back(e);
	}
}

int sib1_soft_accum(sib1_soft &acc, const dci_si &s, int sfn, const std::vector<float> &llr) {
	// Two keys, and BOTH are needed. The grant, because a changed grant is a visible content
	// change. And the SI modification period, because a change of SIB1 content alone
	// (systemInfoValueTag, 36.331 §5.2.1.3) is NOT visible in the grant at all - it would
	// otherwise sum the LLRs of two different transport blocks with nothing to detect it, and
	// the buffer would never recover. The shortest legal period is
	// modificationPeriodCoeff * defaultPagingCycle = 64 frames, so keying on sfn/64 can never
	// straddle a content change whatever the cell configures. It is not a tight cap: SIB1
	// repeats in SF5 of every even SFN, so 64 frames still admits up to 32 occasions - 8 full
	// RV cycles - and the measured gain is already saturated at 4.
	//
	// A window boundary is NOT a content boundary: SIB1 repeats byte-identically through the
	// modification period and its RV cycle repeats with it. For both reference grants a single
	// occasion already carries every mother-code bit (796: K=248 -> K_w=768, 756 non-dummy,
	// against E=960), so the four RVs are informationally equivalent and every combination -
	// same-RV or cross-RV - is pure energy accumulation into the same circular buffer.
	const int mp = sfn / 64;
	if(acc.rb_start != s.rb_start || acc.n_prb != s.n_prb || acc.i_tbs != s.i_tbs ||
	   acc.n_prb_1a != s.n_prb_1a || acc.mod_period != mp) {
		acc.soft.clear();
		acc.rb_start = s.rb_start;
		acc.n_prb = s.n_prb;
		acc.i_tbs = s.i_tbs;
		acc.n_prb_1a = s.n_prb_1a;
		acc.mod_period = mp;
		acc.rv_mask = 0;
		acc.n = 0;
	}
	acc.K = (size_t)s.tbs + 24;
	acc.rv_mask |= (uint8_t)(1u << (s.rv & 3));
	turbo_rate_dematch(llr, acc.K, s.rv, acc.soft);
	return ++acc.n;
}

// How many DISTINCT redundancy versions a mask holds - what `rv_combined` has always claimed
// to mean, as opposed to "more than one occasion" (which is usually the same RV twice).
static int rv_distinct(uint8_t mask) {
	int n = 0;
	for(int i = 0; i < 4; i++) {
		n += (mask >> i) & 1;
	}
	return n;
}

int sib1_try_decode(const std::vector<float> &soft, size_t K, size_t tbs, cell_result &out) {
	std::vector<uint8_t> dec;
	turbo_decode(soft, K, 8, dec);
	if(!crc_check(dec.data(), dec.size(), CRC24A)) {
		return 1;   // ASN.1 rcs are negative, so 1 is free as the CRC verdict
	}
	std::vector<uint8_t> tb;
	bits_to_bytes(dec, tbs, tb);
	return fill_cell_from_tb(out, tb.data(), tb.size());
}

int sib1_decode_occasion(const sib1_soft &acc, const dci_si &s, const std::vector<float> &llr,
                         cell_result &out, bool *combined) {
	if(combined) {
		*combined = false;
	}
	int rc = sib1_try_decode(acc.soft, acc.K, (size_t)s.tbs, out);
	if(rc == 0) {
		if(combined) {
			*combined = rv_distinct(acc.rv_mask) > 1;
		}
		return 0;
	}
	if(acc.n <= 1) {
		return rc;   // the accumulated buffer IS this occasion, nothing left to fall back to
	}
	// Cheap insurance: one poisoned occasion in the sum would otherwise cost the whole burst
	// with no way back. This makes the burst self-healing.
	std::vector<float> solo;
	turbo_rate_dematch(llr, acc.K, s.rv, solo);
	return sib1_try_decode(solo, acc.K, (size_t)s.tbs, out) == 0 ? 0 : rc;
}

bool decode_cell(const std::vector<cf> &x, const found_cell &fc, const wb_num &g,
                 const fft_plan &plan, bool deep, cell_result &out, std::atomic<bool> *cancel) {
	out.pci = fc.pci;
	out.nid1 = fc.nid1;
	out.nid2 = fc.nid2;
	out.snr_db = fc.snr_db;
	// the two raw acquisition numbers, carried verbatim: the escalation gate (job.h) is a
	// statement about them, and without them a doubtful row cannot be judged by a reader either
	out.pss_metric = fc.pss_metric;
	out.sss_margin = fc.sss_margin;
	out.status = "pss_only";

	int64_t slot0 = fc.pss_pos - g.sym_off(6);

	// ---- MIB: seed the 40 ms/half hypothesis from the SSS, verify by decode -------------
	int64_t base = -1;
	int ports = 0, nrb = 0, sfn0 = -1, pd = -1;
	float pr = 0;
	wb_mib_stats mst;            // the counters, summed over every hypothesis of every window
	double best_metric = -1.0;   // and the best SINGLE hypothesis' mean group metric
	int hyp_order[2] = { fc.half, 1 - fc.half };
	// The deep dwell is four times the burst, but the window is bounded (MIB_MAX_FRAMES), so
	// a deep capture had three quarters of it never looked at - while the escalation that
	// produced it exists to give a pss_only cell "a second independent window of TTIs"
	// (job.cpp). Walk the capture in successive windows instead: peak memory stays one
	// window's worth of observations, and each window is that promised second look.
	// Window and half hypothesis are walked as ONE list, window 0's two halves first, so the
	// non-deep case costs exactly what it did.
	const int nwin = mib_windows(deep);
	int win_avail = 0;
	for(int k = 0; k < 2 * nwin && base < 0; k++) {
		const int wi = k / 2, hi = k % 2;
		if(hi == 0) {
			if(wi > 0 && win_avail == 0) {
				break;   // the previous window ran off the end of the capture; so will this
			}
			win_avail = 0;
		}
		int64_t b = slot0 - (int64_t)hyp_order[hi] * 5 * g.subframe;
		if(b < 0) {
			continue;   // this half is simply not in the buffer - the normal case when the
			            // PSS sits in the first frame, and never a short window
		}
		// ---- pass 1: demodulate each frame of the window ONCE and keep the observation.
		// Exactly the work the old loop already did per frame, only kept instead of thrown
		// away after one failed decode - the four frames of a TTI have to meet in a buffer.
		// `fr` counts frames from `b`, not from the window, so the quarter and the SFN a
		// group reports stay anchored to the same frame 0 in every window.
		std::vector<pbch_obs> obs;
		int avail = 0;
		for(int fr = mib_win_begin(wi); fr < mib_win_end(wi); fr++) {
			if(cancel && cancel->load()) {
				return false;
			}
			int64_t s1 = b + (int64_t)fr * g.frame + g.slot;
			if(s1 < 0 || (size_t)(s1 + g.slot) > x.size()) {
				break;   // frames only run later: nothing behind this one fits either
			}
			avail++;
			std::vector<cf> gr[4];
			bool ok = true;
			for(int l = 0; l < 4; l++) {
				gr[l].resize(12 * g.nrb);
				if(!wb_demod_symbol(x.data(), x.size(), s1, l, g, plan, gr[l].data())) {
					ok = false;
					break;
				}
			}
			if(!ok) {
				out.mdiag.demod_fail++;
				continue;
			}
			std::vector<cf> h[4][4];
			if(!wb_pbch_chest(x.data(), x.size(), s1, fc.pci, g, plan, h)) {
				out.mdiag.chest_fail++;
				continue;
			}
			const cf *gp[4] = { gr[0].data(), gr[1].data(), gr[2].data(), gr[3].data() };
			pbch_obs o;
			o.fr = fr;
			o.re.resize(240);
			if(wb_pbch_extract(gp, h, g.nrb, fc.pci, o.re.data(), o.h) != 240) {
				out.mdiag.re_budget++;   // used to be forwarded to the decoder, which
				continue;                // refused it without anyone saying so
			}
			obs.push_back(std::move(o));
		}
		// A short window is one that could not offer a whole 40 ms TTI - four frames - and
		// nothing else. This used to be bumped once for the frame loop reaching the end of the
		// burst, which EVERY finite burst does, and once more for the second half hypothesis'
		// b < 0, which is normal whenever the PSS sits in the first frame: it read >= 1 on
		// essentially every cell and therefore distinguished nothing. A window past the end of
		// the capture (deep only) is not a short window either - there is no window there.
		if(avail < 4 && (wi == 0 || avail > 0)) {
			out.mdiag.win_short++;
		}
		win_avail += avail;
		if(avail == 0) {
			continue;
		}
		out.mdiag.frames += (int)obs.size();   // every other counter covers both halves too

		// ---- pass 2: one hypothesis hq = the quarter the window's FIRST frame carries,
		// which fixes every other frame's quarter AND the TTI grouping at once, so the two
		// can never disagree. A wrong hq descrambles wrongly and dies.
		std::vector<int> fr;
		fr.reserve(obs.size());
		for(const auto &o : obs) {
			fr.push_back(o.fr);
		}
		std::vector<std::pair<size_t, size_t>> runs;
		for(int hq = 0; hq < 4 && base < 0; hq++) {
			// per-hypothesis stats: the metric has to be a mean over THIS hypothesis' groups
			// and a max over the four, never a max over everything - see wb_mib_metric
			wb_mib_stats hs;
			mib_group_runs(fr, hq, runs);
			for(const auto &r : runs) {
				if(cancel && cancel->load()) {
					return false;
				}
				std::vector<const cf *> rp;
				std::vector<const cf (*)[240]> hp;
				for(size_t i = r.first; i < r.second; i++) {
					rp.push_back(obs[i].re.data());
					hp.push_back(obs[i].h);
				}
				int sfn = -1;
				if(wb_mib_decode_n(rp.data(), hp.data(), (int)rp.size(), fc.pci,
				                   mib_quarter(hq, fr[r.first]), &ports, &nrb, &sfn, &pd,
				                   &pr, &hs)) {
					base = b;
					sfn0 = mib_sfn0(sfn, fr[r.first]);
					break;
				}
			}
			mst.bw_reject += hs.bw_reject;
			mst.crc_fail += hs.crc_fail;
			const double m = wb_mib_metric(hs);
			if(m > best_metric) {
				best_metric = m;
			}
		}
	}
	out.mdiag.bw_reject = mst.bw_reject;
	out.mdiag.crc_fail = mst.crc_fail;
	out.mdiag.path_metric = best_metric;
	if(base < 0) {
		return false;   // PSS/SSS only
	}
	out.mib_ok = true;
	out.ports = ports;
	out.nrb = nrb;
	out.sfn = sfn0;
	out.status = "mib_only";
	derive_cell(out);   // bandwidth from N_RB even before SIB1

	// N_RB above 50 does not fit the 15.36 MSps numerology: report honestly.
	if(nrb != 50) {
		return true;
	}

	// ---- SIB1: SF5 of even SFN, accumulate every occasion of the burst -----------------
	sib1_soft acc;

	for(int sf = 0;; sf++) {
		int64_t ss = base + (int64_t)sf * g.subframe;
		if((size_t)(ss + 2 * g.slot) > x.size()) {
			break;
		}
		if(cancel && cancel->load()) {
			return true;   // keep the MIB result
		}
		int ns = 2 * (sf % 10);
		int sfn = sfn0 + sf / 10;
		if(sf % 10 != 5 || (sfn & 1)) {
			continue;
		}
		float mg = 0;
		int cfi = wb_pcfich_decode(x.data(), x.size(), ss, ns, fc.pci, ports, g, plan, &mg);
		if(cfi < 1) {
			out.diag.pcfich_fail++;
			continue;
		}
		std::vector<wb_dci> hits;
		if(wb_pdcch_css_decode(x.data(), x.size(), ss, ns, fc.pci, ports, cfi, pd, pr, g, plan, hits) <= 0) {
			out.diag.no_si_dci++;
			continue;
		}
		for(auto &hd : hits) {
			dci_si s{};
			if(!dci_parse_si(hd.bits, (size_t)hd.size, g.nrb, s)) {
				out.diag.no_si_dci++;   // CRC said SI-RNTI, the payload is not an SI grant
				continue;
			}
			diag_note_dci(out.diag, s);   // record BEFORE the gate: 1C/distributed counts too
			if(s.format != 0x1A || !s.localized) {
				out.diag.dci_rejected++;
				continue;   // 1C / distributed: no localized PDSCH path in v1
			}
			std::vector<float> llr;
			int E = wb_pdsch_cw_llr(x.data(), x.size(), ss, ns, fc.pci, ports, cfi,
			                        s.rb_start, s.n_prb, SI_RNTI, g, plan, llr);
			if(E < 0) {
				out.diag.pdsch_fail++;
				continue;
			}
			sib1_soft_accum(acc, s, sfn, llr);
			bool combined = false;
			int rc = sib1_decode_occasion(acc, s, llr, out, &combined);
			if(rc == 0) {
				out.rv_combined = combined;
				out.status = "decoded";
				return true;
			}
			if(rc > 0) {
				out.diag.crc_fail++;   // also the normal single-RV case, before combining
			} else {
				// CRC passed but grammar rejected the head: keep decoding further windows.
				out.diag.asn1_fail++;
				out.diag.last_asn1_rc = rc;   // the one rc this ladder used to throw away
			}
			break;   // one DCI per subframe is enough
		}
	}
	return true;   // MIB decoded, SIB1 did not verify in this burst
}

} // namespace cellscan

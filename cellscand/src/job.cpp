#include "job.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>

#include "bandplan.h"
#include "cellfind.h"
#include "decode_carrier.h"
#include "plmn.h"
#include "report.h"

namespace cellscan {

scanner::scanner(std::string persist_path) : persist_path_(std::move(persist_path)) {}

scanner::~scanner() {
	cancel_ = true;
	if(worker_.joinable()) {
		worker_.join();
	}
}

bool scanner::load_persist() {
	if(persist_path_.empty()) {
		return false;
	}
	scan_doc d;
	int64_t saved = 0;
	if(!persist_load(persist_path_, d, saved)) {
		return false;
	}
	std::lock_guard<std::mutex> lk(mtx_);
	doc_ = std::move(d);
	if(doc_.state == "running") {
		doc_.state = "error";   // a job that was mid-flight at shutdown never finished
		doc_.error = "daemon restarted mid-scan";
	}
	job_seq_ = doc_.job;
	persist_loaded_ = true;
	persist_saved_at_ = saved;
	return true;
}

uint64_t scanner::start(const job_params &p, std::string &err) {
	if(running_.load()) {
		err = "scan läuft";
		return 0;
	}
	if(p.sweep) {
		for(int b : p.bands) {
			if(!sweep_band_by_id(b)) {
				err = "unbekanntes Band " + std::to_string(b);
				return 0;
			}
		}
		if(sweep_candidates(p.bands).empty()) {
			err = "Bandplan liefert keine Kandidaten";
			return 0;
		}
	} else {
		if(p.freqs_mhz.empty() || p.freqs_mhz.size() > 32) {
			err = "1..32 Frequenzen erforderlich";
			return 0;
		}
		for(double f : p.freqs_mhz) {
			if(f < 400.0 || f > 4000.0) {
				err = "Frequenz ausserhalb 400..4000 MHz";
				return 0;
			}
		}
	}
	if(p.dwell_s != 2 && p.dwell_s != 8) {
		err = "dwell muss 2 oder 8 sein";
		return 0;
	}
	if(!factory_) {
		err = "keine Burst-Quelle";
		return 0;
	}
	if(worker_.joinable()) {
		worker_.join();   // reap the previous (finished) worker
	}
	uint64_t id = job_seq_.load() + 1;
	job_seq_ = id;
	cancel_ = false;
	running_ = true;
	worker_ = std::thread(&scanner::run, this, p, id);
	return id;
}

void scanner::cancel() {
	cancel_ = true;
}

scan_doc scanner::snapshot() const {
	std::lock_guard<std::mutex> lk(mtx_);
	return doc_;
}

void scanner::set_phase(int index, int total, double freq_mhz, const char *phase, int pct) {
	std::lock_guard<std::mutex> lk(mtx_);
	doc_.progress.index = index;
	doc_.progress.total = total;
	doc_.progress.freq_mhz = freq_mhz;
	doc_.progress.phase = phase;
	doc_.progress.pct = pct;
}

// Decode every cell on one already-captured burst into a carrier_result. The burst is mutated
// in place (CFO removal) - the caller passes a buffer it owns.
carrier_result scanner::decode_burst(std::vector<cf> &x, double freq_mhz, int dwell,
                                     std::atomic<bool> &cancel) {
	carrier_result cr;
	cr.freq_mhz = freq_mhz;
	cr.earfcn = earfcn_for_freq_mhz(freq_mhz);
	cr.band = band_for_freq_mhz(freq_mhz);
	cr.dwell_s = dwell;

	// carrier power: mean |x|^2 in dB (uncalibrated - carrier-over-floor, we
	// report the same relative figure, never an invented absolute RSRP dBm)
	double pw = 0;
	size_t np = std::min<size_t>(x.size(), (size_t)(0.1 * burst_source::RATE));
	for(size_t i = 0; i < np; i++) {
		pw += std::norm(x[i]);
	}
	cr.power_db = np ? 10.0 * std::log10(pw / np + 1e-30) : -120.0;

	wb_num g = cellsync::wb_num_for(50);
	fft_plan plan((unsigned)g.fft);
	double cfo = 0;
	std::vector<found_cell> cells = cellfind(x, g, plan, &cfo, &cancel);
	if(cells.empty()) {
		cr.status = cancel.load() ? "capture_error" : "no_signal";
		return cr;
	}
	bool any_decoded = false, any_mib_50 = false, any_mib_wide = false;
	for(const auto &fc : cells) {
		if(cancel.load()) {
			break;
		}
		cell_result c;
		c.power_db = cr.power_db;
		c.cfo_hz = cfo;
		decode_cell(x, fc, g, plan, dwell > 2, c, &cancel);
		if(c.sib1_ok) {
			any_decoded = true;
		} else if(c.mib_ok && c.nrb == 50) {
			any_mib_50 = true;
		} else if(c.mib_ok) {
			any_mib_wide = true;
			c.status = "mib_only";   // too wide for this numerology, but the MIB is real
		}
		// one line per cell into the journal: for a cell that stops short this IS the answer
		// to "where did it die" - the counters are the ladder in the order it runs. A cell
		// without a MIB never got as far as the SIB1 ladder, so it prints the MIB one.
		if(!c.mib_ok) {
			const mib_diag &md = c.mdiag;
			fprintf(stderr, "cellscand: %.3f MHz PCI %d %s - frames %d win_short %u "
			                "demod_fail %u chest_fail %u re_budget %u bw_reject %u "
			                "crc_fail %u best %.3f\n",
			        freq_mhz, c.pci, c.status.c_str(), md.frames, md.win_short, md.demod_fail,
			        md.chest_fail, md.re_budget, md.bw_reject, md.crc_fail, md.path_metric);
		} else {
			const sib1_diag &sd = c.diag;
			fprintf(stderr, "cellscand: %.3f MHz PCI %d %s - pcfich_fail %u no_si_dci %u "
			                "dci_rejected %u pdsch_fail %u crc_fail %u asn1_fail %u (rc %d) dcis %zu\n",
			        freq_mhz, c.pci, c.status.c_str(), sd.pcfich_fail, sd.no_si_dci, sd.dci_rejected,
			        sd.pdsch_fail, sd.crc_fail, sd.asn1_fail, sd.last_asn1_rc, sd.dcis.size());
		}
		cr.cells.push_back(c);
	}
	if(any_decoded) {
		cr.status = "decoded";
	} else if(any_mib_50) {
		cr.status = "mib_only";
	} else if(any_mib_wide) {
		cr.status = "too_wide";
	} else {
		cr.status = "pss_only";
	}
	return cr;
}

void scanner::run(job_params p, uint64_t id) {
	std::vector<sweep_probe> cand;
	if(p.sweep) {
		cand = sweep_candidates(p.bands);
		p.freqs_mhz.clear();   // the stage-2 list is chosen by the probes, never by the caller
	} else {
		antenna_group(p.freqs_mhz, p.antenna);   // one RF-path change per job (bandplan.h)
	}

	// initialise the document for this job
	{
		std::lock_guard<std::mutex> lk(mtx_);
		doc_ = scan_doc();
		doc_.job = id;
		doc_.state = "running";
		doc_.mode = p.sweep ? "sweep" : "scan";
		doc_.started = (int64_t)time(nullptr);
		doc_.freqs_mhz = p.freqs_mhz;
		doc_.dwell_s = p.dwell_s;
		doc_.deep = p.deep;
		doc_.gain = p.gain;
		doc_.antenna = p.antenna.pin.empty() ? "auto" : p.antenna.pin;
		if(p.sweep) {
			doc_.sweep_thresh_db = SWEEP_THRESH_DB;
			doc_.probes.reserve(cand.size());   // grows live: one entry per measured probe
			doc_.progress.total = (int)cand.size();
			doc_.progress.phase = "probe";
		} else {
			doc_.progress.total = (int)p.freqs_mhz.size();
		}
	}
	persist_loaded_ = false;   // a live job supersedes any loaded document

	std::unique_ptr<burst_source> src = factory_();
	bool opened = false;
	std::string final_state = "done";
	std::string final_err;

	// open at the first visited frequency, retune afterwards (open is the expensive part). The
	// antenna travels with every tune: the right RF path is a property of the carrier, not of
	// the job (bandplan.h), and the source only pays for it when the name actually changes.
	auto tune = [&](double fmhz) -> bool {
		const std::string &ant = p.antenna.at(fmhz);
		if(!opened) {
			if(!src->open(fmhz * 1e6, p.gain, p.agc, ant)) {
				final_state = "error";
				final_err = "Radio konnte nicht geoeffnet werden";
				return false;
			}
			opened = true;
			return true;
		}
		if(!src->retune(fmhz * 1e6, ant)) {
			final_state = "error";
			final_err = "Retune fehlgeschlagen";
			return false;
		}
		return true;
	};

	// Progress is a TIME estimate, not a step count: a stage-1 probe costs settle + probe
	// capture (the ~0.35 s retune settling dominates stage 1 - bandplan.h), a stage-2 carrier
	// costs settle + dwell + the offline decode, which is the expensive part (bandplan.h).
	// Stage 1 runs before the decode list exists, so it is budgeted against the WORST case -
	// SWEEP_MAX_DECODES, a hard cap the selection cannot exceed - and stage 2 then maps the
	// remainder of the bar over the carriers actually picked. So the bar can only ever be
	// pessimistic during stage 1 and NEVER walks backwards at the stage boundary (an earlier
	// version guessed 4 decodes, got 15 on the first real board sweep, and jumped 67% -> 45%).
	const double t_probe = SWEEP_SETTLE_S + SWEEP_PROBE_S;
	const double t_carrier = SWEEP_SETTLE_S + (double)p.dwell_s + SWEEP_DECODE_S;
	const double t_stage1 = (double)cand.size() * t_probe;
	// share of the bar stage 1 owns; 0 for a plain /scan, which has no stage 1
	const double pct1 = p.sweep && t_stage1 > 0
	                    ? 100.0 * t_stage1 / (t_stage1 + SWEEP_MAX_DECODES * t_carrier) : 0.0;

	bool aborted = false;

	// ---- stage 1 (sweep only): probe every candidate, publish the levels live ------------
	if(p.sweep) {
		fft_plan probe_plan(1024);
		const int nc = (int)cand.size();
		for(int i = 0; i < nc; i++) {
			if(cancel_.load()) {
				final_state = "cancelled";
				aborted = true;
				break;
			}
			double fmhz = cand[i].freq_mhz;
			set_phase(i, nc, fmhz, "probe", (int)(pct1 * i / nc));
			if(!tune(fmhz)) {
				aborted = true;
				break;
			}
			std::vector<cf> buf;
			uint64_t gap = 0;
			bool cap_ok = src->capture(SWEEP_PROBE_S, buf, cancel_, &gap);
			if(cancel_.load()) {
				final_state = "cancelled";
				aborted = true;
				break;
			}
			cand[i].ok = cap_ok && !buf.empty();
			cand[i].level_db = cand[i].ok
			                   ? sweep_probe_level(buf, probe_plan, burst_source::RATE) : 0.0;
			cand[i].above = cand[i].ok && cand[i].level_db >= SWEEP_THRESH_DB;
			{
				std::lock_guard<std::mutex> lk(mtx_);
				doc_.probes.push_back(cand[i]);
			}
		}
		if(!aborted) {
			// selection: above threshold, grid maxima only, strongest first (bandplan.h)
			std::vector<size_t> picks =
			        sweep_select(cand, SWEEP_THRESH_DB, (size_t)SWEEP_MAX_DECODES);
			for(size_t k : picks) {
				cand[k].picked = true;
				p.freqs_mhz.push_back(cand[k].freq_mhz);
			}
			antenna_group(p.freqs_mhz, p.antenna);   // one RF-path change per job (bandplan.h)
			std::lock_guard<std::mutex> lk(mtx_);
			doc_.probes = cand;
			doc_.freqs_mhz = p.freqs_mhz;
		}
	}

	// ---- stage 2: the per-carrier scan (a plain /scan starts here) -----------------------
	// One escalation budget for the whole job: a sweep may pick up to SWEEP_MAX_DECODES = 16
	// carriers, and with a phantom cell on most of them an ungated gate would ask for 16
	// near-GB captures. What it may spend, and on whom, is job.h.
	deep_budget budget;
	const int total = (int)p.freqs_mhz.size();
	auto pct_at = [&](int i, double frac) -> int {
		if(total <= 0) {
			return 100;
		}
		// stage 2 fills what stage 1 left: [pct1, 100] over the carriers actually picked
		return (int)(pct1 + (100.0 - pct1) * (i + frac) / total);
	};

	for(int i = 0; i < total && !aborted; i++) {
		if(cancel_.load()) {
			final_state = "cancelled";
			break;
		}
		double fmhz = p.freqs_mhz[i];
		set_phase(i, total, fmhz, "tune", pct_at(i, 0.0));

		if(!tune(fmhz)) {
			break;
		}

		// ---- capture (no DSP), one retry on a large timestamp gap --------------------
		// The budget applies to the BASE capture too, and not only to the escalation: a job
		// can simply be started at dwell_s = 8 (POST /scan {"dwell_s":8}), which asks for the
		// same 983 MB on every one of up to 32 carriers with no escalation involved.
		set_phase(i, total, fmhz, "capture", pct_at(i, 0.2));
		std::vector<cf> burst;
		uint64_t gap = 0;
		uint64_t bneed = 0, bavail = 0;
		if(!capture_fits((double)p.dwell_s, &bneed, &bavail)) {
			fprintf(stderr, "cellscand: %.3f MHz - %d s CAPTURE REFUSED: needs %llu MB, "
			                "MemAvailable %llu MB, budget %llu MB (%.0f%%) - carrier reported "
			                "as capture_error, NOT as empty spectrum\n",
			        fmhz, p.dwell_s, (unsigned long long)(bneed >> 20),
			        (unsigned long long)(bavail >> 20),
			        (unsigned long long)((uint64_t)(CAPTURE_MEM_FRACTION * (double)bavail) >> 20),
			        CAPTURE_MEM_FRACTION * 100.0);
			carrier_result mr;
			mr.freq_mhz = fmhz;
			mr.earfcn = earfcn_for_freq_mhz(fmhz);
			mr.band = band_for_freq_mhz(fmhz);
			mr.dwell_s = p.dwell_s;
			mr.status = "capture_error";
			mr.deep_skip = "memory";
			mr.antenna = p.antenna.at(fmhz);
			std::lock_guard<std::mutex> lk(mtx_);
			doc_.carriers.push_back(mr);
			continue;
		}
		bool cap_ok = src->capture(p.dwell_s, burst, cancel_, &gap);
		uint64_t gap_thresh = (uint64_t)(0.002 * burst_source::RATE);   // ~2 ms hole
		if(cap_ok && gap > gap_thresh) {
			cap_ok = src->capture(p.dwell_s, burst, cancel_, &gap);   // one retry
		}
		if(cancel_.load()) {
			final_state = "cancelled";
			break;
		}
		carrier_result cr;
		if(!cap_ok || burst.empty()) {
			cr.freq_mhz = fmhz;
			cr.earfcn = earfcn_for_freq_mhz(fmhz);
			cr.band = band_for_freq_mhz(fmhz);
			cr.dwell_s = p.dwell_s;
			cr.status = "capture_error";   // stream died, a gap survived the one retry, or
			                               // the buffer did not fit the memory budget
		} else {
			// ---- decode (offline) -------------------------------------------------
			set_phase(i, total, fmhz, "decode", pct_at(i, 0.5));
			cr = decode_burst(burst, fmhz, p.dwell_s, cancel_);

			// The burst is dead the moment the carrier result exists (nothing in cr points
			// into it). Release it HERE, before anything else is sized: this is what keeps
			// the peak at ONE capture. The old code kept the 245 MB base burst alive across
			// the 983 MB deep allocation - 1.23 GB resident on a 3.27 GB board, and none of
			// it visible to a budget check, because it lived between two allocations.
			std::vector<cf>().swap(burst);

			// ---- 2 s -> 8 s escalation: a real cell that did not finish, and
			// we are not already at the deep dwell. What the carrier is worth (and what the
			// job may still spend) is job.h's deep_budget, so --selftest can pin it.
			const deep_class dk = carrier_deep_class(cr);
			if(p.deep && dk != DEEP_NONE && p.dwell_s == 2 && !cancel_.load()) {
				const char *why = budget.admit(dk, 8.0);
				if(why) {
					// A silently skipped escalation is indistinguishable from a cell that
					// merely did not decode, so it is said out loud and carried in the row.
					cr.deep_skip = why;
					uint64_t need = 0, avail = 0;
					capture_fits(8.0, &need, &avail);
					fprintf(stderr, "cellscand: %.3f MHz - 8 s DEEP CAPTURE REFUSED (%s): "
					                "class %s, escalations %d/%d (%d/%d without a MIB), needs "
					                "%llu MB, MemAvailable %llu MB, budget %llu MB - KEEPING "
					                "the %d s result (%s)\n",
					        fmhz, why,
					        dk == DEEP_MIB ? "mib" : dk == DEEP_PSS ? "pss_only" : "weak",
					        budget.used, DEEP_MAX_PER_JOB, budget.used_pss,
					        DEEP_MAX_PSS_PER_JOB, (unsigned long long)(need >> 20),
					        (unsigned long long)(avail >> 20),
					        (unsigned long long)((uint64_t)(CAPTURE_MEM_FRACTION *
					                                        (double)avail) >> 20),
					        p.dwell_s, cr.status.c_str());
				} else {
					set_phase(i, total, fmhz, "capture_deep", pct_at(i, 0.6));
					// reuse the buffer just released: one capture is ever resident
					uint64_t dgap = 0;
					if(src->capture(8.0, burst, cancel_, &dgap) && !burst.empty() &&
					   !cancel_.load()) {
						set_phase(i, total, fmhz, "decode_deep", pct_at(i, 0.85));
						carrier_result dcr = decode_burst(burst, fmhz, 8, cancel_);
						// keep whichever carrier decoded more (SIB1 beats MIB beats PSS)
						auto rank = [](const carrier_result &c) {
							return c.status == "decoded" ? 3 : c.status == "mib_only" ? 2
							       : c.status == "too_wide" ? 1 : 0;
						};
						if(rank(dcr) > rank(cr)) {
							cr = std::move(dcr);
						}
					}
					std::vector<cf>().swap(burst);
				}
			}
		}

		// ---- publish this carrier's result live ---------------------------------------
		cr.antenna = p.antenna.at(fmhz);   // the path this row was actually received on
		{
			std::lock_guard<std::mutex> lk(mtx_);
			doc_.carriers.push_back(cr);
			doc_.cells_found = 0;
			doc_.sib1_decoded = 0;
			for(const auto &c : doc_.carriers) {
				for(const auto &cell : c.cells) {
					doc_.cells_found++;
					if(cell.sib1_ok) {
						doc_.sib1_decoded++;
					}
				}
			}
		}
	}

	// ---- finalise ----------------------------------------------------------------------
	{
		std::lock_guard<std::mutex> lk(mtx_);
		doc_.state = (final_state == "done" && cancel_.load()) ? "cancelled" : final_state;
		if(doc_.state == "cancelled") {
			doc_.error.clear();
		} else if(doc_.state == "error") {
			doc_.error = final_err;
		}
		doc_.finished = (int64_t)time(nullptr);
		doc_.progress.phase = "done";
		doc_.progress.pct = 100;
		doc_.progress.index = total;
	}
	if(!persist_path_.empty()) {
		scan_doc snap = snapshot();
		if(!persist_save(persist_path_, snap)) {
			fprintf(stderr, "cellscand: persist to %s failed\n", persist_path_.c_str());
		} else {
			persist_saved_at_ = (int64_t)time(nullptr);
		}
	}
	running_ = false;
}

} // namespace cellscan

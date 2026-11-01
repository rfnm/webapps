#include "report.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <unistd.h>

#include "plmn.h"
#include "rfnm_json.h"

namespace cellscan {

using rfnm::jesc;

// ---- small emit helpers (flat, snprintf-free, the fleet house style) ----------------------
static void num(std::string &j, const char *key, double v, int dec, bool &first) {
	char b[48];
	snprintf(b, sizeof(b), "%.*f", dec, v);
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":";
	j += b;
}
static void inum(std::string &j, const char *key, long long v, bool &first) {
	char b[32];
	snprintf(b, sizeof(b), "%lld", v);
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":";
	j += b;
}
static void boolean(std::string &j, const char *key, bool v, bool &first) {
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":";
	j += v ? "true" : "false";
}
static void str(std::string &j, const char *key, const std::string &v, bool &first) {
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":\"" + jesc(v) + "\"";
}
static void str_or_null(std::string &j, const char *key, const std::string &v, bool &first) {
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":";
	j += v.empty() ? "null" : ("\"" + jesc(v) + "\"");
}
// hex without a printf %X so leading zeros stay under our control
static std::string hex(uint32_t v, int width) {
	char b[16];
	snprintf(b, sizeof(b), "%0*X", width, v);
	return b;
}

// ---- derivation ---------------------------------------------------------------------------
void derive_cell(cell_result &c) {
	c.bw_mhz = bw_mhz_for_nrb(c.nrb);
	if(c.sib1_ok) {
		c.enb = eci_enb(c.eci);
		c.sector = eci_sector(c.eci);
		for(auto &p : c.plmns) {
			p.plmn = p.mcc + "-" + p.mnc;
			p.op = operator_name(p.mcc, p.mnc);
		}
		c.op = c.plmns.empty() ? "" : c.plmns.front().op;
	}
}

// ---- one cell -----------------------------------------------------------------------------
static std::string emit_cell(const cell_result &c) {
	std::string j = "{";
	bool f = true;
	inum(j, "pci", c.pci, f);
	inum(j, "nid1", c.nid1, f);
	inum(j, "nid2", c.nid2, f);
	str(j, "status", c.status, f);
	num(j, "snr_db", c.snr_db, 1, f);
	// The raw acquisition numbers behind snr_db. Emitted for every cell because a row sitting
	// at the detection floors (cellfind.h) is a candidate and not a cell, and this is the only
	// place a reader can tell the two apart - the phantom PCIs that made the escalation budget
	// necessary all lived within a few percent of these floors.
	num(j, "pss_metric", c.pss_metric, 3, f);
	num(j, "sss_margin", c.sss_margin, 2, f);
	num(j, "power_db", c.power_db, 1, f);
	num(j, "cfo_hz", c.cfo_hz, 1, f);
	boolean(j, "mib_ok", c.mib_ok, f);
	boolean(j, "sib1_ok", c.sib1_ok, f);
	if(c.mib_ok) {
		inum(j, "ports", c.ports, f);
		inum(j, "nrb", c.nrb, f);
		num(j, "bw_mhz", c.bw_mhz, 1, f);
		str(j, "cp", c.cp, f);
		str(j, "duplex", c.duplex, f);
		inum(j, "sfn", c.sfn, f);
	}
	if(c.sib1_ok) {
		boolean(j, "rv_combined", c.rv_combined, f);
		// plmns
		j += ",\"plmns\":[";
		for(size_t i = 0; i < c.plmns.size(); i++) {
			const auto &p = c.plmns[i];
			std::string pj = "{";
			bool pf = true;
			str(pj, "mcc", p.mcc, pf);
			str(pj, "mnc", p.mnc, pf);
			str(pj, "plmn", p.plmn, pf);
			str(pj, "operator", p.op, pf);
			boolean(pj, "reserved", p.reserved, pf);
			pj += "}";
			j += (i ? "," : "") + pj;
		}
		j += "]";
		f = false;
		str(j, "operator", c.op, f);
		inum(j, "tac", c.tac, f);
		str(j, "tac_hex", hex((uint32_t)c.tac, 4), f);
		inum(j, "eci", c.eci, f);
		str(j, "eci_hex", hex(c.eci, 7), f);
		inum(j, "enb", c.enb, f);
		inum(j, "sector", c.sector, f);
		inum(j, "band_ind", c.band_ind, f);
		inum(j, "q_rxlevmin", c.q_rxlevmin, f);
		inum(j, "q_rxlevmin_dbm", c.q_rxlevmin_dbm, f);
		inum(j, "si_window_ms", c.si_window_ms, f);
		j += ",\"sibs\":[";
		for(size_t i = 0; i < c.sibs.size(); i++) {
			j += (i ? "," : "") + std::to_string(c.sibs[i]);
		}
		j += "]";
		f = false;
		boolean(j, "sib16", c.sib16, f);
		boolean(j, "cell_barred", c.cell_barred, f);
	}
	// A cell that got the MIB but no SIB1 used to be a silent dead end. Emit the failure
	// ladder so the row says WHERE it died, plus the SI grants that were actually on air.
	if(c.mib_ok && !c.sib1_ok) {
		const sib1_diag &g = c.diag;
		std::string dj = "{";
		bool df = true;
		inum(dj, "pcfich_fail", g.pcfich_fail, df);
		inum(dj, "no_si_dci", g.no_si_dci, df);
		inum(dj, "dci_rejected", g.dci_rejected, df);
		inum(dj, "pdsch_fail", g.pdsch_fail, df);
		inum(dj, "crc_fail", g.crc_fail, df);
		inum(dj, "asn1_fail", g.asn1_fail, df);
		inum(dj, "last_asn1_rc", g.last_asn1_rc, df);
		dj += ",\"dcis\":[";
		for(size_t i = 0; i < g.dcis.size(); i++) {
			const si_dci_seen &e = g.dcis[i];
			std::string ej = "{";
			bool ef = true;
			str(ej, "format", hex((uint32_t)e.format, 2), ef);
			boolean(ej, "localized", e.localized, ef);
			inum(ej, "rb_start", e.rb_start, ef);
			inum(ej, "n_prb", e.n_prb, ef);
			inum(ej, "mcs", e.mcs, ef);
			inum(ej, "rv", e.rv, ef);
			ej += "}";
			dj += (i ? "," : "") + ej;
		}
		dj += "]}";
		j += ",\"sib1_diag\":" + dj;
		f = false;
	}
	// And a cell without a MIB used to be a bare "pss_only" row with no reason attached at
	// all. Same idea one stage earlier: this is what says whether the PBCH was ever in the
	// buffer, and how close the best hypothesis came to closing.
	if(!c.mib_ok) {
		const mib_diag &m = c.mdiag;
		std::string mj = "{";
		bool mf = true;
		inum(mj, "win_short", m.win_short, mf);
		inum(mj, "demod_fail", m.demod_fail, mf);
		inum(mj, "chest_fail", m.chest_fail, mf);
		inum(mj, "re_budget", m.re_budget, mf);
		inum(mj, "bw_reject", m.bw_reject, mf);
		inum(mj, "crc_fail", m.crc_fail, mf);
		inum(mj, "frames", m.frames, mf);
		num(mj, "path_metric", m.path_metric, 4, mf);
		mj += "}";
		j += ",\"mib_diag\":" + mj;
		f = false;
	}
	j += "}";
	return j;
}

static std::string emit_carrier(const carrier_result &cr) {
	std::string j = "{";
	bool f = true;
	num(j, "freq_mhz", cr.freq_mhz, 3, f);
	inum(j, "earfcn", cr.earfcn, f);
	inum(j, "band", cr.band, f);
	inum(j, "dwell_s", cr.dwell_s, f);
	str(j, "antenna", cr.antenna, f);   // per carrier: the policy picks the path per frequency
	str(j, "status", cr.status, f);
	num(j, "power_db", cr.power_db, 1, f);
	// only when the carrier asked for the 8 s escalation and did not get it - a refusal that
	// is not reported reads exactly like a cell that did not decode (job.h deep_budget)
	if(!cr.deep_skip.empty()) {
		str(j, "deep_skip", cr.deep_skip, f);
	}
	j += ",\"cells\":[";
	for(size_t i = 0; i < cr.cells.size(); i++) {
		j += (i ? "," : "") + emit_cell(cr.cells[i]);
	}
	j += "]}";
	return j;
}

std::string emit_scan(const scan_doc &d) {
	std::string j = "{";
	bool f = true;
	inum(j, "job", (long long)d.job, f);
	str(j, "state", d.state, f);
	str(j, "mode", d.mode, f);
	str_or_null(j, "error", d.error, f);
	// unix timestamps; 0 -> null so the panel shows "-" not 1970
	j += ",\"started\":";
	j += d.started ? std::to_string((long long)d.started) : "null";
	j += ",\"finished\":";
	j += d.finished ? std::to_string((long long)d.finished) : "null";
	f = false;
	j += ",\"freqs_mhz\":[";
	for(size_t i = 0; i < d.freqs_mhz.size(); i++) {
		char b[32];
		snprintf(b, sizeof(b), "%.3f", d.freqs_mhz[i]);
		j += (i ? "," : "");
		j += b;
	}
	j += "]";
	inum(j, "dwell_s", d.dwell_s, f);
	boolean(j, "deep", d.deep, f);
	inum(j, "gain", d.gain, f);
	str(j, "antenna", d.antenna, f);
	// sweep only: the stage-1 probe list (the panel's live bar view) and its threshold
	if(d.mode == "sweep") {
		num(j, "sweep_thresh_db", d.sweep_thresh_db, 1, f);
		j += ",\"probes\":[";
		for(size_t i = 0; i < d.probes.size(); i++) {
			const sweep_probe &p = d.probes[i];
			std::string pj = "{";
			bool pf = true;
			num(pj, "freq_mhz", p.freq_mhz, 3, pf);
			inum(pj, "band", p.band, pf);
			num(pj, "level_db", p.level_db, 1, pf);
			boolean(pj, "ok", p.ok, pf);
			boolean(pj, "above", p.above, pf);
			boolean(pj, "picked", p.picked, pf);
			pj += "}";
			j += (i ? "," : "") + pj;
		}
		j += "]";
	}
	// progress
	{
		std::string pj = "{";
		bool pf = true;
		inum(pj, "index", d.progress.index, pf);
		inum(pj, "total", d.progress.total, pf);
		num(pj, "freq_mhz", d.progress.freq_mhz, 3, pf);
		str(pj, "phase", d.progress.phase, pf);
		inum(pj, "pct", d.progress.pct, pf);
		pj += "}";
		j += ",\"progress\":" + pj;
	}
	inum(j, "cells_found", d.cells_found, f);
	inum(j, "sib1_decoded", d.sib1_decoded, f);
	j += ",\"carriers\":[";
	for(size_t i = 0; i < d.carriers.size(); i++) {
		j += (i ? "," : "") + emit_carrier(d.carriers[i]);
	}
	j += "]}";
	return j;
}

// ---- persistence --------------------------------------------------------------------------
bool persist_save(const std::string &path, const scan_doc &d) {
	std::string tmp = path + ".tmp";
	FILE *f = fopen(tmp.c_str(), "wb");
	if(!f) {
		return false;
	}
	std::string body = "{\"saved_at\":" + std::to_string((long long)time(nullptr)) +
	                   ",\"scan\":" + emit_scan(d) + "}\n";
	bool ok = fwrite(body.data(), 1, body.size(), f) == body.size();
	ok = (fflush(f) == 0) && ok;
	fclose(f);
	if(!ok) {
		unlink(tmp.c_str());
		return false;
	}
	if(rename(tmp.c_str(), path.c_str()) != 0) {
		unlink(tmp.c_str());
		return false;
	}
	return true;
}

// ---- load (parse back the scan object) ----------------------------------------------------
static void load_cell(const rfnm::jv &v, cell_result &c) {
	c.pci = (int)v.num("pci", -1);
	c.nid1 = (int)v.num("nid1", -1);
	c.nid2 = (int)v.num("nid2", -1);
	c.status = v.str("status", "pss_only");
	c.snr_db = v.num("snr_db");
	c.pss_metric = v.num("pss_metric");
	c.sss_margin = v.num("sss_margin");
	c.power_db = v.num("power_db");
	c.cfo_hz = v.num("cfo_hz");
	c.mib_ok = v.boolean("mib_ok");
	c.sib1_ok = v.boolean("sib1_ok");
	c.ports = (int)v.num("ports");
	c.nrb = (int)v.num("nrb");
	c.bw_mhz = v.num("bw_mhz");
	c.cp = v.str("cp", "normal");
	c.duplex = v.str("duplex", "fdd");
	c.sfn = (int)v.num("sfn", -1);
	c.rv_combined = v.boolean("rv_combined");
	c.op = v.str("operator");
	c.tac = (int)v.num("tac", -1);
	c.eci = (uint32_t)v.num("eci");
	c.enb = (uint32_t)v.num("enb");
	c.sector = (int)v.num("sector", -1);
	c.band_ind = (int)v.num("band_ind");
	c.q_rxlevmin = (int)v.num("q_rxlevmin");
	c.q_rxlevmin_dbm = (int)v.num("q_rxlevmin_dbm");
	c.si_window_ms = (int)v.num("si_window_ms");
	c.sib16 = v.boolean("sib16");
	c.cell_barred = v.boolean("cell_barred");
	if(const rfnm::jv *pl = v.get("plmns"); pl && pl->t == rfnm::jv::ARR) {
		for(const auto &pv : pl->a) {
			plmn_entry p;
			p.mcc = pv.str("mcc");
			p.mnc = pv.str("mnc");
			p.plmn = pv.str("plmn");
			p.op = pv.str("operator");
			p.reserved = pv.boolean("reserved");
			c.plmns.push_back(p);
		}
	}
	if(const rfnm::jv *dg = v.get("sib1_diag"); dg && dg->t == rfnm::jv::OBJ) {
		c.diag.pcfich_fail = (uint32_t)dg->num("pcfich_fail");
		c.diag.no_si_dci = (uint32_t)dg->num("no_si_dci");
		c.diag.dci_rejected = (uint32_t)dg->num("dci_rejected");
		c.diag.pdsch_fail = (uint32_t)dg->num("pdsch_fail");
		c.diag.crc_fail = (uint32_t)dg->num("crc_fail");
		c.diag.asn1_fail = (uint32_t)dg->num("asn1_fail");
		c.diag.last_asn1_rc = (int)dg->num("last_asn1_rc");
		if(const rfnm::jv *dl = dg->get("dcis"); dl && dl->t == rfnm::jv::ARR) {
			for(const auto &ev : dl->a) {
				si_dci_seen e;
				e.format = (int)strtol(ev.str("format", "0").c_str(), nullptr, 16);
				e.localized = ev.boolean("localized");
				e.rb_start = (int)ev.num("rb_start");
				e.n_prb = (int)ev.num("n_prb");
				e.mcs = (int)ev.num("mcs");
				e.rv = (int)ev.num("rv");
				c.diag.dcis.push_back(e);
			}
		}
	}
	if(const rfnm::jv *mg = v.get("mib_diag"); mg && mg->t == rfnm::jv::OBJ) {
		c.mdiag.win_short = (uint32_t)mg->num("win_short");
		c.mdiag.demod_fail = (uint32_t)mg->num("demod_fail");
		c.mdiag.chest_fail = (uint32_t)mg->num("chest_fail");
		c.mdiag.re_budget = (uint32_t)mg->num("re_budget");
		c.mdiag.bw_reject = (uint32_t)mg->num("bw_reject");
		c.mdiag.crc_fail = (uint32_t)mg->num("crc_fail");
		c.mdiag.frames = (int)mg->num("frames");
		c.mdiag.path_metric = mg->num("path_metric", -1.0);
	}
	if(const rfnm::jv *sb = v.get("sibs"); sb && sb->t == rfnm::jv::ARR) {
		for(const auto &sv : sb->a) {
			if(sv.t == rfnm::jv::NUM) {
				c.sibs.push_back((int)sv.n);
			}
		}
	}
}

bool persist_load(const std::string &path, scan_doc &out, int64_t &saved_at) {
	FILE *f = fopen(path.c_str(), "rb");
	if(!f) {
		return false;
	}
	std::string body;
	char buf[4096];
	size_t n;
	while((n = fread(buf, 1, sizeof(buf), f)) > 0) {
		body.append(buf, n);
	}
	fclose(f);
	rfnm::jv root;
	if(!rfnm::jparse(body, root) || root.t != rfnm::jv::OBJ) {
		return false;   // truncated / corrupt: jparse requires the whole document to close
	}
	saved_at = (int64_t)root.num("saved_at");
	const rfnm::jv *s = root.get("scan");
	if(!s || s->t != rfnm::jv::OBJ) {
		return false;
	}
	scan_doc d;
	d.job = (uint64_t)s->num("job");
	d.state = s->str("state", "done");
	d.mode = s->str("mode", "scan");
	d.sweep_thresh_db = s->num("sweep_thresh_db");
	if(const rfnm::jv *pr = s->get("probes"); pr && pr->t == rfnm::jv::ARR) {
		for(const auto &pv : pr->a) {
			sweep_probe p;
			p.freq_mhz = pv.num("freq_mhz");
			p.band = (int)pv.num("band");
			p.level_db = pv.num("level_db");
			p.ok = pv.boolean("ok", true);
			p.above = pv.boolean("above");
			p.picked = pv.boolean("picked");
			d.probes.push_back(p);
		}
	}
	if(const rfnm::jv *e = s->get("error"); e && e->t == rfnm::jv::STR) {
		d.error = e->s;
	}
	d.started = (int64_t)s->num("started");
	d.finished = (int64_t)s->num("finished");
	d.dwell_s = (int)s->num("dwell_s", 2);
	d.deep = s->boolean("deep", true);
	d.gain = (int)s->num("gain", 12);
	d.antenna = s->str("antenna", "sma_a");
	if(const rfnm::jv *fr = s->get("freqs_mhz"); fr && fr->t == rfnm::jv::ARR) {
		for(const auto &fv : fr->a) {
			if(fv.t == rfnm::jv::NUM) {
				d.freqs_mhz.push_back(fv.n);
			}
		}
	}
	if(const rfnm::jv *pg = s->get("progress"); pg && pg->t == rfnm::jv::OBJ) {
		d.progress.index = (int)pg->num("index");
		d.progress.total = (int)pg->num("total");
		d.progress.freq_mhz = pg->num("freq_mhz");
		d.progress.phase = pg->str("phase", "done");
		d.progress.pct = (int)pg->num("pct");
	}
	d.cells_found = (int)s->num("cells_found");
	d.sib1_decoded = (int)s->num("sib1_decoded");
	if(const rfnm::jv *ca = s->get("carriers"); ca && ca->t == rfnm::jv::ARR) {
		for(const auto &cv : ca->a) {
			carrier_result cr;
			cr.freq_mhz = cv.num("freq_mhz");
			cr.earfcn = (int)cv.num("earfcn", -1);
			cr.band = (int)cv.num("band");
			cr.dwell_s = (int)cv.num("dwell_s", 2);
			cr.antenna = cv.str("antenna");
			cr.status = cv.str("status", "no_signal");
			cr.power_db = cv.num("power_db");
			cr.deep_skip = cv.str("deep_skip");
			if(const rfnm::jv *ce = cv.get("cells"); ce && ce->t == rfnm::jv::ARR) {
				for(const auto &clv : ce->a) {
					cell_result c;
					load_cell(clv, c);
					cr.cells.push_back(c);
				}
			}
			d.carriers.push_back(cr);
		}
	}
	out = std::move(d);
	return true;
}

} // namespace cellscan

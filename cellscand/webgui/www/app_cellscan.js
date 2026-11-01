// Cell Scan panel (cellscand). One button - "Sweep" - probes the whole band plan (B20/B8/B3/
// B1/B7, POST /sweep): stage 1 measures the in-band level on every candidate frequency and this
// panel draws them as live bars (the honest view - an empty band is a row of flat bars), stage 2
// fully decodes only carriers above threshold, strongest first, into the cells table - PCI,
// operator (PLMN), TAC, cellIdentity + eNB/sector, bandwidth, ports and receive power. The old
// explicit frequency-list scan survives as a fold-out detail, no longer the entry point.
// Data: GET /api/app/cellscan/status; actions POST /sweep, /scan, /cancel. No framework; a
// liveBusy guard + print fingerprints keep the DOM from flickering under the 2 s poll, and
// nothing is rebuilt under a text selection.
"use strict";
(function() {

// operator -> accent colour, so the row a human reads is also colour-coded
const OP_COLOR = {
	"Telekom": "#e6197f", "Vodafone": "#e60000", "Telefonica/O2": "#0a7fd6", "1&1": "#f59e0b"
};

const ICON = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.2">' +
	'<path d="M8 6.6 v7.4 M4.6 14 h6.8 M5.1 4.3 a4 4 0 0 1 5.8 0 M3.4 2.7 a6.4 6.4 0 0 1 9.2 0"/>' +
	'<circle cx="8" cy="5.6" r="1.1" fill="currentColor" stroke="none"/></svg>';

// module state: fingerprints of the last-rendered table and probe bars, and whether we've
// seeded the frequency box for this daemon run (so a running scan doesn't fight the user).
const S = { print: "", pprint: "", seeded: false, posting: false };

function injectStyle() {
	if(document.getElementById("cn-style")) { return; }
	const st = document.createElement("style");
	st.id = "cn-style";
	st.textContent = `
	.cn-head { display:flex; align-items:flex-end; gap:12px; flex-wrap:wrap; margin-bottom:10px; }
	.cn-field { display:flex; flex-direction:column; gap:3px; }
	.cn-field label { font-size:10px; text-transform:uppercase; letter-spacing:1px; color:#62798f; }
	.cn-freq { background:#16283f; color:#dae3ec; border:1px solid #2a4260; border-radius:5px;
		font:inherit; font-size:13px; padding:5px 8px; min-width:230px; font-variant-numeric:tabular-nums; }
	.cn-freq:focus { border-color:#3f6f9e; outline:none; }
	.cn-seg { display:inline-flex; border:1px solid #2a4260; border-radius:5px; overflow:hidden; }
	.cn-seg button { background:#101a2b; color:#9ab0c4; border:none; font:inherit; font-size:11px;
		padding:5px 10px; cursor:pointer; }
	.cn-seg button.on { background:#1d3350; color:#dae3ec; }
	.cn-chk { display:inline-flex; align-items:center; gap:5px; color:#9ab0c4; font-size:12px; cursor:pointer; }
	.cn-chk input { accent-color:#10b981; cursor:pointer; margin:0; }
	.cn-btn { background:#12341f; color:#7fe0a6; border:1px solid #1d5a38; border-radius:5px;
		font:inherit; font-size:13px; padding:6px 16px; cursor:pointer; }
	.cn-btn:hover:not(:disabled) { border-color:#2fbd6c; color:#a6f6c8; }
	.cn-btn:disabled { opacity:0.4; cursor:default; }
	.cn-btn.cancel { background:#2a1113; color:#ef8ba1; border-color:#5c2a28; }
	.cn-btn.cancel:hover:not(:disabled) { border-color:#f43f5e; color:#ff9d94; }
	.cn-status { display:flex; align-items:center; gap:10px; flex-wrap:wrap; margin:4px 0 10px;
		color:#9ab0c4; font-size:12px; }
	.cn-badge { font-size:10px; letter-spacing:1px; text-transform:uppercase; padding:3px 9px;
		border-radius:5px; display:inline-flex; align-items:center; gap:6px; }
	.cn-badge.run { color:#e4b664; background:#201c0c; border:1px solid #4f421f; }
	.cn-badge.idle { color:#8fbfe6; background:#0d1a2a; border:1px solid #234258; }
	.cn-badge.done { color:#10b981; background:#0d2a23; border:1px solid #1d5244; }
	.cn-badge.err { color:#f66a85; background:#2a1113; border:1px solid #5c2a28; }
	.cn-badge.cancelled { color:#c0b0d0; background:#1a1526; border:1px solid #3a2f52; }
	.cn-dot { width:8px; height:8px; border-radius:50%; background:currentColor; }
	.cn-badge.run .cn-dot { animation:cn-pulse 1.2s infinite; }
	.cn-bar { flex:1 1 160px; height:8px; min-width:120px; background:#101a2b; border:1px solid #1c2c40;
		border-radius:4px; overflow:hidden; }
	.cn-bar > div { height:100%; background:linear-gradient(90deg,#1d5a38,#2fbd6c); width:0; transition:width .3s; }
	.cn-prog { color:#62798f; font-variant-numeric:tabular-nums; }
	.cn-note { color:#62798f; font-size:11px; }
	.cn-tables { overflow-x:auto; }
	.cn-op { display:inline-flex; align-items:center; gap:6px; }
	.cn-op .cn-opdot { width:8px; height:8px; border-radius:2px; flex:none; }
	.cn-op small { color:#62798f; font-variant-numeric:tabular-nums; }
	.cn-st { font-size:10px; letter-spacing:0.5px; text-transform:uppercase; padding:2px 7px; border-radius:4px; }
	.cn-st.decoded { color:#10b981; background:#0d2a23; border:1px solid #1d5244; }
	.cn-st.mib_only { color:#e4b664; background:#201c0c; border:1px solid #4f421f; }
	.cn-st.too_wide { color:#e08a4f; background:#241606; border:1px solid #4a3410; }
	.cn-st.pss_only, .cn-st.no_signal, .cn-st.capture_error { color:#7c8ba0; background:#0d1421; border:1px solid #223148; }
	.cn-deep { color:#e08a4f; font-size:10px; margin-left:6px; cursor:help; }
	.ap-table td.cn-num { font-variant-numeric:tabular-nums; }
	.cn-err { color:#f66a85; font-size:12px; margin:6px 0; }
	.cn-empty { color:#62798f; font-size:12px; padding:14px 2px; }
	.cn-btn.sweep { font-size:14px; padding:7px 22px; background:#0e2c40; color:#7fc8f0;
		border-color:#1d5a7e; }
	.cn-btn.sweep:hover:not(:disabled) { border-color:#2f9bd6; color:#a6e2ff; }
	.cn-det { flex:1 1 100%; }
	.cn-det summary { color:#62798f; font-size:11px; text-transform:uppercase; letter-spacing:1px;
		cursor:pointer; user-select:none; }
	.cn-det[open] summary { margin-bottom:8px; }
	.cn-det .cn-manual { display:flex; align-items:flex-end; gap:12px; flex-wrap:wrap; }
	.cn-probes { display:flex; align-items:flex-end; gap:14px; flex-wrap:wrap; margin:4px 0 12px; }
	.cn-bgrp { display:flex; flex-direction:column; gap:3px; }
	.cn-bgrp .cn-bars { display:flex; align-items:flex-end; gap:2px; height:78px; position:relative;
		padding:0 2px; background:#0d1421; border:1px solid #1c2c40; border-radius:4px; }
	.cn-bgrp .cn-thr { position:absolute; left:0; right:0; border-top:1px dashed #4f421f; pointer-events:none; }
	.cn-bgrp label { font-size:10px; text-transform:uppercase; letter-spacing:1px; color:#62798f;
		text-align:center; }
	.cn-pbar { width:9px; min-height:2px; background:#22405c; border-radius:2px 2px 0 0; flex:none; }
	.cn-pbar.above { background:#8a6d1f; }
	.cn-pbar.picked { background:#2fbd6c; }
	.cn-pbar.fail { background:#5c2a28; }
	.cn-pbar.pending { background:#141d2e; border:1px dashed #223148; border-bottom:none; }
	.cn-probes-note { flex:1 1 100%; color:#62798f; font-size:11px; margin-top:-6px; }
	@keyframes cn-pulse { 0%,100%{opacity:1;} 50%{opacity:0.5;} }
	`;
	document.head.appendChild(st);
}

function ensurePanel(id) {
	const A = window.RFNM_APPS, ce = A.ui[id];
	if(ce.panel) { return ce.panel; }
	injectStyle();
	ce.live.textContent = "";

	const head = A.el("div", "cn-head");

	// the entry point: one button, the board finds out itself what is receivable
	const sweepBtn = A.el("button", "cn-btn sweep", "Sweep");
	sweepBtn.title = "probe the whole band plan (B20 / B8 / B3 / B1 / B7), then fully decode only carriers above threshold, strongest first";
	const cancelBtn = A.el("button", "cn-btn cancel", "cancel");

	// the explicit frequency list survives as a detail view, no longer the entry
	const det = document.createElement("details");
	det.className = "cn-det";
	const sum = document.createElement("summary");
	sum.textContent = "manual frequency list";
	const manual = A.el("div", "cn-manual");

	const fField = A.el("div", "cn-field");
	fField.append(A.el("label", null, "frequencies (MHz, comma-separated)"));
	const freq = A.el("input", "cn-freq");
	freq.type = "text";
	freq.spellcheck = false;
	freq.placeholder = "796, 806, 816";
	fField.append(freq);

	const dField = A.el("div", "cn-field");
	dField.append(A.el("label", null, "dwell"));
	const seg = A.el("div", "cn-seg");
	const d2 = A.el("button", "on", "2 s");
	const d8 = A.el("button", null, "8 s");
	let dwell = 2;
	d2.onclick = () => { dwell = 2; d2.className = "on"; d8.className = ""; };
	d8.onclick = () => { dwell = 8; d8.className = "on"; d2.className = ""; };
	seg.append(d2, d8);
	dField.append(seg);

	const deepWrap = A.el("label", "cn-chk");
	const deep = document.createElement("input");
	deep.type = "checkbox";
	deep.checked = true;
	deepWrap.append(deep, document.createTextNode("2 s → 8 s escalation"));
	deepWrap.title = "when a cell shows a MIB but SIB1 does not verify in 2 s, capture 8 s and retry";

	const scanBtn = A.el("button", "cn-btn", "scan list");

	manual.append(fField, dField, deepWrap, scanBtn);
	det.append(sum, manual);

	head.append(sweepBtn, cancelBtn, det);

	const status = A.el("div", "cn-status");
	const badge = A.el("span", "cn-badge idle");
	const badgeDot = A.el("span", "cn-dot");
	const badgeTxt = A.el("span", null, "idle");
	badge.append(badgeDot, badgeTxt);
	const bar = A.el("div", "cn-bar");
	const barFill = A.el("div");
	bar.append(barFill);
	const prog = A.el("span", "cn-prog", "");
	const summary = A.el("span", "cn-note", "");
	status.append(badge, bar, prog, summary);

	const errLine = A.el("div", "cn-err", "");
	errLine.style.display = "none";
	const note = A.el("div", "cn-note", "");
	const probes = A.el("div", "cn-probes");
	probes.style.display = "none";
	const tables = A.el("div", "cn-tables");

	sweepBtn.onclick = () => doSweep(id, errLine);
	scanBtn.onclick = () => doScan(id, freq, dwell, deep.checked, errLine);
	cancelBtn.onclick = () => doCancel(id, errLine);

	ce.live.append(head, status, errLine, note, probes, tables);
	S.print = "";
	S.pprint = "";
	ce.panel = { freq, sweepBtn, scanBtn, cancelBtn, badge, badgeTxt, bar, barFill, prog, summary,
		errLine, note, probes, tables, getDwell: () => dwell, getDeep: () => deep.checked };
	return ce.panel;
}

function parseFreqs(str) {
	return str.split(/[,\s]+/).map(s => parseFloat(s)).filter(v => isFinite(v) && v > 0);
}

function doScan(id, freqEl, dwell, deep, errEl) {
	const A = window.RFNM_APPS;
	if(S.posting) { return; }
	const freqs = parseFreqs(freqEl.value || freqEl.placeholder);
	if(!freqs.length) { errEl.textContent = "enter at least one frequency in MHz"; errEl.style.display = "block"; return; }
	if(freqs.length > 32) { errEl.textContent = "at most 32 frequencies per scan"; errEl.style.display = "block"; return; }
	S.posting = true;
	errEl.style.display = "none";
	A.fetchJSON("/api/app/cellscan/scan", { method: "POST",
		body: JSON.stringify({ freqs_mhz: freqs, dwell_s: dwell, deep }) }, 10000)
		.then(() => { S.print = ""; })
		.catch(e => { errEl.textContent = "scan rejected: " + e.message; errEl.style.display = "block"; })
		.finally(() => { S.posting = false; });
}

function doSweep(id, errEl) {
	const A = window.RFNM_APPS;
	if(S.posting) { return; }
	S.posting = true;
	errEl.style.display = "none";
	A.fetchJSON("/api/app/cellscan/sweep", { method: "POST" }, 10000)
		.then(() => { S.print = ""; S.pprint = ""; })
		.catch(e => { errEl.textContent = "sweep rejected: " + e.message; errEl.style.display = "block"; })
		.finally(() => { S.posting = false; });
}

// ---- stage-1 bar view: measured level per candidate frequency, grouped by band -------------
// The most interesting view of the whole run - and the honest one: an empty band is a row of
// flat bars, not an absence of rows. Scale 0..30 dB; the dashed line is the decode threshold.
function renderProbes(A, p, scan, scanning) {
	const probes = Array.isArray(scan.probes) ? scan.probes : [];
	const probing = scanning && scan.progress && scan.progress.phase === "probe";
	if(scan.mode !== "sweep" || (!probes.length && !probing)) {
		p.probes.style.display = "none";
		S.pprint = "";
		return;
	}
	const thresh = Number(scan.sweep_thresh_db) || 6;
	const print = JSON.stringify([thresh, probing,
		probes.map(x => [x.freq_mhz, x.level_db, x.above, x.picked, x.ok])]);
	if(print === S.pprint) { return; }
	if(A.selectionInside(p.probes)) { return; }
	S.pprint = print;
	p.probes.style.display = "";
	p.probes.textContent = "";
	const MAXDB = 30;
	const groups = new Map();
	for(const pr of probes) {
		if(!groups.has(pr.band)) { groups.set(pr.band, []); }
		groups.get(pr.band).push(pr);
	}
	for(const [band, list] of groups) {
		const grp = A.el("div", "cn-bgrp");
		const bars = A.el("div", "cn-bars");
		const thr = A.el("div", "cn-thr");
		thr.style.bottom = Math.max(0, Math.min(100, thresh / MAXDB * 100)) + "%";
		bars.appendChild(thr);
		for(const pr of list) {
			const b = A.el("div", "cn-pbar " + (!pr.ok ? "fail" : pr.picked ? "picked"
				: pr.above ? "above" : ""));
			const lvl = Math.max(0, Math.min(MAXDB, Number(pr.level_db) || 0));
			b.style.height = Math.max(3, lvl / MAXDB * 100) + "%";
			b.title = Number(pr.freq_mhz).toFixed(1) + " MHz · " + f1(pr.level_db) + " dB"
				+ (!pr.ok ? " · capture failed" : pr.picked ? " · decoding" : "");
			bars.appendChild(b);
		}
		grp.append(bars, A.el("label", null, "B" + band));
		p.probes.appendChild(grp);
	}
	const done = probes.length;
	const label = probing
		? "probing the band plan — " + done + " measured, in-band level over out-of-band floor (dB)"
		: "in-band level per candidate (dB) — green bars were decoded, dashed line = " + f1(thresh) + " dB threshold";
	p.probes.appendChild(A.el("div", "cn-probes-note", label));
}

function doCancel(id, errEl) {
	const A = window.RFNM_APPS;
	A.fetchJSON("/api/app/cellscan/cancel", { method: "POST" }, 8000)
		.catch(e => { errEl.textContent = "cancel failed: " + e.message; errEl.style.display = "block"; });
}

function f1(x) { const v = Number(x); return isFinite(v) ? v.toFixed(1) : "—"; }
function fmtWhen(ts) {
	if(!ts) { return ""; }
	try { return new Date(ts * 1000).toLocaleString(); } catch(e) { return ""; }
}

function opCell(A, cell) {
	const wrap = A.el("span", "cn-op");
	const dot = A.el("span", "cn-opdot");
	dot.style.background = OP_COLOR[cell.operator] || "#556";
	const plmns = (cell.plmns || []).map(p => p.plmn).join(", ");
	wrap.append(dot, document.createTextNode(cell.operator || "—"));
	if(plmns) { wrap.append(A.el("small", null, " " + plmns)); }
	return wrap;
}

function statusCell(A, s) {
	const label = s === "decoded" ? "SIB1 ✓" : s === "mib_only" ? "MIB" : s === "too_wide" ? "too wide"
		: s === "pss_only" ? "PSS" : s === "no_signal" ? "no signal" : s === "capture_error" ? "capture err" : s;
	const b = A.el("span", "cn-st " + s, label);
	return b;
}

// The 2 s -> 8 s escalation a carrier asked for and did not get. It belongs beside the status
// because a refused escalation and a cell that simply did not decode produce the SAME row -
// and only one of the two is something the operator can do anything about (job.h deep_budget).
function deepNote(A, c) {
	if(!c.deep_skip) { return null; }
	// capture_error + a reason means the BASE capture was the one refused, not the escalation
	const base = c.status === "capture_error";
	const why = c.deep_skip === "memory"
		? (base ? "it did not fit the memory budget (half of MemAvailable)"
			: "not enough free memory for the 983 MB deep capture")
		: c.deep_skip === "job_cap" ? "the job's escalation budget was already spent on stronger carriers"
		: "the detection sat on the detection floor — not worth an 8 s capture";
	const s = A.el("small", "cn-deep", base ? "mem✗" : "8s✗");
	s.title = (base ? "capture refused: " : "no deep capture: ") + why;
	return s;
}

// one <td> that carries a tooltip and the tabular-num class
function numTd(A, text, title) {
	const td = A.el("td", "cn-num", text);
	if(title) { td.title = title; }
	return td;
}

function render(id) {
	const A = window.RFNM_APPS;
	if(A.liveBusy[id]) { return; }
	A.liveBusy[id] = true;
	const p = ensurePanel(id);
	A.fetchJSON("/api/app/cellscan/status", undefined, 6000).then(d => {
		const scan = d && d.scan ? d.scan : {};
		const scanning = d.state === "scanning";
		const st = scan.state || "idle";

		// badge
		const cls = scanning ? "run" : (st === "done" ? "done" : st === "error" ? "err"
			: st === "cancelled" ? "cancelled" : "idle");
		p.badge.className = "cn-badge " + cls;
		p.badgeTxt.textContent = scanning ? "scanning" : st;

		// progress
		const pr = scan.progress || {};
		const pct = Math.max(0, Math.min(100, Number(pr.pct) || 0));
		p.barFill.style.width = pct + "%";
		if(scanning) {
			p.prog.textContent = (pr.index + 1) + "/" + pr.total + " · " + f1(pr.freq_mhz) + " MHz · " + (pr.phase || "");
		} else {
			p.prog.textContent = "";
		}

		// summary + persisted-result note
		const nc = Number(scan.cells_found) || 0, ns = Number(scan.sib1_decoded) || 0;
		const cellsTxt = nc + " cell" + (nc === 1 ? "" : "s") + " · " + ns + " with SIB1";
		if(scan.mode === "sweep" && Array.isArray(scan.probes) && scan.probes.length) {
			const nd = Array.isArray(scan.freqs_mhz) ? scan.freqs_mhz.length : 0;
			p.summary.textContent = scan.probes.length + " probed · " + nd + " decoded"
				+ ((scan.carriers && scan.carriers.length) ? " · " + cellsTxt : "");
		} else {
			p.summary.textContent = (scan.carriers && scan.carriers.length) ? cellsTxt : "";
		}
		if(!scanning && d.persist && d.persist.loaded && scan.finished) {
			p.note.textContent = "showing the last scan from " + fmtWhen(scan.finished) + " (persisted across restart)";
		} else if(!scanning && scan.finished) {
			p.note.textContent = "scan finished " + fmtWhen(scan.finished);
		} else {
			p.note.textContent = "";
		}

		// buttons
		p.sweepBtn.disabled = scanning || S.posting;
		p.scanBtn.disabled = scanning || S.posting;
		p.cancelBtn.disabled = !scanning;

		// stage-1 bar view (sweep only)
		renderProbes(A, p, scan, scanning);

		// seed the frequency box once per view from the last scan's list (don't fight typing)
		if(!S.seeded && document.activeElement !== p.freq) {
			if(scan.freqs_mhz && scan.freqs_mhz.length) {
				p.freq.value = scan.freqs_mhz.map(x => Number(x).toFixed(1)).join(", ");
			}
			S.seeded = true;
		}

		// error line from the daemon (only when the job itself errored)
		if(st === "error" && scan.error) {
			p.errLine.textContent = "scan error: " + scan.error;
			p.errLine.style.display = "block";
		}

		// ---- results table (rebuilt only on change, never under a selection) ----
		const carriers = Array.isArray(scan.carriers) ? scan.carriers : [];
		const print = JSON.stringify(carriers.map(c => [c.freq_mhz, c.status, c.power_db,
			(c.cells || []).map(x => [x.pci, x.status, x.operator, x.tac, x.enb, x.sector, x.nrb, x.ports, x.snr_db])]));
		if(print !== S.print && !A.selectionInside(p.tables)) {
			S.print = print;
			p.tables.textContent = "";
			if(!carriers.length) {
				p.tables.appendChild(A.el("div", "cn-empty", scanning
					? (pr.phase === "probe" ? "sweeping — carriers above threshold are decoded after the probe pass"
						: "scanning — results appear here as each carrier is decoded")
					: (scan.mode === "sweep" && scan.finished
						? "sweep finished — nothing above threshold, all probed bands are empty here"
						: "no scan yet — press Sweep to let the board find what is receivable")));
			} else {
				const rows = [];
				for(const c of carriers) {
					const cells = c.cells || [];
					if(!cells.length) {
						const tr = A.tr([f1(c.freq_mhz), c.band ? "B" + c.band : "—", "—", "—", "—", "—", "—", "—",
							f1(c.power_db) + " dB", ""]);
						tr.lastChild.textContent = "";
						tr.lastChild.appendChild(statusCell(A, c.status));
						const dn0 = deepNote(A, c);
						if(dn0) { tr.lastChild.appendChild(dn0); }
						rows.push(tr);
						continue;
					}
					for(const cell of cells) {
						const tr = document.createElement("tr");
						tr.appendChild(numTd(A, f1(c.freq_mhz), "EARFCN " + c.earfcn));
						tr.appendChild(A.el("td", null, (c.band ? "B" + c.band : "—") +
							(cell.band_ind && cell.band_ind !== c.band ? " (SIB1 B" + cell.band_ind + ")" : "")));
						tr.appendChild(numTd(A, String(cell.pci), "NID1 " + cell.nid1 + " / NID2 " + cell.nid2));
						const opTd = A.el("td");
						opTd.appendChild(opCell(A, cell));
						tr.appendChild(opTd);
						tr.appendChild(numTd(A, cell.sib1_ok ? (cell.tac_hex ? "0x" + cell.tac_hex : String(cell.tac)) : "—",
							cell.sib1_ok ? String(cell.tac) : ""));
						tr.appendChild(numTd(A, cell.sib1_ok ? (cell.enb + " / " + cell.sector) : "—",
							cell.sib1_ok ? "cellIdentity 0x" + cell.eci_hex : ""));
						tr.appendChild(numTd(A, cell.mib_ok ? f1(cell.bw_mhz) + " MHz" : "—"));
						tr.appendChild(numTd(A, cell.mib_ok ? String(cell.ports) : "—"));
						tr.appendChild(numTd(A, f1(cell.snr_db) + " / " + f1(cell.power_db) + " dB",
							"SNR " + f1(cell.snr_db) + " dB · power " + f1(cell.power_db) + " dB (uncalibrated)" +
							(cell.pss_metric ? " · PSS metric " + Number(cell.pss_metric).toFixed(3)
								+ " · SSS margin " + Number(cell.sss_margin).toFixed(2) : "")));
						const stTd = A.el("td");
						stTd.appendChild(statusCell(A, cell.status));
						if(cell.rv_combined) { stTd.title = "SIB1 recovered by combining redundancy versions (2-port cell)"; }
						const dn = deepNote(A, c);
						if(dn) { stTd.appendChild(dn); }
						tr.appendChild(stTd);
						rows.push(tr);
					}
				}
				p.tables.appendChild(A.table(
					["Freq MHz", "Band", "PCI", "Operator (PLMN)", "TAC", "eNB / Sector", "BW", "Ports", "SNR / Pwr", "Status"],
					rows));
			}
		}
	}).catch(() => {}).finally(() => { A.liveBusy[id] = false; });
}

(window.__RFNM_APP_MODULES = window.__RFNM_APP_MODULES || []).push({
	id: "cellscan", title: "Cell Scan", icon: ICON,
	desc: "One-button LTE band sweep: probe B20/B8/B3/B1/B7 for power, decode carriers above threshold - PCI, operator (PLMN), TAC, cellIdentity/eNB-sector, bandwidth, ports and receive power per cell.",
	live: render,
});

})();

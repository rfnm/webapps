// ELRS panel (elrsd). ExpressLRS 2.4 GHz sniffer: binding phrase -> UID (sent once per "set",
// held in the daemon's RAM only, never stored in the browser), FHSS lock badge, link tiles,
// 16 RC channel bars, the 80-channel occupancy map with the window and the predicted next hops,
// the packet log, and the TX block (arm/disarm behind the board policy, --tx 1 and a power cap).
// Data: GET /api/app/elrs/status + /packets?since=; actions PUT /config, POST /tx/arm|disarm.
// No framework; liveBusy guard + fingerprints keep the DOM from flickering under the 2 s poll,
// nothing is rebuilt under a text selection, and a focused input is never overwritten.
"use strict";
(function() {

const BASE = "/api/app/elrs";

const ICON = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.2">' +
	'<path d="M2 12 h2 v-3 h2 v-5 h2 v8 h2 v-4 h2 v-2 h2"/>' +
	'<path d="M3.2 3.4 a5 5 0 0 1 4.6 -2.4 M2 5.2 a7 7 0 0 1 6 -4"/></svg>';

const S = { lastSeq: 0, packets: [], pktPrint: "", chPrint: "", posting: false, err: "" };

function injectStyle() {
	if(document.getElementById("el-style")) { return; }
	const st = document.createElement("style");
	st.id = "el-style";
	st.textContent = `
	.el-head { display:flex; align-items:flex-end; gap:12px; flex-wrap:wrap; margin-bottom:10px; }
	.el-field { display:flex; flex-direction:column; gap:3px; }
	.el-field label { font-size:10px; text-transform:uppercase; letter-spacing:1px; color:#62798f; }
	.el-in { background:#16283f; color:#dae3ec; border:1px solid #2a4260; border-radius:5px; font:inherit;
		font-size:13px; padding:5px 8px; font-variant-numeric:tabular-nums; }
	.el-in:focus { border-color:#3f6f9e; outline:none; }
	.el-in.phrase { min-width:220px; }
	.el-in.num { width:70px; }
	.el-btn { background:#12341f; color:#7fe0a6; border:1px solid #1d5a38; border-radius:5px; font:inherit;
		font-size:13px; padding:6px 14px; cursor:pointer; }
	.el-btn:hover:not(:disabled) { border-color:#2fbd6c; color:#a6f6c8; }
	.el-btn:disabled { opacity:0.4; cursor:default; }
	.el-btn.arm { background:#2a1113; color:#ef8ba1; border-color:#5c2a28; }
	.el-btn.arm:hover:not(:disabled) { border-color:#f43f5e; color:#ff9d94; }
	.el-badge { font-size:10px; letter-spacing:1px; text-transform:uppercase; padding:3px 9px; border-radius:5px;
		display:inline-flex; align-items:center; gap:6px; }
	.el-badge.ok { color:#10b981; background:#0d2a23; border:1px solid #1d5244; }
	.el-badge.warn { color:#e4b664; background:#201c0c; border:1px solid #4f421f; }
	.el-badge.err { color:#f66a85; background:#2a1113; border:1px solid #5c2a28; }
	.el-badge.idle { color:#8fbfe6; background:#0d1a2a; border:1px solid #234258; }
	.el-badge.off { color:#7c8ba0; background:#0d1421; border:1px solid #223148; }
	.el-row { display:flex; align-items:center; gap:10px; flex-wrap:wrap; margin:4px 0 10px; color:#9ab0c4; font-size:12px; }
	.el-tiles { display:flex; gap:8px; flex-wrap:wrap; margin-bottom:12px; }
	.el-tile { background:#101a2b; border:1px solid #1c2c40; border-radius:6px; padding:7px 11px; min-width:84px; }
	.el-tile label { display:block; font-size:9px; text-transform:uppercase; letter-spacing:1px; color:#62798f; }
	.el-tile b { font-size:16px; color:#dae3ec; font-variant-numeric:tabular-nums; font-weight:600; }
	.el-tile small { color:#62798f; font-size:10px; margin-left:3px; }
	.el-sec { font-size:10px; text-transform:uppercase; letter-spacing:1px; color:#62798f; margin:12px 0 5px; }
	.el-rc { display:grid; grid-template-columns:repeat(16, 1fr); gap:4px; background:#0d1421; border:1px solid #1c2c40;
		border-radius:6px; padding:8px; }
	.el-rcch { display:flex; flex-direction:column; align-items:center; gap:3px; }
	.el-rcbar { width:100%; height:64px; background:#101a2b; border:1px solid #1c2c40; border-radius:3px; position:relative; overflow:hidden; }
	.el-rcbar > div { position:absolute; left:0; right:0; background:#22405c; }
	.el-rcbar.aux > div { background:#2fbd6c; }
	.el-rcbar.arm > div { background:#e4b664; }
	.el-rcbar.dim > div { background:#1c2c40; }
	.el-rcbar .mid { position:absolute; left:0; right:0; top:50%; border-top:1px dashed #2a4260; }
	.el-rcch label { font-size:9px; color:#62798f; }
	.el-rcch b { font-size:10px; color:#9ab0c4; font-variant-numeric:tabular-nums; font-weight:normal; }
	.el-map { display:grid; grid-template-columns:repeat(40, 1fr); gap:2px; }
	.el-cell { height:14px; border-radius:2px; background:#0d1421; border:1px solid #1c2c40; font-size:8px; color:#3d5068;
		display:flex; align-items:center; justify-content:center; }
	.el-cell.win { background:#101f33; border-color:#22405c; }
	.el-cell.seen { background:#1d3350; color:#9ab0c4; }
	.el-cell.hot { background:#22405c; color:#dae3ec; }
	.el-cell.cur { outline:2px solid #10b981; outline-offset:-1px; color:#a6f6c8; }
	.el-cell.next { outline:1px dashed #e4b664; outline-offset:-1px; }
	.el-cell.sync { border-color:#8a6d1f; }
	.el-tables { overflow-x:auto; }
	.ap-table td.el-num { font-variant-numeric:tabular-nums; }
	.ap-table td.el-hex { font-family:ui-monospace, monospace; font-size:11px; color:#9ab0c4; }
	.el-k { font-size:10px; letter-spacing:0.5px; text-transform:uppercase; padding:2px 6px; border-radius:4px; }
	.el-k.rc { color:#10b981; background:#0d2a23; border:1px solid #1d5244; }
	.el-k.sync { color:#e4b664; background:#201c0c; border:1px solid #4f421f; }
	.el-k.bad { color:#f66a85; background:#2a1113; border:1px solid #5c2a28; }
	.el-k.other { color:#7c8ba0; background:#0d1421; border:1px solid #223148; }
	.el-tx { background:#101a2b; border:1px solid #2a4260; border-radius:6px; padding:10px 12px; margin-top:12px; }
	.el-tx.armed { border-color:#f43f5e; box-shadow:0 0 0 1px #5c2a28 inset; }
	.el-warn { color:#e4b664; font-size:11px; margin:6px 0 0; }
	.el-err { color:#f66a85; font-size:12px; margin:6px 0; display:flex; gap:10px; align-items:center; }
	.el-err button { background:none; border:1px solid #5c2a28; color:#ef8ba1; border-radius:4px; font:inherit; font-size:11px; cursor:pointer; padding:1px 7px; }
	.el-note { color:#62798f; font-size:11px; }
	.el-empty { color:#62798f; font-size:12px; padding:10px 2px; }
	`;
	document.head.appendChild(st);
}

function f1(x) { const v = Number(x); return isFinite(v) ? v.toFixed(1) : "—"; }
function n0(x) { const v = Number(x); return isFinite(v) ? String(Math.round(v)) : "—"; }

function tile(A, label, value, unit) {
	const t = A.el("div", "el-tile");
	t.append(A.el("label", null, label));
	const b = A.el("b", null, value);
	t.append(b);
	if(unit) { t.append(A.el("small", null, unit)); }
	t._b = b;
	return t;
}

function showErr(p, msg) { S.err = msg; p.errTxt.textContent = msg; p.err.style.display = msg ? "flex" : "none"; }

function ensurePanel(id) {
	const A = window.RFNM_APPS, ce = A.ui[id];
	if(ce.panel) { return ce.panel; }
	injectStyle();
	ce.live.textContent = "";
	const p = {};

	// ---- head: binding phrase, rate, badges ----
	const head = A.el("div", "el-head");
	const phField = A.el("div", "el-field");
	phField.append(A.el("label", null, "binding phrase (sent to the daemon, kept in RAM only)"));
	const phrase = A.el("input", "el-in phrase");
	phrase.type = "password";
	phrase.autocomplete = "off";
	phrase.spellcheck = false;
	phrase.placeholder = "binding phrase";
	phField.append(phrase);
	const setBtn = A.el("button", "el-btn", "set");
	const rateField = A.el("div", "el-field");
	rateField.append(A.el("label", null, "rate"));
	const rate = document.createElement("select");
	rate.className = "el-in";
	for(const r of ["250hz", "500hz", "150hz", "50hz", "auto"]) {
		const o = document.createElement("option");
		o.value = r; o.textContent = r;
		rate.append(o);
	}
	rateField.append(rate);
	const uidBadge = A.el("span", "el-badge off", "uid unset");
	const lockBadge = A.el("span", "el-badge off", "no lock");
	const stateBadge = A.el("span", "el-badge idle", "idle");
	head.append(phField, setBtn, rateField, uidBadge, lockBadge, stateBadge);

	setBtn.onclick = () => {
		if(S.posting) { return; }
		const body = {};
		if(phrase.value) { body.binding_phrase = phrase.value; }
		body.rate = rate.value;
		S.posting = true;
		A.fetchJSON(BASE + "/config", { method: "PUT", body: JSON.stringify(body) }, 8000)
			.then(() => { phrase.value = ""; showErr(p, ""); })
			.catch(e => showErr(p, "config rejected: " + e.message))
			.finally(() => { S.posting = false; });
	};
	rate.onchange = () => {
		A.fetchJSON(BASE + "/config", { method: "PUT", body: JSON.stringify({ rate: rate.value }) }, 8000)
			.catch(e => showErr(p, "rate rejected: " + e.message));
	};

	const err = A.el("div", "el-err");
	const errTxt = A.el("span");
	const dismiss = A.el("button", null, "dismiss");
	dismiss.onclick = () => showErr(p, "");
	err.append(errTxt, dismiss);
	err.style.display = "none";

	// ---- tiles ----
	const tiles = A.el("div", "el-tiles");
	p.tPkt = tile(A, "packets/s", "—", "");
	p.tLq = tile(A, "LQ", "—", "%");
	p.tSnr = tile(A, "SNR", "—", "dB");
	p.tRssi = tile(A, "RSSI", "—", "dB rel");
	p.tCrc = tile(A, "crc ok / fail", "—", "");
	p.tSlot = tile(A, "slot / nonce", "—", "");
	p.tPtr = tile(A, "fhss ptr / ch", "—", "");
	p.tDrop = tile(A, "drops", "—", "");
	tiles.append(p.tPkt, p.tLq, p.tSnr, p.tRssi, p.tCrc, p.tSlot, p.tPtr, p.tDrop);

	// ---- RC bars ----
	const rcSec = A.el("div", "el-sec", "RC channels (CRSF 172..1811, ch5 = arm bit, aux by switch index)");
	const rc = A.el("div", "el-rc");
	p.rcBars = [];
	for(let i = 0; i < 16; i++) {
		const ch = A.el("div", "el-rcch");
		const bar = A.el("div", "el-rcbar" + (i === 4 ? " arm" : (i >= 5 ? " aux" : "")));
		const fill = A.el("div");
		const mid = A.el("div", "mid");
		bar.append(fill, mid);
		const lbl = A.el("label", null, "ch" + (i + 1));
		const val = A.el("b", null, "—");
		ch.append(bar, lbl, val);
		rc.append(ch);
		p.rcBars.push({ bar, fill, val });
	}
	p.rcNote = A.el("div", "el-note", "");

	// ---- channel map ----
	const mapSec = A.el("div", "el-sec", "FHSS channels 0..79 (2400.4 + i MHz) — window highlighted, current outlined, next 4 dashed, 41 = sync");
	const map = A.el("div", "el-map");
	p.cells = [];
	for(let i = 0; i < 80; i++) {
		const c = A.el("div", "el-cell" + (i === 41 ? " sync" : ""), String(i));
		map.append(c);
		p.cells.push(c);
	}

	// ---- packet table ----
	const pkSec = A.el("div", "el-sec", "packet log (last 50)");
	const tables = A.el("div", "el-tables");

	// ---- TX block ----
	const tx = A.el("div", "el-tx");
	const txHead = A.el("div", "el-row");
	p.txBadge = A.el("span", "el-badge off", "tx off");
	p.txPolicy = A.el("span", null, "");
	txHead.append(A.el("b", null, "TX"), p.txBadge, p.txPolicy);
	const txRow = A.el("div", "el-row");
	const pwField = A.el("div", "el-field");
	pwField.append(A.el("label", null, "power dBm"));
	p.txPower = A.el("input", "el-in num");
	p.txPower.type = "number"; p.txPower.value = "-40"; p.txPower.step = "1";
	pwField.append(p.txPower);
	const antField = A.el("div", "el-field");
	antField.append(A.el("label", null, "antenna"));
	p.txAnt = A.el("input", "el-in num");
	p.txAnt.type = "text"; p.txAnt.value = "sma_a";
	antField.append(p.txAnt);
	const durField = A.el("div", "el-field");
	durField.append(A.el("label", null, "duration s"));
	p.txDur = A.el("input", "el-in num");
	p.txDur.type = "number"; p.txDur.value = "30"; p.txDur.min = "1";
	durField.append(p.txDur);
	p.armBtn = A.el("button", "el-btn arm", "ARM");
	p.disarmBtn = A.el("button", "el-btn", "disarm");
	txRow.append(pwField, antField, durField, p.armBtn, p.disarmBtn);
	p.txStats = A.el("div", "el-row", "");
	p.txWarn = A.el("div", "el-warn",
		"Arming transmits on the selected SMA at 2.4 GHz and stops the receiver while armed. Only with the board's TX policy, " +
		"--tx 1 and a power at or below the daemon cap; auto-disarm after the duration. Never with a receive-side LNA on that port.");
	tx.append(txHead, txRow, p.txStats, p.txWarn);

	p.armBtn.onclick = () => {
		if(S.posting) { return; }
		const body = { power_dbm: Number(p.txPower.value), antenna: p.txAnt.value || "sma_a",
			duration_s: Number(p.txDur.value) || 30, sync_packets: true };
		// second confirmation: arming radiates. Repeat the exact parameters back to the operator.
		if(!window.confirm("ARM the ELRS transmitter?\n\n" +
			"power " + body.power_dbm + " dBm on " + body.antenna + " for " + body.duration_s + " s at 2.4 GHz.\n" +
			"The receiver stops while armed. Make sure no receive-side LNA is on that port.")) { return; }
		S.posting = true;
		A.fetchJSON(BASE + "/tx/arm", { method: "POST", body: JSON.stringify(body) }, 10000)
			.then(() => showErr(p, ""))
			.catch(e => showErr(p, "arm refused: " + e.message))
			.finally(() => { S.posting = false; });
	};
	p.disarmBtn.onclick = () => {
		A.fetchJSON(BASE + "/tx/disarm", { method: "POST" }, 10000)
			.catch(e => showErr(p, "disarm failed: " + e.message));
	};

	ce.live.append(head, err, tiles, rcSec, rc, p.rcNote, mapSec, map, pkSec, tables, tx);
	Object.assign(p, { phrase, rate, uidBadge, lockBadge, stateBadge, err, errTxt, tables, tx, map });
	S.lastSeq = 0; S.packets = []; S.pktPrint = ""; S.chPrint = "";
	ce.panel = p;
	return p;
}

function kindCell(A, e) {
	const cls = !e.crc_ok ? "bad" : (e.kind === "rc" ? "rc" : e.kind === "sync" ? "sync" : "other");
	const label = !e.crc_ok ? (e.kind === "undecodable" ? "undec" : e.kind + " ✗") : e.kind;
	return A.el("span", "el-k " + cls, label);
}

function renderPackets(A, p) {
	const list = S.packets.slice(-50).reverse();
	const print = JSON.stringify(list.map(e => e.seq));
	if(print === S.pktPrint || A.selectionInside(p.tables)) { return; }
	S.pktPrint = print;
	p.tables.textContent = "";
	if(!list.length) {
		p.tables.appendChild(A.el("div", "el-empty", "no packets yet — set the binding phrase, then wait for bursts in the window"));
		return;
	}
	const rows = [];
	for(const e of list) {
		const tr = document.createElement("tr");
		tr.appendChild(A.el("td", "el-num", f1(e.t) + " s"));
		tr.appendChild(A.el("td", "el-num", String(e.slot)));
		const chTd = A.el("td", "el-num", e.channel >= 0 ? String(e.channel) : "—");
		if(e.channel_predicted >= 0 && e.channel_predicted !== e.channel) { chTd.textContent += " (" + e.channel_predicted + ")"; chTd.title = "predicted " + e.channel_predicted; }
		tr.appendChild(chTd);
		const kTd = A.el("td");
		kTd.appendChild(kindCell(A, e));
		tr.appendChild(kTd);
		const nTd = A.el("td", "el-num", e.nonce_conf === "none" ? "—" : String(e.nonce) + (e.nonce_conf === "mod_hop" ? " mod" : ""));
		nTd.title = "nonce confidence: " + e.nonce_conf;
		tr.appendChild(nTd);
		tr.appendChild(A.el("td", "el-num", f1(e.snr_db)));
		tr.appendChild(A.el("td", "el-num", e.crc_ok ? "ok" : "fail"));
		const hx = A.el("td", "el-hex", e.hex || "");
		if(e.truth === true) { hx.title = "matches ground truth"; } else if(e.truth === false) { hx.title = "MISMATCH vs ground truth"; hx.style.color = "#f66a85"; }
		tr.appendChild(hx);
		rows.push(tr);
	}
	p.tables.appendChild(A.table(["t", "slot", "ch", "kind", "nonce", "snr", "crc", "payload"], rows));
}

function renderMap(A, p, d, chans) {
	const per = chans && Array.isArray(chans.per_channel) ? chans.per_channel : [];
	const win = (d.rx && d.rx.window) || {};
	const cur = d.fhss ? d.fhss.channel : -1;
	const next = chans && Array.isArray(chans.next) ? chans.next : [];
	const print = JSON.stringify([per, win.ch_lo, win.ch_hi, cur, next]);
	if(print === S.chPrint) { return; }
	S.chPrint = print;
	let max = 1;
	for(const v of per) { if(v > max) { max = v; } }
	for(let i = 0; i < 80; i++) {
		const c = p.cells[i];
		const n = Number(per[i]) || 0;
		let cls = "el-cell";
		if(i >= win.ch_lo && i <= win.ch_hi) { cls += " win"; }
		if(n > 0) { cls += n >= max * 0.5 ? " hot" : " seen"; }
		if(i === cur) { cls += " cur"; }
		if(next.indexOf(i) >= 0) { cls += " next"; }
		if(i === 41) { cls += " sync"; }
		c.className = cls;
		c.title = "ch " + i + " · " + (2400.4 + i).toFixed(1) + " MHz · " + n + " bursts";
	}
}

function renderRc(A, p, d) {
	const rc = d.rc || {};
	const ch = Array.isArray(rc.ch) && rc.ch.length === 16 ? rc.ch : null;
	const modhop = rc.nonce_conf === "mod_hop";
	for(let i = 0; i < 16; i++) {
		const b = p.rcBars[i];
		const v = ch ? Number(ch[i]) : 992;
		const frac = Math.max(0, Math.min(1, (v - 172) / (1811 - 172)));
		b.fill.style.top = (100 - frac * 100) + "%";
		b.fill.style.bottom = "0";
		b.val.textContent = ch ? String(v) : "—";
		let cls = "el-rcbar" + (i === 4 ? " arm" : (i >= 5 ? " aux" : ""));
		if(!ch || (i >= 5 && modhop)) { cls += " dim"; }
		b.bar.className = cls;
		b.bar.title = "ch" + (i + 1) + (i === 4 ? " (arm)" : i >= 5 ? " (aux " + (i - 4) + ")" : "") + (i >= 5 && modhop ? " — aux slot ambiguous without a full nonce (needs a SYNC packet)" : "");
	}
	p.rcNote.textContent = !ch ? "" : ("last RC: slot " + rc.slot + " · ch4 " + (rc.ch4_high ? "HIGH (armed)" : "low") +
		(rc.switch_index >= 0 ? " · switch index " + rc.switch_index + (rc.aux_channel >= 0 ? " → ch" + (rc.aux_channel + 1) : " (power/telemetry slot)") : " · aux ambiguous (nonce " + rc.nonce_conf + ")"));
}

function render(id) {
	const A = window.RFNM_APPS;
	if(A.liveBusy[id]) { return; }
	A.liveBusy[id] = true;
	const p = ensurePanel(id);
	Promise.all([
		A.fetchJSON(BASE + "/status", undefined, 6000),
		A.fetchJSON(BASE + "/packets?since=" + S.lastSeq + "&max=200", undefined, 6000),
		A.fetchJSON(BASE + "/channels", undefined, 6000).catch(() => null),
	]).then(([d, pk, chans]) => {
		// badges
		p.uidBadge.className = "el-badge " + (d.uid_set ? "ok" : "warn");
		p.uidBadge.textContent = d.uid_set ? "uid set" : "uid unset";
		const lock = d.fhss ? d.fhss.lock : "none";
		p.lockBadge.className = "el-badge " + (lock === "full" ? "ok" : lock === "mod_hop" ? "warn" : "off");
		p.lockBadge.textContent = lock === "none" ? "no lock" : ("lock " + lock + (d.fhss.score ? " · " + d.fhss.score : ""));
		const st = d.state || "idle";
		p.stateBadge.className = "el-badge " + (st === "listening" ? "ok" : st === "tx_armed" ? "err" : "idle");
		p.stateBadge.textContent = st + (d.rate ? " · " + d.rate.name + " SF" + d.rate.sf : "");
		if(document.activeElement !== p.rate && d.rate) {
			const want = (d.rx && d.rx.rate_cfg === "auto") ? "auto" : d.rate.name;
			if(p.rate.value !== want) { p.rate.value = want; }
		}
		if(d.last_error && !S.err) { p.errTxt.textContent = d.last_error; p.err.style.display = "flex"; }

		// tiles
		const c = d.counters || {}, f = d.fhss || {}, rx = d.rx || {};
		p.tPkt._b.textContent = f1(c.packet_rate_hz);
		p.tLq._b.textContent = f1(c.lq_pct);
		p.tSnr._b.textContent = f1(c.snr_avg_db);
		p.tRssi._b.textContent = f1(c.rssi_avg_db);
		p.tCrc._b.textContent = n0(c.crc_ok) + " / " + n0(c.crc_fail);
		p.tSlot._b.textContent = n0(f.slot) + " / " + (f.nonce_conf === "none" ? "—" : n0(f.nonce) + (f.nonce_conf === "mod_hop" ? "m" : ""));
		p.tPtr._b.textContent = (f.index >= 0 ? n0(f.index) : "—") + " / " + (f.channel >= 0 ? n0(f.channel) : "—");
		p.tDrop._b.textContent = n0(rx.drops) + "+" + n0(rx.queue_drops) + "+" + n0(rx.ring_overruns);
		p.tDrop.title = "device drops + queue drops + ring overruns · bursts " + n0(rx.bursts) + " found " + n0(rx.demod_found) + " · cpu " + f1((rx.cpu_busy || 0) * 100) + " %";

		renderRc(A, p, d);
		renderMap(A, p, d, chans);

		// packets (incremental)
		if(pk && Array.isArray(pk.packets)) {
			for(const e of pk.packets) { if(e.seq > S.lastSeq) { S.lastSeq = e.seq; S.packets.push(e); } }
			if(S.packets.length > 200) { S.packets = S.packets.slice(-200); }
		}
		renderPackets(A, p);

		// tx block
		const t = d.tx || {};
		const canArm = t.policy && t.policy !== "none" && t.enabled;
		p.tx.className = "el-tx" + (t.armed ? " armed" : "");
		p.txBadge.className = "el-badge " + (t.armed ? "err" : canArm ? "warn" : "off");
		p.txBadge.textContent = t.armed ? "ARMED · " + f1(t.armed_remaining_s) + " s left" : canArm ? "ready (disarmed)" : "off";
		p.txPolicy.textContent = "board " + (t.board || d.board || "?") + " · policy " + (t.policy || "none") + " · --tx " + (t.enabled ? "1" : "0") +
			" · cap " + n0(t.power_cap_dbm) + " dBm" + (t.power_range ? " · range " + t.power_range[0] + ".." + t.power_range[1] : "");
		p.armBtn.disabled = !canArm || t.armed || S.posting || !d.uid_set;
		p.armBtn.title = !canArm ? "disabled: board policy none or daemon started without --tx 1" : !d.uid_set ? "set the binding phrase first" : "";
		p.disarmBtn.disabled = !t.armed;
		if(document.activeElement !== p.txPower && Number(p.txPower.value) > Number(t.power_cap_dbm)) { p.txPower.value = String(t.power_cap_dbm); }
		p.txStats.textContent = t.armed || t.bursts_sent
			? "LO " + f1((t.freq_lo_hz || 0) / 1e6) + " MHz · " + (t.antenna || "—") + " · " + n0(t.power_dbm_applied) + " dBm applied · sent " + n0(t.bursts_sent) +
			  " · dropped " + n0(t.bursts_dropped) + " · syncs " + n0(t.syncs_sent) + " · retunes " + n0(t.retunes) +
			  " (p50 " + f1(t.retune_ms && t.retune_ms.p50) + " / p99 " + f1(t.retune_ms && t.retune_ms.p99) + " ms)" +
			  (t.health ? " · refusals " + n0(t.health.refusals) : "")
			: "";
		if(t.last_error && !S.err) { p.errTxt.textContent = "tx: " + t.last_error; p.err.style.display = "flex"; }
	}).catch(() => {}).finally(() => { A.liveBusy[id] = false; });
}

(window.__RFNM_APP_MODULES = window.__RFNM_APP_MODULES || []).push({
	id: "elrs", title: "ELRS", icon: ICON,
	desc: "ExpressLRS 2.4 GHz sniffer: LoRa SF5-SF8 demod with the LI interleaver, FHSS follow from the binding phrase, RC channels, link quality, channel map and packet log. TX only when armed via the API and the board allows it.",
	live: render,
});

})();

// ADS-L panel (adsld). The daemon already serves the complete traffic map, so this panel only
// frames it: adsld's www/ uses relative URLs throughout and therefore works unchanged behind
// nginx at /api/app/adsl/. The header shows the state from GET /config + /traffic and links
// the map to a tab of its own. Installed to /rfnm/spectrum/www/ with a <script> tag in index.html.
"use strict";
(function() {

const BASE = "/api/app/adsl/";

const ICON = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.2">' +
	'<path d="M8 13 V8"/><circle cx="8" cy="6.5" r="1.3"/>' +
	'<path d="M4.8 9.7 a4.5 4.5 0 0 1 0 -6.4 M11.2 3.3 a4.5 4.5 0 0 1 0 6.4"/>' +
	'<path d="M2.8 11.7 a7.3 7.3 0 0 1 0 -10.4 M13.2 1.3 a7.3 7.3 0 0 1 0 10.4"/></svg>';

function injectStyle() {
	if(document.getElementById("al-style")) { return; }
	const st = document.createElement("style");
	st.id = "al-style";
	st.textContent = `
	.al-head { display:flex; align-items:center; gap:14px; flex-wrap:wrap; margin-bottom:10px;
		font-size:13px; color:#dae3ec; font-variant-numeric:tabular-nums; }
	.al-head a { color:#7fb2e5; margin-left:auto; }
	.al-frame { display:block; width:100%; height:calc(100vh - 220px); min-height:420px;
		border:1px solid #2a4260; border-radius:5px; background:#0f1c2d; }`;
	document.head.appendChild(st);
}

function ensurePanel(id) {
	const A = window.RFNM_APPS, ce = A.ui[id];
	if(ce.panel) { return ce.panel; }
	injectStyle();
	ce.live.textContent = "";
	const head = A.el("div", "al-head");
	const stat = A.el("span", "", "waiting for adsld …");
	const open = A.el("a", "", "open map in new tab");
	open.href = BASE;
	open.target = "_blank";
	open.rel = "noopener";
	head.appendChild(stat);
	head.appendChild(open);
	const frame = document.createElement("iframe");
	frame.className = "al-frame";
	frame.title = "ADS-L traffic map";
	frame.src = BASE;
	ce.live.appendChild(head);
	ce.live.appendChild(frame);
	ce.panel = { stat };
	return ce.panel;
}

function render(id) {
	const A = window.RFNM_APPS;
	if(A.liveBusy[id]) { return; }
	A.liveBusy[id] = true;
	const p = ensurePanel(id);
	Promise.all([
		A.fetchJSON(BASE + "config", undefined, 6000),
		A.fetchJSON(BASE + "traffic", undefined, 6000),
	]).then(([c, list]) => {
		p.stat.textContent = list.length + " aircraft · " + c.antenna + " · gain " + c.gain +
			" · TX " + (c.tx ? "on" : "off");
	}).catch(() => {
		p.stat.textContent = "adsld not answering";
	}).finally(() => { A.liveBusy[id] = false; });
}

(window.__RFNM_APP_MODULES = window.__RFNM_APP_MODULES || []).push({
	id: "adsl", title: "ADS-L", icon: ICON,
	desc: "ADS-L 868 MHz traffic map.",
	live: render,
});

})();

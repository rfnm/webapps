// LoRa visualizer (app_lora.js): full-screen live panels for lorad, the RX-only LoRa
// sniffer. Registers TWO apps that share this module:
//   - lorawan    : an improved live packet feed (time·MHz·SF·CR·BW·CRC·len·decoded),
//                  filterable, pausable, click-a-row to expand raw hex.
//   - meshtastic : a real mesh visualizer - node table, decoded text-message log, a
//                  force-directed node graph on a <canvas>, and (for POSITION frames)
//                  the nodes plotted on a Web-Mercator map (online tiles with an offline
//                  Natural-Earth coastline fallback, the same substrate as app_adsb.js).
//   - meshcore   : the same visualizer fed from MeshCore frames. Nodes come from adverts
//                  (name, role, position), the log from the default Public channel; private
//                  traffic is only counted by type. Relayed copies of a flood are one message.
//
// Data: /api/app/lora/status ({mode,region,channels,rate,samples,uptime,queue_drops,total,chans})
// and /api/app/lora/packets ({total, packets:[...]}). Meshtastic packets carry a structured
// "mesh":{from,to,id,hop_limit,hop_start,portnum,port,text,lat,lon,alt} object, MeshCore
// packets a "meshcore":{type,route,hops,path,hash,pubkey,name,role,lat,lon,src,dst,sender,
// text,...} object; nodes, edges and messages are accumulated across polls (the daemon ring only holds the last
// few hundred frames). Decode is proven by lorad's built-in selftest; live reception is
// luck indoors, so an empty panel is expected, not a fault.
"use strict";
(function() {

const APP = () => window.RFNM_APPS;

const ICON_LORA = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3"><path d="M8 14 V7 M8 7 a2.2 2.2 0 1 0 -0.01 0" fill="currentColor"/><path d="M3.6 9.2 a5.6 5.6 0 0 1 0-8.0 M12.4 9.2 a5.6 5.6 0 0 0 0-8.0" transform="translate(0,3.4)"/><path d="M5.3 7.9 a3.2 3.2 0 0 1 0-4.6 M10.7 7.9 a3.2 3.2 0 0 0 0-4.6" transform="translate(0,2.2)"/></svg>';
const ICON_MESH = '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3"><circle cx="8" cy="3" r="1.8"/><circle cx="3" cy="12.5" r="1.8"/><circle cx="13" cy="12.5" r="1.8"/><path d="M7 4.6 L4 10.9 M9 4.6 L12 10.9 M4.9 12.5 h6.2"/></svg>';

window.__RFNM_APP_MODULES = window.__RFNM_APP_MODULES || [];
// ONE merged app (panel "lora"): the daemon is mode-switched (lorawan | meshtastic | meshcore) through
// the manifest config, and this panel follows the mode the daemon actually reports.
window.__RFNM_APP_MODULES.push({
	id: "lora", title: "LoRa / Meshtastic / MeshCore", icon: ICON_LORA,
	desc: "RX-only LoRa sniffer (8-channel LoRaWAN dechirp-FFT scan) and Meshtastic / MeshCore mesh visualizer - switch modes from the config row.",
	live: render,
});
// which layout the panel is built for; refreshed from /api/app/lora/status .mode every poll
let meshMode = false;
let coreMode = false;   // meshMode fed from MeshCore frames

// ---- basemap tile sources (loaded by the viewer's browser; https on an http page is OK)
const SUBS = ["a", "b", "c", "d"];
const TILESETS = {
	map: { name: "map", label: "Dark map",
		url: (z, x, y) => "https://tile.openstreetmap.org/" + z + "/" + x + "/" + y + ".png",
		filter: "invert(1) hue-rotate(180deg) brightness(0.9)",
		attr: "© OpenStreetMap contributors" },
	sat: { name: "sat", label: "Satellite",
		url: (z, x, y) => "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/" + z + "/" + y + "/" + x,
		attr: "Imagery © Esri, Maxar, Earthstar Geographics" },
	coast: { name: "coast", label: "Coastline (offline)", attr: "Coastline: Natural Earth" },
};

// ---- scoped styles ---------------------------------------------------------------------
const css = `
.lora-wrap { display: flex; flex-direction: column; gap: 10px; height: 100%; }
.lora-typebar { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
.lora-chip { background: #15243a; border: 1px solid #1c2c40; border-radius: 20px; padding: 2px 11px;
	font-size: 11px; color: #9ab0c4; display: inline-flex; gap: 6px; align-items: center; }
.lora-chip b { color: #dae3ec; font-variant-numeric: tabular-nums; }
.lora-chip .cdot { width: 7px; height: 7px; border-radius: 2px; display: inline-block; }
.lora-ctl { display: flex; gap: 8px; align-items: center; }
.lora-ctl input[type=text] { background: #16283f; color: #dae3ec; border: 1px solid #2a4260; border-radius: 5px;
	font: inherit; padding: 3px 9px; width: 210px; }
.lora-tgl { background: #16283f; color: #9ab0c4; border: 1px solid #2a4260; border-radius: 5px; font: inherit;
	padding: 3px 12px; cursor: pointer; }
.lora-tgl.on { background: #0d2a23; color: #10b981; border-color: #1d5244; }
.lora-spacer { flex: 1 1 auto; }
.lora-count { color: #62798f; font-size: 11px; white-space: nowrap; }
.lora-tblwrap { flex: 1 1 auto; overflow-y: auto; min-height: 0; }
.lora-tblwrap .ap-table th { position: sticky; top: 0; background: #0d1725; z-index: 1; }
.crc { display: inline-block; padding: 0 6px; border-radius: 4px; font-size: 10px; letter-spacing: 0.5px; }
.crc.ok { background: rgba(16,185,129,0.14); color: #10b981; }
.crc.bad { background: rgba(226,69,60,0.18); color: #ff6b60; }
.crc.none { color: #4a6d8f; }
.lora-main { flex: 1 1 auto; display: flex; gap: 10px; min-height: 0; }
.lora-vizwrap { position: relative; flex: 1 1 auto; min-width: 0; }
.lora-canvas { width: 100%; height: 100%; background: #0a1019; border: 1px solid #1c2c40; border-radius: 8px;
	display: block; cursor: crosshair; }
.lora-vctl { position: absolute; top: 8px; left: 8px; display: flex; gap: 6px; align-items: center;
	background: rgba(8,11,18,0.82); border: 1px solid #1c2c40; border-radius: 7px; padding: 5px 8px; backdrop-filter: blur(4px); }
.lora-vctl select { background: #16283f; color: #dae3ec; border: 1px solid #2a4260; border-radius: 5px; font: inherit; padding: 2px 6px; }
.lora-legend { position: absolute; bottom: 8px; left: 8px; display: flex; gap: 10px; align-items: center;
	background: rgba(8,11,18,0.78); border: 1px solid #1c2c40; border-radius: 7px; padding: 4px 9px;
	color: #9ab0c4; font-size: 10px; flex-wrap: wrap; max-width: calc(100% - 40px); }
.lora-legend .sw { width: 9px; height: 9px; border-radius: 2px; display: inline-block; margin-right: 3px; vertical-align: middle; }
.lora-attr { position: absolute; bottom: 6px; right: 8px; color: #6f87a0; font-size: 9.5px;
	background: rgba(4,6,12,0.6); padding: 1px 6px; border-radius: 4px; pointer-events: none; }
.lora-detail { position: absolute; top: 8px; right: 8px; width: 214px; background: rgba(8,11,18,0.94);
	border: 1px solid #2a4260; border-radius: 8px; padding: 10px 12px; box-shadow: 0 6px 24px rgba(0,0,0,0.5); }
.lora-detail h4 { color: #cfe2f2; font-size: 13px; font-weight: normal; margin-bottom: 7px; display: flex; gap: 6px; align-items: center; }
.lora-detail dl { display: grid; grid-template-columns: auto 1fr; gap: 2px 10px; font-size: 11px; }
.lora-detail dt { color: #62798f; }
.lora-detail dd { color: #dae3ec; text-align: right; font-variant-numeric: tabular-nums; word-break: break-all; }
.lora-detail .cx { position: absolute; top: 6px; right: 9px; color: #9ab0c4; cursor: pointer; background: none;
	border: none; font: inherit; font-size: 15px; line-height: 1; }
.lora-detail .cx:hover { color: #f43f5e; }
.lora-vizhint { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
	text-align: center; color: #62798f; font-size: 12px; padding: 0 40px; pointer-events: none; }
.lora-side { flex: 0 0 340px; display: flex; flex-direction: column; gap: 8px; min-height: 0; }
.lora-side-h { color: #62798f; font-size: 10px; text-transform: uppercase; letter-spacing: 1px;
	display: flex; justify-content: space-between; }
.lora-side-h b { color: #9ab0c4; font-weight: normal; }
.lora-nodewrap { flex: 1 1 42%; overflow-y: auto; min-height: 64px; border: 1px solid #16222f; border-radius: 6px; }
.lora-msgwrap { flex: 1 1 58%; overflow-y: auto; min-height: 64px; border: 1px solid #16222f; border-radius: 6px; }
.lora-ntbl { width: 100%; border-collapse: collapse; }
.lora-ntbl th { position: sticky; top: 0; background: #0d1725; color: #62798f; text-align: left; font-weight: normal;
	font-size: 10px; text-transform: uppercase; letter-spacing: 1px; padding: 4px 8px; white-space: nowrap; }
.lora-ntbl td { color: #dae3ec; padding: 3px 8px; border-top: 1px solid #16222f; font-variant-numeric: tabular-nums; white-space: nowrap; }
.lora-ntbl tr.row { cursor: pointer; }
.lora-ntbl tr.row:hover td { background: #15243a; }
.lora-ntbl tr.sel td { background: #0d2a23; }
.lora-ntbl td .ndot { width: 7px; height: 7px; border-radius: 2px; display: inline-block; margin-right: 6px; vertical-align: middle; }
.lora-emptyrow td { color: #4a6d8f; padding: 10px 8px; }
.lora-msg { padding: 5px 9px; border-top: 1px solid #0e1626; font-size: 11px; }
.lora-msg:first-child { border-top: none; }
.lora-msg .m-h { color: #62798f; display: flex; gap: 6px; align-items: baseline; }
.lora-msg .m-from { color: #10b981; } .lora-msg .m-arrow { color: #3e6188; } .lora-msg .m-to { color: #9ab0c4; }
.lora-msg .m-time { margin-left: auto; color: #4a6d8f; font-variant-numeric: tabular-nums; }
.lora-msg .m-t { color: #ecf2f8; margin-top: 3px; word-break: break-word; }
.lora-msg-empty { color: #4a6d8f; padding: 12px 10px; font-size: 12px; text-align: center; }
`;
if(!document.getElementById("lora-style")) {
	const st = document.createElement("style");
	st.id = "lora-style";
	st.textContent = css;
	document.head.appendChild(st);
}

// ---- small helpers ---------------------------------------------------------------------
const rgba = (c, a) => "rgba(" + c[0] + "," + c[1] + "," + c[2] + "," + a + ")";
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const shortId = s => (s && s[0] === "!") ? s.slice(-4) : s;
const nodeLabel = nd => (nd && nd.name) ? nd.name : shortId(nd ? nd.id : "");

// SNR colour ramp (weak red -> strong green), same idea as the ADS-B altitude ramp
const SNR_RAMP = [[-20, [226, 69, 60]], [-13, [255, 140, 66]], [-7, [255, 212, 121]], [-2, [155, 229, 100]], [4, [55, 214, 122]]];
function snrColor(snr) {
	const s = Number(snr);
	if(!isFinite(s)) { return [140, 160, 190]; }
	if(s <= SNR_RAMP[0][0]) { return SNR_RAMP[0][1]; }
	for(let i = 1; i < SNR_RAMP.length; i++) {
		if(s <= SNR_RAMP[i][0]) {
			const [a0, c0] = SNR_RAMP[i - 1], [a1, c1] = SNR_RAMP[i];
			const t = (s - a0) / (a1 - a0);
			return c0.map((v, k) => Math.round(v + (c1[k] - v) * t));
		}
	}
	return SNR_RAMP[SNR_RAMP.length - 1][1];
}
const SNR_LEGEND = [["weak", snrColor(-20)], ["", snrColor(-12)], ["", snrColor(-6)], ["strong", snrColor(2)]];

function crBadge(a, pk) {
	if(!pk.cr) { return "—"; }
	return "4/" + (4 + pk.cr);
}
function crcCell(a, pk) {
	const span = a.el("span");
	if(!pk.crc_present) { span.className = "crc none"; span.textContent = "none"; }
	else if(pk.crc_ok) { span.className = "crc ok"; span.textContent = "OK"; }
	else { span.className = "crc bad"; span.textContent = "BAD"; }
	return span;
}
const portColor = { TEXT: [55, 214, 122], POSITION: [95, 191, 255], NODEINFO: [180, 150, 255],
	TELEMETRY: [255, 212, 121], ROUTING: [140, 160, 190], TRACEROUTE: [255, 140, 66],
	ADVERT: [180, 150, 255], GRP_TXT: [55, 214, 122], TXT_MSG: [95, 191, 255], ACK: [140, 160, 190],
	PATH: [255, 212, 121], TRACE: [255, 140, 66] };
function typeColor(name) { return portColor[name] || [125, 150, 185]; }

// ---- Web Mercator + tiles (map view) ---------------------------------------------------
const RAD = Math.PI / 180;
const merc = { x: lon => (lon + 180) / 360, y: lat => (1 - Math.asinh(Math.tan(lat * RAD)) / Math.PI) / 2 };
const S = { tiles: new Map(), geoBox: null, geoLoading: false, vizPanel: null,
	manual: null, drag: null, dragged: false, lastV: null };   // manual = {lat, lon, z}: user view

function viewOf(cv, center, radiusKm) {
	if(S.manual) { center = S.manual; }
	const w = cv.clientWidth, h = cv.clientHeight;
	const halfPx = Math.min(w, h) / 2 - 22;
	const mppWant = (radiusKm * 1000) / Math.max(halfPx, 1);
	let z = Math.log2(156543.03 * Math.cos(center.lat * RAD) / mppWant);
	z = S.manual ? S.manual.z : clamp(z, 2, 15);
	const world = 256 * Math.pow(2, z);
	const cwx = merc.x(center.lon) * world, cwy = merc.y(center.lat) * world;
	const cx = w / 2, cy = h / 2;
	return { w, h, cx, cy, z, world, c: center,
		mpp: 156543.03 * Math.cos(center.lat * RAD) / Math.pow(2, z),
		toXY: (lon, lat) => [cx + merc.x(lon) * world - cwx, cy + merc.y(lat) * world - cwy] };
}
// ---- manual view: wheel zoom around the pointer, drag pan, double click or Auto = auto-fit --
const unmerc = (fx, fy) => ({
	lon: fx * 360 - 180,
	lat: Math.atan(Math.sinh(Math.PI * (1 - 2 * Math.max(0.001, Math.min(0.999, fy))))) / RAD,
});
function centreFor(fx, fy, dx, dy, z) {
	const world = 256 * Math.pow(2, z);
	return { ...unmerc(fx - dx / world, fy - dy / world), z };
}
function pointerFrac(V, mx, my) {
	return [merc.x(V.c.lon) + (mx - V.cx) / V.world, merc.y(V.c.lat) + (my - V.cy) / V.world];
}
function mapPos(cv, ev) {
	const r = cv.getBoundingClientRect();
	return [ev.clientX - r.left, ev.clientY - r.top];
}
function zoomAt(cv, V, ev) {
	const [mx, my] = mapPos(cv, ev);
	const [fx, fy] = pointerFrac(V, mx, my);
	const px = ev.deltaMode === 1 ? ev.deltaY * 16 : ev.deltaY;
	const z = Math.max(3, Math.min(17, V.z - Math.max(-0.6, Math.min(0.6, px * 0.002))));
	S.manual = centreFor(fx, fy, mx - V.cx, my - V.cy, z);
}
function dragStart(cv, V, ev) {
	const [mx, my] = mapPos(cv, ev);
	const [fx, fy] = pointerFrac(V, mx, my);
	S.drag = { x: mx, y: my, fx, fy, z: V.z, cx: V.cx, cy: V.cy };
	S.dragged = false;
	cv.setPointerCapture(ev.pointerId);
}
function dragMove(cv, ev) {
	const d = S.drag;
	if(!d) { return false; }
	const [mx, my] = mapPos(cv, ev);
	if(!S.dragged && Math.hypot(mx - d.x, my - d.y) < 4) { return false; }
	S.dragged = true;
	cv.style.cursor = "grabbing";
	S.manual = centreFor(d.fx, d.fy, mx - d.cx, my - d.cy, d.z);
	return true;
}
function dragEnd(cv) {
	S.drag = null;
	cv.style.cursor = "";
}

function markVizDirty() { if(S.vizPanel && S.vizPanel._viz) { S.vizPanel._viz.dirty = true; } }
function getTile(set, z, x, y) {
	const key = set.name + "/" + z + "/" + x + "/" + y;
	let img = S.tiles.get(key);
	if(img === undefined) {
		img = new Image();
		img._ok = false;
		img.onload = () => { img._ok = true; markVizDirty(); };
		img.onerror = () => { img._ok = false; markVizDirty(); };
		img.src = set.url(z, x, y);
		S.tiles.set(key, img);
		if(S.tiles.size > 400) { S.tiles.delete(S.tiles.keys().next().value); }
	}
	return img;
}
function drawTiles(g, view, set) {
	const tz = clamp(Math.round(view.z), 0, 19);
	const scale = Math.pow(2, view.z - tz);
	const tworld = 256 * Math.pow(2, tz), nt = Math.pow(2, tz);
	const cwx = merc.x(view.c.lon) * tworld, cwy = merc.y(view.c.lat) * tworld;
	const sx = px => (px - cwx) * scale + view.cx, sy = py => (py - cwy) * scale + view.cy;
	const tx0 = Math.floor((cwx - view.cx / scale) / 256), tx1 = Math.floor((cwx + view.cx / scale) / 256);
	const ty0 = Math.floor((cwy - view.cy / scale) / 256), ty1 = Math.floor((cwy + view.cy / scale) / 256);
	let ok = false;
	if(set.filter) { g.filter = set.filter; }
	for(let tx = tx0; tx <= tx1; tx++) {
		for(let ty = ty0; ty <= ty1; ty++) {
			if(ty < 0 || ty >= nt) { continue; }
			const X = ((tx % nt) + nt) % nt;
			const img = getTile(set, tz, X, ty);
			if(img._ok) {
				const dsz = 256 * scale + 1;
				try { g.drawImage(img, sx(tx * 256), sy(ty * 256), dsz, dsz); ok = true; } catch(e) { /* incomplete */ }
			}
		}
	}
	if(set.filter) { g.filter = "none"; }
	return ok;
}
function ensureGeo() {
	if(window.__RFNM_GEO || S.geoLoading) { return; }
	S.geoLoading = true;
	const s = document.createElement("script");
	s.src = "geodata.js?v=1";
	s.onload = () => {
		const co = (window.__RFNM_GEO || {}).coast || [];
		S.geoBox = co.map(fl => {
			let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
			for(let i = 0; i < fl.length; i += 2) {
				if(fl[i] < x0) { x0 = fl[i]; } if(fl[i] > x1) { x1 = fl[i]; }
				if(fl[i + 1] < y0) { y0 = fl[i + 1]; } if(fl[i + 1] > y1) { y1 = fl[i + 1]; }
			}
			return [x0, y0, x1, y1];
		});
		markVizDirty();
	};
	document.head.appendChild(s);
}
function drawCoast(g, view, radiusKm) {
	ensureGeo();
	if(!window.__RFNM_GEO || !S.geoBox) { return false; }
	const co = window.__RFNM_GEO.coast;
	const kx = 111.320 * Math.cos(view.c.lat * RAD), ky = 110.574;
	const dlon = (radiusKm * 1.8) / kx, dlat = (radiusKm * 1.8) / ky;
	const vx0 = view.c.lon - dlon, vx1 = view.c.lon + dlon, vy0 = view.c.lat - dlat, vy1 = view.c.lat + dlat;
	g.strokeStyle = "#2a4260"; g.lineWidth = 1; g.beginPath();
	for(let k = 0; k < co.length; k++) {
		const b = S.geoBox[k];
		if(b[2] < vx0 || b[0] > vx1 || b[3] < vy0 || b[1] > vy1) { continue; }
		const fl = co[k];
		for(let i = 0; i < fl.length; i += 2) {
			const xy = view.toXY(fl[i], fl[i + 1]);
			if(i === 0) { g.moveTo(xy[0], xy[1]); } else { g.lineTo(xy[0], xy[1]); }
		}
	}
	g.stroke();
	return true;
}

// ---- per-app state ---------------------------------------------------------------------
const ST = {};
function freshState() {
	return {
		lastSamples: null, lastT: null, rate: null,
		tilePrint: "", chanPrint: "", typePrint: "", nodePrint: "", msgPrint: "",
		packets: [], pktPrint: "", expanded: new Set(), filter: "", paused: false, autoscroll: true,
		seen: new Set(), nodes: new Map(), edges: new Map(), messages: [], typeCounts: {}, floods: new Set(),
		pos: new Map(), selNode: null, meshView: "graph", basemap: "map",
	};
}
function stateOf(id) { if(!ST[id]) { ST[id] = freshState(); } return ST[id]; }
function resetState(id) { ST[id] = freshState(); return ST[id]; }

// ==========================================================================================
// entry point (polled ~2 s by the parent while the app is active + shown)
// ==========================================================================================
function render(id) {
	const a = APP();
	if(a.liveBusy[id]) { return; }
	a.liveBusy[id] = true;
	const p = ensurePanel(id);
	const st = stateOf(id);
	const done = () => { a.liveBusy[id] = false; };

	a.fetchJSON("/api/app/lora/status", undefined, 6000).then(status => {
		// mode switch (daemon restarted in the other personality): rebuild the panel layout
		const core = status.mode === "meshcore";
		const mesh = status.mode === "meshtastic" || core;
		if(mesh !== meshMode || core !== coreMode) {
			meshMode = mesh;
			coreMode = core;
			a.ui[id].panel = null;
			a.ui[id].live.textContent = "";
			return;   // the next poll rebuilds with the right layout
		}
		const t = performance.now() / 1000;
		if(st.lastSamples !== null && status.samples < st.lastSamples) {
			// daemon restarted: reset the rate estimate and the accumulated mesh model
			st.lastSamples = st.lastT = null; st.rate = null;
			st.seen.clear(); st.nodes.clear(); st.edges.clear(); st.pos.clear();
			st.messages.length = 0; st.typeCounts = {}; st.selNode = null; st.floods.clear();
			st.nodePrint = st.msgPrint = st.typePrint = "";
		}
		if(st.lastSamples !== null && t > st.lastT) {
			const inst = (status.samples - st.lastSamples) / (t - st.lastT) / 1e6;
			st.rate = st.rate === null ? inst : 0.6 * st.rate + 0.4 * inst;
		}
		st.lastSamples = status.samples; st.lastT = t;
		updateTiles(id, p, st, status);
		updateChans(id, p, st, status);
	}).catch(() => {});

	a.fetchJSON("/api/app/lora/packets", undefined, 6000).then(d => {
		const pkts = d.packets || [];
		if(meshMode) {
			const added = ingestMesh(st, pkts);
			if(added && p._viz) { p._viz.dirty = true; }   // refresh map + reduced-motion graph on new data
			updateTypebar(id, p, st);
			buildNodeTable(id);
			buildMsgLog(id);
		} else {
			if(!st.paused) { st.packets = pkts.slice(-200).reverse(); }
			buildPacketTable(id);
		}
	}).catch(() => {}).finally(done);
}

// ---- panel skeleton --------------------------------------------------------------------
// MeshCore send box. Stays hidden unless the daemon reports TX enabled (board policy + --tx 1).
// Sending needs a separate, time-limited arm; the daemon re-checks every gate on each send.
function buildTxBox(a) {
	const box = a.el("div", "lora-tx");
	box.style.cssText = "display:none;margin-top:10px;padding:8px;border:1px solid #2a4260;border-radius:6px;font-size:12px";
	const state = a.el("div", null, "");
	const row = a.el("div", null, "");
	row.style.cssText = "display:flex;gap:6px;margin-top:6px";
	const chan = document.createElement("select");
	for(const c of ["public", "#ping", "#test", "#hansemesh"]) {
		const o = document.createElement("option");
		o.value = o.textContent = c;
		chan.appendChild(o);
	}
	const fixedChans = Array.from(chan.options).map(o => o.value);
	const addRow = a.el("div", null, "");
	addRow.style.cssText = "display:flex;gap:6px;margin-top:6px";
	const newName = document.createElement("input");
	newName.type = "text";
	newName.maxLength = 32;
	newName.placeholder = "channel name or #hashtag";
	const newKey = document.createElement("input");
	newKey.type = "password";
	newKey.maxLength = 32;
	newKey.placeholder = "key, 32 hex digits (private only)";
	newKey.autocomplete = "off";
	newKey.style.flex = "1";
	const addBtn = a.el("button", "lora-tgl", "add channel");
	const delBtn = a.el("button", "lora-tgl", "remove selected");
	addRow.append(newName, newKey, addBtn, delBtn);
	const text = document.createElement("input");
	text.type = "text";
	text.maxLength = 140;
	text.placeholder = "message";
	text.style.flex = "1";
	const send = a.el("button", "lora-tgl", "send");
	const arm = a.el("button", "lora-tgl", "arm 30 min");
	row.append(chan, text, send, arm);
	const note = a.el("div", null, "");
	note.style.marginTop = "6px";
	box.append(state, row, addRow, note);
	let armed = false;
	const post = (p, body) => fetch("/api/app/lora" + p, { method: "POST", body: JSON.stringify(body || {}) })
		.then(r => r.json().then(j => ({ ok: r.ok, j })));
	const say = r => { note.textContent = r.ok ? (r.j.sent ? "sent, " + r.j.airtime + " s on air" : "ok") : "refused: " + r.j.error; };
	const poll = () => {
		if(!box.isConnected && box.dataset.seen) { return; }
		fetch("/api/app/lora/tx").then(r => r.json()).then(t => {
			box.dataset.seen = "1";
			box.style.display = t.enabled ? "block" : "none";
			armed = t.armed;
			arm.textContent = armed ? "disarm" : "arm 30 min";
			send.disabled = !armed;
			state.textContent = t.node + " " + t.pubkey + " · " + t.power_dbm + " dBm · scope " + t.scope + " · duty " + t.duty_used_s + "/" + t.duty_cap_s + " s" + (armed ? " · ARMED " + t.armed_s + " s" : "");
		}).catch(() => {}).finally(() => { setTimeout(poll, 2000); });
	};
	// names only: the daemon never hands keys back, and the key field clears after each add
	const loadChans = keep => Promise.all([fetch("/api/app/lora/channels").then(r => r.json()),
		fetch("/api/app/lora/contacts").then(r => r.json())]).then(([j, k]) => {
		const cur = keep || chan.value;
		chan.textContent = "";
		for(const c of fixedChans.concat((j.channels || []).filter(n => !fixedChans.includes(n)))) {
			const o = document.createElement("option");
			o.value = o.textContent = c;
			chan.appendChild(o);
		}
		for(const c of k.contacts || []) {
			const o = document.createElement("option");
			o.value = "dm:" + c.key;
			o.textContent = "DM " + c.name;
			chan.appendChild(o);
		}
		if(Array.from(chan.options).some(o => o.value === cur)) { chan.value = cur; }
	}).catch(() => {});
	addBtn.onclick = () => {
		const name = newName.value.trim();
		post("/channels/add", { name, key: newKey.value.trim() }).then(r => {
			say(r);
			if(r.ok) { newName.value = ""; newKey.value = ""; loadChans(name); }
		});
	};
	delBtn.onclick = () => post("/channels/remove", { name: chan.value }).then(r => { say(r); if(r.ok) { loadChans(); } });
	loadChans();
	setInterval(() => { if(box.isConnected && document.activeElement !== chan) { loadChans(); } }, 15000);
	arm.onclick = () => post(armed ? "/tx/disarm" : "/tx/arm", { seconds: 1800 }).then(say);
	send.onclick = () => {
		note.textContent = "waiting for a clear channel…";
		const dest = chan.value.startsWith("dm:") ? { to: chan.value.slice(3) } : { channel: chan.value };
		post("/send", { ...dest, text: text.value }).then(r => { say(r); if(r.ok) { text.value = ""; } });
	};
	poll();
	return box;
}

function ensurePanel(id) {
	const a = APP(), ce = a.ui[id];
	if(ce.panel) { return ce.panel; }
	resetState(id);
	ce.live.textContent = "";
	// make this pane's body a flex column so a height:100% wrap fills the screen (the shared
	// .app-pbody/.ap-live chain is otherwise auto-height and the wrap would collapse). Scoped
	// to this app's own pane, so it never touches other apps.
	const pbody = ce.live.parentElement;
	if(pbody) { pbody.style.display = "flex"; pbody.style.flexDirection = "column"; }
	ce.live.style.flex = "1 1 auto";
	ce.live.style.minHeight = "0";
	const st = stateOf(id);

	const wrap = a.el("div", "lora-wrap");
	const tiles = a.el("div", "ap-tiles");
	const chans = a.el("div", "ap-chanstrip");

	if(meshMode) {
		const typebar = a.el("div", "lora-typebar");
		const main = a.el("div", "lora-main");
		// viz (graph / map)
		const vizwrap = a.el("div", "lora-vizwrap");
		const canvas = document.createElement("canvas");
		canvas.className = "lora-canvas";
		const vctl = a.el("div", "lora-vctl");
		const seg = a.el("span", "ap-seg");
		const bGraph = a.el("button", "on", "Graph");
		const bMap = a.el("button", null, "Map");
		seg.append(bGraph, bMap);
		const bmSel = document.createElement("select");
		for(const k of ["map", "sat", "coast"]) { const o = document.createElement("option"); o.value = k; o.textContent = TILESETS[k].label; bmSel.appendChild(o); }
		bmSel.value = st.basemap;
		bmSel.style.display = "none";
		bmSel.onchange = () => { st.basemap = bmSel.value; markVizDirty(); };
		bGraph.onclick = () => { st.meshView = "graph"; bGraph.className = "on"; bMap.className = ""; bmSel.style.display = "none"; markVizDirty(); };
		bMap.onclick = () => { st.meshView = "map"; bMap.className = "on"; bGraph.className = ""; bmSel.style.display = ""; markVizDirty(); };
		const fit = document.createElement("button");
		fit.type = "button";
		fit.textContent = "Auto";
		fit.title = "back to the automatic view (or double-click the map)";
		fit.onclick = () => { S.manual = null; markVizDirty(); };
		vctl.append(seg, bmSel, fit);
		const legend = a.el("div", "lora-legend");
		const attr = a.el("div", "lora-attr");
		const detail = a.el("div", "lora-detail");
		detail.style.display = "none";
		const vizhint = a.el("div", "lora-vizhint");
		canvas.onclick = ev => {
			if(S.dragged) { S.dragged = false; return; }   // the click that ends a drag selects nothing
			graphHit(id, canvas, ev);
		};
		const onMap = () => stateOf(id).meshView === "map" && S.lastV;
		canvas.ondblclick = () => { if(onMap()) { S.manual = null; markVizDirty(); } };
		canvas.addEventListener("wheel", ev => {
			if(!onMap()) { return; }
			ev.preventDefault();
			zoomAt(canvas, S.lastV, ev);
			markVizDirty();
		}, { passive: false });
		canvas.addEventListener("pointerdown", ev => {
			if(onMap() && ev.button === 0 && ev.pointerType !== "touch") { dragStart(canvas, S.lastV, ev); }
		});
		canvas.addEventListener("pointermove", ev => { if(dragMove(canvas, ev)) { markVizDirty(); } });
		canvas.addEventListener("pointerup", () => dragEnd(canvas));
		canvas.addEventListener("pointercancel", () => dragEnd(canvas));
		vizwrap.append(canvas, vctl, legend, attr, detail, vizhint);
		// side (nodes + messages)
		const side = a.el("div", "lora-side");
		const nodeH = a.el("div", "lora-side-h");
		const nodeHl = a.el("span", null, "Nodes");
		const nodeHc = a.el("b", null, "");
		nodeH.append(nodeHl, nodeHc);
		const nodewrap = a.el("div", "lora-nodewrap");
		const msgH = a.el("div", "lora-side-h");
		const msgHl = a.el("span", null, "Text messages");
		const msgHc = a.el("b", null, "");
		msgH.append(msgHl, msgHc);
		const msgwrap = a.el("div", "lora-msgwrap");
		side.append(nodeH, nodewrap, msgH, msgwrap);
		if(coreMode) { side.append(buildTxBox(a)); }
		main.append(vizwrap, side);
		wrap.append(tiles, typebar, chans, main);
		ce.live.append(wrap);
		ce.panel = { tiles, typebar, chans, canvas, vizwrap, legend, attr, detail, vizhint,
			nodewrap, nodeHc, msgwrap, msgHc, _viz: { raf: 0, dirty: true } };
		startViz(id, ce.panel);
	} else {
		const ctl = a.el("div", "lora-ctl");
		const filter = document.createElement("input");
		filter.type = "text";
		filter.placeholder = "filter (SF, dev, port, hex…)";
		filter.oninput = () => { st.filter = filter.value.trim().toLowerCase(); buildPacketTable(id, true); };
		const bPause = a.el("button", "lora-tgl", "pause");
		bPause.onclick = () => { st.paused = !st.paused; bPause.classList.toggle("on", st.paused); bPause.textContent = st.paused ? "paused" : "pause"; };
		const bScroll = a.el("button", "lora-tgl on", "auto-scroll");
		bScroll.onclick = () => { st.autoscroll = !st.autoscroll; bScroll.classList.toggle("on", st.autoscroll); };
		const spacer = a.el("span", "lora-spacer");
		const count = a.el("span", "lora-count", "");
		ctl.append(filter, bPause, bScroll, spacer, count);
		const tblwrap = a.el("div", "lora-tblwrap");
		const hint = a.el("div", "ap-hint");
		wrap.append(tiles, chans, ctl, tblwrap, hint);
		ce.live.append(wrap);
		ce.panel = { tiles, chans, tblwrap, hint, count };
	}
	return ce.panel;
}

// ---- shared header: tiles + per-channel strip ------------------------------------------
function updateTiles(id, p, st, status) {
	const a = APP();
	const rate = (st.rate && st.rate > 0.01) ? st.rate : (status.rate || 0) / 1e6;
	const bits = [
		["mode / region", (status.mode || "?") + " / " + (status.region || "?")],
		["channels", String(status.channels || 0)],
		["sample rate", rate.toFixed(2) + " MSps"],
		["packets", String(status.total || 0)],
		["queue drops", String(status.queue_drops || 0), (status.queue_drops || 0) > 0],
		["uptime", a.fmtUp(status.uptime)],
	];
	if(meshMode) {
		bits.push(["nodes", String(st.nodes.size)]);
		bits.push(["messages", String(st.messages.length)]);
	}
	const print = JSON.stringify(bits);
	if(print === st.tilePrint) { return; }
	st.tilePrint = print;
	p.tiles.textContent = "";
	for(const b of bits) { p.tiles.append(a.tile(b[1], b[0], b[2])); }
}

function updateChans(id, p, st, status) {
	const a = APP();
	const print = JSON.stringify(status.chans || []);
	if(print === st.chanPrint) { return; }
	st.chanPrint = print;
	p.chans.textContent = "";
	for(const ch of status.chans || []) {
		const cls = ch.packets > 0 ? " hot" : (ch.drops > 0 ? " drop" : "");
		const box = a.el("div", "ap-chan" + cls);
		box.append(a.el("div", "f", (ch.freq / 1e6).toFixed(3)));
		let s = ch.packets + " pkt";
		if(ch.last_snr !== undefined) { s += " · " + ch.last_snr.toFixed(1) + " dB"; }
		if(ch.drops > 0) { s += " · " + ch.drops + " drop"; }
		box.append(a.el("div", "c", s));
		box.title = (ch.freq / 1e6).toFixed(3) + " MHz · queue " + ch.queue;
		p.chans.append(box);
	}
}

// ==========================================================================================
// LoRaWAN: live packet feed
// ==========================================================================================
function pktKey(pk) { return pk.epoch_ms + "|" + (pk.hex || "").slice(0, 20); }

const LCOLS = [
	{ h: "time", get: pk => pk.time },
	{ h: "MHz", get: pk => (pk.freq / 1e6).toFixed(3) },
	{ h: "SF", get: pk => "SF" + pk.sf },
	{ h: "CR", get: (a, pk) => crBadge(a, pk) },
	{ h: "BW", get: pk => pk.bw ? (pk.bw / 1e3).toFixed(0) : "—" },
	{ h: "CRC", node: crcCell },
	{ h: "len", get: pk => pk.len },
	{ h: "decoded", get: pk => pk.info || pk.hex || "" },
];

function buildPacketTable(id, force) {
	const a = APP(), ce = a.ui[id];
	if(!ce.panel) { return; }
	const p = ce.panel, st = stateOf(id);
	let pkts = st.packets;
	if(st.filter) {
		pkts = pkts.filter(pk => ("sf" + pk.sf + " " + (pk.info || "") + " " + (pk.hex || "") + " " +
			(pk.bw ? (pk.bw / 1e3) : "") + " 4/" + (4 + pk.cr)).toLowerCase().includes(st.filter));
	}
	p.count.textContent = pkts.length + (st.paused ? " frames · paused" : " frames");
	if(!pkts.length) {
		const print = "empty|" + st.filter + "|" + st.paused;
		if(!force && print === st.pktPrint) { return; }
		st.pktPrint = print;
		p.tblwrap.textContent = "";
		p.hint.textContent = st.filter ? "no frames match the filter" :
			"listening… no frames decoded yet (indoor reception is luck — the decode chain is proven by lorad's built-in selftest)";
		return;
	}
	const keys = new Set(pkts.map(pktKey));
	for(const k of st.expanded) { if(!keys.has(k)) { st.expanded.delete(k); } }
	const print = JSON.stringify(pkts.map(pk => [pk.time, pk.freq, pk.sf, pk.cr, pk.bw, pk.crc_present, pk.crc_ok, pk.len, pk.info])) +
		"|" + [...st.expanded].join(",") + "|" + st.filter;
	if(!force && (print === st.pktPrint || a.selectionInside(p.tblwrap))) { return; }
	st.pktPrint = print;
	p.hint.textContent = "click a row for the raw payload";

	const rows = [];
	for(const pk of pkts) {
		const key = pktKey(pk);
		const row = a.el("tr", "exp-toggle");
		for(const c of LCOLS) {
			const td = a.el("td");
			if(c.node) { td.append(c.node(a, pk)); }
			else { td.textContent = String(c.get.length === 2 ? c.get(a, pk) : c.get(pk)); }
			row.append(td);
		}
		row.onclick = () => {
			if(st.expanded.has(key)) { st.expanded.delete(key); } else { st.expanded.add(key); }
			buildPacketTable(id, true);
		};
		rows.push(row);
		if(st.expanded.has(key)) {
			const hexRow = a.el("tr", "ap-hexrow");
			const td = a.el("td", null, pk.hex || "(empty payload)");
			td.colSpan = LCOLS.length;
			hexRow.append(td);
			rows.push(hexRow);
		}
	}
	p.tblwrap.textContent = "";
	p.tblwrap.append(a.table(LCOLS.map(c => c.h), rows));
	if(st.autoscroll && !st.paused) { p.tblwrap.scrollTop = 0; }
}

// ==========================================================================================
// Meshtastic: mesh model + visualizer
// ==========================================================================================
function ingestMesh(st, pkts) {
	const keys = new Set();
	let added = 0;
	for(const pk of pkts) {
		const k = pktKey(pk);
		keys.add(k);
		if(st.seen.has(k)) { continue; }
		st.seen.add(k);
		added++;
		if(pk.meshcore) { ingestCore(st, pk, pk.meshcore); continue; }
		const m = pk.mesh;
		// message-type tally (works even for undecodable frames)
		const type = m ? (m.port || (m.decoded ? ("port " + m.portnum) : (pk.len > 16 ? "encrypted" : "flood"))) : "raw";
		st.typeCounts[type] = (st.typeCounts[type] || 0) + 1;
		if(!m || !m.from) { continue; }
		// node bookkeeping (keyed by source id)
		const hopsAway = (m.hop_start != null && m.hop_limit != null) ? (m.hop_start - m.hop_limit) : null;
		let nd = st.nodes.get(m.from);
		if(!nd) { nd = { id: m.from, first: pk.epoch_ms, packets: 0, hopMin: null, hopStart: m.hop_start }; st.nodes.set(m.from, nd); }
		nd.packets++;
		nd.last = pk.epoch_ms;
		nd.snr = pk.snr;
		nd.hopStart = m.hop_start;
		if(hopsAway != null) { nd.hopMin = nd.hopMin == null ? hopsAway : Math.min(nd.hopMin, hopsAway); }
		if(m.has_pos || (m.lat != null && m.lon != null)) { nd.lat = m.lat; nd.lon = m.lon; nd.alt = m.alt; }
		if(m.text) { nd.lastText = m.text; }
		// edge (source -> dest, incl. the broadcast hub)
		if(m.to) {
			const ek = m.from + ">" + m.to;
			let ed = st.edges.get(ek);
			if(!ed) { ed = { from: m.from, to: m.to, count: 0 }; st.edges.set(ek, ed); }
			ed.count++;
			ed.last = pk.epoch_ms;
		}
		// text-message log
		if(m.text) {
			st.messages.push({ t: pk.time, from: m.from, to: m.to, text: m.text, snr: pk.snr });
			if(st.messages.length > 200) { st.messages.shift(); }
		}
	}
	// FIFO ring: a key that left the daemon buffer never returns, so keep only current keys
	st.seen = keys;
	return added;
}

// a MeshCore path hash is the first 1..3 bytes of a public key: name it only if it is unambiguous
function coreNode(st, hash) {
	if(!hash) { return null; }
	let hit = null;
	for(const id of st.nodes.keys()) {
		if(id.startsWith(hash)) {
			if(hit) { return null; }
			hit = id;
		}
	}
	return hit;
}
function coreEdge(st, from, to, t) {
	if(!from || !to || from === to) { return; }
	const ek = from + ">" + to;
	let ed = st.edges.get(ek);
	if(!ed) { ed = { from, to, count: 0 }; st.edges.set(ek, ed); }
	ed.count++;
	ed.last = t;
}
function ingestCore(st, pk, m) {
	st.typeCounts[m.type] = (st.typeCounts[m.type] || 0) + 1;
	// repeaters relay every flood, and each copy shares the packet hash
	const repeat = !!m.hash && st.floods.has(m.hash);
	if(m.hash) {
		st.floods.add(m.hash);
		if(st.floods.size > 4000) { st.floods.delete(st.floods.values().next().value); }
	}
	if(m.pubkey && m.type === "ADVERT") {
		let nd = st.nodes.get(m.pubkey);
		if(!nd) { nd = { id: m.pubkey, first: pk.epoch_ms, packets: 0, hopMin: null }; st.nodes.set(m.pubkey, nd); }
		nd.packets++;
		nd.last = pk.epoch_ms;
		nd.snr = pk.snr;
		nd.hopMin = nd.hopMin == null ? m.hops : Math.min(nd.hopMin, m.hops);
		if(m.name) { nd.name = m.name; }
		if(m.role) { nd.role = m.role; }
		if(m.lat != null && m.lon != null) { nd.lat = m.lat; nd.lon = m.lon; }
	}
	// the path is the relay chain as heard: draw the links whose hashes resolve to a known node
	const chain = (m.type === "ADVERT" ? [m.pubkey] : [coreNode(st, m.src)]).concat((m.path || []).map(h => coreNode(st, h)));
	for(let i = 0; i + 1 < chain.length; i++) { coreEdge(st, chain[i], chain[i + 1], pk.epoch_ms); }
	if(m.src && m.dst) { coreEdge(st, coreNode(st, m.src), coreNode(st, m.dst), pk.epoch_ms); }
	if(m.text && !repeat) {
		st.messages.push({ t: pk.time, from: m.sender || "?", to: m.channel_name || m.channel, text: m.text, snr: pk.snr, core: true });
		if(st.messages.length > 200) { st.messages.shift(); }
	}
}

function updateTypebar(id, p, st) {
	const a = APP();
	const entries = Object.entries(st.typeCounts).sort((x, y) => y[1] - x[1]);
	const print = JSON.stringify(entries);
	if(print === st.typePrint) { return; }
	st.typePrint = print;
	p.typebar.textContent = "";
	if(!entries.length) {
		p.typebar.append(a.el("span", "lora-count", "message types appear here as frames arrive"));
		return;
	}
	p.typebar.append(a.el("span", "lora-count", "message types:"));
	for(const [name, n] of entries) {
		const chip = a.el("span", "lora-chip");
		const dot = a.el("span", "cdot");
		dot.style.background = rgba(typeColor(name), 1);
		chip.append(dot, document.createTextNode(name + " "), a.el("b", null, String(n)));
		p.typebar.append(chip);
	}
}

function nodeList(st) {
	return [...st.nodes.values()].sort((x, y) => (y.last || 0) - (x.last || 0));
}

function buildNodeTable(id) {
	const a = APP(), ce = a.ui[id];
	if(!ce.panel) { return; }
	const p = ce.panel, st = stateOf(id);
	const nodes = nodeList(st);
	p.nodeHc.textContent = String(nodes.length);
	const now = Date.now();
	const print = JSON.stringify(nodes.map(n => [n.id, n.packets, Math.round((now - (n.last || now)) / 1000), n.hopMin, n.snr, !!n.lat])) + "|" + st.selNode;
	if(print === st.nodePrint || a.selectionInside(p.nodewrap)) { return; }
	st.nodePrint = print;

	const t = a.el("table", "lora-ntbl");
	const tr0 = a.el("tr");
	for(const h of ["node", "heard", "pkts", "hops"]) { tr0.append(a.el("th", null, h)); }
	t.append(tr0);
	if(!nodes.length) {
		const r = a.el("tr", "lora-emptyrow");
		const td = a.el("td", null, "no nodes heard yet");
		td.colSpan = 4;
		r.append(td);
		t.append(r);
	}
	for(const n of nodes) {
		const row = a.el("tr", "row" + (n.id === st.selNode ? " sel" : ""));
		const first = a.el("td");
		const dot = a.el("span", "ndot");
		dot.style.background = rgba(snrColor(n.snr), 1);
		first.append(dot, document.createTextNode(n.name || n.id));
		if(n.lat != null) { first.append(document.createTextNode(" ")); const gp = a.el("span"); gp.textContent = "◉"; gp.style.color = "#5da8dc"; gp.title = "position known"; first.append(gp); }
		row.append(first);
		const age = (now - (n.last || now)) / 1000;
		row.append(a.el("td", null, a.fmtUp(age)));
		row.append(a.el("td", null, String(n.packets)));
		row.append(a.el("td", null, n.hopMin == null ? "?" : (n.hopMin === 0 ? "direct" : n.hopMin)));
		row.onclick = () => { st.selNode = st.selNode === n.id ? null : n.id; st.nodePrint = ""; buildNodeTable(id); updateNodeDetail(id); markVizDirty(); };
		t.append(row);
	}
	p.nodewrap.textContent = "";
	p.nodewrap.append(t);
	updateNodeDetail(id);
}

function buildMsgLog(id) {
	const a = APP(), ce = a.ui[id];
	if(!ce.panel) { return; }
	const p = ce.panel, st = stateOf(id);
	p.msgHc.textContent = String(st.messages.length);
	const msgs = st.messages.slice(-80).reverse();
	const print = JSON.stringify(msgs.map(m => [m.t, m.from, m.to, m.text]));
	if(print === st.msgPrint || a.selectionInside(p.msgwrap)) { return; }
	st.msgPrint = print;
	p.msgwrap.textContent = "";
	if(!msgs.length) {
		p.msgwrap.append(a.el("div", "lora-msg-empty", coreMode ? "no decoded text messages yet — Public and added channel messages appear here" :
			"no decoded text messages yet — default-key (LongFast) TEXT frames appear here"));
		return;
	}
	for(const m of msgs) {
		const row = a.el("div", "lora-msg");
		const head = a.el("div", "m-h");
		head.append(a.el("span", "m-from", m.core ? m.from : shortId(m.from)), a.el("span", "m-arrow", "→"),
			a.el("span", "m-to", m.to === "^all" ? "all" : (m.core ? m.to : shortId(m.to))), a.el("span", "m-time", m.t));
		row.append(head, a.el("div", "m-t", m.text));
		p.msgwrap.append(row);
	}
}

// ---- visualizer (canvas): force-directed graph OR position map -------------------------
function startViz(id, p) {
	S.vizPanel = p;
	const reduce = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
	const tick = () => {
		const a = APP(), ce = a.ui[id];
		if(ce.panel !== p) { return; }   // panel replaced/nulled (app stopped): stop the loop
		S.vizPanel = p;
		const shown = ce.pane && ce.pane.classList.contains("show");
		if(!document.hidden && shown) {
			const st = stateOf(id);
			if(st.meshView === "graph") {
				if(!reduce) { stepGraph(id, p, st); drawGraph(id, p, st); }
				else if(p._viz.dirty) { for(let i = 0; i < 240; i++) { stepGraph(id, p, st); } drawGraph(id, p, st); p._viz.dirty = false; }
			} else {
				if(p._viz.dirty) { drawMap(id, p, st); p._viz.dirty = false; }
			}
		}
		p._viz.raf = requestAnimationFrame(tick);
	};
	p._viz.raf = requestAnimationFrame(tick);
}

function graphNodes(st) {
	// real nodes + the broadcast hub when any frame targets ^all
	const arr = [...st.nodes.values()].map(n => ({ id: n.id, node: n }));
	let hub = false;
	for(const e of st.edges.values()) { if(e.to === "^all") { hub = true; break; } }
	if(hub) { arr.push({ id: "^all", hub: true }); }
	return arr;
}
function initPos(st, id, w, h) {
	if(st.pos.has(id)) { return st.pos.get(id); }
	// deterministic ring placement so the layout is stable across rebuilds
	let seed = 0;
	for(let i = 0; i < id.length; i++) { seed = (seed * 31 + id.charCodeAt(i)) & 0xffff; }
	const ang = (seed / 0xffff) * Math.PI * 2;
	const r = 60 + (seed % 40);
	const pt = { x: w / 2 + Math.cos(ang) * r, y: h / 2 + Math.sin(ang) * r, vx: 0, vy: 0 };
	st.pos.set(id, pt);
	return pt;
}
function stepGraph(id, p, st) {
	const cv = p.canvas;
	const w = cv.clientWidth || 600, h = cv.clientHeight || 400;
	const nodes = graphNodes(st);
	if(!nodes.length) { return; }
	for(const n of nodes) { n.p = initPos(st, n.id, w, h); }
	const REP = 9000, SPRING = 0.014, REST = 92, GRAV = 0.006, DAMP = 0.86, MAXV = 14;
	for(let i = 0; i < nodes.length; i++) {
		const a = nodes[i].p;
		for(let j = i + 1; j < nodes.length; j++) {
			const b = nodes[j].p;
			let dx = a.x - b.x, dy = a.y - b.y;
			let d2 = dx * dx + dy * dy;
			if(d2 < 1) { d2 = 1; dx = (i - j) || 1; dy = 1; }
			const f = REP / d2;
			const d = Math.sqrt(d2);
			const fx = f * dx / d, fy = f * dy / d;
			a.vx += fx; a.vy += fy; b.vx -= fx; b.vy -= fy;
		}
	}
	const byId = new Map(nodes.map(n => [n.id, n.p]));
	for(const e of st.edges.values()) {
		const a = byId.get(e.from), b = byId.get(e.to);
		if(!a || !b) { continue; }
		const dx = b.x - a.x, dy = b.y - a.y;
		const d = Math.hypot(dx, dy) || 1;
		const f = SPRING * (d - REST);
		const fx = f * dx / d, fy = f * dy / d;
		a.vx += fx; a.vy += fy; b.vx -= fx; b.vy -= fy;
	}
	for(const n of nodes) {
		const pt = n.p;
		pt.vx += (w / 2 - pt.x) * GRAV;
		pt.vy += (h / 2 - pt.y) * GRAV;
		pt.vx = clamp(pt.vx * DAMP, -MAXV, MAXV);
		pt.vy = clamp(pt.vy * DAMP, -MAXV, MAXV);
		pt.x = clamp(pt.x + pt.vx, 16, w - 16);
		pt.y = clamp(pt.y + pt.vy, 16, h - 16);
	}
}

function prepCanvas(cv) {
	const dpr = window.devicePixelRatio || 1;
	const w = cv.clientWidth, h = cv.clientHeight;
	if(!w || !h) { return null; }
	if(cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
	const g = cv.getContext("2d");
	g.setTransform(dpr, 0, 0, dpr, 0, 0);
	g.clearRect(0, 0, w, h);
	return { g, w, h };
}

function drawGraph(id, p, st) {
	const c = prepCanvas(p.canvas);
	if(!c) { return; }
	const { g, w, h } = c;
	g.font = "10px ui-monospace, monospace";
	const nodes = graphNodes(st);
	for(const n of nodes) { n.p = initPos(st, n.id, w, h); }
	const byId = new Map(nodes.map(n => [n.id, n]));
	const now = Date.now();

	// edges
	for(const e of st.edges.values()) {
		const a = byId.get(e.from), b = byId.get(e.to);
		if(!a || !b) { continue; }
		const age = (now - (e.last || now)) / 1000;
		const alpha = age < 8 ? 0.55 : age < 30 ? 0.34 : 0.16;
		g.strokeStyle = "rgba(120,150,190," + alpha + ")";
		g.lineWidth = clamp(0.6 + Math.log2(1 + e.count) * 0.5, 0.6, 3);
		g.beginPath(); g.moveTo(a.p.x, a.p.y); g.lineTo(b.p.x, b.p.y); g.stroke();
	}
	g.lineWidth = 1;
	// nodes
	p._hit = [];
	for(const n of nodes) {
		const x = n.p.x, y = n.p.y;
		if(n.hub) {
			g.fillStyle = "rgba(90,110,140,0.9)";
			g.strokeStyle = "rgba(140,165,200,0.8)";
			g.beginPath(); g.rect(x - 6, y - 6, 12, 12); g.fill(); g.stroke();
			g.fillStyle = "#9ab0c4"; g.textAlign = "center";
			g.fillText("broadcast", x, y - 11); g.textAlign = "left";
			continue;
		}
		const nd = n.node;
		const r = clamp(4 + Math.sqrt(nd.packets) * 2.1, 4.5, 17);
		const age = (now - (nd.last || now)) / 1000;
		const alpha = age < 10 ? 1 : age < 45 ? 0.72 : 0.45;
		const col = snrColor(nd.snr);
		p._hit.push({ id: n.id, x, y, r });
		g.beginPath(); g.arc(x, y, r, 0, 2 * Math.PI);
		g.fillStyle = rgba(col, alpha); g.fill();
		g.lineWidth = 0.9; g.strokeStyle = "rgba(4,6,12,0.7)"; g.stroke();
		if(nd.lat != null) { g.strokeStyle = "rgba(95,191,255,0.9)"; g.lineWidth = 1.4; g.beginPath(); g.arc(x, y, r + 2.5, 0, 2 * Math.PI); g.stroke(); }
		if(n.id === st.selNode) { g.strokeStyle = "#fff"; g.lineWidth = 1.6; g.beginPath(); g.arc(x, y, r + 4.5, 0, 2 * Math.PI); g.stroke(); }
		g.lineWidth = 3; g.strokeStyle = "rgba(4,6,12,0.75)"; g.strokeText(nodeLabel(nd), x + r + 3, y + 3);
		g.fillStyle = n.id === st.selNode ? "#fff" : rgba([210, 222, 240], alpha); g.fillText(nodeLabel(nd), x + r + 3, y + 3);
	}
	g.lineWidth = 1;
	drawLegend(p, [["weak", snrColor(-20)], ["…", snrColor(-10)], ["strong", snrColor(2)]], "node = source · ring = has position · size = packets");
	p.attr.textContent = "";
	p.vizhint.textContent = nodes.length ? "" : coreMode ?
		"no MeshCore adverts heard yet — nodes appear here when they advertise (decode proven by lorad --selftest)" :
		"no mesh nodes heard yet — LongFast frames appear here when received (decode proven by lorad --selftest)";
}

function drawLegend(p, ramp, label) {
	const a = APP();
	p.legend.textContent = "";
	p.legend.append(a.el("span", null, "SNR:"));
	for(const [lbl, col] of ramp) {
		const sp = a.el("span");
		const sw = a.el("span", "sw");
		sw.style.background = rgba(col, 1);
		sp.append(sw, document.createTextNode(lbl));
		p.legend.append(sp);
	}
	p.legend.append(a.el("span", null, "· " + label));
}

function drawMap(id, p, st) {
	const c = prepCanvas(p.canvas);
	if(!c) { return; }
	const { g, w, h } = c;
	g.font = "10px ui-monospace, monospace";
	const positioned = [...st.nodes.values()].filter(n => n.lat != null && n.lon != null);
	if(!positioned.length) {
		p.vizhint.textContent = coreMode ? "no adverts with a position yet — nodes that advertise one get plotted here" :
			"no POSITION frames decoded yet — nodes with a default-key POSITION payload get plotted here";
		p.legend.textContent = ""; p.attr.textContent = "";
		return;
	}
	p.vizhint.textContent = "";
	const lats = positioned.map(n => n.lat).sort((x, y) => x - y);
	const lons = positioned.map(n => n.lon).sort((x, y) => x - y);
	const center = { lat: lats[lats.length >> 1], lon: lons[lons.length >> 1] };
	let radius = 2;
	const kx = 111.320 * Math.cos(center.lat * RAD), ky = 110.574;
	for(const n of positioned) { radius = Math.max(radius, Math.hypot((n.lon - center.lon) * kx, (n.lat - center.lat) * ky)); }
	radius = Math.max(radius * 1.4, 1);
	const V = viewOf(p.canvas, center, radius);
	S.lastV = V;
	if(S.manual) { radius = V.mpp * Math.max(V.w, V.h) / 1000; }   // coastline culling follows the view

	let drew = false, attr = "";
	const set = TILESETS[st.basemap];
	if(st.basemap !== "coast" && navigator.onLine !== false) { drew = drawTiles(g, V, set); attr = set.attr; }
	if(!drew) { drew = drawCoast(g, V, radius); attr = (st.basemap === "coast" ? "" : "offline · ") + TILESETS.coast.attr; }
	p.attr.textContent = attr;

	// edges between positioned pairs
	const now = Date.now();
	for(const e of st.edges.values()) {
		const a = st.nodes.get(e.from), b = st.nodes.get(e.to);
		if(!a || !b || a.lat == null || b.lat == null) { continue; }
		const pa = V.toXY(a.lon, a.lat), pb = V.toXY(b.lon, b.lat);
		g.strokeStyle = "rgba(120,150,190,0.4)"; g.lineWidth = 1;
		g.beginPath(); g.moveTo(pa[0], pa[1]); g.lineTo(pb[0], pb[1]); g.stroke();
	}
	p._hit = [];
	for(const n of positioned) {
		const [x, y] = V.toXY(n.lon, n.lat);
		const age = (now - (n.last || now)) / 1000;
		const alpha = age < 10 ? 1 : age < 45 ? 0.72 : 0.45;
		const col = snrColor(n.snr);
		p._hit.push({ id: n.id, x, y, r: 8 });
		g.beginPath(); g.arc(x, y, 5, 0, 2 * Math.PI);
		g.fillStyle = rgba(col, alpha); g.fill();
		g.lineWidth = 1; g.strokeStyle = "rgba(4,6,12,0.8)"; g.stroke();
		if(n.id === st.selNode) { g.strokeStyle = "#fff"; g.lineWidth = 1.6; g.beginPath(); g.arc(x, y, 9, 0, 2 * Math.PI); g.stroke(); }
		g.lineWidth = 3; g.strokeStyle = "rgba(4,6,12,0.75)"; g.strokeText(nodeLabel(n), x + 8, y + 3);
		g.fillStyle = n.id === st.selNode ? "#fff" : rgba([210, 222, 240], alpha); g.fillText(nodeLabel(n), x + 8, y + 3);
	}
	g.lineWidth = 1;
	drawLegend(p, [["weak", snrColor(-20)], ["…", snrColor(-10)], ["strong", snrColor(2)]], positioned.length + " node(s) with position");
}

function graphHit(id, cv, ev) {
	const a = APP(), p = a.ui[id].panel, st = stateOf(id);
	if(!p || !p._hit) { return; }
	const rect = cv.getBoundingClientRect();
	const mx = ev.clientX - rect.left, my = ev.clientY - rect.top;
	let best = null, bd = 1e9;
	for(const hnode of p._hit) {
		const d = (hnode.x - mx) ** 2 + (hnode.y - my) ** 2;
		const rr = (hnode.r + 6) ** 2;
		if(d < rr && d < bd) { bd = d; best = hnode.id; }
	}
	st.selNode = (best && best === st.selNode) ? null : best;
	st.nodePrint = "";
	buildNodeTable(id);
	updateNodeDetail(id);
	markVizDirty();
}

function fld(a, dl, k, v) {
	if(v === undefined || v === null || v === "") { return; }
	dl.append(a.el("dt", null, k), a.el("dd", null, String(v)));
}
function updateNodeDetail(id) {
	const a = APP(), p = a.ui[id].panel;
	if(!p || !p.detail) { return; }
	const st = stateOf(id);
	const nd = st.selNode ? st.nodes.get(st.selNode) : null;
	if(!nd) { p.detail.style.display = "none"; return; }
	p.detail.style.display = "block";
	p.detail.textContent = "";
	const close = a.el("button", "cx", "×");
	close.onclick = () => { st.selNode = null; st.nodePrint = ""; p.detail.style.display = "none"; buildNodeTable(id); markVizDirty(); };
	const h = a.el("h4");
	const sw = a.el("span", "sw");
	sw.style.cssText = "width:10px;height:10px;border-radius:2px;background:" + rgba(snrColor(nd.snr), 1);
	h.append(sw, document.createTextNode(nd.name || nd.id));
	const dl = a.el("dl");
	const now = Date.now();
	if(nd.name) { fld(a, dl, "public key", nd.id + "…"); }
	fld(a, dl, "role", nd.role);
	fld(a, dl, "packets", nd.packets);
	fld(a, dl, "last heard", a.fmtUp((now - (nd.last || now)) / 1000) + " ago");
	fld(a, dl, "hops away", nd.hopMin == null ? "?" : (nd.hopMin === 0 ? "direct" : nd.hopMin));
	fld(a, dl, "hop start", nd.hopStart);
	fld(a, dl, "last SNR", nd.snr !== undefined ? nd.snr.toFixed(1) + " dB" : undefined);
	if(nd.lat != null) { fld(a, dl, "position", nd.lat.toFixed(5) + ", " + nd.lon.toFixed(5)); }
	if(nd.alt != null) { fld(a, dl, "altitude", nd.alt + " m"); }
	if(nd.lastText) { fld(a, dl, "last text", "“" + nd.lastText + "”"); }
	p.detail.append(close, h, dl);
}

})();

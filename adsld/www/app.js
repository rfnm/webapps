/* adsld map UI. Plain JS + vendored Leaflet; polls the daemon's JSON API (/traffic 1 Hz,
   /status 2 s) and renders one rotated per-category SVG icon per aircraft, short client-side
   trails, a sortable list and a status strip. MPL-2.0. */
"use strict";

/* ---- ADS-L category icons -----------------------------------------------------------
   Own simple designs, all in a 24x24 viewBox pointing NORTH (up) so a CSS rotate by
   track_deg gives the course. fill/stroke use currentColor; state (normal / emergency /
   ground / stale) is a CSS class on the wrapper. */
var SW = ' fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round"';
var ICONS = {
	// 0 unknown: diamond
	unknown: '<path d="M12 3 20 12 12 21 4 12Z"/>',
	// 1 fixed-wing small: classic straight-wing GA silhouette
	plane: '<path d="M12 1.5 13.2 5v4L22 12v2l-8.8-1.4V18l3.3 2v1.6L12 20.6l-4.5 1L7.5 20l3.3-2v-5.4L2 14v-2l8.8-3V5Z"/>',
	// 2 fixed-wing large: swept wings, wider span
	heavy: '<path d="M12 1 13.7 4.4v4L23 14.6v2.2l-9.3-3.2V18l3.8 2.6v1.9L12 21l-5.5 1.5v-1.9L10.3 18v-4.4L1 16.8v-2.2l9.3-6.2v-4Z"/>',
	// 3 / 15 helicopter & other rotorcraft: cabin + tail boom + crossed rotor
	heli: '<path d="M12 6.2c2 0 2.9 2.4 2.9 5.3S13.3 16 12 16s-2.9-1.6-2.9-4.5 .9-5.3 2.9-5.3Z"/>' +
	      '<path d="M11.3 15.6h1.4V21h-1.4Z"/><path d="M9.6 20.3h4.8v1.4H9.6Z"/>' +
	      '<path d="M5 4.5 19 18.5M19 4.5 5 18.5"' + SW + '/>',
	// 4 glider: very long thin wings, slim fuselage, T-tail
	glider: '<path d="M12 2 12.9 5v4.4l10.6 1.1v1.6l-10.6-.3V18l2.6 1.5V21L12 20.4 9 21v-1.5L11.1 18v-6.2L.5 12.1v-1.6L11.1 9.4V5Z"/>',
	// 5 balloon / LTA: envelope + basket
	balloon: '<circle cx="12" cy="9" r="6.6"/><path d="M9.7 15 10.4 18h3.2L14.3 15Z"/>' +
	         '<path d="M10.2 18.6h3.6v2.8h-3.6Z"/>',
	// 6 ultralight / trike: flexible delta wing with a hanging pod
	trike: '<path d="M12 4 22.5 10.5 12 8.6 1.5 10.5Z"/><path d="M12 8.6v4.4"' + SW + '/>' +
	       '<circle cx="12" cy="15.4" r="2.4"/>',
	// 7 paraglider: canopy arc + lines + pilot
	para: '<path d="M2.5 9.5A9.7 6.5 0 0 1 21.5 9.5L19.6 12A7.8 4.6 0 0 0 4.4 12Z"/>' +
	      '<path d="M6 11.5 11.2 17M18 11.5 12.8 17"' + SW.replace('1.7', '1.1') + '/>' +
	      '<circle cx="12" cy="17.5" r="2"/>',
	// 8 skydiver: small square canopy + spread-limb jumper
	skydiver: '<path d="M5 4h14l-1.6 3.6H6.6Z"/>' +
	          '<path d="M7.5 7.6 11 13M16.5 7.6 13 13"' + SW.replace('1.7', '1.1') + '/>' +
	          '<circle cx="12" cy="13.6" r="1.7"/>' +
	          '<path d="M12 15.3v2.9m0-2.2-3 2.5m3-2.5 3 2.5m-3-.7-2 4m2-4 2 4"' + SW.replace('1.7', '1.4') + '/>',
	// 9 eVTOL: wing + fuselage + tip lift rotors
	evtol: '<path d="M11 4.6h2V19h-2Z"/><path d="M3.4 10.6h17.2v2H3.4Z"/>' +
	       '<path d="M9 18h6v1.6H9Z"/>' +
	       '<circle cx="4.4" cy="8.2" r="2.6"' + SW.replace('1.7', '1.3') + '/>' +
	       '<circle cx="19.6" cy="8.2" r="2.6"' + SW.replace('1.7', '1.3') + '/>',
	// 10 gyrocopter: pod + prop disc behind + single teetering rotor across
	gyro: '<path d="M12 6.8c1.7 0 2.6 2.2 2.6 4.9S13.2 16.5 12 16.5 9.4 14.4 9.4 11.7 10.3 6.8 12 6.8Z"/>' +
	      '<path d="M11.3 16.2h1.4v3.6h-1.4Z"/><path d="M8.9 19.4h6.2v1.3H8.9Z"/>' +
	      '<path d="M3.5 5.5 20.5 10.5"' + SW + '/>',
	// 11/12/13 drone / UAS: quad frame
	quad: '<circle cx="6" cy="6" r="3"' + SW.replace('1.7', '1.5') + '/>' +
	      '<circle cx="18" cy="6" r="3"' + SW.replace('1.7', '1.5') + '/>' +
	      '<circle cx="6" cy="18" r="3"' + SW.replace('1.7', '1.5') + '/>' +
	      '<circle cx="18" cy="18" r="3"' + SW.replace('1.7', '1.5') + '/>' +
	      '<path d="M8 8 16 16M16 8 8 16"' + SW.replace('1.7', '1.5') + '/>' +
	      '<rect x="10.4" y="10.4" width="3.2" height="3.2" rx="1"/>',
	// 14 model plane: the small fixed-wing, scaled down
	model: '<g transform="translate(3.6 3.6) scale(.7)">' +
	       '<path d="M12 1.5 13.2 5v4L22 12v2l-8.8-1.4V18l3.3 2v1.6L12 20.6l-4.5 1L7.5 20l3.3-2v-5.4L2 14v-2l8.8-3V5Z"/></g>',
	// 16 hang-glider: swallow delta
	hang: '<path d="M12 4.5 23 14.5 12 10.8 1 14.5Z"/><path d="M12 10.8v5"' + SW.replace('1.7', '1.4') + '/>',
	// 17 paramotor: paraglider canopy + prop cage
	pmotor: '<path d="M2.5 8.5A9.7 6.5 0 0 1 21.5 8.5L19.6 11A7.8 4.6 0 0 0 4.4 11Z"/>' +
	        '<circle cx="12" cy="16.5" r="3.4"' + SW.replace('1.7', '1.3') + '/>' +
	        '<circle cx="12" cy="16.5" r="1.3"/>'
};

// ADS-L Category(5) -> [icon key, display name]
var CATS = {
	0: ['unknown', 'unknown'],
	1: ['plane', 'fixed-wing (small)'],
	2: ['heavy', 'fixed-wing (large)'],
	3: ['heli', 'helicopter'],
	4: ['glider', 'glider'],
	5: ['balloon', 'balloon / LTA'],
	6: ['trike', 'ultralight / trike'],
	7: ['para', 'paraglider'],
	8: ['skydiver', 'skydiver'],
	9: ['evtol', 'eVTOL'],
	10: ['gyro', 'gyrocopter'],
	11: ['quad', 'drone'],
	12: ['quad', 'UAS'],
	13: ['quad', 'drone (heavy)'],
	14: ['model', 'model aircraft'],
	15: ['heli', 'rotorcraft (other)'],
	16: ['hang', 'hang-glider'],
	17: ['pmotor', 'paramotor']
};
function catInfo(cat) { return CATS[cat] || ['unknown', 'category ' + cat]; }
function catSvg(cat) {
	return '<svg viewBox="0 0 24 24" fill="currentColor">' + ICONS[catInfo(cat)[0]] + '</svg>';
}

var AMT_NAMES = { 0: 'RANDOM', 5: 'ICAO', 6: 'FLARM', 7: 'OGN', 8: 'FANET' };

var AMT_NAMES = { 0: 'AMT 0', 5: 'ICAO', 6: 'FLARM', 7: 'OGN', 8: 'FANET' };
function amtName(a) { return AMT_NAMES[a] || (a >= 9 ? 'MFR' + a : 'AMT' + a); }
var FS_NAMES = { 0: 'unknown', 1: 'on ground', 2: 'airborne' };
var CH_MHZ = { 0: '868.2', 1: '868.4' };
var STALE_S = 30;
/* An aircraft not heard for this long leaves the map, the list and the counters: longer than the
   10 s on-ground beacon interval. The daemon keeps its record longer, so an aircraft heard again
   is back at once. */
var HIDE_S = 20;
var TRAIL_MAX = 60;

function $(id) { return document.getElementById(id); }
function esc(s) {
	return String(s === undefined || s === null ? '' : s).replace(/[&<>"']/g, function (c) {
		return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
	});
}

/* ---- theme --------------------------------------------------------------------------- */
try {
	if (localStorage.getItem('adsl-theme') === 'light') { document.body.classList.add('light'); }
} catch (e) { /* storage blocked - dark default */ }
$('themebtn').addEventListener('click', function () {
	document.body.classList.toggle('light');
	try {
		localStorage.setItem('adsl-theme', document.body.classList.contains('light') ? 'light' : 'dark');
	} catch (e) { /* fine */ }
});

/* ---- map ----------------------------------------------------------------------------- */
var map = L.map('map', { zoomControl: true, attributionControl: true }).setView([48, 10], 5);
var tilesBroken = false;
var tileLayer = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
	maxZoom: 19,
	attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
});
tileLayer.on('tileerror', function () {
	// offline / no internet: the CSS grid under the map stays, just say so once
	tilesBroken = true;
	$('offline-badge').hidden = false;
});
tileLayer.on('tileload', function () {
	if (tilesBroken) {
		tilesBroken = false;
		$('offline-badge').hidden = true;
	}
});
tileLayer.addTo(map);

/* ---- state --------------------------------------------------------------------------- */
var aircraft = {};        // key (amt:addr) -> {data, marker, trail, path:[]}
var selectedKey = null;
var sortBy = 'age';
var didFit = false;
var connOk = null;

function key(t) { return t.amt + ':' + t.addr; }
function fmt(v, d) { return v === undefined || v === null ? '–' : v.toFixed(d === undefined ? 0 : d); }

function stateClass(t) {
	var cls = '';
	if (t.emg > 1) { cls = 'emg'; }
	else if (t.fs === 1) { cls = 'ground'; }
	if (t.age_s > STALE_S) { cls += ' stale'; }
	return cls;
}

function makeIcon(t, selected) {
	var rot = (typeof t.track_deg === 'number') ? ' style="transform:rotate(' + t.track_deg.toFixed(0) + 'deg)"' : '';
	var html = '<div class="acwrap ' + stateClass(t) + (selected ? ' selected' : '') + '"' + rot + '>' +
		catSvg(t.cat) + '</div><span class="aclabel">' + esc(t.addr) + '</span>';
	return L.divIcon({ className: 'acdiv', html: html, iconSize: [34, 34], iconAnchor: [17, 17] });
}

function detailRows(t) {
	var spd = typeof t.speed_mps === 'number'
		? (t.speed_mps * 3.6).toFixed(0) + ' km/h (' + t.speed_mps.toFixed(1) + ' m/s)' : '–';
	var vr = typeof t.vrate_mps === 'number'
		? (t.vrate_mps > 0.05 ? '↑ ' : t.vrate_mps < -0.05 ? '↓ ' : '') + t.vrate_mps.toFixed(1) + ' m/s' : '–';
	return '<table>' +
		'<tr><td>address</td><td>' + esc(t.addr) + ' (' + amtName(t.amt) + ')</td></tr>' +
		'<tr><td>category</td><td>' + catInfo(t.cat)[1] + ' (' + t.cat + ')</td></tr>' +
		'<tr><td>state</td><td>' + (FS_NAMES[t.fs] || t.fs) + (t.emg > 1 ? ' · <b class="emgtxt">EMERGENCY ' + t.emg + '</b>' : '') + '</td></tr>' +
		'<tr><td>altitude</td><td>' + fmt(t.alt_m) + ' m</td></tr>' +
		'<tr><td>speed</td><td>' + spd + '</td></tr>' +
		'<tr><td>vert rate</td><td>' + vr + '</td></tr>' +
		'<tr><td>track</td><td>' + (typeof t.track_deg === 'number' ? t.track_deg.toFixed(0) + '°' : '–') + '</td></tr>' +
		'<tr><td>rssi</td><td>' + fmt(t.rssi, 1) + ' dBFS</td></tr>' +
		'<tr><td>channel</td><td>' + (CH_MHZ[t.ch] || t.ch) + ' MHz</td></tr>' +
		'<tr><td>packets</td><td>' + t.count + '</td></tr>' +
		'<tr><td>age</td><td>' + fmt(t.age_s, 1) + ' s</td></tr>' +
		'</table>';
}

function detailHtml(t) { return '<h2>' + esc(t.addr) + '</h2>' + detailRows(t); }

function select(k, pan) {
	selectedKey = k;
	var a = aircraft[k];
	var det = $('detail');
	if (!a) { det.hidden = true; renderList(); return; }
	if (pan && a.marker) { map.panTo(a.marker.getLatLng()); }
	if (a.marker) { a.marker.openPopup(); }
	det.hidden = false;
	det.innerHTML = detailHtml(a.data);
	renderList();
}

function renderList() {
	var ul = $('aclist');
	var entries = Object.keys(aircraft).map(function (k) { return { k: k, t: aircraft[k].data }; });
	entries.sort(function (x, y) {
		return sortBy === 'rssi' ? (y.t.rssi || -999) - (x.t.rssi || -999)
		                         : (x.t.age_s || 0) - (y.t.age_s || 0);
	});
	if (!entries.length) {
		ul.innerHTML = '<li class="lempty">no aircraft heard</li>';
		return;
	}
	ul.innerHTML = entries.map(function (e) {
		var t = e.t;
		var cls = (t.emg > 1 ? 'emg ' : '') + (t.fs === 1 ? 'ground ' : '') +
			(t.age_s > STALE_S ? 'stale ' : '') + (e.k === selectedKey ? 'selected' : '');
		return '<li class="' + cls + '" data-k="' + esc(e.k) + '">' +
			'<span class="licon">' + catSvg(t.cat) + '</span>' +
			'<span class="laddr">' + esc(t.addr) + '</span>' +
			'<span class="lcat">' + catInfo(t.cat)[1] + '</span>' +
			'<span class="lnum"><b>' + fmt(t.alt_m) + ' m</b>' +
			fmt(t.rssi, 0) + ' dB · ' + fmt(t.age_s, 0) + ' s</span>' +
			'</li>';
	}).join('');
	Array.prototype.forEach.call(ul.children, function (li) {
		li.addEventListener('click', function () { select(li.getAttribute('data-k'), true); });
	});
}

function setSort(s) {
	sortBy = s;
	$('sort-age').classList.toggle('sorton', s === 'age');
	$('sort-rssi').classList.toggle('sorton', s === 'rssi');
	renderList();
}
$('sort-age').addEventListener('click', function () { setSort('age'); });
$('sort-rssi').addEventListener('click', function () { setSort('rssi'); });

/* ---- polling ------------------------------------------------------------------------- */
function setConn(ok) {
	if (ok === connOk) { return; }
	connOk = ok;
	$('conn').className = 'dot ' + (ok ? 'ok' : 'bad');
	$('connlabel').textContent = ok ? 'live' : 'connection lost';
}

/* The first view once traffic arrives: the aircraft, at most zoom 15 (one aircraft alone is a
   zero-size box and Leaflet would zoom to the street). */
function fitWorkArea() {
	var pts = Object.keys(aircraft).map(function (k) { return aircraft[k].marker.getLatLng(); });
	if (pts.length) { map.fitBounds(L.latLngBounds(pts).pad(0.25), { maxZoom: 15 }); }
}

function updateTraffic(list) {
	list = list.filter(function (t) { return !(typeof t.age_s === 'number' && t.age_s > HIDE_S); });
	var seen = {};
	list.forEach(function (t) {
		if (typeof t.lat !== 'number' || typeof t.lon !== 'number') { return; }   // no fix: unmappable
		var k = key(t);
		seen[k] = true;
		var a = aircraft[k];
		var ll = [t.lat, t.lon];
		if (!a) {
			a = aircraft[k] = {
				data: t,
				path: [ll],
				trail: L.polyline([ll], { className: 'trail', weight: 2, opacity: 0.55, interactive: false }).addTo(map),
				marker: L.marker(ll, { icon: makeIcon(t, false) }).addTo(map)
			};
			a.marker.bindPopup(function () {
				return '<div class="popup"><h3>' + esc(a.data.addr) + '</h3>' + detailRows(a.data) + '</div>';
			}, { maxWidth: 280 });
			a.marker.on('click', function () { select(k, false); });
		} else {
			a.data = t;
			var last = a.path[a.path.length - 1];
			if (last[0] !== ll[0] || last[1] !== ll[1]) {
				a.path.push(ll);
				if (a.path.length > TRAIL_MAX) { a.path.shift(); }
				a.trail.setLatLngs(a.path);
			}
			a.marker.setLatLng(ll);
		}
		a.marker.setIcon(makeIcon(t, k === selectedKey));
		// the popup's content is evaluated only when it opens; re-run it while it stays open
		if (a.marker.isPopupOpen()) { a.marker.getPopup().update(); }
	});
	// entries the daemon aged out
	Object.keys(aircraft).forEach(function (k) {
		if (!seen[k]) {
			map.removeLayer(aircraft[k].marker);
			map.removeLayer(aircraft[k].trail);
			delete aircraft[k];
			if (selectedKey === k) { selectedKey = null; $('detail').hidden = true; }
		}
	});
	if (!didFit && Object.keys(aircraft).length) {
		didFit = true;
		fitWorkArea();
	}
	$('st-traffic').textContent = list.length;
	if (selectedKey && aircraft[selectedKey]) {
		$('detail').innerHTML = detailHtml(aircraft[selectedKey].data);
	}
	renderList();
}

function pollTraffic() {
	fetch('traffic', { cache: 'no-store' })
		.then(function (r) { if (!r.ok) { throw 0; } return r.json(); })
		.then(function (list) { setConn(true); updateTraffic(list); })
		.catch(function () { setConn(false); })
		.then(function () { setTimeout(pollTraffic, 1000); });
}

function pollStatus() {
	fetch('status', { cache: 'no-store' })
		.then(function (r) { if (!r.ok) { throw 0; } return r.json(); })
		.then(function (s) {
			$('st-rxok').textContent = s.rx_ok;
			$('st-unknown').textContent = s.unknown_rx;
			if (s.channels && s.channels.length > 1) {
				$('st-ch0').textContent = s.channels[0].rssi.toFixed(0) + '/' + s.channels[0].noise.toFixed(0) + ' dB';
				$('st-ch1').textContent = s.channels[1].rssi.toFixed(0) + '/' + s.channels[1].noise.toFixed(0) + ' dB';
			}
			$('st-txwrap').hidden = !s.tx_enabled;
			if (s.tx_enabled) { $('st-tx').textContent = s.tx_count; }
			if ((beaconTick++ % 5) === 0) { beaconRefresh(); }
		})
		.catch(function () { /* the traffic poll owns the link dot */ })
		.then(function () { setTimeout(pollStatus, 2000); });
}

/* ---- own station: address, position, Emergency Status ----------------------------------------
     GET  config  -> {addr, amt, lat, lon, alt, pos_valid, emergency}   (every 5th status poll)
     POST config  <- {emergency: n}   (no restart: G.1.4 is meant to change in operation) */
var beaconTick = 0;
var emgDirty = false;          // the operator picked a value that is not sent yet
var rxMarker = null;
var EMG_NAMES = ['undefined', 'no emergency', 'general emergency', 'lifeguard / medical',
	'no communications', 'unlawful interference', 'downed aircraft'];

function beaconRefresh() {
	return fetch('config', { cache: 'no-store' })
		.then(function (r) { if (!r.ok) { throw 0; } return r.json(); })
		.then(function (c) {
			$('v-addr').textContent = String(c.addr) + ' (' + amtName(c.amt) + ')';
			$('v-pos').textContent = c.pos_valid
				? c.lat.toFixed(5) + ', ' + c.lon.toFixed(5) + ', ' + c.alt.toFixed(0) + ' m' : 'not set';
			if (!emgDirty) { $('f-emg').value = String(c.emergency); }
			if (c.pos_valid) {
				if (!rxMarker) {
					// className instead of a colour option: the stylesheet carries both themes
					rxMarker = L.circleMarker([c.lat, c.lon], {
						radius: 6, weight: 2, className: 'rxdot', fillOpacity: 0.3, interactive: false
					}).addTo(map);
					if (!didFit) { map.setView([c.lat, c.lon], 11); }
				} else {
					rxMarker.setLatLng([c.lat, c.lon]);
				}
			}
		})
		.catch(function () { /* the next poll tries again */ });
}

function emgMsg(text, kind) {
	var el = $('emg-msg');
	el.textContent = text;
	el.className = 'pmsg' + (kind ? ' ' + kind : '');
	el.hidden = !text;
}

$('f-emg').addEventListener('change', function () { emgDirty = true; emgMsg(''); });
$('b-emg').addEventListener('click', function () {
	var n = parseInt($('f-emg').value, 10);
	fetch('config', {
		method: 'POST', cache: 'no-store',
		headers: { 'Content-Type': 'application/json' },
		body: JSON.stringify({ emergency: n })
	}).then(function (r) {
		if (!r.ok) { throw new Error('HTTP ' + r.status); }
		return r.json();
	}).then(function (c) {
		emgDirty = false;
		var on = (typeof c.emergency === 'number') ? c.emergency : n;
		emgMsg('Emergency status ' + on + ' (' + (EMG_NAMES[on] || on) + ') is in every Traffic ' +
			'frame from now on.', on > 1 ? 'caution' : 'ok');
		return beaconRefresh();
	}).catch(function (e) { emgMsg('Setting the emergency status failed: ' + e.message, 'err'); });
});

beaconRefresh();
pollTraffic();
pollStatus();

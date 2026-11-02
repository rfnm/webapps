#include "meshcore_tx.h"
#include "meshcore.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <cstring>

#include "aes128.h"
#include "sha256.h"
#include "vendor/monocypher-ed25519.h"
#include "vendor/monocypher.h"

namespace lorad {

static const int TYPE_TXT_MSG = 2;
static const int TYPE_ACK = 3;
static const int TYPE_PATH = 8;
static const int ROUTE_NODE_CHAT = 1;
static const int TYPE_GRP_TXT = 5;
static const int TYPE_ADVERT = 4;
static const int ROUTE_TRANSPORT_FLOOD = 0;
static const int ROUTE_FLOOD = 1;
static const int ADV_TYPE_CHAT = 1;
static const int ADV_LATLON = 0x10;
static const int ADV_NAME = 0x80;
static const size_t MAX_APP_DATA = 32;
static const size_t MAX_PAYLOAD = 184;
static const size_t MAX_TEXT = 160;     // MAX_TEXT_LEN of the firmware

static const uint8_t PUBLIC_KEY[16] = {
	0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a, 0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
};

static void sha(const void *p, size_t n, uint8_t out[32]) {
	sha256 s;
	s.update((const uint8_t *)p, n);
	s.finish(out);
}

void mc_identity_from_seed(const uint8_t seed[32], mc_identity &id) {
	uint8_t s[32];
	memcpy(s, seed, 32);
	crypto_ed25519_key_pair(id.secret, id.pub, s);   // wipes s
	id.ok = true;
}

bool mc_identity_load(const std::string &path, mc_identity &id, std::string &err) {
	uint8_t seed[32];
	FILE *f = fopen(path.c_str(), "rb");
	if(f) {
		size_t n = fread(seed, 1, 32, f);
		fclose(f);
		if(n != 32) {
			err = "identity file is not 32 bytes: " + path;
			return false;
		}
		mc_identity_from_seed(seed, id);
		return true;
	}
	FILE *r = fopen("/dev/urandom", "rb");
	if(!r || fread(seed, 1, 32, r) != 32) {
		if(r) {
			fclose(r);
		}
		err = "no random source";
		return false;
	}
	fclose(r);
	std::string tmp = path + ".tmp";
	int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
	if(fd < 0) {
		err = "cannot create " + tmp;
		return false;
	}
	bool ok = write(fd, seed, 32) == 32 && fsync(fd) == 0;
	close(fd);
	if(!ok || rename(tmp.c_str(), path.c_str()) != 0) {
		err = "cannot write " + path;
		return false;
	}
	mc_identity_from_seed(seed, id);
	return true;
}

mc_scope mc_scope_named(const std::string &name) {
	mc_scope s;
	if(name.empty() || name == "*" || name == "off") {
		return s;
	}
	uint8_t d[32];
	sha(name.data(), name.size(), d);
	memcpy(s.key, d, 16);
	s.on = true;
	return s;
}

uint16_t mc_transport_code(const uint8_t key[16], int type, const uint8_t *payload, size_t len) {
	std::vector<uint8_t> m(1, (uint8_t)type);
	m.insert(m.end(), payload, payload + len);
	uint8_t d[32];
	hmac_sha256(key, 16, m.data(), m.size(), d);
	uint16_t c = (uint16_t)(d[0] | (d[1] << 8));
	return c == 0 ? 1 : (c == 0xFFFF ? 0xFFFE : c);   // 0000 and FFFF are reserved
}

bool mc_channel_key(const std::string &name, uint8_t key[16]) {
	if(name == "public" || name == "Public") {
		memcpy(key, PUBLIC_KEY, 16);
		return true;
	}
	if(mc_channel_lookup(name, key)) {
		return true;
	}
	if(name.size() < 2 || name[0] != '#') {
		return false;
	}
	uint8_t d[32];
	sha(name.data(), name.size(), d);
	memcpy(key, d, 16);
	return true;
}

// header, optional transport codes, path length 0: this node is the origin of the flood
static std::vector<uint8_t> wrap(int type, const std::vector<uint8_t> &payload, const mc_scope &scope) {
	std::vector<uint8_t> f;
	f.push_back((uint8_t)((scope.on ? ROUTE_TRANSPORT_FLOOD : ROUTE_FLOOD) | (type << 2)));
	if(scope.on) {
		uint16_t c = mc_transport_code(scope.key, type, payload.data(), payload.size());
		f.push_back((uint8_t)c);
		f.push_back((uint8_t)(c >> 8));
		f.push_back(0);
		f.push_back(0);
	}
	f.push_back(0);
	f.insert(f.end(), payload.begin(), payload.end());
	return f;
}

std::vector<uint8_t> mc_build_advert(const mc_identity &id, uint32_t ts, const std::string &name, bool has_pos,
                                     double lat, double lon, const mc_scope &scope) {
	std::vector<uint8_t> app;
	app.push_back((uint8_t)(ADV_TYPE_CHAT | (has_pos ? ADV_LATLON : 0) | (name.empty() ? 0 : ADV_NAME)));
	if(has_pos) {
		int32_t v[2] = { (int32_t)lround(lat * 1e6), (int32_t)lround(lon * 1e6) };
		for(int32_t x : v) {
			for(int i = 0; i < 4; i++) {
				app.push_back((uint8_t)(((uint32_t)x) >> (8 * i)));
			}
		}
	}
	size_t room = MAX_APP_DATA - app.size();
	app.insert(app.end(), name.begin(), name.begin() + (long)std::min(room, name.size()));

	std::vector<uint8_t> msg(id.pub, id.pub + 32);
	for(int i = 0; i < 4; i++) {
		msg.push_back((uint8_t)(ts >> (8 * i)));
	}
	msg.insert(msg.end(), app.begin(), app.end());
	uint8_t sig[64];
	crypto_ed25519_sign(sig, id.secret, msg.data(), msg.size());

	std::vector<uint8_t> p(id.pub, id.pub + 32);
	for(int i = 0; i < 4; i++) {
		p.push_back((uint8_t)(ts >> (8 * i)));
	}
	p.insert(p.end(), sig, sig + 64);
	p.insert(p.end(), app.begin(), app.end());
	return wrap(TYPE_ADVERT, p, scope);
}

bool mc_build_group_text(const uint8_t key[16], uint32_t ts, const std::string &sender, const std::string &text,
                         const mc_scope &scope, std::vector<uint8_t> &frame) {
	std::string line = sender + ": " + text;
	if(line.empty() || line.size() > MAX_TEXT) {
		return false;
	}
	std::vector<uint8_t> plain;
	for(int i = 0; i < 4; i++) {
		plain.push_back((uint8_t)(ts >> (8 * i)));
	}
	plain.push_back(0);   // TXT_TYPE_PLAIN
	plain.insert(plain.end(), line.begin(), line.end());
	while(plain.size() % 16) {
		plain.push_back(0);
	}
	aes128 aes(key);
	std::vector<uint8_t> ct(plain.size());
	for(size_t i = 0; i < plain.size(); i += 16) {
		aes.encrypt_block(&plain[i], &ct[i]);
	}
	// the MAC key is the channel secret zero-padded to PUB_KEY_SIZE, the channel hash covers the 16 bytes
	uint8_t mkey[32] = { 0 }, d[32], ch[32];
	memcpy(mkey, key, 16);
	hmac_sha256(mkey, 32, ct.data(), ct.size(), d);
	sha(key, 16, ch);
	std::vector<uint8_t> p;
	p.push_back(ch[0]);
	p.push_back(d[0]);
	p.push_back(d[1]);
	p.insert(p.end(), ct.begin(), ct.end());
	if(p.size() > MAX_PAYLOAD) {
		return false;
	}
	frame = wrap(TYPE_GRP_TXT, p, scope);
	return true;
}

// ---- private messages ---------------------------------------------------------------------
struct contact {
	std::string name;
	uint8_t pub[32];
	bool has_route = false;
	uint8_t route_len = 0;
	std::vector<uint8_t> route;
};
static std::mutex g_dm_mtx;
static std::vector<contact> g_contacts;
static std::string g_contacts_path;
static bool g_self_ok = false;
static uint8_t g_self_pub[32], g_self_scalar[32];
static std::function<void(const mc_dm_event &)> g_dm_cb;
static std::vector<uint32_t> g_dm_seen;
static std::function<void(uint32_t)> g_ack_cb;

// a flood lists the relays from the sender towards us: reversed, it is our route to the sender
static std::vector<uint8_t> reverse_path(const std::vector<uint8_t> &path, uint8_t path_len) {
	size_t n = (size_t)(path_len >> 6) + 1;
	std::vector<uint8_t> r;
	for(size_t i = path.size(); i >= n; i -= n) {
		r.insert(r.end(), path.begin() + (long)(i - n), path.begin() + (long)i);
	}
	return r;
}

static std::string hex6(const uint8_t *p) {
	char b[13];
	for(int i = 0; i < 6; i++) {
		snprintf(b + 2 * i, 3, "%02x", p[i]);
	}
	return b;
}

// Identity::calcSharedSecret: the Ed25519 scalar against the peer's key as a Montgomery point.
// The result is used raw, without a hash: 16 bytes AES key, all 32 as HMAC key.
static void shared_secret(const uint8_t peer[32], uint8_t out[32]) {
	uint8_t x[32];
	crypto_eddsa_to_x25519(x, peer);
	crypto_x25519(out, g_self_scalar, x);
}

// Utils::encryptThenMAC: AES-128-ECB with zero padding, two-byte MAC in front
static std::vector<uint8_t> seal(const uint8_t secret[32], std::vector<uint8_t> plain) {
	while(plain.size() % 16) {
		plain.push_back(0);
	}
	aes128 aes(secret);
	std::vector<uint8_t> out(2 + plain.size());
	for(size_t i = 0; i < plain.size(); i += 16) {
		aes.encrypt_block(&plain[i], &out[2 + i]);
	}
	uint8_t d[32];
	hmac_sha256(secret, 32, &out[2], plain.size(), d);
	out[0] = d[0];
	out[1] = d[1];
	return out;
}

void mc_dm_set_identity(const mc_identity &id) {
	uint8_t h[64];
	crypto_sha512(h, id.secret, 32);
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	memcpy(g_self_scalar, h, 32);
	memcpy(g_self_pub, id.pub, 32);
	g_self_ok = true;
	crypto_wipe(h, sizeof(h));
}

void mc_dm_set_callback(std::function<void(const mc_dm_event &)> cb) {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	g_dm_cb = std::move(cb);
}

static void contacts_save_locked() {
	if(g_contacts_path.empty()) {
		return;
	}
	std::string tmp = g_contacts_path + ".new";
	FILE *f = fopen(tmp.c_str(), "w");
	if(!f) {
		return;
	}
	for(const contact &c : g_contacts) {
		for(int i = 0; i < 32; i++) {
			fprintf(f, "%02x", c.pub[i]);
		}
		fprintf(f, " %s\n", c.name.c_str());
	}
	fclose(f);
	rename(tmp.c_str(), g_contacts_path.c_str());
}

// Routes live beside the contacts: without them a restart puts every DM back on a flood, and in
// field tests a peer answered 0 of 7 flood DMs with an ACK that arrived here.
static void routes_save_locked() {
	if(g_contacts_path.empty()) {
		return;
	}
	std::string path = g_contacts_path + ".routes", tmp = path + ".new";
	FILE *f = fopen(tmp.c_str(), "w");
	if(!f) {
		return;
	}
	for(const contact &c : g_contacts) {
		if(!c.has_route) {
			continue;
		}
		for(int i = 0; i < 32; i++) {
			fprintf(f, "%02x", c.pub[i]);
		}
		fprintf(f, " %u ", (unsigned)c.route_len);
		for(uint8_t b : c.route) {
			fprintf(f, "%02x", b);
		}
		fprintf(f, "\n");
	}
	fclose(f);
	rename(tmp.c_str(), path.c_str());
}

static void routes_load_locked() {
	FILE *f = fopen((g_contacts_path + ".routes").c_str(), "r");
	if(!f) {
		return;
	}
	char line[400];
	while(fgets(line, sizeof(line), f)) {
		char hex[65], rt[257] = "";
		unsigned len = 0;
		if(sscanf(line, "%64s %u %256s", hex, &len, rt) < 2 || strlen(hex) != 64 || strlen(rt) % 2 != 0) {
			continue;
		}
		size_t want = (size_t)(len & 63) * (size_t)((len >> 6) + 1);
		if(strlen(rt) / 2 != want) {
			continue;              // a damaged line must not send DMs down a wrong path
		}
		for(contact &c : g_contacts) {
			char have[65];
			for(int i = 0; i < 32; i++) {
				snprintf(have + 2 * i, 3, "%02x", c.pub[i]);
			}
			if(strcmp(have, hex) != 0) {
				continue;
			}
			c.has_route = true;
			c.route_len = (uint8_t)len;
			c.route.clear();
			for(size_t i = 0; i < want; i++) {
				unsigned v = 0;
				sscanf(rt + 2 * i, "%2x", &v);
				c.route.push_back((uint8_t)v);
			}
		}
	}
	fclose(f);
}

static bool add_locked(const uint8_t pub[32], const std::string &name) {
	for(contact &c : g_contacts) {
		if(memcmp(c.pub, pub, 32) == 0) {
			if(c.name == name) {
				return false;
			}
			c.name = name;
			return true;
		}
	}
	contact c;
	c.name = name;
	memcpy(c.pub, pub, 32);
	g_contacts.push_back(c);
	return true;
}

void mc_contacts_open(const std::string &path) {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	g_contacts_path = path;
	FILE *f = fopen(path.c_str(), "r");
	if(!f) {
		return;
	}
	char line[160];
	while(fgets(line, sizeof(line), f)) {
		uint8_t pub[32];
		bool ok = strlen(line) > 65 && line[64] == ' ';
		for(int i = 0; ok && i < 32; i++) {
			unsigned v;
			ok = sscanf(line + 2 * i, "%2x", &v) == 1;
			pub[i] = (uint8_t)v;
		}
		if(ok) {
			std::string name(line + 65);
			while(!name.empty() && (name.back() == '\n' || name.back() == '\r')) {
				name.pop_back();
			}
			add_locked(pub, name);
		}
	}
	fclose(f);
	routes_load_locked();
}

bool mc_contact_add(const uint8_t pub[32], const std::string &name) {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	bool changed = add_locked(pub, name);
	if(changed) {
		contacts_save_locked();
	}
	return changed;
}

bool mc_contact_find(const std::string &prefix_hex, uint8_t pub[32]) {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	for(const contact &c : g_contacts) {
		if(prefix_hex.size() >= 2 && hex6(c.pub).compare(0, prefix_hex.size(), prefix_hex) == 0) {
			memcpy(pub, c.pub, 32);
			return true;
		}
	}
	return false;
}

std::vector<std::pair<std::string, std::string>> mc_contacts() {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	std::vector<std::pair<std::string, std::string>> v;
	for(const contact &c : g_contacts) {
		v.push_back({ c.name, hex6(c.pub) });
	}
	return v;
}

void mc_contacts_clear() {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	g_contacts.clear();
	g_dm_seen.clear();
}

void mc_contact_seen(const std::vector<uint8_t> &frame, int node_type, const std::string &name) {
	if(node_type != ROUTE_NODE_CHAT || name.empty() || name.find('\n') != std::string::npos || !mc_verify_advert(frame)) {
		return;
	}
	size_t i = 1;
	int route = frame[0] & 3;
	if(route == 0 || route == 3) {
		i = 5;
	}
	int pl = frame[i++];
	i += (size_t)(pl & 63) * ((pl >> 6) + 1);
	if(i + 32 <= frame.size()) {
		mc_contact_add(&frame[i], name);
	}
}

void mc_dm_receive(meshcore_fields &f, const uint8_t *b, int len, uint8_t path_len, const uint8_t *path, size_t path_bytes) {
	int ct_len = len - 4;
	if(ct_len <= 0 || ct_len % 16 != 0) {
		return;
	}
	std::unique_lock<std::mutex> lk(g_dm_mtx);
	if(!g_self_ok || b[0] != g_self_pub[0]) {
		return;
	}
	for(contact &c : g_contacts) {
		if(c.pub[0] != b[1]) {
			continue;
		}
		uint8_t secret[32], mac[32];
		shared_secret(c.pub, secret);
		hmac_sha256(secret, 32, b + 4, (size_t)ct_len, mac);
		if(memcmp(mac, b + 2, 2) != 0) {
			continue;
		}
		std::vector<uint8_t> pt((size_t)ct_len + 1, 0);
		aes128 aes(secret);
		for(int o = 0; o < ct_len; o += 16) {
			aes.decrypt_block(b + 4 + o, pt.data() + o);
		}
		if(pt.size() < 7 || (pt[4] >> 2) != 0) {
			return;           // only plain text
		}
		size_t text_len = strnlen((const char *)&pt[5], pt.size() - 6);
		f.encrypted = false;
		f.has_text = true;
		f.sender = c.name;
		f.text.assign((const char *)&pt[5], text_len);
		f.channel_name = "DM";
		mc_dm_event e;
		memcpy(e.peer, c.pub, 32);
		memcpy(e.secret, secret, 32);
		sha256 s;
		s.update(pt.data(), 5 + text_len);
		s.update(c.pub, 32);
		uint8_t d[32];
		s.finish(d);
		memcpy(e.ack, d, 4);
		e.ack[4] = 5 + text_len + 1 < pt.size() ? pt[5 + text_len + 1] : 0;
		e.ack[5] = (uint8_t)rand();
		e.flood = f.route == 0 || f.route == 1;
		e.scoped = f.route == 0;
		e.path_len = path_len;
		e.path.assign(path, path + path_bytes);
		if(e.flood) {
			std::vector<uint8_t> learned = reverse_path(e.path, path_len);
			bool changed = !c.has_route || c.route_len != path_len || c.route != learned;
			c.has_route = true;
			c.route_len = path_len;
			c.route = learned;
			if(changed) {
				routes_save_locked();
			}
		}
		e.has_route = c.has_route;
		e.route_len = c.route_len;
		e.route = c.route;
		uint32_t id;
		memcpy(&id, d, 4);
		if(std::find(g_dm_seen.begin(), g_dm_seen.end(), id) != g_dm_seen.end()) {
			return;           // a relayed copy: shown, but acknowledged only once
		}
		g_dm_seen.push_back(id);
		if(g_dm_seen.size() > 64) {
			g_dm_seen.erase(g_dm_seen.begin());
		}
		auto cb = g_dm_cb;
		lk.unlock();
		if(cb) {
			cb(e);
		}
		return;
	}
}

void mc_ack_set_callback(std::function<void(uint32_t)> cb) {
	std::lock_guard<std::mutex> lk(g_dm_mtx);
	g_ack_cb = std::move(cb);
}

void mc_ack_seen(uint32_t ack) {
	std::function<void(uint32_t)> cb;
	{
		std::lock_guard<std::mutex> lk(g_dm_mtx);
		cb = g_ack_cb;
	}
	if(cb) {
		cb(ack);
	}
}

// A flood DM is answered by a PATH return from the recipient, with the ACK inside the ciphertext:
// path length byte, path, type of the extra data, extra data. Mesh::createPathReturn.
void mc_path_receive(const uint8_t *b, int len) {
	int ct_len = len - 4;
	if(ct_len <= 0 || ct_len % 16 != 0) {
		return;
	}
	uint32_t ack = 0;
	bool found = false;
	{
		std::lock_guard<std::mutex> lk(g_dm_mtx);
		if(!g_self_ok || b[0] != g_self_pub[0]) {
			return;
		}
		for(const contact &c : g_contacts) {
			if(c.pub[0] != b[1]) {
				continue;
			}
			uint8_t secret[32], mac[32];
			shared_secret(c.pub, secret);
			hmac_sha256(secret, 32, b + 4, (size_t)ct_len, mac);
			if(memcmp(mac, b + 2, 2) != 0) {
				continue;
			}
			std::vector<uint8_t> pt((size_t)ct_len, 0);
			aes128 aes(secret);
			for(int o = 0; o < ct_len; o += 16) {
				aes.decrypt_block(b + 4 + o, pt.data() + o);
			}
			size_t k = 1 + (size_t)(pt[0] & 63) * (size_t)((pt[0] >> 6) + 1);
			if(k + 5 <= pt.size() && (pt[k] & 15) == TYPE_ACK) {
				ack = (uint32_t)pt[k + 1] | (uint32_t)pt[k + 2] << 8 | (uint32_t)pt[k + 3] << 16 | (uint32_t)pt[k + 4] << 24;
				found = true;
			}
			break;
		}
	}
	if(found) {
		mc_ack_seen(ack);
	}
}

bool mc_build_dm(const mc_identity &id, const uint8_t peer[32], uint32_t ts, const std::string &text,
                 const mc_scope &scope, std::vector<uint8_t> &frame, int attempt, uint32_t *expected_ack, bool *routed) {
	if(text.empty() || text.size() > MAX_TEXT) {
		return false;
	}
	std::vector<uint8_t> plain;
	for(int i = 0; i < 4; i++) {
		plain.push_back((uint8_t)(ts >> (8 * i)));
	}
	plain.push_back((uint8_t)(attempt & 3));   // TXT_TYPE_PLAIN, attempt in the low bits
	plain.insert(plain.end(), text.begin(), text.end());
	if(expected_ack) {
		sha256 s;
		s.update(plain.data(), plain.size());
		s.update(id.pub, 32);
		uint8_t d[32];
		s.finish(d);
		*expected_ack = (uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24;
	}
	uint8_t secret[32];
	bool has_route = false;
	uint8_t route_len = 0;
	std::vector<uint8_t> route;
	{
		std::lock_guard<std::mutex> lk(g_dm_mtx);
		shared_secret(peer, secret);
		for(const contact &c : g_contacts) {
			if(routed && c.has_route && memcmp(c.pub, peer, 32) == 0) {
				has_route = true;
				route_len = c.route_len;
				route = c.route;
			}
		}
	}
	std::vector<uint8_t> p = { peer[0], id.pub[0] };
	std::vector<uint8_t> sealed = seal(secret, plain);
	p.insert(p.end(), sealed.begin(), sealed.end());
	if(p.size() > MAX_PAYLOAD) {
		return false;
	}
	if(routed) {
		*routed = has_route;
	}
	if(has_route) {
		// A routed DM is answered along the sender's known route, not by a PATH flood. In field
		// tests no ACK arrived for seven flood DMs the peer did read, plain or scoped, while the
		// peer's routed DMs reached this node every time.
		frame.clear();
		frame.push_back((uint8_t)(2 | (TYPE_TXT_MSG << 2)));     // ROUTE_DIRECT
		frame.push_back(route_len);
		frame.insert(frame.end(), route.begin(), route.end());
		frame.insert(frame.end(), p.begin(), p.end());
		return true;
	}
	frame = wrap(TYPE_TXT_MSG, p, scope);
	return true;
}

std::vector<uint8_t> mc_build_path_ack(const mc_identity &id, const mc_dm_event &e, const mc_scope &scope) {
	std::vector<uint8_t> data;
	data.push_back(e.path_len);
	data.insert(data.end(), e.path.begin(), e.path.end());
	data.push_back((uint8_t)TYPE_ACK);
	data.insert(data.end(), e.ack, e.ack + 6);
	std::vector<uint8_t> p = { e.peer[0], id.pub[0] };
	std::vector<uint8_t> sealed = seal(e.secret, data);
	p.insert(p.end(), sealed.begin(), sealed.end());
	return wrap(TYPE_PATH, p, scope);
}

std::vector<uint8_t> mc_build_direct_ack(const mc_dm_event &e) {
	std::vector<uint8_t> f;
	f.push_back((uint8_t)(2 | (TYPE_ACK << 2)));     // ROUTE_DIRECT
	f.push_back(e.route_len);
	f.insert(f.end(), e.route.begin(), e.route.end());
	f.insert(f.end(), e.ack, e.ack + 6);
	return f;
}

std::vector<uint8_t> mc_build_flood_ack(const mc_dm_event &e, const mc_scope &scope) {
	return wrap(TYPE_ACK, std::vector<uint8_t>(e.ack, e.ack + 6), scope);
}

bool mc_verify_advert(const std::vector<uint8_t> &f) {
	if(f.size() < 2) {
		return false;
	}
	int route = f[0] & 3;
	if(((f[0] >> 2) & 15) != TYPE_ADVERT) {
		return false;
	}
	size_t i = 1;
	if(route == 0 || route == 3) {
		i += 4;
	}
	if(i >= f.size()) {
		return false;
	}
	int plen = f[i++];
	i += (size_t)(((plen >> 6) + 1) * (plen & 63));
	if(f.size() < i + 100) {
		return false;
	}
	const uint8_t *pub = &f[i], *sig = &f[i + 36];
	std::vector<uint8_t> msg(f.begin() + (long)i, f.begin() + (long)i + 36);
	msg.insert(msg.end(), f.begin() + (long)i + 100, f.end());
	return crypto_ed25519_check(sig, pub, msg.data(), msg.size()) == 0;
}

phy_params mc_phy() {
	phy_params p;
	p.sf = 8;
	p.cr = 4;
	p.ldro = false;
	p.has_crc = true;
	p.sync = 0x12;
	p.preamble_len = 32;     // RadioLibWrappers.h: 32 symbols at SF <= 8
	return p;
}

double mc_airtime_s(size_t frame_len) {
	phy_params p = mc_phy();
	double sym = (double)(1 << p.sf) / 62500.0;
	return sym * (p.preamble_len + 4.25 + 8 + n_payload_symbols(p, (int)frame_len));
}

std::vector<int16_t> mc_render_iq(const std::vector<uint8_t> &frame, double rate_hz, double offset_hz) {
	return lora_render_iq(mc_phy(), 62500.0, frame, rate_hz, offset_hz);
}

std::vector<int16_t> lora_render_iq(const phy_params &p, double bw_hz, const std::vector<uint8_t> &frame, double rate_hz, double offset_hz) {
	std::vector<uint16_t> syms = encode_frame(p, frame.data(), (int)frame.size());
	const int n = 1 << p.sf;
	const int os = (int)lround(rate_hz / bw_hz);
	const double fo = offset_hz / rate_hz;
	std::vector<float> iq;
	double ph = 0;
	auto put = [&](double f) {
		ph += 2.0 * M_PI * (f / os + fo);
		if(ph > M_PI) {
			ph -= 2.0 * M_PI * floor((ph + M_PI) / (2.0 * M_PI));
		}
		iq.push_back((float)cos(ph));
		iq.push_back((float)sin(ph));
	};
	auto up = [&](int s) {
		for(int k = 0; k < n * os; k++) {
			double pos = fmod((double)s + (double)k / os, (double)n);
			put((pos + 0.5) / n - 0.5);
		}
	};
	auto down = [&](int len) {
		for(int k = 0; k < len; k++) {
			put(0.5 - ((double)k / os + 0.5) / n);
		}
	};
	for(int i = 0; i < p.preamble_len; i++) {
		up(0);
	}
	up(((p.sync >> 4) & 0xF) << 3);
	up((p.sync & 0xF) << 3);
	down(n * os);
	down(n * os);
	down(n * os / 4);
	for(uint16_t s : syms) {
		up(s);
	}
	const float amp = 14000.0f;
	std::vector<int16_t> out;
	out.reserve(iq.size() + 1024);
	const size_t total = iq.size() / 2, ramp = 256;
	for(size_t i = 0; i < total; i++) {
		float g = amp;
		if(i < ramp) {
			g *= (float)i / ramp;
		} else if(total - i < ramp) {
			g *= (float)(total - i) / ramp;
		}
		out.push_back((int16_t)lrintf(iq[2 * i] * g));
		out.push_back((int16_t)lrintf(iq[2 * i + 1] * g));
	}
	while((out.size() / 2) % 256) {
		out.push_back(0);
		out.push_back(0);
	}
	return out;
}

void mc_duty::trim(double now_s) {
	while(!log_.empty() && log_.front().t + window_ < now_s) {
		log_.pop_front();
	}
}

double mc_duty::used(double now_s) {
	trim(now_s);
	double u = 0;
	for(const entry &e : log_) {
		u += e.a;
	}
	return u;
}

bool mc_duty::allow(double now_s, double airtime_s) {
	return used(now_s) + airtime_s <= cap_;
}

const char *mc_tx_gate(const mc_gate_in &g) {
	if(!g.tx_mode) {
		return "TX needs meshcore mode, or meshtastic mode with a private channel";
	}
	if(!g.board_all) {
		return "board policy refuses TX (TX_ALLOWED must be 1, Lime TX is SMA-A only)";
	}
	if(!g.tx_flag) {
		return "daemon not started with --tx 1";
	}
	if(g.power_dbm > LORAD_TX_HARD_CAP_DBM) {
		return "power above the compiled hard cap";
	}
	if(!g.armed) {
		return "not armed";
	}
	return nullptr;
}

} // namespace lorad

#include "meshtastic_tx.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#include <mutex>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "aes128.h"
#include "sha256.h"
#include "vendor/monocypher.h"

namespace lorad {

static const size_t MAX_BODY = 237;    // MAX_LORA_PAYLOAD_LEN 255 minus the 16 byte header, Data overhead
static const uint32_t BROADCAST = 0xffffffffu;
static const int PORT_TEXT = 1, PORT_NODEINFO = 4, PORT_ROUTING = 5;
static const int FLAG_WANT_ACK = 8;
static const size_t PKC_OVERHEAD = 12, CCM_M = 8;
static const uint8_t DEFAULT_PSK_FIRST = 0xd4;   // the public "AQ==" key starts d4 f1 bb 3a

uint8_t mt_channel_hash(const std::string &name, const uint8_t psk[16]) {
	uint8_t h = 0;
	for(char c : name) {
		h ^= (uint8_t)c;
	}
	for(int i = 0; i < 16; i++) {
		h ^= psk[i];
	}
	return h;
}

bool mt_channel_load(const std::string &path, mt_channel &ch, std::string &err) {
	std::ifstream f(path);
	std::string name, hex;
	if(!f || !(f >> name >> hex)) {
		err = "no private meshtastic channel in " + path;
		return false;
	}
	if(hex.size() != 32) {
		err = "channel key must be 32 hex digits";
		return false;
	}
	for(int i = 0; i < 16; i++) {
		unsigned b = 0;
		if(sscanf(hex.c_str() + 2 * i, "%2x", &b) != 1) {
			err = "channel key must be 32 hex digits";
			return false;
		}
		ch.psk[i] = (uint8_t)b;
	}
	if(ch.psk[0] == DEFAULT_PSK_FIRST && ch.psk[1] == 0xf1 && ch.psk[2] == 0xbb && ch.psk[3] == 0x3a) {
		err = "refusing the public default key";
		return false;
	}
	if(name.size() > 11) {
		err = "channel name longer than 11 bytes";
		return false;
	}
	ch.name = name;
	ch.hash = mt_channel_hash(name, ch.psk);
	ch.ok = true;
	return true;
}

phy_params mt_phy() {
	phy_params p;
	p.sf = 11;
	p.cr = 1;
	p.ldro = false;      // 8.2 ms symbols stay below the 16 ms that would ask for it
	p.has_crc = true;
	p.sync = 0x2B;
	p.preamble_len = 16;
	return p;
}

double mt_airtime_s(size_t frame_len) {
	phy_params p = mt_phy();
	double sym = (double)(1 << p.sf) / 250000.0;
	return sym * (p.preamble_len + 4.25 + 8 + n_payload_symbols(p, (int)frame_len));
}

static void put_varint(std::vector<uint8_t> &b, uint32_t v) {
	while(v >= 0x80) {
		b.push_back((uint8_t)(v | 0x80));
		v >>= 7;
	}
	b.push_back((uint8_t)v);
}

static void put_bytes(std::vector<uint8_t> &b, int field, const std::string &s) {
	put_varint(b, (uint32_t)(field << 3 | 2));
	put_varint(b, (uint32_t)s.size());
	b.insert(b.end(), s.begin(), s.end());
}

static void put_le32(std::vector<uint8_t> &b, uint32_t v) {
	for(int i = 0; i < 4; i++) {
		b.push_back((uint8_t)(v >> (8 * i)));
	}
}

static std::vector<uint8_t> data_msg(int port, const std::string &payload, bool want_response = false, uint32_t request_id = 0) {
	std::vector<uint8_t> body;
	put_varint(body, 1 << 3);     // Data.portnum, varint
	put_varint(body, (uint32_t)port);
	put_bytes(body, 2, payload);
	if(want_response) {
		put_varint(body, 3 << 3);
		put_varint(body, 1);
	}
	if(request_id) {
		put_varint(body, 6 << 3 | 5);   // Data.request_id, fixed32
		put_le32(body, request_id);
	}
	return body;
}

static void header(std::vector<uint8_t> &frame, uint32_t to, uint32_t from, uint32_t id, int flags, int hop_limit, uint8_t chan) {
	frame.clear();
	put_le32(frame, to);
	put_le32(frame, from);
	put_le32(frame, id);
	frame.push_back((uint8_t)(hop_limit | flags | (hop_limit << 5)));   // hop_limit, want_ack, hop_start
	frame.push_back(chan);
	frame.push_back(0);           // next hop: none
	frame.push_back(0);           // relay node: none
}

// a meshtastic.Data message under the 16 byte radio header, on the channel key
static bool seal(uint32_t from, uint32_t id, uint32_t to, int flags, int hop_limit, const mt_channel &ch, std::vector<uint8_t> body,
                 std::vector<uint8_t> &frame) {
	if(!ch.ok || hop_limit < 0 || hop_limit > 7 || body.size() > MAX_BODY) {
		return false;
	}
	uint8_t nonce[16];
	memset(nonce, 0, sizeof(nonce));   // packet id as u64 LE, sender LE, 32-bit block counter
	memcpy(nonce, &id, 4);
	memcpy(nonce + 8, &from, 4);
	aes128 aes(ch.psk);
	aes.ctr_crypt(nonce, body.data(), body.size());
	header(frame, to, from, id, flags, hop_limit, ch.hash);
	frame.insert(frame.end(), body.begin(), body.end());
	return true;
}

bool mt_build_text(uint32_t from, uint32_t id, int hop_limit, const mt_channel &ch, const std::string &text,
                   std::vector<uint8_t> &frame) {
	return !text.empty() && seal(from, id, BROADCAST, 0, hop_limit, ch, data_msg(PORT_TEXT, text), frame);
}

bool mt_build_nodeinfo(uint32_t from, uint32_t id, int hop_limit, const mt_channel &ch, const std::string &long_name,
                       const std::string &short_name, std::vector<uint8_t> &frame, const uint8_t *public_key, bool want_response) {
	char node[12];
	snprintf(node, sizeof(node), "!%08x", from);
	std::vector<uint8_t> user;
	put_bytes(user, 1, node);          // User.id
	put_bytes(user, 2, long_name);
	put_bytes(user, 3, short_name);
	put_varint(user, 5 << 3);          // User.hw_model: PRIVATE_HW
	put_varint(user, 255);
	if(public_key) {
		put_bytes(user, 8, std::string((const char *)public_key, 32));
	}
	return seal(from, id, BROADCAST, 0, hop_limit, ch, data_msg(PORT_NODEINFO, std::string(user.begin(), user.end()), want_response), frame);
}

void mt_keys_from_private(const uint8_t priv[32], mt_keys &k) {
	memcpy(k.priv, priv, 32);
	crypto_x25519_public_key(k.pub, k.priv);
	k.ok = true;
}

bool mt_keys_load(const std::string &path, mt_keys &k, std::string &err) {
	uint8_t priv[32];
	FILE *f = fopen(path.c_str(), "rb");
	if(f) {
		size_t n = fread(priv, 1, 32, f);
		fclose(f);
		if(n != 32) {
			err = "bad key file " + path;
			return false;
		}
	} else {
		int r = open("/dev/urandom", O_RDONLY);
		if(r < 0 || read(r, priv, 32) != 32) {
			err = "no random bytes";
			return false;
		}
		close(r);
		int w = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
		if(w < 0 || write(w, priv, 32) != 32) {
			err = "cannot write " + path;
			return false;
		}
		close(w);
	}
	mt_keys_from_private(priv, k);
	crypto_wipe(priv, 32);
	return true;
}

void mt_shared_key(const mt_keys &k, const uint8_t peer[32], uint8_t out[32]) {
	uint8_t raw[32];
	crypto_x25519(raw, k.priv, peer);
	sha256 s;
	s.update(raw, 32);
	s.finish(out);
	crypto_wipe(raw, 32);
}

// RFC 3610 with M = 8, L = 2 and no associated data: 13 byte nonce from id, sender and extra
static void ccm_nonce(uint32_t from, uint32_t id, uint32_t extra, uint8_t nonce[13]) {
	memset(nonce, 0, 13);
	memcpy(nonce, &id, 4);
	memcpy(nonce + 4, &extra, 4);
	memcpy(nonce + 8, &from, 4);
}

static void ccm_tag(const aes256 &aes, const uint8_t nonce[13], const std::vector<uint8_t> &plain, uint8_t tag[8]) {
	uint8_t b[16], x[16];
	b[0] = (uint8_t)(((CCM_M - 2) / 2) << 3 | 1);
	memcpy(b + 1, nonce, 13);
	b[14] = (uint8_t)(plain.size() >> 8);
	b[15] = (uint8_t)plain.size();
	aes.encrypt_block(b, x);
	for(size_t i = 0; i < plain.size(); i += 16) {
		for(size_t j = 0; j < 16 && i + j < plain.size(); j++) {
			x[j] ^= plain[i + j];
		}
		aes.encrypt_block(x, x);
	}
	uint8_t a[16] = { 1 }, s0[16];
	memcpy(a + 1, nonce, 13);
	aes.encrypt_block(a, s0);
	for(size_t i = 0; i < CCM_M; i++) {
		tag[i] = (uint8_t)(x[i] ^ s0[i]);
	}
}

static void ccm_crypt(const aes256 &aes, const uint8_t nonce[13], const uint8_t *in, size_t len, uint8_t *out) {
	uint8_t a[16] = { 1 }, st[16];
	memcpy(a + 1, nonce, 13);
	for(size_t i = 0; i < len; i += 16) {
		size_t ctr = i / 16 + 1;
		a[14] = (uint8_t)(ctr >> 8);
		a[15] = (uint8_t)ctr;
		aes.encrypt_block(a, st);
		for(size_t j = 0; j < 16 && i + j < len; j++) {
			out[i + j] = (uint8_t)(in[i + j] ^ st[j]);
		}
	}
}

std::vector<uint8_t> mt_pkc_seal(const uint8_t shared[32], uint32_t from, uint32_t id, const std::vector<uint8_t> &plain, uint32_t extra) {
	aes256 aes(shared);
	uint8_t nonce[13], tag[8];
	ccm_nonce(from, id, extra, nonce);
	ccm_tag(aes, nonce, plain, tag);
	std::vector<uint8_t> out(plain.size());
	ccm_crypt(aes, nonce, plain.data(), plain.size(), out.data());
	out.insert(out.end(), tag, tag + 8);
	put_le32(out, extra);
	return out;
}

bool mt_pkc_open(const uint8_t shared[32], uint32_t from, uint32_t id, const uint8_t *ct, size_t len, std::vector<uint8_t> &plain) {
	if(len <= PKC_OVERHEAD) {
		return false;
	}
	size_t n = len - PKC_OVERHEAD;
	uint32_t extra;
	memcpy(&extra, ct + n + 8, 4);
	aes256 aes(shared);
	uint8_t nonce[13], tag[8];
	ccm_nonce(from, id, extra, nonce);
	plain.resize(n);
	ccm_crypt(aes, nonce, ct, n, plain.data());
	ccm_tag(aes, nonce, plain, tag);
	uint8_t diff = 0;
	for(int i = 0; i < 8; i++) {
		diff |= (uint8_t)(tag[i] ^ ct[n + (size_t)i]);
	}
	return diff == 0;
}

struct node {
	uint32_t id;
	uint8_t pub[32];
	bool has_key;
	std::string name;
};
static std::mutex g_nodes_mtx;
static std::vector<node> g_nodes;
static std::string g_nodes_path;

static void nodes_save_locked() {
	if(g_nodes_path.empty()) {
		return;
	}
	std::string tmp = g_nodes_path + ".new";
	FILE *f = fopen(tmp.c_str(), "w");
	if(!f) {
		return;
	}
	for(const node &n : g_nodes) {
		fprintf(f, "%08x ", n.id);
		for(int i = 0; i < 32; i++) {
			fprintf(f, "%02x", n.pub[i]);
		}
		fprintf(f, " %s\n", n.name.c_str());
	}
	fclose(f);
	rename(tmp.c_str(), g_nodes_path.c_str());
}

void mt_nodes_clear() {
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	g_nodes.clear();
	g_nodes_path.clear();
}

void mt_nodes_open(const std::string &path) {
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	g_nodes_path = path;
	std::ifstream f(path);
	std::string line;
	while(std::getline(f, line)) {
		unsigned id = 0;
		char hex[65];
		int used = 0;
		if(sscanf(line.c_str(), "%8x %64s %n", &id, hex, &used) < 2 || strlen(hex) != 64) {
			continue;
		}
		node n;
		n.id = id;
		n.has_key = true;
		for(int i = 0; i < 32; i++) {
			unsigned b = 0;
			sscanf(hex + 2 * i, "%2x", &b);
			n.pub[i] = (uint8_t)b;
		}
		n.name = line.substr((size_t)used);
		g_nodes.push_back(n);
	}
}

bool mt_node_add(uint32_t id, const uint8_t pub[32], const std::string &name) {
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	for(node &n : g_nodes) {
		if(n.id == id) {
			// first key seen stays: a NODEINFO is unsigned, so a later one must not replace it
			if(memcmp(n.pub, pub, 32) != 0) {
				return false;
			}
			if(n.name != name) {
				n.name = name;
				nodes_save_locked();
			}
			return true;
		}
	}
	node n;
	n.id = id;
	memcpy(n.pub, pub, 32);
	n.has_key = true;
	n.name = name;
	g_nodes.push_back(n);
	nodes_save_locked();
	return true;
}

static bool node_key(uint32_t id, uint8_t pub[32]) {
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	for(const node &n : g_nodes) {
		if(n.id == id) {
			memcpy(pub, n.pub, 32);
			return true;
		}
	}
	return false;
}

bool mt_node_find(const std::string &prefix_hex, uint32_t &id, uint8_t pub[32], bool &has_key) {
	std::string want = prefix_hex;
	if(!want.empty() && want[0] == '!') {
		want = want.substr(1);
	}
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	for(const node &n : g_nodes) {
		char b[12];
		snprintf(b, sizeof(b), "%08x", n.id);
		if(!want.empty() && std::string(b).compare(0, want.size(), want) == 0) {
			id = n.id;
			memcpy(pub, n.pub, 32);
			has_key = n.has_key;
			return true;
		}
	}
	return false;
}

std::vector<std::pair<std::string, std::string>> mt_nodes() {
	std::lock_guard<std::mutex> lk(g_nodes_mtx);
	std::vector<std::pair<std::string, std::string>> out;
	for(const node &n : g_nodes) {
		char b[12];
		snprintf(b, sizeof(b), "%08x", n.id);
		out.push_back({ n.name, b });
	}
	return out;
}

bool mt_build_dm(const mt_keys &k, uint32_t from, uint32_t id, uint32_t to, const uint8_t *peer_pub, int hop_limit,
                 const mt_channel &ch, const std::string &text, std::vector<uint8_t> &frame) {
	if(text.empty() || hop_limit < 0 || hop_limit > 7) {
		return false;
	}
	std::vector<uint8_t> body = data_msg(PORT_TEXT, text);
	if(!peer_pub) {
		return seal(from, id, to, FLAG_WANT_ACK, hop_limit, ch, body, frame);
	}
	if(!k.ok || body.size() + PKC_OVERHEAD > MAX_BODY) {
		return false;
	}
	uint8_t shared[32];
	mt_shared_key(k, peer_pub, shared);
	uint32_t extra = id ^ 0x5a5a5a5au;
	int r = open("/dev/urandom", O_RDONLY);
	if(r >= 0) {
		if(read(r, &extra, 4) != 4) {
			extra = id ^ 0x5a5a5a5au;
		}
		close(r);
	}
	extra |= 1;            // zero would leave the id bytes in the nonce
	std::vector<uint8_t> ct = mt_pkc_seal(shared, from, id, body, extra);
	crypto_wipe(shared, 32);
	header(frame, to, from, id, FLAG_WANT_ACK, hop_limit, 0);
	frame.insert(frame.end(), ct.begin(), ct.end());
	return true;
}

bool mt_build_ack(uint32_t from, uint32_t id, uint32_t to, int hop_limit, const mt_channel &ch, uint32_t request_id, int error,
                  std::vector<uint8_t> &frame) {
	std::string routing;
	routing.push_back((char)(3 << 3));   // Routing.error_reason: a oneof, so zero is sent too
	routing.push_back((char)error);
	return seal(from, id, to, 0, hop_limit, ch, data_msg(PORT_ROUTING, routing, false, request_id), frame);
}

static bool get_varint(const uint8_t *p, size_t n, size_t &i, uint64_t &v) {
	v = 0;
	for(int s = 0; s < 64 && i < n; s += 7) {
		uint8_t b = p[i++];
		v |= (uint64_t)(b & 0x7f) << s;
		if(!(b & 0x80)) {
			return true;
		}
	}
	return false;
}

// walks a protobuf message: cb(field, varint value, bytes, length)
template <class F> static bool walk(const uint8_t *p, size_t n, F cb) {
	size_t i = 0;
	while(i < n) {
		uint64_t tag;
		if(!get_varint(p, n, i, tag)) {
			return false;
		}
		int field = (int)(tag >> 3), wire = (int)(tag & 7);
		uint64_t v = 0;
		if(wire == 0) {
			if(!get_varint(p, n, i, v)) {
				return false;
			}
			cb(field, v, nullptr, (size_t)0);
		} else if(wire == 2) {
			if(!get_varint(p, n, i, v) || v > n - i) {
				return false;
			}
			cb(field, (uint64_t)0, p + i, (size_t)v);
			i += (size_t)v;
		} else if(wire == 5) {
			if(n - i < 4) {
				return false;
			}
			uint32_t f;
			memcpy(&f, p + i, 4);
			cb(field, (uint64_t)f, nullptr, (size_t)0);
			i += 4;
		} else if(wire == 1) {
			if(n - i < 8) {
				return false;
			}
			i += 8;
		} else {
			return false;
		}
	}
	return true;
}

mt_event mt_receive(const std::vector<uint8_t> &p, uint32_t self, const mt_keys &k, const mt_channel &ch) {
	mt_event e;
	if(p.size() < 16) {
		return e;
	}
	memcpy(&e.to, &p[0], 4);
	memcpy(&e.from, &p[4], 4);
	memcpy(&e.id, &p[8], 4);
	if(e.from == self) {
		return e;
	}
	e.want_ack = (p[12] & FLAG_WANT_ACK) != 0;
	e.hop_start = (p[12] >> 5) & 7;
	e.for_us = e.to == self;
	std::vector<uint8_t> body;
	if(p[13] == 0 && e.for_us && k.ok) {
		uint8_t pub[32], shared[32];
		if(!node_key(e.from, pub)) {
			e.unreadable = true;
			return e;
		}
		mt_shared_key(k, pub, shared);
		bool ok = mt_pkc_open(shared, e.from, e.id, p.data() + 16, p.size() - 16, body);
		crypto_wipe(shared, 32);
		if(!ok) {
			e.unreadable = true;
			return e;
		}
		e.sealed = true;
	} else if(ch.ok && p[13] == ch.hash) {
		body.assign(p.begin() + 16, p.end());
		uint8_t nonce[16];
		memset(nonce, 0, sizeof(nonce));
		memcpy(nonce, &e.id, 4);
		memcpy(nonce + 8, &e.from, 4);
		aes128 aes(ch.psk);
		aes.ctr_crypt(nonce, body.data(), body.size());
	} else {
		return e;
	}
	std::vector<uint8_t> inner;
	bool ok = walk(body.data(), body.size(), [&](int f, uint64_t v, const uint8_t *b, size_t n) {
		if(f == 1) {
			e.port = (int)v;
		} else if(f == 2 && b) {
			inner.assign(b, b + n);
		} else if(f == 3) {
			e.want_response = v != 0;
		} else if(f == 6) {
			e.request_id = (uint32_t)v;
		}
	});
	if(!ok || e.port == 0) {
		return e;
	}
	e.valid = true;
	if(e.port == PORT_TEXT) {
		e.text.assign(inner.begin(), inner.end());
	} else if(e.port == PORT_ROUTING) {
		e.routing_error = 0;
		walk(inner.data(), inner.size(), [&](int f, uint64_t v, const uint8_t *, size_t) {
			if(f == 3) {
				e.routing_error = (int)v;
			}
		});
	} else if(e.port == PORT_NODEINFO) {
		std::string name;
		std::vector<uint8_t> key;
		walk(inner.data(), inner.size(), [&](int f, uint64_t, const uint8_t *b, size_t n) {
			if(f == 2 && b) {
				name.assign((const char *)b, n);
			} else if(f == 8 && b) {
				key.assign(b, b + n);
			}
		});
		if(key.size() == 32) {
			mt_node_add(e.from, key.data(), name);
		}
	}
	return e;
}

} // namespace lorad

#include "meshcore.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "aes128.h"
#include "packets.h"
#include "parse.h"
#include "sha256.h"

namespace lorad {

// limits from src/MeshCore.h
static const int PUB_KEY_SIZE = 32;
static const int SIGNATURE_SIZE = 64;
static const int MAX_ADVERT_DATA_SIZE = 32;
static const int CIPHER_BLOCK_SIZE = 16;
static const int CIPHER_MAC_SIZE = 2;
static const int MAX_PATH_SIZE = 64;
static const int MAX_PACKET_PAYLOAD = 184;
static const int PUBKEY_PREFIX = 6;

enum {
	TYPE_REQ = 0, TYPE_RESPONSE = 1, TYPE_TXT_MSG = 2, TYPE_ACK = 3, TYPE_ADVERT = 4, TYPE_GRP_TXT = 5,
	TYPE_GRP_DATA = 6, TYPE_ANON_REQ = 7, TYPE_PATH = 8, TYPE_TRACE = 9, TYPE_MULTIPART = 10,
	TYPE_CONTROL = 11, TYPE_RAW_CUSTOM = 15,
};

// default "Public" channel key: PUBLIC_GROUP_PSK = "izOH6cXN6mrJ5e26oRXNcg==" in
// examples/companion_radio/MyMesh.cpp. It is public by design.
static const uint8_t PUBLIC_KEY[16] = {
	0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a, 0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72,
};
static const uint8_t PUBLIC_HASH = 0x11; // first byte of SHA-256(PUBLIC_KEY)

static uint32_t le32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static std::string hex_of(const uint8_t *p, size_t n) {
	return hex_string(std::vector<uint8_t>(p, p + n));
}

const char *meshcore_type_name(int type) {
	static const char *NAMES[16] = {
		"REQ", "RESPONSE", "TXT_MSG", "ACK", "ADVERT", "GRP_TXT", "GRP_DATA", "ANON_REQ",
		"PATH", "TRACE", "MULTIPART", "CONTROL", "TYPE12", "TYPE13", "TYPE14", "RAW_CUSTOM",
	};
	return NAMES[type & 15];
}

static const char *route_name(int route) {
	static const char *NAMES[4] = { "transport-flood", "flood", "direct", "transport-direct" };
	return NAMES[route & 3];
}

static const char *role_name(int t) {
	switch(t) {
	case 1: return "chat";
	case 2: return "repeater";
	case 3: return "room";
	case 4: return "sensor";
	default: return "";
	}
}

// Packet::calculatePacketHash: SHA-256 over type + payload, truncated to MAX_HASH_SIZE.
// TRACE also mixes in path_len, a little-endian uint16 in the firmware's Packet struct.
static std::string packet_hash(int type, int path_len, const uint8_t *payload, size_t len) {
	sha256 h;
	uint8_t t = (uint8_t)type;
	h.update(&t, 1);
	if(type == TYPE_TRACE) {
		uint8_t pl[2] = { (uint8_t)path_len, 0 };
		h.update(pl, 2);
	}
	h.update(payload, len);
	uint8_t out[32];
	h.finish(out);
	return hex_of(out, 8);
}

// the plaintext is a C string zero-padded to the AES block: cut at the first NUL
static std::vector<uint8_t> c_string(const uint8_t *p, size_t n) {
	const void *z = memchr(p, 0, n);
	return std::vector<uint8_t>(p, z ? (const uint8_t *)z : p + n);
}

static void decode_advert(meshcore_fields &f, const uint8_t *b, int len) {
	if(len < PUB_KEY_SIZE + 4 + SIGNATURE_SIZE) {
		f.truncated = true;
		return;
	}
	f.has_identity = true;
	f.pubkey = hex_of(b, PUBKEY_PREFIX);
	f.adv_time = le32(b + PUB_KEY_SIZE);
	const uint8_t *app = b + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE;
	int app_len = len - (PUB_KEY_SIZE + 4 + SIGNATURE_SIZE);
	if(app_len > MAX_ADVERT_DATA_SIZE) {
		app_len = MAX_ADVERT_DATA_SIZE; // the firmware ignores anything beyond this too
	}
	f.has_advert = true;
	if(app_len < 1) {
		return;
	}
	uint8_t flags = app[0];
	f.node_type = flags & 0x0F;
	int i = 1;
	bool has_loc = flags & 0x10;
	int need = i + (has_loc ? 8 : 0) + ((flags & 0x20) ? 2 : 0) + ((flags & 0x40) ? 2 : 0);
	if(need > app_len) {
		f.truncated = true; // flags promise more than the frame holds: AdvertDataParser rejects it
		return;
	}
	if(has_loc) {
		f.lat = (double)(int32_t)le32(app + i) / 1e6;
		f.lon = (double)(int32_t)le32(app + i + 4) / 1e6;
		f.has_pos = f.lat >= -90 && f.lat <= 90 && f.lon >= -180 && f.lon <= 180 && (f.lat != 0 || f.lon != 0);
		i += 8;
	}
	i = need;
	if((flags & 0x80) && i < app_len) {
		std::vector<uint8_t> nm = c_string(app + i, (size_t)(app_len - i));
		if(clean_text(nm)) {
			f.name.assign(nm.begin(), nm.end());
		}
	}
}

// MAC-then-decrypt against a channel key, as Utils::MACThenDecrypt: HMAC-SHA-256 keyed
// with the 32-byte secret buffer (16-byte key, zero padded), truncated to 2 bytes, over
// the ciphertext; then AES-128-ECB. Any other key fails the MAC and stays ciphertext.
static bool group_decrypt(const uint8_t key[16], const uint8_t *mac_and_ct, int len, std::vector<uint8_t> &plain) {
	int ct_len = len - CIPHER_MAC_SIZE;
	if(ct_len <= 0 || ct_len % CIPHER_BLOCK_SIZE != 0) {
		return false;
	}
	const uint8_t *ct = mac_and_ct + CIPHER_MAC_SIZE;
	uint8_t secret[PUB_KEY_SIZE];
	memset(secret, 0, sizeof(secret));
	memcpy(secret, key, 16);
	uint8_t mac[32];
	hmac_sha256(secret, sizeof(secret), ct, (size_t)ct_len, mac);
	if(memcmp(mac, mac_and_ct, CIPHER_MAC_SIZE) != 0) {
		return false;
	}
	aes128 aes(key);
	plain.resize((size_t)ct_len);
	for(int o = 0; o < ct_len; o += CIPHER_BLOCK_SIZE) {
		aes.decrypt_block(ct + o, plain.data() + o);
	}
	return true;
}

struct user_channel {
	std::string name;
	uint8_t key[16];
	uint8_t hash;
};
static std::mutex g_channels_mtx;
static std::vector<user_channel> g_channels;

bool mc_channel_name_ok(const std::string &name) {
	if(name.empty() || name.size() > 32 || name == "public" || name == "Public") {
		return false;
	}
	for(size_t i = 0; i < name.size(); i++) {
		char c = name[i];
		bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || (i == 0 && c == '#');
		if(!ok) {
			return false;
		}
	}
	return name != "#";
}

bool mc_channel_add(const std::string &name, const uint8_t key[16]) {
	if(!mc_channel_name_ok(name)) {
		return false;
	}
	user_channel c;
	c.name = name;
	memcpy(c.key, key, 16);
	uint8_t d[32];
	sha256 s;
	s.update(key, 16);
	s.finish(d);
	c.hash = d[0];
	std::lock_guard<std::mutex> lk(g_channels_mtx);
	for(auto &o : g_channels) {
		if(o.name == name) {
			o = c;
			return true;
		}
	}
	g_channels.push_back(c);
	return true;
}

bool mc_channel_remove(const std::string &name) {
	std::lock_guard<std::mutex> lk(g_channels_mtx);
	auto it = std::remove_if(g_channels.begin(), g_channels.end(), [&](const user_channel &c) { return c.name == name; });
	bool found = it != g_channels.end();
	g_channels.erase(it, g_channels.end());
	return found;
}

bool mc_channel_lookup(const std::string &name, uint8_t key[16]) {
	std::lock_guard<std::mutex> lk(g_channels_mtx);
	for(const auto &c : g_channels) {
		if(c.name == name) {
			memcpy(key, c.key, 16);
			return true;
		}
	}
	return false;
}

std::vector<std::string> mc_channel_names() {
	std::lock_guard<std::mutex> lk(g_channels_mtx);
	std::vector<std::string> v;
	for(const auto &c : g_channels) {
		v.push_back(c.name);
	}
	return v;
}

bool mc_channels_load(const std::string &path) {
	FILE *f = fopen(path.c_str(), "r");
	if(!f) {
		return false;
	}
	char name[40], hex[40];
	while(fscanf(f, "%39s %39s", name, hex) == 2) {
		uint8_t key[16];
		bool ok = strlen(hex) == 32;
		for(int i = 0; ok && i < 16; i++) {
			unsigned v;
			ok = sscanf(hex + 2 * i, "%2x", &v) == 1;
			key[i] = (uint8_t)v;
		}
		if(ok) {
			mc_channel_add(name, key);
		}
	}
	fclose(f);
	return true;
}

bool mc_channels_save(const std::string &path) {
	std::string tmp = path + ".new";
	int fd = open(tmp.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0600);
	if(fd < 0) {
		return false;
	}
	FILE *f = fdopen(fd, "w");
	{
		std::lock_guard<std::mutex> lk(g_channels_mtx);
		for(const auto &c : g_channels) {
			fprintf(f, "%s ", c.name.c_str());
			for(int i = 0; i < 16; i++) {
				fprintf(f, "%02x", c.key[i]);
			}
			fprintf(f, "\n");
		}
	}
	fclose(f);
	return rename(tmp.c_str(), path.c_str()) == 0;
}

// a one-byte hash collides on purpose often enough: the MAC decides, not the hash
static bool user_channel_decrypt(uint8_t hash, const uint8_t *mac_and_ct, int len, std::vector<uint8_t> &pt, std::string &name) {
	std::lock_guard<std::mutex> lk(g_channels_mtx);
	for(const auto &c : g_channels) {
		if(c.hash == hash && group_decrypt(c.key, mac_and_ct, len, pt)) {
			name = c.name;
			return true;
		}
	}
	return false;
}

static void decode_group(meshcore_fields &f, const uint8_t *b, int len) {
	if(len < 1 + CIPHER_MAC_SIZE + 1) {
		f.truncated = true;
		return;
	}
	f.channel = b[0];
	f.encrypted = true;
	std::vector<uint8_t> pt;
	if(b[0] == PUBLIC_HASH && group_decrypt(PUBLIC_KEY, b + 1, len - 1, pt)) {
		f.public_channel = true;
		f.channel_name = "Public";
	} else if(!user_channel_decrypt(b[0], b + 1, len - 1, pt, f.channel_name)) {
		return;
	}
	f.encrypted = false;
	if(f.type == TYPE_GRP_TXT) {
		// timestamp(4), txt_type/attempt(1), "<sender>: <text>". Only plain text is valid here
		if(pt.size() < 5 || (pt[4] >> 2) != 0) {
			return;
		}
		f.msg_time = le32(pt.data());
		std::vector<uint8_t> s = c_string(pt.data() + 5, pt.size() - 5);
		if(!clean_text(s)) {
			return;
		}
		std::string all(s.begin(), s.end());
		size_t sep = all.find(": ");
		f.has_text = true;
		if(sep != std::string::npos) {
			f.sender = all.substr(0, sep);
			f.text = all.substr(sep + 2);
		} else {
			f.text = all;
		}
	} else if(pt.size() >= 3) {
		f.has_group_data = true;
		f.data_type = pt[0] | (pt[1] << 8);
		f.data_len = pt[2];
	}
}

meshcore_fields meshcore_decode(const std::vector<uint8_t> &p) {
	meshcore_fields f;
	size_t i = 0;
	if(p.size() < 2) {
		return f;
	}
	f.route = p[0] & 3;
	f.type = (p[0] >> 2) & 15;
	f.version = p[0] >> 6;
	f.has_transport = f.route == 0 || f.route == 3;
	i = 1;
	if(f.has_transport) {
		if(p.size() < 6) {
			return f;
		}
		f.transport[0] = (uint16_t)(p[1] | (p[2] << 8));
		f.transport[1] = (uint16_t)(p[3] | (p[4] << 8));
		i = 5;
	}
	int path_len = p[i++];
	f.hash_size = (path_len >> 6) + 1;
	f.hops = path_len & 63;
	int path_bytes = f.hops * f.hash_size;
	// Packet::isValidPathLen and readFrom: reserved size 4, oversize path, empty or oversize payload
	if(f.hash_size == 4 || path_bytes > MAX_PATH_SIZE || i + (size_t)path_bytes >= p.size()) {
		return f;
	}
	const size_t payload_off = i + (size_t)path_bytes;
	f.payload_len = (int)(p.size() - payload_off);
	if(f.payload_len > MAX_PACKET_PAYLOAD) {
		return f;
	}
	for(int h = 0; h < f.hops; h++) {
		f.path.push_back(hex_of(&p[i + (size_t)h * f.hash_size], (size_t)f.hash_size));
	}
	f.valid = true;
	const uint8_t *b = &p[payload_off];
	f.hash = packet_hash(f.type, path_len, b, (size_t)f.payload_len);
	if(f.version != 0) {
		return f; // future payload format: header only
	}

	switch(f.type) {
	case TYPE_ADVERT:
		decode_advert(f, b, f.payload_len);
		if(f.has_advert) {
			mc_contact_seen(p, f.node_type, f.name);
		}
		break;
	case TYPE_GRP_TXT:
	case TYPE_GRP_DATA:
		decode_group(f, b, f.payload_len);
		break;
	case TYPE_REQ:
	case TYPE_RESPONSE:
	case TYPE_TXT_MSG:
	case TYPE_PATH:
		// dest_hash, src_hash, MAC, ciphertext: end-to-end encrypted with an ECDH secret
		if(f.payload_len < 2 + CIPHER_MAC_SIZE) {
			f.truncated = true;
		} else {
			f.dst = b[0];
			f.src = b[1];
			f.encrypted = true;
			if(f.type == TYPE_TXT_MSG) {
				mc_dm_receive(f, b, f.payload_len, (uint8_t)path_len, &p[i], (size_t)path_bytes);
			} else if(f.type == TYPE_PATH) {
				mc_path_receive(b, f.payload_len);
			}
		}
		break;
	case TYPE_ANON_REQ:
		// dest_hash, sender public key in the clear, MAC, ciphertext
		if(f.payload_len < 1 + PUB_KEY_SIZE + CIPHER_MAC_SIZE) {
			f.truncated = true;
		} else {
			f.dst = b[0];
			f.has_identity = true;
			f.pubkey = hex_of(b + 1, PUBKEY_PREFIX);
			f.src = b[1];
			f.encrypted = true;
		}
		break;
	case TYPE_ACK:
		if(f.payload_len < 4) {
			f.truncated = true;
		} else {
			f.ack = le32(b);
			f.has_ack = true;
			mc_ack_seen(f.ack);
		}
		break;
	case TYPE_CONTROL:
		f.control_sub = b[0] >> 4;
		break;
	default:
		break;
	}
	return f;
}

std::string meshcore_info(const meshcore_fields &f) {
	if(!f.valid) {
		return "";
	}
	char b[160];
	snprintf(b, sizeof(b), "%s %s hops=%d", meshcore_type_name(f.type), route_name(f.route), f.hops);
	std::string s = b;
	if(f.version != 0) {
		snprintf(b, sizeof(b), " ver=%d", f.version + 1);
		s += b;
	}
	if(f.has_advert) {
		if(!f.name.empty()) {
			s += " \"" + f.name + "\"";
		}
		if(role_name(f.node_type)[0]) {
			s += std::string(" ") + role_name(f.node_type);
		}
		s += " pk=" + f.pubkey;
		if(f.has_pos) {
			snprintf(b, sizeof(b), " pos=%.6f,%.6f", f.lat, f.lon);
			s += b;
		}
	} else if(f.has_identity) {
		s += " pk=" + f.pubkey;
	}
	if(f.channel >= 0) {
		snprintf(b, sizeof(b), " ch=%02x", f.channel);
		s += b;
		if(!f.channel_name.empty()) {
			s += " " + f.channel_name;
		}
	}
	if(f.has_text) {
		s += " \"" + (f.sender.empty() ? f.text : f.sender + ": " + f.text) + "\"";
	}
	if(f.has_group_data) {
		snprintf(b, sizeof(b), " data_type=%d len=%d", f.data_type, f.data_len);
		s += b;
	}
	if(f.dst >= 0) {
		if(f.src >= 0) {
			snprintf(b, sizeof(b), " %02x>%02x", f.src, f.dst);
		} else {
			snprintf(b, sizeof(b), " >%02x", f.dst);
		}
		s += b;
	}
	if(f.has_ack) {
		snprintf(b, sizeof(b), " ack=%08x", f.ack);
		s += b;
	}
	if(f.control_sub >= 0) {
		snprintf(b, sizeof(b), " sub=%d", f.control_sub);
		s += b;
	}
	if(f.encrypted) {
		s += " (encrypted)";
	}
	if(f.truncated) {
		s += " (truncated)";
	}
	return s;
}

static std::string jstr(const std::string &s) {
	return "\"" + packet_ring::json_escape(s) + "\"";
}

std::string meshcore_json(const meshcore_fields &f) {
	char b[192];
	snprintf(b, sizeof(b), "{\"type\":\"%s\",\"type_id\":%d,\"route\":\"%s\",\"version\":%d,\"hops\":%d,\"hash_size\":%d,\"payload_len\":%d,\"hash\":",
	         meshcore_type_name(f.type), f.type, route_name(f.route), f.version + 1, f.hops, f.hash_size, f.payload_len);
	std::string o = b;
	o += jstr(f.hash);
	o += ",\"encrypted\":";
	o += f.encrypted ? "true" : "false";
	o += ",\"path\":[";
	for(size_t i = 0; i < f.path.size(); i++) {
		o += (i ? "," : "") + jstr(f.path[i]);
	}
	o += "]";
	if(f.has_transport) {
		snprintf(b, sizeof(b), ",\"transport\":[%u,%u]", f.transport[0], f.transport[1]);
		o += b;
	}
	if(f.truncated) {
		o += ",\"truncated\":true";
	}
	if(f.has_identity) {
		o += ",\"pubkey\":" + jstr(f.pubkey);
	}
	if(f.has_advert) {
		snprintf(b, sizeof(b), ",\"adv_time\":%u,\"node_type\":%d", f.adv_time, f.node_type);
		o += b;
		if(role_name(f.node_type)[0]) {
			o += ",\"role\":" + jstr(role_name(f.node_type));
		}
		if(!f.name.empty()) {
			o += ",\"name\":" + jstr(f.name);
		}
		if(f.has_pos) {
			snprintf(b, sizeof(b), ",\"lat\":%.6f,\"lon\":%.6f", f.lat, f.lon);
			o += b;
		}
	}
	if(f.dst >= 0) {
		snprintf(b, sizeof(b), ",\"dst\":\"%02x\"", f.dst);
		o += b;
	}
	if(f.src >= 0) {
		snprintf(b, sizeof(b), ",\"src\":\"%02x\"", f.src);
		o += b;
	}
	if(f.channel >= 0) {
		snprintf(b, sizeof(b), ",\"channel\":\"%02x\"", f.channel);
		o += b;
		if(!f.channel_name.empty()) {
			o += ",\"channel_name\":\"" + f.channel_name + "\"";   // validated: no quotes possible
		}
	}
	if(f.has_text) {
		snprintf(b, sizeof(b), ",\"msg_time\":%u", f.msg_time);
		o += b;
		if(!f.sender.empty()) {
			o += ",\"sender\":" + jstr(f.sender);
		}
		o += ",\"text\":" + jstr(f.text);
	}
	if(f.has_group_data) {
		snprintf(b, sizeof(b), ",\"data_type\":%d,\"data_len\":%d", f.data_type, f.data_len);
		o += b;
	}
	if(f.has_ack) {
		snprintf(b, sizeof(b), ",\"ack\":%u", f.ack);
		o += b;
	}
	if(f.control_sub >= 0) {
		snprintf(b, sizeof(b), ",\"control_sub\":%d", f.control_sub);
		o += b;
	}
	return o + "}";
}

void meshcore_fill(packet_entry &e, const std::vector<uint8_t> &payload, bool crc_ok) {
	if(!crc_ok) {
		return;
	}
	meshcore_fields f = meshcore_decode(payload);
	e.info = meshcore_info(f);
	if(f.valid) {
		e.meshcore_json = meshcore_json(f);
	}
}

} // namespace lorad

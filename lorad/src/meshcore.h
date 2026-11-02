// lorad: MeshCore frame interpretation (RX only). Wire format per docs/packet_format.md
// and docs/payloads.md of github.com/meshcore-dev/MeshCore, implemented independently
// from those documents.
//
// Sniffer semantics, like the rest of lorad: fields are informational. Advert Ed25519
// signatures are NOT verified, so name/position are claims, not proof. Only the default
// "Public" group channel is decrypted, because its key ships in the MeshCore firmware
// source. Everything else (private messages, requests, other channels) stays ciphertext
// and only its plaintext routing header is reported.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lorad {

struct packet_entry;

// Radio chain for --mode meshcore, shared with the selftest so it proves what the daemon
// runs. A 10 ppm crystal is 8.7 kHz at 869 MHz, 14 % of the channel, and at this bandwidth
// its clock error slips a long frame by a whole sample, so the channel is kept at
// 4 x BW for sub-sample timing (see receiver.h). The cutoff of 0.65 x BW keeps the chirp
// when crystal error pushes it past the channel edge.
static const double MESHCORE_BW = 62500.0;
static const int MESHCORE_OS = 4;
static const double MESHCORE_DDC_CUTOFF = 0.65 / MESHCORE_OS; // fraction of the DDC output rate
static const int MESHCORE_DDC_TAPS = 32;                      // per decimation step
static const double MESHCORE_LO_OFFSET = 250e3;               // keeps zero-IF DC out of the channel

struct meshcore_fields {
	bool valid = false;        // routing header parsed and path/payload sizes are legal
	int route = 0;             // 0 transport-flood, 1 flood, 2 direct, 3 transport-direct
	int type = 0;              // PAYLOAD_TYPE_*
	int version = 0;           // payload version field; only 0 (v1) is interpreted
	bool has_transport = false;
	uint16_t transport[2] = { 0, 0 };
	int hash_size = 1;         // bytes per path hash, 1..3
	int hops = 0;              // path hash count
	std::vector<std::string> path;   // hex, one per hop
	std::string hash;          // 8-byte packet hash: the same flood relayed twice matches
	int payload_len = 0;
	bool truncated = false;    // payload too short for its declared type

	// ADVERT and other frames that carry a plaintext identity
	bool has_identity = false;
	std::string pubkey;        // first 6 bytes of the Ed25519 public key, hex
	bool has_advert = false;
	uint32_t adv_time = 0;
	int node_type = 0;         // ADV_TYPE_*: 1 chat, 2 repeater, 3 room, 4 sensor
	std::string name;
	bool has_pos = false;
	double lat = 0, lon = 0;   // degrees

	// REQ / RESPONSE / TXT_MSG / PATH / ANON_REQ: first byte(s) of the node public keys
	int dst = -1, src = -1;

	// GRP_TXT / GRP_DATA
	int channel = -1;          // first byte of SHA-256 of the channel key
	bool public_channel = false;   // MAC verified against the default Public key
	std::string channel_name;      // set when a known key verified the MAC, "DM" for a private message
	bool has_text = false;     // GRP_TXT with clean UTF-8 text
	uint32_t msg_time = 0;
	std::string sender;        // self-declared name prefix: unverified by protocol design
	std::string text;
	bool has_group_data = false;
	int data_type = 0, data_len = 0;

	bool encrypted = false;    // carries ciphertext lorad has no key for
	uint32_t ack = 0;          // ACK checksum
	bool has_ack = false;
	int control_sub = -1;      // CONTROL sub_type
};

meshcore_fields meshcore_decode(const std::vector<uint8_t> &p);
std::string meshcore_info(const meshcore_fields &f);
std::string meshcore_json(const meshcore_fields &f);   // the "meshcore":{...} object body
const char *meshcore_type_name(int type);

// fill the display fields of a /packets entry. A frame failing the LoRa CRC gets no
// structured decode: a corrupt advert must not create a phantom node.
void meshcore_fill(packet_entry &e, const std::vector<uint8_t> &payload, bool crc_ok);

// User channels: a "#name" hashtag channel or a private channel with its own 16-byte key. Decoded
// alongside Public, a frame only counts when its MAC verifies. Keys stay in the state file (0600)
// and never leave the daemon through the API.
struct meshcore_fields;
void mc_contact_seen(const std::vector<uint8_t> &frame, int node_type, const std::string &name);
void mc_ack_seen(uint32_t ack);
void mc_path_receive(const uint8_t *b, int len);
void mc_dm_receive(meshcore_fields &f, const uint8_t *b, int len, uint8_t path_len, const uint8_t *path, size_t path_bytes);
bool mc_channel_name_ok(const std::string &name);
bool mc_channel_add(const std::string &name, const uint8_t key[16]);
bool mc_channel_remove(const std::string &name);
bool mc_channel_lookup(const std::string &name, uint8_t key[16]);
std::vector<std::string> mc_channel_names();
bool mc_channels_load(const std::string &path);
bool mc_channels_save(const std::string &path);

} // namespace lorad

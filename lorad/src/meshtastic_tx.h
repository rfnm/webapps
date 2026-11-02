// lorad: Meshtastic transmit side, radio-free: LongFast PHY, private channel, text and NODEINFO
// frames. There is deliberately no way to build a frame for the public default channel here:
// sending needs a channel with its own key, loaded from a file.
//
// Wire format per the Meshtastic firmware (GPL-3.0, read for the format, no code taken):
// src/mesh/RadioInterface.h PacketHeader, src/mesh/CryptoEngine.cpp nonce, Channels.cpp xorHash.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "lora_phy.h"

namespace lorad {

struct mt_channel {
	bool ok = false;
	std::string name;
	uint8_t psk[16] = { 0 };
	uint8_t hash = 0;
};

// the on-air channel id: XOR over the name bytes and the key bytes
uint8_t mt_channel_hash(const std::string &name, const uint8_t psk[16]);
// One line "name 32-hex-digit-key". Refuses the public default key: see the header comment.
bool mt_channel_load(const std::string &path, mt_channel &ch, std::string &err);

phy_params mt_phy();                     // SF11 CR4/5, sync 0x2B, 16 symbol preamble
double mt_airtime_s(size_t frame_len);   // at 250 kHz

// from: this node's id. hop_limit 0 means nobody relays the frame.
bool mt_build_text(uint32_t from, uint32_t id, int hop_limit, const mt_channel &ch, const std::string &text,
                   std::vector<uint8_t> &frame);
bool mt_build_nodeinfo(uint32_t from, uint32_t id, int hop_limit, const mt_channel &ch, const std::string &long_name,
                       const std::string &short_name, std::vector<uint8_t> &frame, const uint8_t *public_key = nullptr,
                       bool want_response = false);

// ---- direct messages ----------------------------------------------------------------------
// A DM is sealed for the recipient alone: X25519, SHA-256, AES-256-CCM with an 8 byte tag and 4
// random nonce bytes, under channel hash 0. With no key for the peer it falls back to the channel
// key. The ACK is a ROUTING_APP packet on the channel: the firmware never seals those.
struct mt_keys {
	bool ok = false;
	uint8_t priv[32];
	uint8_t pub[32];
};
bool mt_keys_load(const std::string &path, mt_keys &k, std::string &err);   // created 0600 if missing
void mt_keys_from_private(const uint8_t priv[32], mt_keys &k);
void mt_shared_key(const mt_keys &k, const uint8_t peer[32], uint8_t out[32]);
std::vector<uint8_t> mt_pkc_seal(const uint8_t shared[32], uint32_t from, uint32_t id, const std::vector<uint8_t> &plain, uint32_t extra);
bool mt_pkc_open(const uint8_t shared[32], uint32_t from, uint32_t id, const uint8_t *ct, size_t len, std::vector<uint8_t> &plain);

void mt_nodes_open(const std::string &path);
void mt_nodes_clear();
bool mt_node_add(uint32_t id, const uint8_t pub[32], const std::string &name);
bool mt_node_find(const std::string &prefix_hex, uint32_t &id, uint8_t pub[32], bool &has_key);
std::vector<std::pair<std::string, std::string>> mt_nodes();    // name, node id hex

bool mt_build_dm(const mt_keys &k, uint32_t from, uint32_t id, uint32_t to, const uint8_t *peer_pub, int hop_limit,
                 const mt_channel &ch, const std::string &text, std::vector<uint8_t> &frame);
bool mt_build_ack(uint32_t from, uint32_t id, uint32_t to, int hop_limit, const mt_channel &ch, uint32_t request_id, int error,
                  std::vector<uint8_t> &frame);

struct mt_event {
	bool valid = false, for_us = false, want_ack = false, want_response = false;
	bool sealed = false;           // a DM sealed for this node
	bool unreadable = false;       // sealed for this node, but the sender's key is unknown
	uint32_t from = 0, to = 0, id = 0, request_id = 0;
	int hop_start = 0, port = 0;
	int routing_error = -1;        // a ROUTING_APP answer: 0 is the ACK
	std::string text;
};
// reads the frame, learns keys from NODEINFO
mt_event mt_receive(const std::vector<uint8_t> &frame, uint32_t self, const mt_keys &k, const mt_channel &ch);

} // namespace lorad

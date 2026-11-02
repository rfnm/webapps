// lorad: MeshCore transmit side, radio-free. Node identity, signed adverts, group text frames,
// airtime and duty-cycle accounting, and the IQ render. Nothing here can emit RF: the radio
// path lives in capture_rfnm.cpp behind the gates of mc_tx_gate().
//
// Wire format per docs/packet_format.md and docs/payloads.md, recipes per src/Mesh.cpp,
// src/Utils.cpp and src/helpers/BaseChatMesh.cpp of github.com/meshcore-dev/MeshCore (MIT).
#pragma once

#include <functional>
#include <utility>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "lora_phy.h"

// Compile-time ceiling that no flag or API call can raise (20 dBm: below the 27 dBm e.r.p. limit of 869.4-869.65 MHz; the setting is uncalibrated here).
#ifndef LORAD_TX_HARD_CAP_DBM
#define LORAD_TX_HARD_CAP_DBM 20
#endif

namespace lorad {

struct mc_identity {
	uint8_t secret[64];
	uint8_t pub[32];
	bool ok = false;
};

// Load the node key from path, or create it: 32 random bytes, mode 0600, written atomically.
bool mc_identity_load(const std::string &path, mc_identity &id, std::string &err);
void mc_identity_from_seed(const uint8_t seed[32], mc_identity &id);

// Region scope of a flood, such as "#de-nord". Off means a plain flood, which
// repeaters take as the wildcard region unless they are configured to deny it.
struct mc_scope {
	bool on = false;
	uint8_t key[16] = { 0 };
};
mc_scope mc_scope_named(const std::string &name);
uint16_t mc_transport_code(const uint8_t key[16], int type, const uint8_t *payload, size_t len);

// "public" is the firmware's default channel, "#name" a public hashtag channel, anything else a
// channel added with mc_channel_add.
bool mc_channel_key(const std::string &name, uint8_t key[16]);

std::vector<uint8_t> mc_build_advert(const mc_identity &id, uint32_t ts, const std::string &name, bool has_pos,
                                     double lat, double lon, const mc_scope &scope);
// false when the text does not fit one frame
bool mc_build_group_text(const uint8_t key[16], uint32_t ts, const std::string &sender, const std::string &text,
                         const mc_scope &scope, std::vector<uint8_t> &frame);

// ---- private messages ---------------------------------------------------------------------
// Contacts come from ADVERT frames with a valid signature. A TXT_MSG for this node is matched
// against them, decrypted with the X25519 secret of the two identities, and reported through the
// callback, which owes the sender an ACK.
struct mc_dm_event {
	uint8_t peer[32];
	uint8_t secret[32];
	uint8_t ack[6];
	bool flood = false;
	bool scoped = false;           // arrived with transport codes
	bool has_route = false;        // route back known from an earlier flood
	uint8_t route_len = 0;         // raw path length byte of that route
	std::vector<uint8_t> route;    // hashes towards the sender, nearest first
	uint8_t path_len = 0;          // raw path length byte of the received frame
	std::vector<uint8_t> path;
};
void mc_dm_set_identity(const mc_identity &id);
void mc_dm_set_callback(std::function<void(const mc_dm_event &)> cb);
void mc_contacts_open(const std::string &path);
bool mc_contact_add(const uint8_t pub[32], const std::string &name);
bool mc_contact_find(const std::string &prefix_hex, uint8_t pub[32]);
std::vector<std::pair<std::string, std::string>> mc_contacts();   // name, key prefix hex
void mc_contacts_clear();
// attempt (0-3) goes into the plaintext as in the firmware, so each try has its own ACK hash and
// expected_ack, when given, receives the one this try must see.
bool mc_build_dm(const mc_identity &id, const uint8_t peer[32], uint32_t ts, const std::string &text,
                 const mc_scope &scope, std::vector<uint8_t> &frame, int attempt = 0, uint32_t *expected_ack = nullptr,
                 bool *routed = nullptr);   // routed given: go along the learned route if there is one
// every ACK heard, bare or inside a PATH return addressed to this node
void mc_ack_set_callback(std::function<void(uint32_t)> cb);
// the reply to a flood DM: encrypted return path carrying the ACK, as Mesh::createPathReturn
std::vector<uint8_t> mc_build_path_ack(const mc_identity &id, const mc_dm_event &e, const mc_scope &scope);
// the reply to a routed DM: along the known route, as sendAckTo
std::vector<uint8_t> mc_build_direct_ack(const mc_dm_event &e);
// the same with no route known: a plain flood ACK
std::vector<uint8_t> mc_build_flood_ack(const mc_dm_event &e, const mc_scope &scope);

// Ed25519 check of an ADVERT frame.
bool mc_verify_advert(const std::vector<uint8_t> &frame);

phy_params mc_phy();                                // SF8 CR4/8, sync 0x12, 32 symbol preamble
double mc_airtime_s(size_t frame_len);
std::vector<int16_t> mc_render_iq(const std::vector<uint8_t> &frame, double rate_hz, double offset_hz);
// the same chirp synthesis for any LoRa parameters, at bandwidth bw_hz
std::vector<int16_t> lora_render_iq(const phy_params &p, double bw_hz, const std::vector<uint8_t> &frame, double rate_hz, double offset_hz);

// Sliding-window airtime budget. 869.40-869.65 MHz allows < 10 % duty (CEPT ERC/DEC/(01)04).
class mc_duty {
public:
	explicit mc_duty(double window_s = 3600.0, double fraction = 0.10) : window_(window_s), cap_(window_s * fraction) {}
	bool allow(double now_s, double airtime_s);
	void commit(double now_s, double airtime_s) { log_.push_back({ now_s, airtime_s }); }
	double used(double now_s);
	double cap() const { return cap_; }

private:
	struct entry {
		double t, a;
	};
	void trim(double now_s);
	double window_, cap_;
	std::deque<entry> log_;
};

struct mc_gate_in {
	bool board_all = false;     // /rfnm/board.conf TX_ALLOWED=1; "embed" does not count: Lime TX is SMA-A only
	bool tx_flag = false;       // --tx 1
	bool armed = false;         // POST /tx/arm, unexpired
	int power_dbm = 0;
	bool tx_mode = false;       // meshcore, or meshtastic with a private channel
};
// nullptr = all gates open, else the reason for the refusal
const char *mc_tx_gate(const mc_gate_in &g);

} // namespace lorad

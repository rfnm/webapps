// vrtd: daemon core - VRT exporter (RX -> UDP), ingest (UDP -> TX) and the 49.2
// control dispatcher. Split from main.cpp so the selftest can run the whole engine
// in-process over loopback sockets with the mock radio.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>

#include "radio.h"
#include "vrt.h"

namespace vrtd {

struct config {
	std::string dest_host = "";        // VRT data+context destination; empty = no export
	uint16_t dest_port = 4991;
	uint16_t listen_port = 4991;       // control + TX-data ingest (0 = disabled)
	uint32_t sid_rx = 1;               // stream id of the RX export stream
	uint32_t sid_tx = 2;               // stream id TX-data packets must carry
	uint32_t controllee_id = 0;        // 0 = accept commands for any controllee id
	uint32_t oui = 0;                  // 0 = omit class id from emitted packets
	uint16_t info_class = 0;
	uint16_t packet_class = 0;
	size_t samples_per_pkt = 360;      // 1440 payload bytes: fits standard MTU
	double context_interval_s = 1.0;   // periodic context re-emit
	// TX SAFETY (fail-safe default): the engine drops TX-data and error-acks TX commands
	// unless the board policy allows TX. The embed-vs-all path choice is the radio layer's
	// job (rfnm_radio forces the embedded antenna under an embed policy); this flag is the
	// engine-level gate. main.cpp fills it from /rfnm/board.conf TX_ALLOWED.
	bool tx_allowed = false;
};

struct stats {
	std::atomic<uint64_t> data_pkts_out{ 0 };
	std::atomic<uint64_t> ctx_pkts_out{ 0 };
	std::atomic<uint64_t> samples_out{ 0 };
	std::atomic<uint64_t> data_pkts_in{ 0 };
	std::atomic<uint64_t> samples_in{ 0 };
	std::atomic<uint64_t> seq_gaps_in{ 0 };
	std::atomic<uint64_t> tx_drops{ 0 };
	std::atomic<uint64_t> cmds_in{ 0 };
	std::atomic<uint64_t> cmds_rejected{ 0 };
	std::atomic<uint64_t> acks_out{ 0 };
	std::atomic<uint64_t> malformed_in{ 0 };
	std::atomic<uint64_t> tx_forbidden{ 0 };   // TX data/commands refused by board policy
};

class core {
public:
	core(const config &cfg, radio *r);
	~core();

	bool start();                      // opens sockets, starts RX export + ingest loop
	void stop();

	// exposed for the selftest and the status endpoint
	stats st;
	radio *rad() { return rad_; }
	int listen_fd() const { return in_fd_; }

private:
	void on_rx(const int16_t *iq, size_t n);
	void emit_context(bool changed);
	void ingest_loop();
	void handle_command(const vrt::command_packet &c, const sockaddr_in &from);
	void send_ack(vrt::command_packet ack, const sockaddr_in &to);

	config cfg_;
	radio *rad_;
	int out_fd_ = -1;
	int in_fd_ = -1;
	sockaddr_in dest_{};
	bool has_dest_ = false;

	std::atomic<bool> run_{ false };
	std::thread ingest_thr_;

	// exporter state (touched only on the radio RX thread)
	std::vector<int16_t> stage_;
	std::vector<uint8_t> pktbuf_;
	uint8_t data_count_ = 0;
	uint8_t ctx_count_ = 0;
	uint8_t ack_count_ = 0;
	uint64_t sample_clock_ = 0;        // TSF free-running sample count
	double next_ctx_ = 0;

	// ingest state
	int last_in_count_ = -1;

	std::mutex ctl_mtx_;               // serializes control ops vs context emission
};

} // namespace vrtd

// lorad: per-channel LoRa receiver. Consumes complex samples at fs == BW and emits
// decoded packets. One receiver instance scans all configured SFs simultaneously;
// when a preamble locks, the winning SF runs the frame to completion.
#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <vector>
#include "lora_phy.h"

namespace lorad {

struct rx_packet {
	double freq_hz = 0;
	double bw_hz = 0;
	int sf = 0;
	int cr = 0;
	float snr_db = 0;
	bool crc_ok = false;
	bool crc_present = false;
	bool fec_clean = true;
	std::vector<uint8_t> payload;
	uint64_t sample_pos = 0; // absolute sample index of the frame start (fs == BW)
};

class channel_receiver {
public:
	// sync: expected sync word; sf_mask: bit s set == scan SF s (bits 7..12)
	channel_receiver(double freq_hz, uint8_t sync, unsigned sf_mask, bool bw250,
	                 std::function<void(const rx_packet &)> on_packet);
	~channel_receiver(); // out-of-line: sf_state is only complete in receiver.cpp

	void feed(const cf *samples, size_t count);

private:
	struct sf_state;

	void scan(sf_state &st);
	bool try_frame(sf_state &st); // false: not enough buffered data yet, retry on next feed
	int demod(const sf_state &st, uint64_t pos, float cfo_bins, float *peak_db, float *noise_db) const;

	double freq_hz_;
	uint8_t sync_;
	bool bw250_;
	std::function<void(const rx_packet &)> on_packet_;

	std::vector<cf> ring_;
	uint64_t ring_base_ = 0;   // absolute index of ring_[0]
	uint64_t total_in_ = 0;

	std::vector<std::unique_ptr<sf_state>> sfs_;
	uint64_t skip_until_ = 0;  // after a decode, resume scanning here

	const cf *at(uint64_t abs) const { return &ring_[(size_t)(abs - ring_base_)]; }
	uint64_t avail() const { return ring_base_ + ring_.size(); }
	void trim();
};

} // namespace lorad

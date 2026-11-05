// lorad: per-channel LoRa receiver. Consumes complex samples at fs == os * BW and emits
// decoded packets. One receiver instance scans all configured SFs simultaneously;
// when a preamble locks, the winning SF runs the frame to completion.
//
// os == 1 is the critically sampled path. Sub-sample timing error puts a phase step at
// every chirp's wrap there, which no integer window position can remove; that costs little
// at BW 125 and up, where a frame is short against crystal error, but breaks long frames
// at BW 62.5. With os > 1 detection still runs on every os-th sample, while the data
// symbols are read at 1/os sample resolution, with the frame's timing searched and the
// crystal's clock slip followed symbol by symbol.
#pragma once

#include <atomic>

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
	// counts preamble candidates; a moving value means a frame is on air (listen-before-talk)
	uint64_t candidates() const { return cand_count_.load(); }
	// sync: expected sync word; sf_mask: bit s set == scan SF s (bits 7..12)
	// bw_hz: LoRa bandwidth; the input sample rate is os * bw_hz
	channel_receiver(double freq_hz, uint8_t sync, unsigned sf_mask, double bw_hz,
	                 std::function<void(const rx_packet &)> on_packet, int os = 1);
	~channel_receiver(); // out-of-line: sf_state is only complete in receiver.cpp

	void feed(const cf *samples, size_t count);

private:
	std::atomic<uint64_t> cand_count_{0};
	struct sf_state;

	void scan(sf_state &st);
	bool try_frame(sf_state &st); // false: not enough buffered data yet, retry on next feed
	int demod(const sf_state &st, uint64_t pos, float cfo_bins, float *peak_db, float *noise_db) const;
	int demod_window(const sf_state &st, const cf *x, int stride, float cfo_bins, float *peak_db, float *noise_db) const;
	int demod_fine(const sf_state &st, double pos, float tone_bins, float *peak_db, float *noise_db) const;
	bool fine_offset(const sf_state &st, uint64_t sfd_start, float &bins) const;
	bool search_timing(const sf_state &st, uint64_t sfd_start, uint64_t data_start, float tone, double slip, std::vector<float> &timings) const;

	double freq_hz_;
	uint8_t sync_;
	double bw_hz_;
	int os_;
	std::function<void(const rx_packet &)> on_packet_;

	std::vector<cf> ring_;     // every os-th input sample: the fs == BW view
	std::vector<cf> ring_hi_;  // os > 1: all input samples, ring_hi_[0] aligns with ring_[0]
	uint64_t hi_total_ = 0;    // input samples seen
	uint64_t ring_base_ = 0;   // absolute index of ring_[0]
	uint64_t total_in_ = 0;

	std::vector<std::unique_ptr<sf_state>> sfs_;
	uint64_t skip_until_ = 0;  // after a decode, resume scanning here

	const cf *at(uint64_t abs) const { return &ring_[(size_t)(abs - ring_base_)]; }
	uint64_t avail() const { return ring_base_ + ring_.size(); }
	void trim();
};

} // namespace lorad

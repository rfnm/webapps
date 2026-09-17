// elrsd: wide-band sample sources (int16 interleaved IQ at fs, centred on fc).
// The replay source is host-buildable; the RFNM source lives in
// src/capture_rfnm.cpp and is compiled only WITH_RFNM=1 (never include librfnm here).
//
// Contract: the source pushes raw cs16 blocks to a callback on ITS OWN thread and reports
// the absolute sample index of block[0] on the source clock (fs). The daemon derives every
// timestamp from (sample_index / fs); device ts_ns is carried only as an annotation.
// IQ sign convention: the host TCP capture path (rfnm_tcp_capture) negates Q, the board
// TRANSPORT_LOCAL path does not (2026-08-07) -> `conj` is a runtime option, the replay
// selftest runs both signs and reports which one decodes.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace elrsd {

struct source_block {
	const int16_t *iq;        // interleaved I,Q int16, `count` complex samples
	size_t count;
	uint64_t sample_index;    // absolute index of iq[0] on the source clock
	int64_t ts_ns;            // device timestamp of iq[0] if known, else -1
};

struct source_stats {
	uint64_t samples = 0;     // delivered complex samples
	uint64_t drops = 0;       // discontinuities / dropped blocks (device counter or gaps)
	uint64_t reads = 0;
	uint64_t empty_reads = 0; // consecutive-empty watchdog input (live only)
	bool eof = false;         // replay: file exhausted
};

class sample_source {
public:
	virtual ~sample_source() {}
	virtual double fs_hz() const = 0;
	virtual double fc_hz() const = 0;
	// blocking; returns when stop is set or the source ends. false = setup failure (logged).
	virtual bool run(std::atomic<bool> &stop, const std::function<void(const source_block &)> &cb) = 0;
	virtual void stats(source_stats &out) const = 0;
	virtual std::string describe() const = 0;
};

// --- replay: cs16 file (int16 LE interleaved), e.g. the 2026-09-14 freerun captures ---
struct replay_config {
	std::string path;
	double fs_hz = 25e6;
	double fc_hz = 2415e6;
	size_t block_samples = 65536;    // like the live read size
	bool realtime = false;           // pace to wall clock (default: as fast as possible)
	double start_s = 0, stop_s = 0;  // window inside the file (0 = whole file)
	bool loop = false;
};
sample_source *make_replay_source(const replay_config &cfg);

// --- RFNM live RX (src/capture_rfnm.cpp, WITH_RFNM only; every factory below returns nullptr
//     and logs "built without librfnm" in host builds) ---
//
// Device ownership (/): there is exactly ONE rfnm::device per process. The
// daemon (C) opens it once with rfnm_device_open() - TRANSPORT_LOCAL, CS16, set_samp_rate(fs)
// at rest - and hands the opaque handle to BOTH the RX source (B) and the TX adapter (D,
// tx_chain.h make_rfnm_tx). A second open would fight over the radio; a sample-rate change
// is a re-open (rate changes only at rest). The handle is an rfnm::device* behind void* so
// that this header stays librfnm-free.
void *rfnm_device_open(double fs_hz, std::string &error);   // nullptr + error on failure
void rfnm_device_close(void *dev);                          // parks TX/RX RF_OFF, deletes
struct rfnm_rx_config {
	double fs_hz = 25e6;             // the only verified rate (4 ms = 100 000 samples); must equal the opened rate
	double fc_hz = 2441.4e6;         // window centre (default: the SYNC channel 41)
	int gain_db = -24;               // clamped to rx->gain_range
	bool agc = false;
	std::string antenna = "sma_a";  // resolved via string_to_rf_path / path_possible, fallback preferred
	int empty_read_limit = 300;      // ~30 s of empty reads -> run() returns false (systemd restarts)
	bool park_tx = true;             // park TX ch0 RF_OFF/STREAM_OFF with an in-range freq before RX apply (duplex trap)
	// additions (--selfloop-device): the RX channel index (0 = the card's rx0; a
	// second Lime card's rx0 is channel 1 on that board) and a strict path: when set the
	// requested antenna is used literally (never silently replaced by path_preferred; a name
	// string_to_rf_path does not understand fails run()) - a path that path_possible[] does not
	// advertise is only warned about, the device apply decides ("loopback" on the Lime card is
	// accepted by the driver's sysfs parser but not advertised).
	int rx_channel = 0;
	bool strict_path = false;
};
// run() applies ONLY the RX channel (rx_channel_apply_flags[0]) on the shared device, streams
// until `stop`, then stops the stream and applies RX RF_OFF (so that the TX adapter can take
// the radio: v1 TX mode = RX source stopped). run() may be called again after it
// returned (fc/gain/antenna change = stop + run with a new config on the same device).
sample_source *make_rfnm_rx_source(const rfnm_rx_config &cfg, void *dev);

// --- synthetic source for selftests: caller-provided int16 buffer, chunked like a device ---
sample_source *make_buffer_source(const int16_t *iq, size_t count, double fs_hz, double fc_hz, size_t block_samples = 65536);

} // namespace elrsd

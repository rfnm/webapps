// elrsd: RX signal chain - wide-band burst detector, per-burst channel estimate, narrow-band
// extraction (resampler to fs = OS * 812.5 kHz), CSS (LoRa) demodulator SF5..8, frame sync
// -> raw payload symbols + ref + timestamp + channel. NO ELRS bit decoding here (that is
// elrs_codec, done by the daemon on the rx_burst it receives).
// Consumed by C (daemon) and D (loopback selftest of the TX synthesis).
//
// Why burst-gated: 25 MSps cs16 is 100 MB/s. A full lorad-style channelizer (80 or even
// 24 DDCs at 25 MSps) is out of reach for 4x A53; ELRS bursts occupy
// < 85 % of a slot on ONE channel, so the chain (1) runs a cheap int16 envelope over the
// wide stream, (2) cuts a burst segment out of a raw ring, (3) estimates the channel from
// the burst middle (FFT centroid on the 1 MHz grid), (4) mixes + resamples ONLY that
// segment, (5) demodulates. With an FHSS lock the daemon can hand a channel prediction in
// (prediction beats estimation on weak bursts); without one the estimate stands alone.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include "dsp.h"
#include "elrs_codec.h"

namespace elrsd {

// ---- CSS modem (dechirp + FFT + OS-fold), fs = os * bw ---------------------------------
class css_modem {
public:
	css_modem(int sf, int os = 1, double bw_hz = ELRS_BW_HZ);
	int sf() const { return sf_; }
	int n() const { return n_; }                   // 2^sf bins
	int os() const { return os_; }
	int samples_per_symbol() const { return n_ * os_; }
	// demodulate ONE symbol at x (samples_per_symbol samples): dechirp with conj(upchirp),
	// FFT (n*os), fold the os aliases (sum |X[k + j*n]|), argmax over n bins.
	// peak/noise (optional): linear peak power and mean floor for SNR/quality.
	int demod_symbol(const cf *x, float *peak = nullptr, float *noise = nullptr) const;
	// same with a downchirp reference (SFD detection)
	int demod_symbol_down(const cf *x, float *peak = nullptr, float *noise = nullptr) const;
	// argmax plus the fractional bin offset (-0.5..0.5) of the peak by parabolic interpolation
	// on the folded magnitudes (added by B: frame_sync uses it to rotate the burst so that the
	// preamble tone sits on an integer bin before ref/payload rounding - removes the half-bin
	// ambiguity that a fractional CFO or a sub-sample timing offset would otherwise leave)
	int demod_symbol_frac(const cf *x, float *frac, float *peak = nullptr, float *noise = nullptr) const;
	// main-lobe energy (peak bin +-1 of the folded magnitudes, squared) with the up- or the
	// down-chirp reference - a scallop-free power measure (added by B: frame_sync ranks the
	// candidate timings by the SFD edge with it)
	float peak_power3(const cf *x, bool down) const;
	// demodulate count consecutive symbols starting at x
	void demod(const cf *x, size_t count, std::vector<uint16_t> &out, std::vector<float> *snr_db = nullptr) const;
	// reference chirps at fs = os * bw (used by B's own selftest; NOT by tx_chain, whose
	// synthesis runs at the device rate and must not link against this module)
	const std::vector<cf> &upchirp() const { return up_; }
	const std::vector<cf> &downchirp() const { return down_; }
	// apply a CFO rotation (Hz) in place
	static void rotate_cfo(cf *x, size_t count, double cfo_hz, double fs_hz, double phase0 = 0);
private:
	int sf_, n_, os_;
	double bw_;
	std::vector<cf> up_, down_, upconj_;
	fft_plan fft_;
};

// ---- frame sync on a narrow-band burst -----------------------------------------------------
struct frame_sync_result {
	bool found = false;
	int sf = 0;
	int preamble_run = 0;        // consecutive equal preamble symbols found (quality)
	int ref = 0;                 // majority preamble symbol under the final timing
	size_t preamble_start = 0;   // sample index (narrow-band) of the first preamble symbol
	size_t payload_start = 0;    // preamble_start + round(frame_offset_symbols * n * os) (+ fine shift)
	double cfo_hz = 0;           // estimated (0 if not estimated)
	float snr_db = 0;            // preamble SNR estimate
	bool sfd_seen = false;       // downchirp SFD confirmed
	int sfd_bin_up = 0, sfd_bin_down = 0;
};

// Locate preamble/SFD in a narrow-band burst and demodulate the payload symbols.
// The algorithm mirrors the Python reference (1123/1181 on the SF6 capture): envelope
// onset -> scan sub-symbol shifts (n*os/32 step) for the longest run of equal up-chirp
// argmaxes (>= preamble_len - 4) -> ref = majority of the run excluding the last 2
// (sync-word) symbols -> payload at +frame_offset_symbols. Optional lorad-style
// refinement (SFD up/down pair -> CFO + STO) when refine = true.
class frame_sync {
public:
	frame_sync(const rate_params &rp, int os = 4, bool refine = false);
	// x: narrow-band burst (fs = os * bw) starting ~0.3 ms before the expected preamble.
	// symbols receives n_sym payload symbols demodulated at the NOMINAL frame offset (empty
	// if !found). The tracker cannot re-demodulate (it never sees x), so alternative offsets
	// are internal only (Python: the nominal offset wins on every capture).
	frame_sync_result run(const cf *x, size_t count, std::vector<uint16_t> &symbols) const;
	// candidate payload offsets: the nominal one first, then +-0.25 sym - for
	// B's selftest/diagnostics, not part of the rx_burst contract
	std::vector<double> candidate_offsets() const;
	const css_modem &modem() const { return modem_; }
private:
	rate_params rp_;
	css_modem modem_;
	bool refine_;
};

// ---- the burst-gated wide-band front end -------------------------------------------------------
struct rx_burst {
	uint64_t sample_start;     // wide-stream sample index of the burst onset (envelope)
	double t_start_s;          // sample_start / fs
	int64_t ts_ns;             // device stamp interpolated to sample_start, -1 if unknown
	double duration_s;         // envelope duration
	double freq_offset_hz;     // burst centre vs fc (centroid)
	int channel;               // nearest ELRS channel (0..79) or -1
	int channel_predicted;     // channel handed in by the tracker (-1 = none)
	int rate_index;            // rate used for demod (from duration or forced)
	frame_sync_result sync;    // narrow-band result (sync.found -> symbols valid)
	std::vector<uint16_t> symbols;   // n_sym raw payload symbols
	float rssi_db;             // burst power vs the quiet floor (relative, not calibrated)
	float snr_db;
	bool conj;                 // sign convention used
};

struct rx_chain_config {
	double fs_hz = 25e6;
	double fc_hz = 2415e6;
	int os = 4;                        // 4 = Python-verified path (13/100), 1 = lorad-style (13/400)
	int rate_index = 0;                // 0 = auto (from burst duration), else force 4..9
	bool conj = false;                 // negate Q of the wide stream
	bool auto_conj = false;            // try the other sign when the preamble is not found (selftest/replay aid)
	double env_block_s = 125e-6;       // envelope block (3125 samples @25 MSps)
	// Schmitt-trigger envelope gate (hysteresis). ENTER stays at the conservative 6.0x so trigger
	// sensitivity (and the noise-driven false-burst rate) is unchanged from the proven strong-signal
	// path. SUSTAIN is lower: once a burst has fired, a marginal-SNR burst that dips between the two
	// factors is held together instead of chopping into sub-1 ms fragments that get rejected as
	// too_short. Measured on the two-board link (peak ~3.8x floor): enter 6 / sustain 3 cut
	// bursts_too_short 30->20 and raised demod_attempts 4->7 with NO extra triggers. For a very weak
	// link the enter factor can be lowered via ELRSD_ENV_ENTER (offline replay tuning).
	double env_thresh_factor = 6.0;    // ENTER a burst when env > floor * factor (+ margin)
	double env_sustain_factor = 3.0;   // stay in the burst until env <= floor * factor (+ margin)
	double min_burst_s = 1.0e-3;       // shorter = ignore (SF5 burst = 1.5 ms)
	double max_burst_s = 12.0e-3;      // longer = split / ignore (SF8 burst = 10.8 ms)
	double ring_s = 0.060;             // raw int16 ring depth (25e6 * 0.06 * 4 B = 6 MB)
	double pre_s = 0.3e-3;             // extract this much before the onset
	bool refine = false;               // lorad-style CFO/STO refinement in frame_sync
};

// Thread model: feed() runs on ONE thread (the daemon's DSP thread); set_channel_hint /
// set_rate_index / set_conj / get_stats may be called concurrently from other threads (HTTP)
// AND re-entrantly from inside the on_burst callback (the tracker hands the next hint back
// while still inside on_burst) -> they take a knob mutex that feed() only holds between
// bursts, never while calling on_burst. The chain owns no other threads.
class rx_chain {
public:
	explicit rx_chain(const rx_chain_config &cfg);
	~rx_chain();
	const rx_chain_config &config() const { return cfg_; }

	// feed a wide-band block (any size, from the source thread). Calls on_burst on the SAME
	// thread for each completed burst. Demod work happens here -> the daemon runs feed() on a
	// dedicated DSP thread, not on the capture thread.
	void feed(const int16_t *iq, size_t count, uint64_t sample_index, int64_t ts_ns);
	void set_on_burst(std::function<void(const rx_burst &)> cb);
	// tracker hint: the channel expected for bursts starting in [t0, t1) (source seconds);
	// -1 clears. The chain still estimates and reports both.
	void set_channel_hint(double t0_s, double t1_s, int channel);
	// runtime knobs (daemon PUT /config): rate change reallocates the modem. idx 0 = auto:
	// choose among the rates only (rate_indices_verified(): 4/6/7/9, burst lengths 1.5 /
	// 3.3 / 5.9 / 10.8 ms are > 40 % apart; OTA8 rates 5/8 would overlap and are unsupported)
	void set_rate_index(int idx);
	void set_conj(bool conj);

	struct stats {
		uint64_t samples = 0;
		uint64_t bursts = 0;           // envelope events
		uint64_t bursts_too_short = 0, bursts_too_long = 0;
		uint64_t demod_attempts = 0, demod_found = 0;   // frame_sync found
		uint64_t ring_overruns = 0;    // burst fell out of the ring before extraction
		uint64_t bursts_no_rate = 0;   // duration matched no rate in auto mode (added by B)
		double floor_level = 0;        // current envelope floor (int16 units)
		double cpu_busy = 0;           // fraction of wall time spent in feed() (1 s IIR)
		int last_channel = -1;
	};
	void get_stats(stats &out) const;

private:
	struct impl;
	impl *p_;
	rx_chain_config cfg_;
};

// rate from a burst duration (rate table burst_s +- 20 %, rates 4/6/7/9 only); 0 = none
int rate_index_from_duration(double duration_s);

// selftest: fft vs naive DFT, resampler passband/rate, envelope, css_modem symbol
// round trip at os 1 and 4 (all SF5..8), frame_sync on synthesized frames with CFO/STO/noise
// (needs tx_chain's synthesizer -> the RX selftest implements a minimal chirp frame builder of its
// own in rx_selftest.cpp so B never depends on D), rx_chain end-to-end on a synthetic wide
// stream with 3 bursts on 3 channels, both conj signs.
int run_selftest_rx();

// --bench driver (src/rx_selftest.cpp; added by B): synthesizes a
// 25 MSps int16 stream with one SF6 burst per 4 ms slot on random in-window channels plus
// noise, runs it through rx_chain for `seconds` of signal and prints samples, bursts found,
// wall time and realtime_factor (signal seconds / wall seconds). Returns 0 on success.
int run_bench_rx(double seconds);

// --replay summary. The driver itself is run_replay() in daemon.h: it
// wires make_replay_source -> rx_chain -> rx_tracker and the freerun ground truth; B only
// guarantees that rx_chain + replay source deliver the bursts.
struct replay_report {
	uint64_t bursts = 0, found = 0, decoded = 0, crc_ok = 0, truth_match = 0, truth_checked = 0;
	int conj_used = 0;
	double seconds = 0, wall_s = 0;
};

} // namespace elrsd

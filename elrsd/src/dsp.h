// elrsd: DSP primitives - complex type, radix-2 FFT, NCO+FIR decimator, rational resampler.
// fft_plan and ddc are lorad's (rfnm-webapps/lorad/src/fft.h, channelizer.h)
// re-homed into namespace elrsd; keep the NEON/scalar bit-identity property. The scalar path
// is the numeric reference (the x86 selftest only exercises the scalar path).
// Build note: -fcx-limited-range is load-bearing (see Makefile).
#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace elrsd {

typedef std::complex<float> cf;

// iterative radix-2 complex FFT, n = any power of two >= 4 (SF5..8 at OS 1..4 -> 32..1024)
class fft_plan {
public:
	explicit fft_plan(unsigned n);
	unsigned size() const { return n_; }
	void run(const cf *in, cf *out) const;      // out-of-place, out has n elements
private:
	unsigned n_, log2n_;
	std::vector<cf> tw_, stw_;
	std::vector<unsigned> stage_off_, rev_;
};

// NCO + windowed-sinc FIR, INTEGER decimation only (in_rate / out_rate must be integral).
// offset_hz: channel offset from the capture centre (positive = above centre).
class ddc {
public:
	ddc(double in_rate, double out_rate, double offset_hz);
	void process(const cf *in, size_t count, std::vector<cf> &out);   // appends
	void reset();                                                     // clears FIR state and NCO phase
	int decim() const { return decim_; }
private:
	int decim_, nt_, w_, cnt_ = 0;
	cf phase_, rot_, rot2_, rot3_, rot4_;
	std::vector<float> taps_;
	std::vector<cf> buf_;
};

// Rational polyphase resampler out = in * up / down with an integrated NCO mixer
// (mix at the INPUT rate before filtering). Needed because 25 MSps / 812.5 kHz is not an
// integer: 25e6 -> 3.25e6 is (13, 100) [the Python-verified path, OS 4], 25e6 -> 812.5e3
// is (13, 400) [OS 1]. Cost is paid at the output rate (one polyphase branch per output).
class resampler {
public:
	// cutoff_hz = 0 -> 0.45 * out_rate (generic anti-alias); rx_chain passes 0.55 * ELRS_BW_HZ so
	// the adjacent 1-MHz channel is rejected while the full 812.5 kHz chirp passes (added by B,
	// trailing defaulted parameter: existing calls are unchanged)
	resampler(unsigned up, unsigned down, double in_rate, double offset_hz, int taps_per_branch = 24, double cutoff_hz = 0);
	void process(const cf *in, size_t count, std::vector<cf> &out);   // appends
	// int16 interleaved input (the capture/replay format); scale = 1/32768; conj = negate Q
	void process_i16(const int16_t *iq, size_t count, std::vector<cf> &out, bool conj = false);
	void reset();
	double out_rate() const { return out_rate_; }
private:
	unsigned up_, down_;
	double in_rate_, out_rate_;
	cf phase_, rot_;
	std::vector<float> taps_;   // up * taps_per_branch, polyphase-ordered
	std::vector<cf> hist_;      // 2 * taps_per_branch double-write ring (newest-first window)
	uint64_t phase_acc_ = 0;    // (m * down) mod up bookkeeping
	// internal state (private only, no interface change): exact NCO re-phasing
	// every 1024 samples from a double accumulator, history write index
	double offset_hz_d_ = 0;
	uint64_t nco_n_ = 0;
	int hw_ = 0;
};

// cheap wide-band burst envelope on raw int16 IQ: sum of |I| + |Q| over blocks of
// block_len samples (NEON-able, no float conversion of the wide stream). out gets one
// uint32 per complete block; a partial block is carried to the next call.
class envelope_i16 {
public:
	explicit envelope_i16(size_t block_len);
	void process(const int16_t *iq, size_t count, std::vector<uint32_t> &out);   // appends
	size_t block_len() const { return blk_; }
	void reset();
	// DC tracking (added by B, default off): the RFNM capture carries a large DC offset
	// (2026-09-14 file: I +13614, Q -3245 on a 2400-unit noise floor), which would swamp the
	// |I| + |Q| envelope. When on, every block also sums I and Q, the DC estimate follows the
	// block means with a 1/32 IIR (first block locks instantly) and the NEXT block's magnitudes
	// are taken after a saturating subtraction of that estimate (scalar and NEON bit-identical).
	void set_dc_tracking(bool on);
	int dc_i() const { return dc_i_; }
	int dc_q() const { return dc_q_; }
private:
	size_t blk_, fill_ = 0;
	uint64_t acc_ = 0;
	bool dc_track_ = false, dc_locked_ = false;
	int dc_i_ = 0, dc_q_ = 0;
	int64_t sum_i_ = 0, sum_q_ = 0;
};

} // namespace elrsd

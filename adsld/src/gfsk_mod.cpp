// adsld: GFSK burst modulator - see gfsk_mod.h.
//
// Chips -> NRZ (+1/-1) at S = fs/chip_rate samples per chip -> Gaussian pulse-shaping
// FIR (BT = 0.5 relative to the chip rate, GFSK_SPAN_CHIPS span, unit DC gain so a run
// of equal chips settles at full deviation) -> instantaneous frequency +-ADSL_DEV_HZ ->
// phase accumulator -> complex exponential, mixed to offset_hz inside the same
// accumulator (mathematically identical to a separate exp(j*2*pi*offset*t) mixer).
//
// The burst is book-ended by GFSK_RAMP_CHIPS of settled carrier at the first/last
// chip's frequency; the raised-cosine amplitude ramps live entirely inside that lead-in
// and tail, so the discriminator sees valid history before the first preamble chip and
// the shaped preamble itself is never amplitude-distorted.
#include "gfsk_mod.h"

#include <cmath>

namespace adsld {

// Gaussian premod filter taps at S samples/chip: impulse response with
// sigma_t = sqrt(ln 2) / (2*pi*B), B = BT * chip rate, truncated to GFSK_SPAN_CHIPS
// and normalized to unit sum.
static std::vector<float> gaussian_taps(int S) {
	const int half = GFSK_SPAN_CHIPS * S / 2;
	const double B = ADSL_BT * ADSL_CHIP_RATE_HZ;
	const double sigma_t = std::sqrt(std::log(2.0)) / (2.0 * M_PI * B);
	const double dt = 1.0 / (ADSL_CHIP_RATE_HZ * S);   // sample period
	std::vector<float> taps(2 * (size_t)half + 1);
	double sum = 0;
	for(size_t i = 0; i < taps.size(); i++) {
		double t = (double)((int)i - half) * dt;
		double v = std::exp(-0.5 * t * t / (sigma_t * sigma_t));
		taps[i] = (float)v;
		sum += v;
	}
	for(float &t : taps) {
		t = (float)(t / sum);
	}
	return taps;
}

std::vector<cf> gfsk_render_burst(const bits &chips, double fs_hz, double offset_hz,
                                  float amplitude) {
	if(chips.empty()) {
		return {};
	}
	const long S_l = std::lround(fs_hz / ADSL_CHIP_RATE_HZ);
	if(S_l < 2 || fs_hz != (double)S_l * ADSL_CHIP_RATE_HZ) {
		return {};   // fs must be an integer multiple of the chip rate
	}
	const int S = (int)S_l;
	const std::vector<float> taps = gaussian_taps(S);
	const int half = ((int)taps.size() - 1) / 2;

	const int lead = GFSK_RAMP_CHIPS;                        // settled-carrier chips per edge
	const size_t nout = (chips.size() + 2 * (size_t)lead) * (size_t)S;
	const int ramp = GFSK_RAMP_CHIPS * S;                    // amplitude ramp, samples

	// NRZ chip value at an extended-stream sample index; clamping replicates the first/
	// last chip across the lead-in/tail (and virtually beyond, for the FIR edges)
	auto nrz_at = [&](long n) -> float {
		long c = n / S - lead;
		if(c < 0) {
			c = 0;
		}
		if(c >= (long)chips.size()) {
			c = (long)chips.size() - 1;
		}
		return chips[(size_t)c] ? 1.0f : -1.0f;
	};

	std::vector<cf> out(nout);
	double phase = 0.0;   // starts at 0 on the first sample
	for(size_t n = 0; n < nout; n++) {
		double env = 1.0;
		if((long)n < ramp) {
			env = 0.5 * (1.0 - std::cos(M_PI * (double)(n + 1) / ramp));
		}
		long from_end = (long)(nout - 1 - n);
		if(from_end < ramp) {
			double e = 0.5 * (1.0 - std::cos(M_PI * (double)(from_end + 1) / ramp));
			if(e < env) {
				env = e;
			}
		}
		out[n] = (float)(amplitude * env) * cf((float)std::cos(phase), (float)std::sin(phase));

		double f = 0;   // shaped NRZ, settles to +-1
		for(int k = 0; k < (int)taps.size(); k++) {
			f += taps[k] * nrz_at((long)n + k - half);
		}
		phase += 2.0 * M_PI * (ADSL_DEV_HZ * f + offset_hz) / fs_hz;
		while(phase > M_PI) {
			phase -= 2.0 * M_PI;
		}
		while(phase < -M_PI) {
			phase += 2.0 * M_PI;
		}
	}
	return out;
}

std::vector<int16_t> gfsk_render_burst_cs16(const bits &chips, double fs_hz, double offset_hz,
                                            int16_t amplitude) {
	std::vector<cf> iq = gfsk_render_burst(chips, fs_hz, offset_hz, (float)amplitude);
	std::vector<int16_t> out(iq.size() * 2);
	for(size_t i = 0; i < iq.size(); i++) {
		long I = std::lround((double)iq[i].real());
		long Q = std::lround((double)iq[i].imag());
		I = I > 32767 ? 32767 : (I < -32768 ? -32768 : I);
		Q = Q > 32767 ? 32767 : (Q < -32768 ? -32768 : Q);
		out[2 * i] = (int16_t)I;
		out[2 * i + 1] = (int16_t)Q;
	}
	return out;
}

} // namespace adsld

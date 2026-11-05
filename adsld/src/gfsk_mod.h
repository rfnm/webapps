// adsld: GFSK burst modulator (TX + selftest/loopback stimulus). Renders an ADS-L chip
// stream to complex baseband: chips -> Gaussian FIR (BT=0.5, 4-chip span) -> frequency
// path +-ADSL_DEV_HZ -> phase accumulator -> complex exponential -> mix to the
// channel offset. Short raised-cosine amplitude ramps at the burst edges keep the
// occupied bandwidth clean.
#pragma once

#include <cstdint>
#include <vector>

#include "adsl_phy.h"
#include "dsp.h"

namespace adsld {

constexpr int GFSK_SPAN_CHIPS = 4;    // Gaussian pulse span (chips)
constexpr int GFSK_RAMP_CHIPS = 2;    // raised-cosine amplitude ramp at each burst edge

// Render one burst at sample rate fs_hz (must be an integer multiple of the chip rate),
// mixed to offset_hz from the capture/TX center, peak amplitude `amplitude`. The output
// covers the chips plus the edge ramps; phase starts at 0.
std::vector<cf> gfsk_render_burst(const bits &chips, double fs_hz, double offset_hz,
                                  float amplitude);

// Same burst as interleaved CS16 I/Q (2 * nsamples int16 values), peak DAC amplitude
// `amplitude` (~14000 for the RFNM TX path).
std::vector<int16_t> gfsk_render_burst_cs16(const bits &chips, double fs_hz, double offset_hz,
                                            int16_t amplitude);

} // namespace adsld

// adsld: the one shared numeric type. ADS-L needs no FFT (unlike lorad's fft.h), so this
// is just the complex-float sample type the DDC, modulator and receiver pass around.
#pragma once

#include <complex>

namespace adsld {

typedef std::complex<float> cf;

} // namespace adsld

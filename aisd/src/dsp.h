// aisd: the one shared numeric type. AIS needs no FFT (unlike lorad's fft.h), so this
// is just the complex-float sample type the DDC and receiver pass around.
#pragma once

#include <complex>

namespace aisd {

typedef std::complex<float> cf;

} // namespace aisd

# aisd build. Host selftest/bench build: `make` (no librfnm needed).
# Board build (in the chroot, native aarch64 -- adds the A53 tuning via ARCHFLAGS):
#   make WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 \
#        ASIO_INC='-I/rfnm/apps/src/asio-src/asio/include -I/usr/local/include' \
#        LDFLAGS='-L/usr/local/lib -Wl,-rpath,/usr/local/lib'
#
# ARCHFLAGS is empty on the x86 host (so -mcpu=cortex-a53 never reaches the host
# compiler); set it to -mcpu=cortex-a53 only for the aarch64 board/cross build.
# -fcx-limited-range is critical: without it GCC emits a libgcc __mulsc3 call for every
# std::complex<float> multiply (the inf/NaN-safe path), catastrophic in the DDC down-mix
# and FIR. It inlines the multiply to (ac-bd, ad+bc), safe for the normalized IQ this DSP
# handles. Full -ffast-math is deliberately NOT used: it can shift the near-threshold
# noisy/CFO selftest cases.
CXX ?= g++
ARCHFLAGS ?=
CXXFLAGS ?= -O3 -g
CXXFLAGS += -I$(COMMON)
CXXFLAGS += -std=c++17 -Wall -Wextra -pthread -fcx-limited-range -ftree-vectorize $(ARCHFLAGS)
LDFLAGS ?=
LDLIBS = -pthread

COMMON ?= ../common
SRCS = src/main.cpp src/ais_phy.cpp src/receiver.cpp src/parse.cpp src/selftest.cpp
ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM=1 $(ASIO_INC)
SRCS += src/capture_rfnm.cpp
LDLIBS += -lrfnm
endif

OBJS = $(SRCS:.cpp=.o) src/rfnm_httpd.o
HDRS = $(wildcard src/*.h)

aisd: $(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

selftest: aisd
	./aisd --selftest

clean:
	rm -f aisd $(OBJS)

.PHONY: selftest clean

# the shared fleet httpd (apps/common) builds into this tree so `make clean` sweeps it
src/rfnm_httpd.o: $(COMMON)/rfnm_httpd.cpp $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

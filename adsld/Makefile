# adsld build. Host selftest/mock build: `make` (no librfnm needed).
# Board build: cross-compile for aarch64 against the board sysroot, see README.md.
#   make WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 ASIO_INC=-I<librfnm>/asio/asio/include
#
# ARCHFLAGS is empty on the x86 host (so -mcpu=cortex-a53 never reaches the host compiler).
# -fcx-limited-range matters for the same reason as in lorad: without it every
# std::complex<float> multiply calls libgcc __mulsc3, catastrophic in the DDC down-mix and
# FIR. Full -ffast-math is deliberately not used: it can shift the near-threshold noisy/CFO
# selftest cases.

CXX ?= g++
ARCHFLAGS ?=
ASIO_INC ?=
COMMON ?= ../common
CXXFLAGS ?= -O3 -g -std=c++17 -Wall -Wextra -pthread -fcx-limited-range -ftree-vectorize $(ARCHFLAGS)
CXXFLAGS += -I$(COMMON)
LDFLAGS ?=

OBJS = src/main.o src/adsl_codec.o src/adsl_phy.o src/gfsk_mod.o src/receiver.o \
       src/radio_mock.o src/txsched.o src/position.o src/traffic.o src/demo.o \
       src/websrv.o src/conform.o src/selftest.o src/selftest_codec.o src/selftest_phy.o \
       src/selftest_rx.o src/selftest_sched.o src/selftest_loopback.o src/selftest_status.o \
       src/selftest_conform.o

ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM=1 $(ASIO_INC)
OBJS += src/radio_rfnm.o
LIBS = -lrfnm
else
LIBS =
endif

adsld: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) -pthread

src/%.o: src/%.cpp src/*.h $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h $(COMMON)/rfnm_boardconf.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o adsld

.PHONY: clean

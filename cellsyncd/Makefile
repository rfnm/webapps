# cellsyncd build. Host selftest/sim build: `make` (no librfnm needed).
# Board build: CROSS-COMPILE ONLY (aarch64 toolchain against the board sysroot;
# never on-target):  source the cross env, then  make clean && make WITH_RFNM=1
# -fcx-limited-range matters for the same reason as in lorad: without it every
# std::complex<float> multiply calls libgcc __mulsc3.

CXX ?= g++
ARCHFLAGS ?=
ASIO_INC ?=
COMMON ?= ../common
CXXFLAGS ?= -O3 -g -std=c++17 -Wall -Wextra -pthread -fcx-limited-range -ftree-vectorize $(ARCHFLAGS)
CXXFLAGS += -I$(COMMON)
LDFLAGS ?=

OBJS = src/main.o src/lte_seq.o src/tracker.o src/steer.o src/rfnm_httpd.o src/selftest.o

ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM $(ASIO_INC)
OBJS += src/capture_rfnm.o
LIBS = -lrfnm
else
LIBS =
endif

cellsyncd: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) -pthread

src/%.o: src/%.cpp src/*.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# the shared fleet httpd (apps/common) builds into this tree so `make clean` sweeps it
src/rfnm_httpd.o: $(COMMON)/rfnm_httpd.cpp $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o cellsyncd

.PHONY: clean

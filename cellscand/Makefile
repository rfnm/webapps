# cellscand build. Host selftest build: `make` (no librfnm needed).
# Board build: CROSS-COMPILE ONLY (aarch64 toolchain against the board sysroot; never
# on-target):  source the cross env, then  make clean && make WITH_RFNM=1
#
# The verified SIB1 chain is NOT forked: this app compiles the production sources straight
# out of ../cellsyncd/src (lte_seq/lte_coding/lte_wb/lte_rrc), the same way every app in the
# bundle compiles ../common/rfnm_httpd.cpp. cellsyncd's 152 --selftest checks stay the single
# source of truth for the DSP conventions.
# -fcx-limited-range matters for the same reason as in lorad/cellsyncd: without it every
# std::complex<float> multiply calls libgcc __mulsc3.

CXX ?= g++
ARCHFLAGS ?=
ASIO_INC ?=
COMMON ?= ../common
CHAIN ?= ../cellsyncd/src
CXXFLAGS ?= -O3 -g -std=c++17 -Wall -Wextra -pthread -fcx-limited-range -ftree-vectorize $(ARCHFLAGS)
CXXFLAGS += -I$(COMMON) -I$(CHAIN) -Isrc
LDFLAGS ?=

# cellscand's own objects
OBJS = src/main.o src/report.o src/cellfind.o src/decode_carrier.o src/burst.o src/job.o src/selftest.o
# the shared fleet httpd (built into this tree so `make clean` sweeps it)
OBJS += src/rfnm_httpd.o
# the verified LTE chain, compiled from ../cellsyncd/src into this tree (own .o names, so the
# cellsyncd build is never touched)
CHAIN_OBJS = src/lte_seq.o src/lte_coding.o src/lte_wb.o src/lte_rrc.o
OBJS += $(CHAIN_OBJS)

ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM $(ASIO_INC)
OBJS += src/capture_rfnm.o
LIBS = -lrfnm
else
LIBS =
endif

cellscand: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) -pthread

# cellscand's own sources
src/%.o: src/%.cpp src/*.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# the shared fleet httpd
src/rfnm_httpd.o: $(COMMON)/rfnm_httpd.cpp $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# the verified chain, referenced (never copied) from cellsyncd
src/lte_seq.o: $(CHAIN)/lte_seq.cpp $(CHAIN)/lte_seq.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
src/lte_coding.o: $(CHAIN)/lte_coding.cpp $(CHAIN)/lte_coding.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
src/lte_wb.o: $(CHAIN)/lte_wb.cpp $(CHAIN)/lte_wb.h $(CHAIN)/lte_seq.h $(CHAIN)/lte_coding.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<
src/lte_rrc.o: $(CHAIN)/lte_rrc.cpp $(CHAIN)/lte_rrc.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# the RFNM capture front-end (board build only), also referenced from cellsyncd
src/capture_rfnm.o: $(CHAIN)/capture_rfnm.cpp $(CHAIN)/capture.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o cellscand

.PHONY: clean

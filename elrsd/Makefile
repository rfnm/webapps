# elrsd build. Host selftest/replay build: `make` (no librfnm needed).
# Board build is driven by the bundle's top Makefile `cross` target, which passes
#   WITH_RFNM=1 CXX="aarch64-linux-gnu-g++ --sysroot=$S" ARCHFLAGS=-mcpu=cortex-a53
#   ASIO_INC="-I$S/usr/local/include" LDFLAGS="-L$S/usr/local/lib -Wl,-rpath-link,..."
# so every variable below is only ever set with ?= or += (contract).
#
# ARCHFLAGS is empty on the x86 host. -fcx-limited-range is load-bearing: without it GCC
# emits __mulsc3 for every std::complex<float> multiply (FFT butterflies, NCO). Full
# -ffast-math is deliberately NOT used (it can shift near-threshold selftest cases).
CXX ?= g++
ARCHFLAGS ?=
CXXFLAGS ?= -O3 -g
CXXFLAGS += -I$(COMMON)
CXXFLAGS += -std=c++17 -Wall -Wextra -pthread -fcx-limited-range -ftree-vectorize $(ARCHFLAGS)
LDFLAGS ?=
LDLIBS = -pthread

COMMON ?= ../common
# embedded codec vectors (TEST-UID only); repo-relative, never a scratch path
VECTORS ?= tools/elrsd_vectors.json

# every src/*.cpp except the librfnm adapters (which need WITH_RFNM=1).
RFNM_SRCS = src/capture_rfnm.cpp src/tx_rfnm.cpp
SRCS = $(filter-out $(RFNM_SRCS),$(wildcard src/*.cpp))

# ---- parallel-work rule: a source that references symbols of another module's
# module declares that module's .cpp files in a leading comment line
#     // NEEDS: src/elrs_codec.cpp src/rx_chain.cpp
# and is compiled only once all of them exist (and are themselves buildable). Until then the
# weak selftest hooks in main.cpp report the module as [SKIP]. Three pruning passes reach the
# fixed point for the dependency depth we have (codec -> link -> rx/tx -> daemon).
needs_of = $(shell sed -n '1,40{s|^//[ ]*NEEDS:||p}' $(1))
prune = $(foreach f,$(1),$(if $(filter-out $(1),$(call needs_of,$(f))),,$(f)))
SRCS := $(call prune,$(call prune,$(call prune,$(SRCS))))
PRUNED = $(filter-out $(SRCS) $(RFNM_SRCS),$(wildcard src/*.cpp))
ifneq ($(PRUNED),)
$(info elrsd: not built yet (missing NEEDS): $(PRUNED))
endif

ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM=1 $(ASIO_INC)
SRCS += $(wildcard $(RFNM_SRCS))
LDLIBS += -lrfnm
endif

OBJS = $(SRCS:.cpp=.o) src/rfnm_httpd.o
HDRS = $(wildcard src/*.h)

# relink when the object LIST changes (a pruned/added module changes no listed prerequisite,
# so without this stamp a stale binary would keep the old module set)
$(shell echo "$(OBJS)" | cmp -s - .objlist 2>/dev/null || echo "$(OBJS)" > .objlist)

elrsd: $(OBJS) .objlist
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

selftest: elrsd
	./elrsd --selftest

# regenerate the embedded codec vectors header (tools/vectors_to_h.py). The
# generated src/elrs_vectors_gen.h is checked in so that `make` never needs python.
vectors:
	python3 tools/vectors_to_h.py $(VECTORS) > src/elrs_vectors_gen.h.tmp && mv src/elrs_vectors_gen.h.tmp src/elrs_vectors_gen.h

clean:
	rm -f elrsd src/*.o src/elrs_vectors_gen.h.tmp .objlist

.PHONY: selftest clean vectors

# the shared fleet httpd (apps/common) builds into this tree so `make clean` sweeps it
src/rfnm_httpd.o: $(COMMON)/rfnm_httpd.cpp $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# clockgend build. Host: `make && ./clockgend --selftest`.
# Cross build: source the aarch64 cross env, then `make clean && make`
# (CXX/ARCHFLAGS come from the env; no librfnm, no external deps - pure sysfs client).
CXX ?= g++
ARCHFLAGS ?=
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra $(ARCHFLAGS)

clockgend: src/clockgend.cpp
	$(CXX) $(CXXFLAGS) src/clockgend.cpp -o clockgend

selftest: clockgend
	./clockgend --selftest

clean:
	rm -f clockgend

.PHONY: selftest clean

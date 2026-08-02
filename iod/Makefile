# iod build. Host: `make && ./iod --selftest`. Cross: cross-compile-imx8mp env (kernel
# uapi <linux/gpio.h> only - no libgpiod, no librfnm).
CXX ?= g++
ARCHFLAGS ?=
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra $(ARCHFLAGS)

iod: src/iod.cpp
	$(CXX) $(CXXFLAGS) src/iod.cpp -o iod

selftest: iod
	./iod --selftest

clean:
	rm -f iod

.PHONY: selftest clean

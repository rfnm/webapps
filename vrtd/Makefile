# vrtd build. Host selftest/mock build: `make`. Board build (in the rootfs chroot):
#   make WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 \
#        ASIO_INC='-I/rfnm/apps/src/asio-src/asio/include -I/usr/local/include' \
#        LDFLAGS='-L/usr/local/lib -Wl,-rpath,/usr/local/lib'

CXX ?= g++
ARCHFLAGS ?=
ASIO_INC ?=
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -pthread $(ARCHFLAGS)
CXXFLAGS += -I$(COMMON)
LDFLAGS ?=

COMMON ?= ../common
OBJS = src/main.o src/vrt.o src/core.o src/mock_radio.o src/rfnm_httpd.o src/selftest.o

ifeq ($(WITH_RFNM),1)
CXXFLAGS += -DWITH_RFNM $(ASIO_INC)
OBJS += src/rfnm_radio.o
LIBS = -lrfnm
else
LIBS =
endif

vrtd: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) -pthread

src/%.o: src/%.cpp src/*.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o vrtd

.PHONY: clean

src/rfnm_httpd.o: $(COMMON)/rfnm_httpd.cpp $(COMMON)/rfnm_httpd.h $(COMMON)/rfnm_json.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

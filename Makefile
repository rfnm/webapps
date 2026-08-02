# RFNM on-board apps bundle.
#   make            host build of every app (each has a native selftest - no board needed)
#   make cross      aarch64 board build (needs RFNM_SYSROOT + a librfnm cross build, see
#                   the SDK: ./rfnm-sdk build apps)
APPS := aisd cellsyncd clockgend iod lorad vrtd
CROSS ?= aarch64-linux-gnu-
LIBRFNM ?= $(abspath ../librfnm)

all: $(APPS)
$(APPS):
	$(MAKE) -C $@

cross:
	@test -n "$(RFNM_SYSROOT)" || { echo "set RFNM_SYSROOT (board sysroot path)"; exit 1; }
	for a in $(APPS); do \
	  $(MAKE) -C $$a WITH_RFNM=1 \
	    CXX="$(CROSS)g++ --sysroot=$(RFNM_SYSROOT)" \
	    ARCHFLAGS=-mcpu=cortex-a53 \
	    ASIO_INC="-I$(LIBRFNM)/asio/asio/include -I$(RFNM_SYSROOT)/usr/local/include" \
	    LDFLAGS="-L$(RFNM_SYSROOT)/usr/local/lib -Wl,-rpath-link,$(RFNM_SYSROOT)/usr/lib/aarch64-linux-gnu" \
	    || exit 1; \
	done

clean:
	for a in $(APPS); do $(MAKE) -C $$a clean; done
.PHONY: all cross clean $(APPS)

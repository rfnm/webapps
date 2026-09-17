// elrsd: ExpressLRS 2.4 GHz sniffer / bench transmitter for the RFNM web GUI.
// *** PERMANENT BOOTSTRAP (architect). Do not replace. ***
// Anyone can `make && ./elrsd --selftest` from day one: the module selftests are weak
// symbols, a module that is not written (or pruned by the Makefile's NEEDS rule) is reported
// as SKIP. The real daemon entry point is `int daemon_main(int, char **)` in
// src/daemon_main.cpp (which NEEDS every other module and is therefore pruned until they
// exist); it is also weak here, so a tree without it still builds and reports exit 2.
#include <cstdio>
#include <cstring>
#include "selftest.h"

namespace elrsd {
__attribute__((weak)) int run_selftest_codec();
__attribute__((weak)) int run_selftest_link();
__attribute__((weak)) int run_selftest_rx();
__attribute__((weak)) int run_selftest_tx();
__attribute__((weak)) int run_selftest_daemon();
__attribute__((weak)) int daemon_main(int argc, char **argv);   // src/daemon_main.cpp
}

using namespace elrsd;

static int run_one(const char *name, int (*fn)()) {
	if(!fn) {
		printf("[SKIP] %s: module not built yet\n", name);
		return 0;
	}
	printf("== %s ==\n", name);
	return fn();
}

int main(int argc, char **argv) {
	bool selftest = false;
	for(int i = 1; i < argc; i++) {
		if(!strcmp(argv[i], "--selftest")) {
			selftest = true;
		}
	}
	if(!selftest) {
		if(daemon_main) {
			return daemon_main(argc, argv);
		}
		fprintf(stderr, "elrsd: bootstrap only - the daemon (src/daemon_main.cpp) is not built yet; only --selftest works\n");
		return 2;
	}
	int fails = 0;
	fails += run_one("codec", run_selftest_codec);
	fails += run_one("link", run_selftest_link);
	fails += run_one("rx", run_selftest_rx);
	fails += run_one("tx", run_selftest_tx);
	fails += run_one("daemon", run_selftest_daemon);
	if(fails) {
		printf("SELFTEST FAILED: %d case(s)\n", fails);
		return 1;
	}
	printf("SELFTEST OK\n");
	return 0;
}

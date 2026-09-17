// elrsd: shared selftest reporting. Every module selftest prints "[PASS] name" / "[FAIL] name: detail"
// through check() and returns its failure count; main aggregates and prints SELFTEST OK /
// SELFTEST FAILED: N case(s). Deterministic seeds only, no hardware, no network, no files
// outside the tree (the embedded vectors header is compiled in).
#pragma once

#include <cstdio>
#include <string>

namespace elrsd {

struct selftest_counter {
	int fails = 0;
	int passes = 0;
	void check(bool ok, const char *name, const std::string &detail = "") {
		printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : ": ", detail.c_str());
		fflush(stdout);
		if(ok) passes++; else fails++;
	}
};

// module entry points (each owned by one agent)
int run_selftest_codec();   // A  src/elrs_codec_selftest.cpp
int run_selftest_link();    // A  src/elrs_link_selftest.cpp (may live in the same file)
int run_selftest_rx();      // B  src/rx_selftest.cpp
int run_selftest_tx();      // D  src/tx_selftest.cpp
int run_selftest_daemon();  // C  src/daemon_selftest.cpp (config precedence, JSON schemas, packet ring, tracker glue)

} // namespace elrsd

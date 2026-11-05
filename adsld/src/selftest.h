// adsld --selftest: shared harness for the per-module selftest translation units. Each
// module entry point returns its fail count; selftest.cpp sums them and prints the final
// SELFTEST OK / SELFTEST FAILED verdict. Everything runs from fixed-seed PRNGs and
// standard KATs; a pass is a real pass, never luck.
#pragma once

#include <cstdio>
#include <string>

namespace adsld {

// per-module entry points (selftest_codec.cpp etc.); each returns its fail count
int selftest_codec();
int selftest_phy();
int selftest_rx();
int selftest_sched();
int selftest_loopback();
int selftest_status();
int selftest_conform();

// shared check helper: prints [PASS]/[FAIL] and bumps fails on failure
inline void check(int &fails, bool ok, const char *name, const std::string &detail = "") {
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : ": ", detail.c_str());
	if(!ok) {
		fails++;
	}
}

} // namespace adsld

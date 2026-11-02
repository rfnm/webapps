// adsld --selftest: deterministic, hardware-free acceptance test. Sums the per-module
// fail counts (selftest_codec/phy/rx/sched/loopback/status/conform.cpp) and prints the final
// verdict.
#include <cstdio>

#include "selftest.h"

namespace adsld {

int run_selftest() {
	int fails = 0;
	fails += selftest_codec();
	fails += selftest_phy();
	fails += selftest_rx();
	fails += selftest_sched();
	fails += selftest_loopback();
	// Status rides the same PHY and MAC, so it runs once the loopback gates have shown both
	fails += selftest_status();
	// Timestamp, privacy address and conformity notes build on the scheduler just exercised
	fails += selftest_conform();
	if(fails == 0) {
		printf("SELFTEST OK\n");
	} else {
		printf("SELFTEST FAILED: %d check(s)\n", fails);
	}
	return fails ? 1 : 0;
}

} // namespace adsld

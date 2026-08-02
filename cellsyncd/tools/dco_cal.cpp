// dco_cal: measure the Si5510 MR DCO transfer function (sign + gain) at the CHIP level.
// Instrument: LA9310 phytimer tick (61.44 MHz nominal, derived from DCS/OUT13 <- Fvco <- MR,
// so it moves 1:1 fractionally with any MR DCO pull) regressed against CLOCK_MONOTONIC_RAW
// (A53 arch timer from the SoC's own 24 MHz crystal - NOT Si5510-derived). No RF, no RX
// stream, no cellsyncd pipeline: this isolates the chip transfer from every measurement-
// frame and observable-conversion question.
//
// Plateaus of ABSOLUTE DCO steps (interleaved with 0-plateaus so crystal drift cancels):
//   0, +100000, 0, -100000, 0
// Each plateau: settle, then (t_mono, tick) pairs, linear regression -> tick rate.
// ppm(plateau) = (rate - rate0_local) / rate0_local * 1e6, rate0_local = mean of the
// neighbouring zero plateaus.
#include <librfnm/device.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <thread>
#include <vector>

static bool set_dco(int32_t steps) {
	std::ofstream f("/sys/bus/i2c/devices/0-0058/rfnm_set_dco_steps");
	if(!f) {
		return false;
	}
	f << steps << "\n";
	return f.good();
}

static double plateau_rate(rfnm::device *dev, double secs, int period_ms) {
	std::vector<std::pair<double, double>> v;
	auto t_end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
	while(std::chrono::steady_clock::now() < t_end) {
		struct timespec a, b;
		uint64_t tick = 0;
		clock_gettime(CLOCK_MONOTONIC_RAW, &a);
		if(dev->clock_now(&tick) != RFNM_API_OK) {
			fprintf(stderr, "clock_now failed\n");
			exit(1);
		}
		clock_gettime(CLOCK_MONOTONIC_RAW, &b);
		double ta = a.tv_sec + a.tv_nsec * 1e-9, tb = b.tv_sec + b.tv_nsec * 1e-9;
		v.emplace_back(0.5 * (ta + tb), (double)tick);
		std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
	}
	double n = (double)v.size(), sx = 0, sy = 0, sxx = 0, sxy = 0;
	double x0 = v[0].first, y0 = v[0].second;
	for(const auto &p : v) {
		double x = p.first - x0, y = p.second - y0;
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
	}
	return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

int main(int argc, char **argv) {
	double secs = (argc > 1) ? atof(argv[1]) : 30.0;
	int period_ms = (argc > 2) ? atoi(argv[2]) : 50;

	rfnm::device *dev;
	try {
		dev = new rfnm::device(rfnm::TRANSPORT_LOCAL, "");
	} catch(const std::exception &e) {
		fprintf(stderr, "open failed: %s\n", e.what());
		return 1;
	}

	const int32_t plan[5] = { 0, 100000, 0, -100000, 0 };
	double rate[5];

	for(int i = 0; i < 5; i++) {
		if(!set_dco(plan[i])) {
			fprintf(stderr, "sysfs dco write failed\n");
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		rate[i] = plateau_rate(dev, secs, period_ms);
		printf("plateau dco=%+7d steps: rate=%.3f Hz\n", plan[i], rate[i]);
		fflush(stdout);
	}
	set_dco(0);

	// each pulled plateau vs the mean of its neighbouring zeros (kills linear crystal drift)
	double z01 = 0.5 * (rate[0] + rate[2]), z23 = 0.5 * (rate[2] + rate[4]);
	double ppm_pos = (rate[1] - z01) / z01 * 1e6;
	double ppm_neg = (rate[3] - z23) / z23 * 1e6;
	printf("\nRESULT +100000 steps -> %+.3f ppm (commanded +10.000 at 0.1 ppb/step, sign=+1)\n", ppm_pos);
	printf("RESULT -100000 steps -> %+.3f ppm (commanded -10.000)\n", ppm_neg);
	printf("RESULT sign=%+d gain=%.4f (|measured|/|commanded|, avg of both plateaus)\n",
	       (ppm_pos > 0) ? 1 : -1, 0.5 * (std::fabs(ppm_pos) + std::fabs(ppm_neg)) / 10.0);
	printf("RESULT ppb_per_step=%+.6f\n", 0.5 * (ppm_pos - ppm_neg) * 1000.0 / 100000.0);
	return 0;
}

// elrsd: replay smoke tool (NOT part of the daemon build - src/*.cpp only). Runs the
// burst-gated RX chain over a cs16 capture and prints counts; optionally dumps every found
// burst (sample_start, channel, rate, ref, symbols) to a text file.
//   g++ -O3 -std=c++17 -fcx-limited-range -Isrc -o /tmp/rx_smoke tools/rx_smoke.cpp src/dsp.cpp src/rx_chain.cpp src/source_replay.cpp src/source_buffer.cpp -pthread
//   /tmp/rx_smoke FILE fs fc seconds rate_idx(0=auto) conj(0|1) [dump.txt]
#include "rx_chain.h"
#include "sample_source.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
using namespace elrsd;

int main(int argc, char **argv) {
	if(argc >= 2 && !strcmp(argv[1], "--bench")) return run_bench_rx(argc > 2 ? atof(argv[2]) : 2.0);
	if(argc < 7) {
		fprintf(stderr, "usage: rx_smoke FILE fs fc seconds rate_idx conj [dump.txt] [os]\n");
		return 2;
	}
	replay_config rc;
	rc.path = argv[1];
	rc.fs_hz = atof(argv[2]);
	rc.fc_hz = atof(argv[3]);
	rc.stop_s = atof(argv[4]);
	rx_chain_config cc;
	cc.fs_hz = rc.fs_hz;
	cc.fc_hz = rc.fc_hz;
	cc.rate_index = atoi(argv[5]);
	cc.conj = atoi(argv[6]) != 0;
	cc.os = argc > 8 ? atoi(argv[8]) : 4;
	FILE *dump = argc > 7 ? fopen(argv[7], "w") : nullptr;
	rx_chain chain(cc);
	uint64_t found = 0, cb = 0;
	std::map<int, int> per_ch, per_rate, runs;
	double snr_sum = 0;
	chain.set_on_burst([&](const rx_burst &b) {
		cb++;
		if(!b.sync.found) return;
		found++;
		per_ch[b.channel]++;
		per_rate[b.rate_index]++;
		runs[b.sync.preamble_run]++;
		snr_sum += b.snr_db;
		if(dump) {
			fprintf(dump, "%llu %.6f %d %d %d %.1f %.1f %d %.0f", (unsigned long long)b.sample_start, b.t_start_s, b.channel, b.rate_index, b.sync.ref, b.snr_db, b.rssi_db, b.sync.preamble_run, b.sync.cfo_hz);
			for(uint16_t s : b.symbols) fprintf(dump, " %d", s);
			fprintf(dump, "\n");
		}
	});
	sample_source *src = make_replay_source(rc);
	std::atomic<bool> stop{false};
	auto t0 = std::chrono::steady_clock::now();
	bool ok = src->run(stop, [&](const source_block &blk) { chain.feed(blk.iq, blk.count, blk.sample_index, blk.ts_ns); });
	double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	source_stats ss;
	src->stats(ss);
	rx_chain::stats st;
	chain.get_stats(st);
	printf("replay %s: ok=%d samples=%llu (%.2f s) wall=%.2f s realtime=%.2fx\n", argv[1], ok, (unsigned long long)ss.samples, ss.samples / rc.fs_hz, wall, ss.samples / rc.fs_hz / wall);
	printf("bursts=%llu too_short=%llu too_long=%llu no_rate=%llu overruns=%llu attempts=%llu found=%llu callbacks=%llu floor=%.0f conj_final=%d\n",
	       (unsigned long long)st.bursts, (unsigned long long)st.bursts_too_short, (unsigned long long)st.bursts_too_long, (unsigned long long)st.bursts_no_rate,
	       (unsigned long long)st.ring_overruns, (unsigned long long)st.demod_attempts, (unsigned long long)st.demod_found, (unsigned long long)cb, st.floor_level, chain.config().conj);
	printf("per_rate:");
	for(auto &kv : per_rate) printf(" %d:%d", kv.first, kv.second);
	printf("\nper_channel:");
	for(auto &kv : per_ch) printf(" %d:%d", kv.first, kv.second);
	printf("\npreamble_run:");
	for(auto &kv : runs) printf(" %d:%d", kv.first, kv.second);
	printf("\nsnr_avg=%.1f dB\n", found ? snr_sum / found : 0.0);
	if(dump) fclose(dump);
	delete src;
	return 0;
}

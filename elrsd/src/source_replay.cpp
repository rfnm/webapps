// elrsd: cs16 replay source. int16 LE interleaved IQ read in device-sized blocks;
// sample_index = file position / 4 (absolute in the file, also when start_s > 0), so the
// replay driver's ground truth (k from sample_start) is anchored to the file.
#include "sample_source.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include <sys/stat.h>

namespace elrsd {

namespace {

class replay_source : public sample_source {
public:
	explicit replay_source(const replay_config &cfg) : cfg_(cfg) {
		if(cfg_.block_samples == 0) cfg_.block_samples = 65536;
	}
	double fs_hz() const override { return cfg_.fs_hz; }
	double fc_hz() const override { return cfg_.fc_hz; }
	std::string describe() const override {
		char b[256];
		snprintf(b, sizeof b, "replay %s fs=%.0f fc=%.0f%s", cfg_.path.c_str(), cfg_.fs_hz, cfg_.fc_hz, cfg_.realtime ? " realtime" : "");
		return b;
	}
	void stats(source_stats &out) const override {
		out.samples = samples_.load();
		out.drops = 0;
		out.reads = reads_.load();
		out.empty_reads = 0;
		out.eof = eof_.load();
	}

	bool run(std::atomic<bool> &stop, const std::function<void(const source_block &)> &cb) override {
		eof_ = false;
		FILE *f = fopen(cfg_.path.c_str(), "rb");
		if(!f) {
			fprintf(stderr, "elrsd: replay: cannot open %s\n", cfg_.path.c_str());
			return false;
		}
		struct stat st;
		if(fstat(fileno(f), &st) != 0 || st.st_size < 4) {
			fprintf(stderr, "elrsd: replay: cannot stat %s\n", cfg_.path.c_str());
			fclose(f);
			return false;
		}
		const uint64_t total = (uint64_t)st.st_size / 4;
		uint64_t first = 0, last = total;
		if(cfg_.start_s > 0) first = (uint64_t)(cfg_.start_s * cfg_.fs_hz);
		if(cfg_.stop_s > 0) last = (uint64_t)(cfg_.stop_s * cfg_.fs_hz);
		if(last > total) last = total;
		if(first >= last) {
			fprintf(stderr, "elrsd: replay: empty window %.3f..%.3f s in %s (%.3f s)\n", cfg_.start_s, cfg_.stop_s, cfg_.path.c_str(), total / cfg_.fs_hz);
			fclose(f);
			return false;
		}
		std::vector<int16_t> buf(cfg_.block_samples * 2);
		const double block_s = (double)cfg_.block_samples / cfg_.fs_hz;
		bool ok = true;
		do {
			if(fseeko(f, (off_t)first * 4, SEEK_SET) != 0) {
				fprintf(stderr, "elrsd: replay: seek failed\n");
				ok = false;
				break;
			}
			uint64_t pos = first;
			auto t0 = std::chrono::steady_clock::now();
			uint64_t delivered = 0;
			while(!stop && pos < last) {
				size_t want = (size_t)std::min<uint64_t>(cfg_.block_samples, last - pos);
				size_t got = fread(buf.data(), 4, want, f);
				reads_++;
				if(got == 0) {
					break;
				}
				source_block b;
				b.iq = buf.data();
				b.count = got;
				b.sample_index = pos;
				b.ts_ns = -1;
				cb(b);
				pos += got;
				samples_ += got;
				delivered += got;
				if(cfg_.realtime) {
					double due = delivered / cfg_.fs_hz;
					auto now = std::chrono::steady_clock::now();
					double el = std::chrono::duration<double>(now - t0).count();
					if(due > el + block_s) {
						std::this_thread::sleep_for(std::chrono::duration<double>(due - el));
					}
				}
			}
		} while(cfg_.loop && !stop && ok);
		eof_ = true;
		fclose(f);
		return ok;
	}

private:
	replay_config cfg_;
	std::atomic<uint64_t> samples_{0}, reads_{0};
	std::atomic<bool> eof_{false};
};

} // namespace

sample_source *make_replay_source(const replay_config &cfg) {
	return new replay_source(cfg);
}

} // namespace elrsd

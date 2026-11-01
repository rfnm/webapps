#include "burst.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace cellscan {

// ---- capture sizing and the memory budget ---------------------------------------------------
size_t capture_reset(std::vector<cf> &out, size_t n) {
	// swap-with-a-temporary is the only way to make a vector give its storage BACK: clear()
	// keeps the capacity and shrink_to_fit() is non-binding. The old block is therefore gone
	// before the new one is asked for, and the two never coexist.
	std::vector<cf>().swap(out);
	out.reserve(n);
	return n;
}

static std::atomic<uint64_t> g_mem_override{ 0 };

void set_mem_available_override(uint64_t bytes) {
	g_mem_override.store(bytes);
}

uint64_t mem_available_bytes() {
	uint64_t ov = g_mem_override.load();
	if(ov) {
		return ov;
	}
#ifdef __linux__
	FILE *f = fopen("/proc/meminfo", "r");
	if(!f) {
		return 0;
	}
	char line[256];
	uint64_t kb = 0;
	while(fgets(line, sizeof(line), f)) {
		// MemAvailable, never MemFree: MemFree ignores the reclaimable page cache and would
		// refuse every deep capture on a board that has merely been reading files.
		if(strncmp(line, "MemAvailable:", 13) == 0) {
			kb = strtoull(line + 13, nullptr, 10);
			break;
		}
	}
	fclose(f);
	return kb * 1024ull;
#else
	return 0;   // unknown - see the header: unknown never blocks
#endif
}

bool capture_fits(double dwell_s, uint64_t *need, uint64_t *avail) {
	const uint64_t n = capture_bytes(dwell_s);
	const uint64_t a = mem_available_bytes();
	if(need) {
		*need = n;
	}
	if(avail) {
		*avail = a;
	}
	if(a == 0) {
		return true;   // cannot measure -> must not invent a verdict
	}
	return (double)n <= CAPTURE_MEM_FRACTION * (double)a;
}

// ---- file replay --------------------------------------------------------------------------
bool file_burst_source::open(double, int, bool, const std::string &) {
	FILE *f = fopen(path_.c_str(), "rb");
	if(!f) {
		perror(path_.c_str());
		return false;
	}
	fclose(f);
	return true;
}

bool file_burst_source::capture(double dwell_s, std::vector<cf> &out, std::atomic<bool> &cancel,
                                uint64_t *gap_samples) {
	if(gap_samples) {
		*gap_samples = 0;
	}
	if(!loaded_) {
		FILE *f = fopen(path_.c_str(), "rb");
		if(!f) {
			return false;
		}
		cache_.clear();
		std::vector<cf> buf(1 << 16);
		size_t got;
		while((got = fread(buf.data(), sizeof(cf), buf.size(), f)) > 0) {
			if(cancel.load()) {
				fclose(f);
				return false;
			}
			cache_.insert(cache_.end(), buf.begin(), buf.begin() + got);
		}
		fclose(f);
		if(conj_) {
			for(auto &v : cache_) {
				v = std::conj(v);
			}
		}
		loaded_ = true;
	}
	size_t n = std::min(capture_samples(dwell_s), cache_.size());
	// free whatever `out` held BEFORE sizing it: out.assign() over a longer previous capture
	// reallocates, and a realloc holds the old block and the new one at the same time.
	capture_reset(out, n);
	out.insert(out.end(), cache_.begin(), cache_.begin() + n);
	return !out.empty();
}

#ifdef WITH_RFNM
} // namespace cellscan

#include "capture.h"   // cellsyncd rx_session

namespace cellscan {

struct rfnm_burst_source::impl {
	cellsync::rx_session ses;
	bool open = false;
};

rfnm_burst_source::rfnm_burst_source() : p_(new impl) {}
rfnm_burst_source::~rfnm_burst_source() { delete p_; }

bool rfnm_burst_source::open(double center_hz, int gain_db, bool agc, const std::string &antenna) {
	p_->open = p_->ses.open(center_hz, RATE, gain_db, agc, antenna);
	return p_->open;
}

bool rfnm_burst_source::retune(double center_hz, const std::string &antenna) {
	return p_->open && p_->ses.retune(center_hz, antenna);
}

bool rfnm_burst_source::capture(double dwell_s, std::vector<cf> &out, std::atomic<bool> &cancel,
                                uint64_t *gap_samples) {
	if(!p_->open) {
		return false;
	}
	// Last line of defence on the budget (job.cpp decides the escalation policy; this catches
	// every other caller, including a job that was simply STARTED at dwell_s = 8). Loud,
	// because a capture that never happened must not read as a carrier that held nothing.
	uint64_t need = 0, avail = 0;
	if(!capture_fits(dwell_s, &need, &avail)) {
		fprintf(stderr, "cellscand: CAPTURE REFUSED - %.1f s needs %llu MB, MemAvailable is "
		                "%llu MB, budget is %llu MB (%.0f%%)\n",
		        dwell_s, (unsigned long long)(need >> 20), (unsigned long long)(avail >> 20),
		        (unsigned long long)((uint64_t)(CAPTURE_MEM_FRACTION * (double)avail) >> 20),
		        CAPTURE_MEM_FRACTION * 100.0);
		return false;
	}
	const uint64_t want = (uint64_t)capture_samples(dwell_s);
	// Size once, up front - and free what `out` held FIRST, so
	// the previous capture and this one are never both resident. The old code reserved
	// want + 0.05*RATE on a vector that still held the 2 s burst: 6 MB of slack the fill loop
	// (which stops at `want`) could never use, on top of a transient 1.23 GB double buffer.
	capture_reset(out, (size_t)want);
	uint64_t worst_gap = 0;
	std::atomic<uint64_t> samp{ 0 };
	std::atomic<bool> stop{ false };
	// settle discard: skip an initial slice so LO/FE transients (and IQ already in flight from
	// the old carrier after a retune) never enter the decoded burst.
	uint64_t settle = (uint64_t)(0.05 * RATE);
	bool ok = p_->ses.run(stop, samp, [&](const cf *x, size_t n, uint64_t gap) {
		if(cancel.load()) {
			stop = true;
			return;
		}
		if(gap > worst_gap) {
			worst_gap = gap;
		}
		if(settle > 0) {
			uint64_t drop = std::min<uint64_t>(settle, n);
			settle -= drop;
			x += drop;
			n -= drop;
		}
		if(n == 0) {
			return;
		}
		uint64_t room = out.size() < want ? want - out.size() : 0;
		size_t take = (size_t)std::min<uint64_t>(room, n);
		out.insert(out.end(), x, x + take);
		if(out.size() >= want) {
			stop = true;
		}
	});
	if(gap_samples) {
		*gap_samples = worst_gap;
	}
	if(cancel.load()) {
		return false;
	}
	return ok && out.size() >= want / 2;   // a stream that died early is a capture failure
}
#endif

} // namespace cellscan

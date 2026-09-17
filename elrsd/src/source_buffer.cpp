// elrsd: in-memory sample source for selftests + the host-build stubs of the
// librfnm factories (the real ones live in src/capture_rfnm.cpp, WITH_RFNM=1 only).
#include "sample_source.h"

#include <cstdio>
#include <algorithm>

namespace elrsd {

namespace {

class buffer_source : public sample_source {
public:
	buffer_source(const int16_t *iq, size_t count, double fs, double fc, size_t blk)
	    : iq_(iq), count_(count), fs_(fs), fc_(fc), blk_(blk ? blk : 65536) {}
	double fs_hz() const override { return fs_; }
	double fc_hz() const override { return fc_; }
	std::string describe() const override { return "buffer"; }
	void stats(source_stats &out) const override {
		out.samples = samples_.load();
		out.drops = 0;
		out.reads = reads_.load();
		out.empty_reads = 0;
		out.eof = eof_.load();
	}
	bool run(std::atomic<bool> &stop, const std::function<void(const source_block &)> &cb) override {
		eof_ = false;
		size_t pos = 0;
		while(!stop && pos < count_) {
			size_t n = std::min(blk_, count_ - pos);
			source_block b;
			b.iq = iq_ + 2 * pos;
			b.count = n;
			b.sample_index = pos;
			b.ts_ns = -1;
			cb(b);
			pos += n;
			samples_ += n;
			reads_++;
		}
		eof_ = true;
		return true;
	}
private:
	const int16_t *iq_;
	size_t count_;
	double fs_, fc_;
	size_t blk_;
	std::atomic<uint64_t> samples_{0}, reads_{0};
	std::atomic<bool> eof_{false};
};

} // namespace

sample_source *make_buffer_source(const int16_t *iq, size_t count, double fs_hz, double fc_hz, size_t block_samples) {
	return new buffer_source(iq, count, fs_hz, fc_hz, block_samples);
}

#ifndef WITH_RFNM
// host build: no librfnm. The daemon reports "built without librfnm" and refuses to go live.
void *rfnm_device_open(double, std::string &error) {
	error = "built without librfnm";
	fprintf(stderr, "elrsd: rfnm_device_open: built without librfnm\n");
	return nullptr;
}
void rfnm_device_close(void *) {}
sample_source *make_rfnm_rx_source(const rfnm_rx_config &, void *) {
	fprintf(stderr, "elrsd: make_rfnm_rx_source: built without librfnm\n");
	return nullptr;
}
#endif

} // namespace elrsd

// vrtd: deterministic mock backend for the selftest and --mock runs. RX emits a ramp
// pattern (sample n -> I = n & 0x7FFF, Q = ~n & 0x7FFF) so a consumer can verify
// sample integrity and ordering; TX just counts what it swallows.
#include "radio.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace vrtd {

class mock_radio : public radio {
public:
	bool start_rx(const rx_cb &cb) override {
		stop_rx();
		run_ = true;
		thr_ = std::thread([this, cb] {
			std::vector<int16_t> buf(2 * 1024);
			while(run_) {
				for(size_t k = 0; k < 1024; k++) {
					uint32_t n = seq_++;
					buf[2 * k] = (int16_t)(n & 0x7FFF);
					buf[2 * k + 1] = (int16_t)(~n & 0x7FFF);
				}
				cb(buf.data(), 1024);
				// pace vaguely like a real radio so loopback tests exercise timing paths
				std::this_thread::sleep_for(std::chrono::microseconds(200));
			}
		});
		st_.rx_on = true;
		return true;
	}
	void stop_rx() override {
		run_ = false;
		if(thr_.joinable()) {
			thr_.join();
		}
		st_.rx_on = false;
	}
	bool start_tx() override {
		st_.tx_on = true;
		return true;
	}
	void stop_tx() override { st_.tx_on = false; }
	bool submit_tx(const int16_t *iq, size_t nsamples) override {
		(void)iq;
		tx_samples += nsamples;
		return true;
	}
	bool set_rx_freq(double hz) override {
		if(hz < 10e6 || hz > 3500e6) {
			return false;                    // mirror the Lime tuning range
		}
		st_.rx_freq_hz = hz;
		return true;
	}
	bool set_tx_freq(double hz) override {
		if(hz < 10e6 || hz > 3500e6) {
			return false;
		}
		st_.tx_freq_hz = hz;
		return true;
	}
	bool set_samp_rate(double hz) override {
		if(hz < 195375.0 || hz > 122.88e6) {
			return false;                    // mirror the DCS-honesty floor
		}
		st_.samp_rate_hz = hz;
		return true;
	}
	bool set_rx_gain(double db) override {
		if(db < -24 || db > 30) {
			return false;
		}
		st_.rx_gain_db = db;
		return true;
	}
	bool set_tx_power(double dbm) override {
		st_.tx_power_dbm = dbm;
		return true;
	}
	radio_state state() override { return st_; }
	std::string describe() override { return "mock"; }

	std::atomic<uint64_t> tx_samples{ 0 };

private:
	radio_state st_;
	std::atomic<bool> run_{ false };
	std::atomic<uint32_t> seq_{ 0 };
	std::thread thr_;
};

radio *make_mock_radio() {
	return new mock_radio();
}

} // namespace vrtd

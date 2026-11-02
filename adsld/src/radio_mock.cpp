// adsld: deterministic mock radio for the hardware-free selftests - see radio.h.
#include "radio.h"

#include <mutex>

namespace adsld {

namespace {

class mock_impl : public mock_radio {
public:
	// ---- radio ----
	bool start_rx(double rate_hz, double freq_hz, int gain_db,
	              const std::string &antenna, const rx_cb &cb) override {
		(void)rate_hz;
		(void)freq_hz;
		(void)gain_db;
		(void)antenna;
		std::lock_guard<std::mutex> lk(mtx_);
		rx_cb_ = cb;
		rx_on_ = true;
		return true;
	}

	void stop_rx() override {
		std::lock_guard<std::mutex> lk(mtx_);
		rx_on_ = false;
		rx_cb_ = nullptr;
	}

	bool start_tx(double rate_hz, double freq_hz, double power_dbm,
	              const std::string &antenna) override {
		(void)rate_hz;
		(void)freq_hz;
		(void)power_dbm;
		(void)antenna;
		std::lock_guard<std::mutex> lk(mtx_);
		if(policy_ == tx_allowed::none) {
			return false;
		}
		tx_on_ = true;
		return true;
	}

	void stop_tx() override {
		std::lock_guard<std::mutex> lk(mtx_);
		tx_on_ = false;
	}

	bool submit_tx(const int16_t *iq, size_t nsamples) override {
		std::lock_guard<std::mutex> lk(mtx_);
		if(!tx_on_) {
			return false;
		}
		tx_.insert(tx_.end(), iq, iq + 2 * nsamples);
		return true;
	}

	tx_allowed tx_policy() const override { return policy_; }

	uint64_t now_ms() override {
		std::lock_guard<std::mutex> lk(mtx_);
		return now_ms_;
	}

	std::string describe() override { return "mock radio"; }

	// ---- mock controls ----
	void inject_rx(const int16_t *iq, size_t nsamples) override {
		rx_cb cb;
		{
			std::lock_guard<std::mutex> lk(mtx_);
			if(!rx_on_) {
				return;
			}
			cb = rx_cb_;
		}
		if(cb) {
			cb(iq, nsamples);
		}
	}

	std::vector<int16_t> tx_captured() override {
		std::lock_guard<std::mutex> lk(mtx_);
		std::vector<int16_t> out;
		out.swap(tx_);
		return out;
	}

	void set_now_ms(uint64_t ms) override {
		std::lock_guard<std::mutex> lk(mtx_);
		now_ms_ = ms;
	}

	void advance_ms(uint64_t ms) override {
		std::lock_guard<std::mutex> lk(mtx_);
		now_ms_ += ms;
	}

	void set_tx_policy(tx_allowed p) override { policy_ = p; }

private:
	std::mutex mtx_;
	rx_cb rx_cb_;
	bool rx_on_ = false;
	bool tx_on_ = false;
	tx_allowed policy_ = tx_allowed::all;
	uint64_t now_ms_ = 0;
	std::vector<int16_t> tx_;
};

} // namespace

mock_radio *make_mock_radio() {
	return new mock_impl();
}

} // namespace adsld

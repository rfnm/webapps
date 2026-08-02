#include "receiver.h"

#include <cmath>

namespace aisd {

channel_receiver::channel_receiver(double freq_hz, char channel, double out_rate,
                                   std::function<void(const ais_packet &)> on_packet)
	: freq_hz_(freq_hz), channel_(channel), sps_(out_rate / AIS_BAUD),
	  on_packet_(std::move(on_packet)) {
	phases_.reserve(PHASES);
	for(int j = 0; j < PHASES; j++) {
		phases_.emplace_back([this](const bits &m) { on_wire_msg(m); });
		// spread the K strobes evenly across one symbol; each advances by sps thereafter,
		// so together they cover every sub-symbol alignment a burst can arrive at
		phases_.back().t_strobe = sps_ * 2.0 + (double)j * sps_ / PHASES;
	}
}

// linear interpolation of the cum ring at absolute (fractional) time t
static inline double interp_cum(const double *ring, int cap, double t) {
	long i = (long)std::floor(t);
	double f = t - (double)i;
	int a = (int)(((i % cap) + cap) % cap);
	int b = (int)((((i + 1) % cap) + cap) % cap);
	return ring[a] * (1.0 - f) + ring[b] * f;
}

void channel_receiver::feed(const cf *samples, size_t count) {
	for(size_t i = 0; i < count; i++) {
		cf x = samples[i];
		mag_ema_ += 0.001f * (std::abs(x) - mag_ema_);
		if(!have_prev_) {
			prev_ = x;
			have_prev_ = true;
			continue;
		}
		// FM discriminator: instantaneous frequency = phase of x * conj(prev). The running
		// integral lets each hypothesis read out the phase change across its symbol window
		// (an integrate-and-dump matched filter) by interpolating two points of the sum.
		double dd = std::arg(x * std::conj(prev_));
		prev_ = x;
		cum_ += dd;
		cum_ring_[n_ % RING] = cum_;
		n_++;

		// once a strobe centre plus a half-symbol of look-ahead is buffered, dump its symbol
		for(auto &p : phases_) {
			while((double)(n_ - 1) >= p.t_strobe + sps_ / 2.0 + 1.0) {
				double acc = interp_cum(cum_ring_, RING, p.t_strobe + sps_ / 2.0) -
				             interp_cum(cum_ring_, RING, p.t_strobe - sps_ / 2.0);
				int level = acc > 0 ? 1 : 0;
				p.hdlc.push_bit(level == p.prev_line ? 1 : 0);   // NRZI: no transition -> 1
				p.prev_line = level;
				p.t_strobe += sps_;
			}
		}
	}
}

void channel_receiver::on_wire_msg(const bits &wire_msg) {
	// suppress the same frame arriving from several adjacent timing hypotheses
	for(int k = 0; k < 4; k++) {
		if(!last_msgs_[k].empty() && last_msgs_[k] == wire_msg && n_ - last_n_[k] < (long)(400 * sps_)) {
			return;
		}
	}
	last_msgs_[last_pos_] = wire_msg;
	last_n_[last_pos_] = n_;
	last_pos_ = (last_pos_ + 1) % 4;

	ais_packet pkt;
	pkt.freq_hz = freq_hz_;
	pkt.channel = channel_;
	pkt.level_db = 20.0f * std::log10(mag_ema_ + 1e-9f);
	pkt.logical = wire_to_logical(wire_msg);
	pkt.type = (int)bits_get_uint(pkt.logical, 0, 6);
	pkt.mmsi = (uint32_t)bits_get_uint(pkt.logical, 8, 30);
	on_packet_(pkt);
}

} // namespace aisd

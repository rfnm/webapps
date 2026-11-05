// adsld: per-channel receiver implementation - see receiver.h for the architecture
// (FM discriminator -> K-phase integrate-and-dump chip bank -> sync64 correlator ->
// block capture -> differential Manchester soft decision -> CRC24 selects length and phase).
#include "receiver.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

namespace adsld {

namespace {

// linear interpolation of the cum ring at absolute (fractional) time t
inline double interp_cum(const double *ring, int cap, double t) {
	long i = (long)std::floor(t);
	double f = t - (double)i;
	int a = (int)(((i % cap) + cap) % cap);
	int b = (int)((((i + 1) % cap) + cap) % cap);
	return ring[a] * (1.0 - f) + ring[b] * f;
}

// ADSL_SYNC64 packed into one uint64, first on-air chip in the MSB - matches a shift
// register that shifts left and appends the newest hard chip at the LSB
constexpr uint64_t sync64_traffic() {
	uint64_t p = 0;
	for(int i = 0; i < 8; i++) {
		p = (p << 8) | ADSL_SYNC64[i];
	}
	return p;
}

// Manchester(b) as 16 chips, first on-air chip in the MSB (1 -> 01, 0 -> 10)
constexpr uint64_t manchester16(uint8_t b) {
	uint64_t p = 0;
	for(int k = 7; k >= 0; k--) {
		p = (p << 2) | (((b >> k) & 1) ? 1u : 2u);
	}
	return p;
}

// The 64-chip detection pattern for a given LEN byte: the preamble tail and the sync word
// (48 chips, the same for every packet) followed by Manchester(LEN). LEN 0x18 rebuilds
// ADSL_SYNC64 exactly.
constexpr uint64_t sync64_pattern(uint8_t len) {
	return (sync64_traffic() & ~0xFFFFull) | manchester16(len);
}

static_assert(sync64_pattern(ADSL_LEN_TRAFFIC) == sync64_traffic(),
              "Manchester(LEN) must rebuild ADSL_SYNC64 for a Traffic frame");

} // namespace

struct adsl_receiver::impl {
	static constexpr int RING = 128;   // cum history for fractional interpolation (>= 2*sps)
	// The sync64 pattern ends at the LEN byte, so a match leaves the rest of the packet to
	// capture: 24 bytes = 384 chips for Traffic, 16 bytes = 256 chips for Status. The soft
	// buffer is sized for the longer one.
	static constexpr int CAPTURE_CHIPS = ((int)ADSL_PKT_LEN - 1) * 16;

	int channel;
	double freq_hz;
	double out_rate;
	packet_cb cb;

	double sps = 4;          // samples per chip
	float pwr_alpha = 0;     // ~1 ms power EMA coefficient

	// FM discriminator + running integral of the discriminator (integrate-and-dump source)
	cf prev{0, 0};
	bool have_prev = false;
	long n = 0;
	double cum = 0;
	double cum_ring[RING] = {0};

	// LBT observables: short-window power EMA and min-statistics noise floor. The dB
	// values are published to atomics so the scheduler thread can read them lock-free.
	static constexpr int NF_RING = 8;
	float pwr_ema = 0;
	int pub_cnt = 0;
	long subwin_len = 0;
	long subwin_cnt = 0;
	float subwin_min = 1e30f;
	float nf_ring[NF_RING] = {0};
	int nf_pos = 0;
	int nf_filled = 0;
	std::atomic<float> rssi_db{-120.0f};
	std::atomic<float> noise_db{-120.0f};

	// one decode hypothesis per fixed sub-chip timing phase
	struct phase_dec {
		double t_strobe = 0;   // absolute (fractional) sample time of the next chip strobe
		double dc = 0;         // chip-integral DC estimate (CFO tracker; Manchester is DC-free)
		uint64_t shift = 0;    // hard-chip shift register for the sync64 correlator
		long chips_seen = 0;   // chips since last reset; need 64 before correlating
		int cap_n = -1;        // -1 = hunting for sync, else soft chips captured so far
		int cap_chips = 0;     // chips to capture: enough for the longest candidate length
		uint8_t cand[2] = {0}; // LEN candidates for the CRC to decide, most likely first
		int cand_n = 0;
		float soft[CAPTURE_CHIPS] = {0};
		float len_soft[16] = {0};   // last 16 soft chips while hunting: the LEN byte at a hit
		int len_pos = 0;            // ring index of the oldest entry of len_soft
	};
	phase_dec phases[PHASES];

	// duplicate suppression: adjacent phases decode the same burst within a few chips
	uint8_t last_pkts[4][ADSL_PKT_LEN] = {{0}};
	bool last_valid[4] = {false, false, false, false};
	long last_n[4] = {0, 0, 0, 0};
	int last_pos = 0;

	void feed(const cf *samples, size_t count);
	void on_chip(int pi, double acc);
	void finish(int pi);
};

adsl_receiver::adsl_receiver(int channel, double freq_hz, double out_rate, packet_cb cb)
	: im_(new impl) {
	im_->channel = channel;
	im_->freq_hz = freq_hz;
	im_->out_rate = out_rate;
	im_->cb = std::move(cb);
	im_->sps = out_rate / ADSL_CHIP_RATE_HZ;
	im_->pwr_alpha = (float)(1.0 / (1e-3 * out_rate));      // ~1 ms EMA
	im_->subwin_len = (long)llround(0.5 * out_rate);        // 0.5 s min-statistics subwindow
	for(int j = 0; j < PHASES; j++) {
		// spread the K strobes evenly across one chip; each advances by sps thereafter,
		// so together they cover every sub-chip alignment a burst can arrive at
		im_->phases[j].t_strobe = im_->sps * 2.0 + (double)j * im_->sps / PHASES;
	}
}

adsl_receiver::~adsl_receiver() {
	delete im_;
}

void adsl_receiver::feed(const cf *samples, size_t count) {
	im_->feed(samples, count);
}

void adsl_receiver::impl::feed(const cf *samples, size_t count) {
	for(size_t i = 0; i < count; i++) {
		cf x = samples[i];

		float p = x.real() * x.real() + x.imag() * x.imag();
		pwr_ema += pwr_alpha * (p - pwr_ema);
		if(++pub_cnt >= 64) {
			pub_cnt = 0;
			rssi_db.store(10.0f * std::log10(pwr_ema + 1e-20f), std::memory_order_relaxed);
		}
		// min-statistics noise floor: the minimum of the 1 ms power EMA per subwindow,
		// then the minimum over the last NF_RING subwindows - a burst (<= 5 ms) can never
		// drag it down, quiet air between bursts always can
		subwin_min = std::min(subwin_min, pwr_ema);
		if(++subwin_cnt >= subwin_len) {
			subwin_cnt = 0;
			nf_ring[nf_pos] = subwin_min;
			nf_pos = (nf_pos + 1) % NF_RING;
			nf_filled = std::min(nf_filled + 1, NF_RING);
			subwin_min = 1e30f;
			float m = 1e30f;
			for(int k = 0; k < nf_filled; k++) {
				m = std::min(m, nf_ring[k]);
			}
			noise_db.store(10.0f * std::log10(m + 1e-20f), std::memory_order_relaxed);
		}

		if(!have_prev) {
			prev = x;
			have_prev = true;
			continue;
		}
		// FM discriminator: instantaneous frequency = phase of x * conj(prev). The running
		// integral lets each hypothesis read out the phase change across its chip window
		// (an integrate-and-dump matched filter) by interpolating two points of the sum.
		double dd = std::arg(x * std::conj(prev));
		prev = x;
		cum += dd;
		cum_ring[n % RING] = cum;
		n++;

		// once a strobe centre plus a half-chip of look-ahead is buffered, dump its chip
		for(int pi = 0; pi < PHASES; pi++) {
			phase_dec &ph = phases[pi];
			while((double)(n - 1) >= ph.t_strobe + sps / 2.0 + 1.0) {
				double acc = interp_cum(cum_ring, RING, ph.t_strobe + sps / 2.0) -
				             interp_cum(cum_ring, RING, ph.t_strobe - sps / 2.0);
				on_chip(pi, acc);
				ph.t_strobe += sps;
			}
		}
	}
}

void adsl_receiver::impl::on_chip(int pi, double acc) {
	phase_dec &ph = phases[pi];
	// DC estimate (slow EMA of the chip integrals): preamble + Manchester coding keep the
	// chip stream DC-balanced, so any mean is CFO - subtract it before hard slicing. The
	// bit decisions later are differential and cancel whatever residual is left.
	double soft = acc - ph.dc;
	ph.dc += (acc - ph.dc) * (1.0 / 64.0);

	if(ph.cap_n >= 0) {
		ph.soft[ph.cap_n++] = (float)soft;
		if(ph.cap_n >= ph.cap_chips) {
			finish(pi);
		}
		return;
	}

	ph.len_soft[ph.len_pos] = (float)soft;
	ph.len_pos = (ph.len_pos + 1) & 15;
	ph.shift = (ph.shift << 1) | (uint64_t)(soft > 0 ? 1 : 0);
	ph.chips_seen++;
	if(ph.chips_seen < ADSL_SYNC64_CHIPS) {
		return;
	}
	// preamble tail + sync word first: these 48 chips are the same for every length, and a
	// miss here is by far the common case, so it costs one popcount
	constexpr uint64_t HEAD = ~0xFFFFull;
	if(__builtin_popcountll((ph.shift ^ sync64_traffic()) & HEAD) > SYNC_MAX_CHIP_ERRORS) {
		return;
	}
	// The Traffic (0x18) and Status (0x10) patterns differ in only the two chips of LEN bit
	// 3, fewer than the SYNC_MAX_CHIP_ERRORS a match tolerates, so the hard chips cannot
	// always tell the lengths apart. Whenever both are in reach, capture enough for the
	// longer one and let the CRC decide in finish().
	const int d_traffic = __builtin_popcountll(ph.shift ^ sync64_pattern(ADSL_LEN_TRAFFIC));
	const int d_status = __builtin_popcountll(ph.shift ^ sync64_pattern(ADSL_LEN_STATUS));
	const bool ok_traffic = d_traffic <= SYNC_MAX_CHIP_ERRORS;
	const bool ok_status = d_status <= SYNC_MAX_CHIP_ERRORS;
	if(!ok_traffic && !ok_status) {
		return;
	}
	// candidate order: the closer pattern first; a tie by the differential soft decision on
	// the LEN byte, bit = sign(second chip - first chip), oldest chip at len_pos
	bool status_first = ok_status && (!ok_traffic || d_status < d_traffic);
	if(ok_status && ok_traffic && d_status == d_traffic) {
		uint8_t soft_len = 0;
		for(int k = 0; k < 8; k++) {
			float c0 = ph.len_soft[(ph.len_pos + 2 * k) & 15];
			float c1 = ph.len_soft[(ph.len_pos + 2 * k + 1) & 15];
			soft_len = (uint8_t)((soft_len << 1) | (c1 > c0 ? 1 : 0));
		}
		status_first = soft_len == ADSL_LEN_STATUS;
	}
	ph.cand_n = 0;
	if(status_first) {
		ph.cand[ph.cand_n++] = ADSL_LEN_STATUS;
		if(ok_traffic) {
			ph.cand[ph.cand_n++] = ADSL_LEN_TRAFFIC;
		}
	} else {
		ph.cand[ph.cand_n++] = ADSL_LEN_TRAFFIC;
		if(ok_status) {
			ph.cand[ph.cand_n++] = ADSL_LEN_STATUS;
		}
	}
	ph.cap_chips = ((int)(ok_traffic ? ADSL_PKT_LEN : ADSL_STATUS_PKT_LEN) - 1) * 16;
	ph.cap_n = 0;  // sync: block-capture the remaining packet chips
}

void adsl_receiver::impl::finish(int pi) {
	phase_dec &ph = phases[pi];
	// back to hunting; require 64 fresh chips so the stale register cannot re-trigger
	ph.cap_n = -1;
	ph.shift = 0;
	ph.chips_seen = 0;

	// differential Manchester decision on the soft chip integrals: bit 1 flew as chips
	// "01", bit 0 as "10", so bit = sign(second integral - first) - CFO-immune
	const size_t nbytes = (size_t)ph.cap_chips / 16;   // bytes captured after the LEN byte
	uint8_t raw[ADSL_PKT_LEN] = { 0 };
	for(size_t b = 0; b < nbytes; b++) {
		uint8_t v = 0;
		for(int k = 0; k < 8; k++) {
			size_t bit_i = b * 8 + (size_t)k;
			v = (uint8_t)((v << 1) | (ph.soft[2 * bit_i + 1] > ph.soft[2 * bit_i] ? 1 : 0));
		}
		raw[1 + b] = v;
	}

	// CRC24 selects the length and the winning hypothesis: an exact CRC on any candidate
	// first, then a single-bit syndrome patch in candidate order. The CRC covers NetHdr +
	// data, i.e. the LEN value minus the three CRC bytes.
	uint8_t pkt[ADSL_PKT_LEN] = { 0 };
	bool found = false;
	bool corrected = false;
	for(int pass = 0; pass < 2 && !found; pass++) {
		for(int i = 0; i < ph.cand_n && !found; i++) {
			const size_t len = adsl_onair_len(ph.cand[i]);
			const size_t span = len - 4;
			uint8_t trial[ADSL_PKT_LEN] = { 0 };
			trial[0] = ph.cand[i];
			memcpy(trial + 1, raw + 1, len - 1);
			const uint32_t rx_crc = ((uint32_t)trial[span + 1] << 16) |
			                        ((uint32_t)trial[span + 2] << 8) | trial[span + 3];
			if(pass == 0) {
				if(crc24(trial + 1, span) != rx_crc) {
					continue;
				}
			} else {
				if(crc24_correct_1bit(trial + 1, span + 3) < 0) {
					continue;
				}
				corrected = true;
			}
			memcpy(pkt, trial, sizeof(pkt));
			found = true;
		}
	}
	if(!found) {
		return;
	}

	// suppress the same frame arriving from several adjacent timing hypotheses
	long win = (long)(2.0 * ADSL_FRAME_CHIPS * sps);
	for(int k = 0; k < 4; k++) {
		if(last_valid[k] && n - last_n[k] < win &&
		   memcmp(last_pkts[k], pkt, ADSL_PKT_LEN) == 0) {
			return;
		}
	}
	memcpy(last_pkts[last_pos], pkt, ADSL_PKT_LEN);
	last_valid[last_pos] = true;
	last_n[last_pos] = n;
	last_pos = (last_pos + 1) % 4;

	adsl_rx_meta meta;
	meta.channel = channel;
	meta.freq_hz = freq_hz;
	meta.phase = pi;
	meta.rssi_db = 10.0f * std::log10(pwr_ema + 1e-20f);
	meta.corrected = corrected;
	cb(pkt, meta);
}

float adsl_receiver::rssi_db() const {
	return im_->rssi_db.load(std::memory_order_relaxed);
}

float adsl_receiver::noise_floor_db() const {
	return im_->noise_db.load(std::memory_order_relaxed);
}

} // namespace adsld

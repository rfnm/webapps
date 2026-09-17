// elrsd: TX chain - chirp synthesis at the device rate, TX policy, the mock radio,
// the hop planner and the slot-loop transmitter, plus run_selftest_tx.
// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp
//
// Frame layout synthesized here (measured on the 2026-09-14 captures with
// a local probe script, quarter-symbol dechirp):
//   preamble_len up-chirps (value 0)
//   2 sync-word up-chirps with values 8 and 16   = ((0x12>>4)&0xF)<<3, (0x12&0xF)<<3
//                                                  SF5, SF6, SF7 (ref+8, ref+16 on every burst probed)
//   2.25 down-chirps (SFD)                         (down-dechirp peaks at symbols pre+2, pre+3, quarter at pre+4)
//   SF5/SF6 only: 2 up-chirps with value 1        SF5 + SF6: the two symbols after the SFD demodulate as
//                                                  ref+1 on every burst, i.e. a payload-style symbol with v = 0
//   n_sym payload symbols (li_codec::encode, ref 0: s = gray_decode(v) << (sf-w) + 1)
// -> payload start = preamble_len + 2 + 2.25 (+2) = rate_params::frame_offset_symbols (18.25 / 20.25 / 16.25).
//
// Chirp convention = css_modem.py / rx_chain: base up-chirp phase phi(t) = 2*pi*(bw/(2T) t^2 - bw/2 t),
// t in [0, T), T = 2^sf / bw; symbol s = cyclic shift by s*T/M (the dechirped tone lands in FFT bin s).
// Down-chirp = conjugate. Generated at the DEVICE rate (25 MSps, os = 30.77 non-integer) with a
// second-order phase recursion per linear frequency piece (2 complex multiplies per sample, double
// precision) - no sin/cos per sample, no resampling, no dependency on B's css_modem.
#include "tx_chain.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <numeric>
#include <thread>
#include "rfnm_boardconf.h"
#include "selftest.h"

namespace elrsd {

// =====================================================================================
// burst_synth
// =====================================================================================

namespace {

// one linear-frequency piece of the burst: f(tau) = f0 + k * tau for tau in [0, dur), phase phi0 at tau = 0
struct chirp_piece {
	double t_start;   // seconds from burst start
	double dur;
	double f0;        // Hz at piece start (baseband, before the carrier offset)
	double k;         // Hz/s
	double phi0;      // radians at piece start
};

// phase of the base up-chirp at local time tau in [0, T)
static inline double upchirp_phase(double tau, double bw, double T) {
	return 2.0 * M_PI * (bw / (2.0 * T) * tau * tau - 0.5 * bw * tau);
}

// append the pieces of one symbol: up-chirp of value s (or down-chirp, value 0) lasting `frac` symbols
static void add_symbol(std::vector<chirp_piece> &pcs, double &t, int sf, double bw, double T, bool up, int s, double frac) {
	const int M = 1 << sf;
	const double dur_total = frac * T;
	double shift = (double)(s % M) * T / M;      // cyclic shift of the base chirp
	double sign = up ? 1.0 : -1.0;
	// the symbol starts at base-chirp local time `shift` and runs to the end of the symbol
	double consumed = 0.0;
	double tau = shift;
	while(consumed < dur_total - 1e-15) {
		double remain_in_base = T - tau;
		double dur = std::min(remain_in_base, dur_total - consumed);
		chirp_piece p;
		p.t_start = t + consumed;
		p.dur = dur;
		p.f0 = sign * (bw * tau / T - 0.5 * bw);
		p.k = sign * bw / T;
		p.phi0 = sign * upchirp_phase(tau, bw, T);
		pcs.push_back(p);
		consumed += dur;
		tau = 0.0;   // wrapped to the start of the base chirp
	}
	t += dur_total;
}

} // namespace

burst_synth::burst_synth(const rate_params &rp, const burst_synth_config &cfg) : rp_(rp), cfg_(cfg) {
	if(cfg_.amplitude > 14000) cfg_.amplitude = 14000;   // fleet ceiling, never above
	if(cfg_.amplitude < 1) cfg_.amplitude = 1;
}

size_t burst_synth::burst_samples() const {
	return (size_t)std::llround(rp_.burst_s() * cfg_.fs_hz);
}

size_t burst_synth::synth(const std::vector<uint16_t> &symbols, double baseband_offset_hz, std::vector<int16_t> &out) const {
	const int sf = rp_.sf;
	const double bw = cfg_.bw_hz;
	const double T = (double)(1 << sf) / bw;
	const double fs = cfg_.fs_hz;
	std::vector<chirp_piece> pcs;
	pcs.reserve(2 * (rp_.preamble_len + 6 + rp_.n_sym) + 4);
	double t = 0.0;
	for(int i = 0; i < rp_.preamble_len; i++) add_symbol(pcs, t, sf, bw, T, true, 0, 1.0);
	// sync word 0x12 (SX1280 private default) -> up-chirps 8 and 16 (measured, see file header)
	add_symbol(pcs, t, sf, bw, T, true, ((0x12 >> 4) & 0xF) << 3, 1.0);
	add_symbol(pcs, t, sf, bw, T, true, (0x12 & 0xF) << 3, 1.0);
	// SFD: 2.25 down-chirps
	add_symbol(pcs, t, sf, bw, T, false, 0, 1.0);
	add_symbol(pcs, t, sf, bw, T, false, 0, 1.0);
	add_symbol(pcs, t, sf, bw, T, false, 0, 0.25);
	if(rp_.fine_sync) {
		// SF5/SF6: two symbols carrying v = 0 (ref + 1)
		add_symbol(pcs, t, sf, bw, T, true, 1, 1.0);
		add_symbol(pcs, t, sf, bw, T, true, 1, 1.0);
	}
	size_t n_sym = (size_t)rp_.n_sym;
	for(size_t i = 0; i < n_sym; i++) {
		int s = i < symbols.size() ? (int)symbols[i] : 0;
		add_symbol(pcs, t, sf, bw, T, true, s, 1.0);
	}
	const size_t n = burst_samples();
	out.assign(2 * n, 0);
	const double A = (double)cfg_.amplitude;
	const double inv = cfg_.iq_invert ? -1.0 : 1.0;   // invertIQ mirrors the chirp (uid[5] & 1)
	const double dt = 1.0 / fs;
	size_t nidx = 0;
	for(size_t pi = 0; pi < pcs.size() && nidx < n; pi++) {
		const chirp_piece &p = pcs[pi];
		double t_end = p.t_start + p.dur;
		size_t n_end = (size_t)std::ceil(t_end * fs - 1e-9);
		if(n_end > n) n_end = n;
		if(n_end <= nidx) continue;
		// closed-form phase/frequency at the first sample of the piece, then the recursion
		double tau0 = nidx * dt - p.t_start;
		double phi = inv * (p.phi0 + 2.0 * M_PI * (p.f0 * tau0 + 0.5 * p.k * tau0 * tau0)) + 2.0 * M_PI * baseband_offset_hz * nidx * dt;
		double f_now = inv * (p.f0 + p.k * tau0) + baseband_offset_hz;
		double dphi = 2.0 * M_PI * (f_now * dt + 0.5 * inv * p.k * dt * dt);
		std::complex<double> z(std::cos(phi), std::sin(phi));
		std::complex<double> w(std::cos(dphi), std::sin(dphi));
		const double ddphi = 2.0 * M_PI * inv * p.k * dt * dt;
		const std::complex<double> c(std::cos(ddphi), std::sin(ddphi));
		for(size_t i = nidx; i < n_end; i++) {
			out[2 * i] = (int16_t)std::lround(A * z.real());
			out[2 * i + 1] = (int16_t)std::lround(A * z.imag());
			z *= w;
			w *= c;
		}
		nidx = n_end;
	}
	return n;
}

// =====================================================================================
// TX policy (board.conf) - the canonical parser lives in common/rfnm_boardconf.h
// =====================================================================================

const char *tx_policy_str(tx_policy p) {
	return p == tx_policy::ALL ? "all" : (p == tx_policy::EMBED ? "embed" : "none");
}

tx_policy tx_policy_from_boardconf(const char *path, std::string *board) {
	char b[64];
	enum vsg_tx_policy p = vsg_boardconf_parse(path ? path : "/rfnm/board.conf", b, sizeof(b));
	if(board) *board = b;
	return p == VSG_TXPOL_ALL ? tx_policy::ALL : (p == VSG_TXPOL_EMBED ? tx_policy::EMBED : tx_policy::NONE);
}

// =====================================================================================
// radio_tx_mock
// =====================================================================================

// =====================================================================================
// tx_feed - the continuous feed core (host-testable, see tx_chain.h)
// =====================================================================================

struct tx_feed::impl {
	mutable std::mutex mtx;
	struct burst { uint64_t pos; std::vector<int16_t> iq; size_t done; };
	std::deque<burst> q;
	uint64_t frontier = 0;
	uint64_t last_end = 0;          // end of the last queued burst (queue order = position order)
	double tone_step = 0;           // radians per sample (0 = zeros)
	double tone_phase = 0;
	int16_t tone_amp = 0;
	stats st;
	static const size_t MAX_Q = 64;
};

tx_feed::tx_feed(uint64_t frontier) : p_(new impl) { p_->frontier = frontier; p_->last_end = frontier; }

void tx_feed::set_idle_tone(double cycles_per_sample, int16_t amplitude) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->tone_amp = amplitude;
	p_->tone_step = amplitude ? 2.0 * M_PI * cycles_per_sample : 0.0;
}

bool tx_feed::queue_burst(uint64_t pos, const int16_t *iq, size_t samples, std::string *err) {
	std::lock_guard<std::mutex> l(p_->mtx);
	auto fail = [&](const char *w) { if(err) *err = w; return false; };
	if(!iq || !samples) return fail("empty burst");
	if(pos < p_->frontier) { p_->st.refused_late++; return fail("late: position behind the feed frontier"); }
	if(pos < p_->last_end) { p_->st.refused_late++; return fail("overlap: position inside the previous burst"); }
	if(p_->q.size() >= impl::MAX_Q) return fail("burst queue full");
	impl::burst b;
	b.pos = pos;
	b.iq.assign(iq, iq + 2 * samples);
	b.done = 0;
	p_->q.push_back(std::move(b));
	p_->last_end = pos + samples;
	p_->st.queued++;
	return true;
}

size_t tx_feed::next(int16_t *out, size_t max_samples) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(!max_samples) return 0;
	size_t n;
	if(!p_->q.empty() && p_->q.front().pos <= p_->frontier) {
		// inside (or exactly at the start of) the next burst: emit its remainder
		impl::burst &b = p_->q.front();
		size_t total = b.iq.size() / 2;
		size_t off = (size_t)(p_->frontier - b.pos);      // == b.done by construction
		n = std::min(max_samples, total - off);
		memcpy(out, b.iq.data() + 2 * off, n * 4);
		b.done = off + n;
		p_->st.burst_samples += n;
		if(b.done >= total) { p_->q.pop_front(); p_->st.fed++; }
	} else {
		// idle fill up to the next burst start
		n = max_samples;
		if(!p_->q.empty()) n = (size_t)std::min<uint64_t>(n, p_->q.front().pos - p_->frontier);
		if(p_->tone_amp) {
			for(size_t i = 0; i < n; i++) {
				out[2 * i] = (int16_t)std::lrint((double)p_->tone_amp * std::cos(p_->tone_phase));
				out[2 * i + 1] = (int16_t)std::lrint((double)p_->tone_amp * std::sin(p_->tone_phase));
				p_->tone_phase += p_->tone_step;
				if(p_->tone_phase > 2.0 * M_PI) p_->tone_phase -= 2.0 * M_PI;
			}
		} else {
			memset(out, 0, n * 4);
		}
		p_->st.zero_samples += n;
	}
	p_->frontier += n;
	if(p_->last_end < p_->frontier) p_->last_end = p_->frontier;
	return n;
}

uint64_t tx_feed::frontier() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->frontier; }
size_t tx_feed::pending() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->q.size(); }
tx_feed::stats tx_feed::get_stats() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->st; }
void tx_feed::reset(uint64_t frontier) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->q.clear();
	p_->frontier = frontier;
	p_->last_end = frontier;
	p_->tone_phase = 0;
}

// =====================================================================================
// radio_tx_mock - the host adapter on the same tx_feed core the rfnm adapter uses
// =====================================================================================

struct radio_tx_mock::impl {
	mutable std::mutex mtx;
	tx_policy policy = tx_policy::ALL;
	std::vector<std::string> possible{"sma_a", "embed_ant"};
	std::string preferred = "sma_a";
	int pmin = -60, pmax = 0;
	long retune_us = 200;
	uint64_t fake_now = 0;
	bool fake_now_set = false;
	std::chrono::steady_clock::time_point t_open = std::chrono::steady_clock::now();
	// session
	bool running = false;           // RF up
	bool session = false;           // feed session (anchor + feeder)
	uint64_t t0_ext = 0;
	uint32_t epoch = 0;
	bool anchor_moved = false;
	int session_starts = 0;
	double fs = 25e6;
	double freq_lo = 0;
	double tone_offset_hz = 0;
	int16_t tone_amp = 0;
	int power_applied = 0;
	std::string path_applied;
	tx_health health;
	tx_feed feed{0};
	std::vector<mock_tx_event> events;
	std::vector<double> retunes_us;
	static const uint64_t TICK_HZ = 61440000ULL;   // dcs_clk/2 of the 122.88 MHz plan (default)
	uint64_t tick_hz = TICK_HZ;                     // configurable (set_clock) to model R = 4/2 etc.
	uint32_t rnum_override = 0, rden_override = 0;  // report an unreduced ratio (e.g. 4/2) when set
	int64_t feed_axis_skew = 0;                     // pos0 offset from the tick axis (models the observed re-mint)
	uint32_t lead_ticks() const { return (uint32_t)(tick_hz / 500); }   // 2 ms honest feed lead at tick_hz

	uint64_t now_tick() const {
		if(fake_now_set) return fake_now;
		auto d = std::chrono::steady_clock::now() - t_open;
		return (uint64_t)(std::chrono::duration<double>(d).count() * (double)tick_hz);
	}
	void ratio(uint32_t &num, uint32_t &den) const {
		if(rnum_override && rden_override) { num = rnum_override; den = rden_override; return; }
		uint64_t a = tick_hz, b = (uint64_t)std::llround(fs);
		uint64_t g = std::gcd(a, b);
		num = (uint32_t)(a / g);
		den = (uint32_t)(b / g);
	}
	uint64_t pos_of_tick(uint64_t tick) const {
		uint32_t num, den;
		ratio(num, den);
		if(tick <= t0_ext) return 0;
		return (tick - t0_ext) * den / num;
	}
	bool refuse(const char *why) {
		health.refusals++;
		health.last_error = why;
		return false;
	}
	// anchor now, first feed position = now + lead (the pace the scheduler is told), 256-floor never
	// backward. feed_axis_skew models a device whose reclock/RX-apply re-minted the anchor so that the
	// returned pos0 no longer matches (now - t0_ext)*R (the observed anchor re-mint): t0_ext stays put, pos0 jumps.
	bool start_session_locked(uint64_t *pos0) {
		t0_ext = now_tick();
		epoch++;
		anchor_moved = false;
		uint64_t pos = pos_of_tick(now_tick() + lead_ticks());
		pos = (pos + 255) / 256 * 256;
		pos += (uint64_t)feed_axis_skew;
		feed.reset(pos);
		feed.set_idle_tone(tone_offset_hz / fs, tone_amp);
		session = true;
		session_starts++;
		if(pos0) *pos0 = pos;
		return true;
	}
	void merge_feed_stats() {
		tx_feed::stats fs_ = feed.get_stats();
		health.bursts_queued = fs_.queued;
		health.bursts_fed = fs_.fed;
		health.bursts_late = fs_.refused_late;
		health.zero_samples = fs_.zero_samples;
		health.burst_samples = fs_.burst_samples;
	}
};

radio_tx_mock::radio_tx_mock() : p_(new impl) {}
void radio_tx_mock::set_policy(tx_policy p) { std::lock_guard<std::mutex> l(p_->mtx); p_->policy = p; }
void radio_tx_mock::set_paths(const std::vector<std::string> &possible, const std::string &preferred) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->possible = possible;
	p_->preferred = preferred;
}
void radio_tx_mock::set_power_range(int min_dbm, int max_dbm) { std::lock_guard<std::mutex> l(p_->mtx); p_->pmin = min_dbm; p_->pmax = max_dbm; }
void radio_tx_mock::set_retune_us(long us) { std::lock_guard<std::mutex> l(p_->mtx); p_->retune_us = us; }
void radio_tx_mock::set_fake_now(uint64_t tick) { std::lock_guard<std::mutex> l(p_->mtx); p_->fake_now = tick; p_->fake_now_set = true; }
void radio_tx_mock::set_clock(uint64_t tick_hz, uint32_t rnum, uint32_t rden) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->tick_hz = tick_hz ? tick_hz : impl::TICK_HZ;
	p_->rnum_override = rnum;
	p_->rden_override = rden;
}
void radio_tx_mock::set_feed_axis_skew(int64_t samples) { std::lock_guard<std::mutex> l(p_->mtx); p_->feed_axis_skew = samples; }
const std::vector<mock_tx_event> &radio_tx_mock::captured() const { return p_->events; }
std::vector<double> radio_tx_mock::retunes() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->retunes_us; }
tx_policy radio_tx_mock::policy() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->policy; }
void radio_tx_mock::set_anchor_moved(bool moved) { std::lock_guard<std::mutex> l(p_->mtx); p_->anchor_moved = moved; }
int radio_tx_mock::session_starts() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->session_starts; }

size_t radio_tx_mock::drain(std::vector<int16_t> &out, size_t max_samples) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(!p_->session) return 0;
	size_t got = 0;
	std::vector<int16_t> chunk(2 * 1024);
	while(got < max_samples) {
		size_t n = p_->feed.next(chunk.data(), std::min<size_t>(1024, max_samples - got));
		if(!n) break;
		out.insert(out.end(), chunk.begin(), chunk.begin() + 2 * n);
		got += n;
	}
	p_->merge_feed_stats();
	return got;
}

bool radio_tx_mock::start(const tx_start_params &p, uint64_t *pos0) {
	std::lock_guard<std::mutex> l(p_->mtx);
	// ---- the adapter's own gates (never trust the caller) ----
	if(p_->policy == tx_policy::NONE) return p_->refuse("policy none: TX refused by the adapter");
	if(p.power_dbm > p.power_cap_dbm) return p_->refuse("power above cap: refused by the adapter");
	std::string path;
	if(p_->policy == tx_policy::EMBED) {
		if(std::find(p_->possible.begin(), p_->possible.end(), "embed_ant") == p_->possible.end())
			return p_->refuse("policy embed but no embed_ant path on this board: refused");
		path = "embed_ant";
	} else {
		if(std::find(p_->possible.begin(), p_->possible.end(), p.antenna) != p_->possible.end()) path = p.antenna;
		else path = p_->preferred;
	}
	if(p_->running) return p_->refuse("already running");
	// the clamp below never raises the power above the cap: a board whose minimum lies above it is refused
	if(p_->pmin > p.power_cap_dbm) return p_->refuse("power_range.min above the cap: refused by the adapter");
	int pw = std::max(p_->pmin, std::min(std::min(p_->pmax, p.power_cap_dbm), p.power_dbm));
	p_->fs = p.fs_hz;
	p_->freq_lo = p.freq_lo_hz;
	p_->power_applied = pw;
	p_->path_applied = path;
	p_->running = true;
	p_->session = false;
	p_->events.clear();
	p_->health = tx_health();
	if(pos0) *pos0 = 0;
	if(!p.start_session) return true;
	return p_->start_session_locked(pos0);
}

bool radio_tx_mock::session_start(uint64_t *pos0) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(p_->policy == tx_policy::NONE) return p_->refuse("policy none: session refused by the adapter");
	if(!p_->running) return p_->refuse("session_start: RF not up");
	if(p_->session) return p_->refuse("session already running");
	return p_->start_session_locked(pos0);
}

bool radio_tx_mock::session_restart(uint64_t *pos0) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(p_->policy == tx_policy::NONE) return p_->refuse("policy none: session refused by the adapter");
	if(!p_->running) return p_->refuse("session_restart: RF not up");
	p_->health.rebases++;
	p_->session = false;
	return p_->start_session_locked(pos0);
}

void radio_tx_mock::stop() {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->running = false;
	p_->session = false;
}
bool radio_tx_mock::running() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->running; }
bool radio_tx_mock::session_running() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->session; }

bool radio_tx_mock::write_at(uint64_t pos, const int16_t *iq, size_t samples) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(p_->policy == tx_policy::NONE) return p_->refuse("policy none: write refused by the adapter");
	if(!p_->running) return p_->refuse("not running");
	if(!p_->session) return p_->refuse("no feed session");
	if(!iq && samples) return p_->refuse("null buffer");
	std::string err;
	if(!p_->feed.queue_burst(pos, iq, samples, &err)) { p_->health.last_error = err; p_->merge_feed_stats(); return false; }
	mock_tx_event ev;
	ev.pos = pos;
	ev.freq_lo_hz = p_->freq_lo;
	ev.iq.assign(iq, iq + 2 * samples);
	p_->events.push_back(std::move(ev));
	p_->health.writes++;
	p_->health.packets += (samples + 255) / 256;
	p_->merge_feed_stats();
	return true;
}

bool radio_tx_mock::flush(bool end_of_burst) {
	std::lock_guard<std::mutex> l(p_->mtx);
	(void)end_of_burst;
	return p_->session;
}

uint64_t radio_tx_mock::pos() { std::lock_guard<std::mutex> l(p_->mtx); return p_->feed.frontier(); }

void radio_tx_mock::set_idle_tone(double offset_hz, int16_t amplitude) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->tone_offset_hz = offset_hz;
	p_->tone_amp = amplitude;
	p_->feed.set_idle_tone(offset_hz / p_->fs, amplitude);
}

long radio_tx_mock::retune(double freq_lo_hz, bool *timeline_broken) {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(timeline_broken) *timeline_broken = false;
	if(p_->policy == tx_policy::NONE) { p_->refuse("policy none"); return -1; }
	p_->freq_lo = freq_lo_hz;
	if(p_->fake_now_set) p_->fake_now += (uint64_t)p_->retune_us * p_->tick_hz / 1000000ULL;
	p_->retunes_us.push_back((double)p_->retune_us);
	return p_->retune_us;
}

bool radio_tx_mock::timing(tx_timing &out) {
	std::lock_guard<std::mutex> l(p_->mtx);
	out.tick_hz = p_->tick_hz;
	p_->ratio(out.r_num, out.r_den);
	out.t0_ext = p_->t0_ext;
	out.epoch = p_->epoch;
	out.feed_lead_ticks = p_->lead_ticks();
	out.now_tick = p_->now_tick();
	out.anchor_moved = p_->anchor_moved;
	if(p_->anchor_moved) p_->health.anchor_moves++;
	p_->anchor_moved = false;
	return p_->running && p_->session;
}

void radio_tx_mock::health(tx_health &out) { std::lock_guard<std::mutex> l(p_->mtx); p_->merge_feed_stats(); out = p_->health; }
int radio_tx_mock::power_applied_dbm() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->power_applied; }
std::string radio_tx_mock::path_applied() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->path_applied; }
bool radio_tx_mock::power_range(int *min_dbm, int *max_dbm) const {
	std::lock_guard<std::mutex> l(p_->mtx);
	if(min_dbm) *min_dbm = p_->pmin;
	if(max_dbm) *max_dbm = p_->pmax;
	return true;
}
std::string radio_tx_mock::describe() const {
	std::lock_guard<std::mutex> l(p_->mtx);
	return std::string("mock(policy=") + tx_policy_str(p_->policy) + ")";
}

#ifndef WITH_RFNM
// host build: there is no librfnm adapter (the daemon answers 409 "no tx radio")
radio_tx *make_rfnm_tx(void *, tx_policy) { return nullptr; }
#endif

// =====================================================================================
// hop_planner
// =====================================================================================

hop_planner::hop_planner(const tx_plan_config &cfg, const rate_params &rp) : cfg_(cfg), rp_(rp) {
	cfg_.wide_window = false;   // never active without a measurement
}

uint64_t hop_planner::slot_samples() const {
	return (uint64_t)std::llround((double)rp_.interval_us * cfg_.fs_hz / 1e6);
}

// LO for the longest run of upcoming dwells that fits into one window; prefers an LO that keeps
// every channel of that run >= lo_min_offset_hz away from the LO leak (-12..-15 dBc).
double hop_planner::choose_lo(const std::array<uint8_t, FHSS_SEQ_LEN> &seq, int ptr, int n_dwells, const tx_plan_config &cfg) {
	const double usable = cfg.window_hz - 2.0 * cfg.guard_hz;
	if(n_dwells < 1) n_dwells = 1;
	std::vector<double> f;
	for(int i = 0; i < n_dwells; i++) f.push_back(fhss_channel_freq_hz_sx1280(seq[(size_t)((ptr + i) % FHSS_SEQ_LEN)]));
	for(int L = n_dwells; L >= 1; L--) {
		double fmin = *std::min_element(f.begin(), f.begin() + L);
		double fmax = *std::max_element(f.begin(), f.begin() + L);
		if(fmax - fmin > usable) continue;
		// feasible LO interval for these L channels
		double lo_lo = fmax - usable / 2, lo_hi = fmin + usable / 2;
		double centre = 0.5 * (fmin + fmax);
		auto leak_ok = [&](double lo) {
			for(int i = 0; i < L; i++) if(std::fabs(f[(size_t)i] - lo) < cfg.lo_min_offset_hz) return false;
			return true;
		};
		if(leak_ok(centre)) return centre;
		// walk outwards from the centre on a 100 kHz grid until every channel clears the leak zone
		for(double d = 0.1e6; d <= usable; d += 0.1e6) {
			if(centre + d <= lo_hi && leak_ok(centre + d)) return centre + d;
			if(centre - d >= lo_lo && leak_ok(centre - d)) return centre - d;
		}
		return centre;   // cannot avoid the leak zone for this run; accept (still inside the window)
	}
	return fhss_channel_freq_hz_sx1280(seq[(size_t)(ptr % FHSS_SEQ_LEN)]) + cfg.lo_min_offset_hz;
}

slot_plan hop_planner::plan_slot(const link_model &lm, int64_t slot, double current_lo_hz, uint64_t slot0_feed_pos, int64_t slot0) const {
	slot_plan sp;
	sp.slot = slot;
	sp.channel = lm.channel_at(slot);
	sp.f_ch_hz = fhss_channel_freq_hz_sx1280(sp.channel);
	sp.skip = false;
	sp.skip_reason = "";
	sp.feed_pos = slot0_feed_pos + (uint64_t)(slot - slot0) * slot_samples();
	const double half = cfg_.window_hz / 2 - cfg_.guard_hz;
	if(current_lo_hz > 0 && std::fabs(sp.f_ch_hz - current_lo_hz) <= half) {
		sp.f_lo_hz = current_lo_hz;
		sp.retune_before = false;
	} else {
		sp.f_lo_hz = choose_lo(lm.sequence(), lm.fhss_ptr_at(slot), 8, cfg_);
		sp.retune_before = true;
	}
	sp.baseband_offset_hz = sp.f_ch_hz - sp.f_lo_hz;
	return sp;
}

// =====================================================================================
// elrs_transmitter
// =====================================================================================

struct elrs_transmitter::impl {
	radio_tx &radio;
	tx_plan_config plan_cfg;
	int cap_dbm;
	bool enabled;
	mutable std::mutex mtx;
	bool threaded = true;
	std::thread thread;
	std::atomic<bool> thread_stop{false};
	std::thread::id thread_id;

	// configuration
	bool uid_set = false;
	uid_t6 uid{};
	std::array<uint16_t, 16> rc{};
	tx_arm_request req;
	// state
	bool armed = false;
	bool session = false;        // feed session + slot grid laid (arm(start_session) / session_start())
	tx_status st;
	std::chrono::steady_clock::time_point t_armed;
	// timeline
	const rate_params *rp = nullptr;
	link_model *lm = nullptr;
	hop_planner *planner = nullptr;
	burst_synth *synth = nullptr;
	li_codec *codec = nullptr;
	int64_t next_slot = 0;
	int64_t slot0 = 0;
	uint64_t slot0_feed_pos = 0;
	double current_lo = 0;
	uint64_t last_air_end = 0;   // feed position where the previous written burst ends
	bool have_prev = false;
	int sync_slot = 0;
	double last_sync_t = -1e9;
	double sync_interval_s = 1.0;   // SyncPktIntervalDisconnected shape (1 s; one SYNC per sync-channel visit anyway)
	// Feed-axis skew (samples), latched at lay_grid from the frontier the adapter just returned:
	//   axis_skew = pos0 - (air_now0 + pace)
	// The scheduler's slot grid MUST live on the feed-sample axis (the feeder writes from pos0 and
	// tx_feed refuses anything behind that frontier), but pos_now is derived from the tick anchor
	// t0_ext. On a healthy device the two axes coincide and axis_skew ~ 0. After a set_samp_rate
	// reclock / RX-apply re-mints the anchor, the device returned a pos0 that sat ~2^31 samples above
	// (tm.now_tick - t0_ext)*R while t0_ext stayed put (one board, DCS 200 MHz, R 4/2: frontier 85 s ahead
	// of pos_now, every burst read "ahead", tx sent 0). Adding axis_skew back onto pos_now re-bases the
	// scheduler's "now" onto the SAME axis as the frontier so the offset cancels. See/
	double axis_skew = 0.0;
	std::vector<double> retune_ms;
	std::vector<int16_t> iq;

	impl(radio_tx &r, const tx_plan_config &pc, int cap, bool en) : radio(r), plan_cfg(pc), cap_dbm(cap), enabled(en) {
		rc.fill(992);
		rc[4] = 191;   // disarmed (ch4 < 992)
		st.enabled = en;
		st.power_cap_dbm = cap;
		st.policy = r.policy();
		st.window_hz = pc.window_hz;
	}
	~impl() { release(); }
	void release() {
		delete lm; lm = nullptr;
		delete planner; planner = nullptr;
		delete synth; synth = nullptr;
		delete codec; codec = nullptr;
	}
	double elapsed_s() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_armed).count(); }

	// stop the radio and drop the armed state (mtx held)
	void disarm_locked(const char *why) {
		if(armed) {
			radio.stop();
			armed = false;
			session = false;
			st.armed = false;
			st.session = false;
			st.armed_remaining_s = 0;
			if(why && *why) st.last_error = why;
		}
	}

	void update_retune_stats() {
		if(retune_ms.empty()) { st.retune_ms_p50 = st.retune_ms_p99 = st.retune_ms_max = 0; return; }
		std::vector<double> v = retune_ms;
		std::sort(v.begin(), v.end());
		auto pct = [&](double p) { size_t i = (size_t)std::floor(p * (double)(v.size() - 1) + 0.5); return v[std::min(i, v.size() - 1)]; };
		st.retune_ms_p50 = pct(0.5);
		st.retune_ms_p99 = pct(0.99);
		st.retune_ms_max = v.back();
	}

	// lay (or re-lay) the slot grid from the session's first feed position (mtx held): the
	// first slot is the first grid point that satisfies the lead rule from pos0
	bool lay_grid(uint64_t pos0, int64_t first_slot) {
		tx_timing tm;
		if(!radio.timing(tm) || tm.r_num == 0) return false;
		double lead = (double)tm.feed_lead_ticks * (double)tm.r_den / (double)tm.r_num;
		uint64_t slot_n = planner->slot_samples();
		// re-anchor the scheduler's clock onto the feed-sample axis the adapter just handed us: the
		// frontier (pos0) is expected to sit `lead` (pace) ahead of the tick-derived air-now; whatever
		// residual offset remains is the axis skew a reclock/RX-apply introduced, and we keep it so
		// pos_now and the slot grid share one axis (see the axis_skew note above).
		double air_now0 = tm.now_tick > tm.t0_ext ? (double)(tm.now_tick - tm.t0_ext) * (double)tm.r_den / (double)tm.r_num : 0.0;
		axis_skew = (double)pos0 - (air_now0 + lead);
		slot0 = first_slot;
		next_slot = first_slot;
		slot0_feed_pos = ((pos0 + (uint64_t)(lead + plan_cfg.lead_margin_s * plan_cfg.fs_hz)) + slot_n - 1) / slot_n * slot_n;
		have_prev = false;
		session = true;
		st.session = true;
		return true;
	}

	// re-anchor after the device moved its TX anchor / a retune broke the timeline (mtx held)
	bool rebase(const char *why_s, std::string *why) {
		uint64_t pos0 = 0;
		if(!radio.session_restart(&pos0) || !lay_grid(pos0, next_slot)) {
			disarm_locked("session restart failed");
			if(why) *why = "restart_failed";
			return false;
		}
		// (the adapter counts the restart in health.rebases; status() merges it)
		fprintf(stderr, "elrsd tx: session re-anchored (%s): slot %lld from feed pos %llu\n", why_s, (long long)next_slot, (unsigned long long)slot0_feed_pos);
		if(why) *why = "rebased";
		return false;
	}

	// one scheduler iteration (mtx held). Returns true when a slot was consumed (written or skipped).
	bool step(std::string *why) {
		auto say = [&](const char *s) { if(why) *why = s; };
		if(!armed) { say("not armed"); return false; }
		// auto-disarm
		double el = elapsed_s();
		double dur = req.duration_s;
		st.armed_remaining_s = dur > 0 ? std::max(0.0, dur - el) : 0;
		if(dur > 0 && el >= dur) { disarm_locked("auto-disarm: duration elapsed"); say("auto-disarm"); return false; }
		if(req.max_bursts && st.bursts_sent >= req.max_bursts) { disarm_locked("auto-disarm: max_bursts reached"); say("auto-disarm"); return false; }
		if(!session) { say("no session"); return false; }          // armed, RF up, feed session not started yet
		tx_timing tm;
		if(!radio.timing(tm) || tm.r_num == 0) { disarm_locked("radio timing lost"); say("timing"); return false; }
		if(tm.anchor_moved) return rebase("device anchor moved", why);
		if(req.cw_only) { say("cw"); return false; }                // the idle tone is the only content
		auto ticks_to_samples = [&](uint64_t ticks) { return (double)ticks * (double)tm.r_den / (double)tm.r_num; };
		// pos_now on the FEED-sample axis: air-now (from the tick anchor) + the axis skew latched at
		// lay_grid, so it is comparable to the feed positions the frontier judges (see axis_skew note).
		double pos_now = (tm.now_tick > tm.t0_ext ? ticks_to_samples(tm.now_tick - tm.t0_ext) : 0.0) + axis_skew;
		const uint64_t slot_n = planner->slot_samples();
		const double lead_samples = ticks_to_samples(tm.feed_lead_ticks) + plan_cfg.lead_margin_s * plan_cfg.fs_hz;

		slot_plan sp = planner->plan_slot(*lm, next_slot, current_lo, slot0_feed_pos, slot0);
		// lookahead cap: never more than lookahead_slots ahead of the feed cursor - but always at
		// least one slot beyond the lead rule (short slots, e.g. 2 ms at 500 Hz, would otherwise
		// never satisfy "not ahead" and "not late" at the same time)
		const double ahead_cap = std::max((double)plan_cfg.lookahead_slots * (double)slot_n, lead_samples + (double)slot_n);
		if((double)sp.feed_pos > pos_now + ahead_cap) { say("ahead"); return false; }
		const size_t burst_n = synth->burst_samples();
		if(sp.retune_before) {
			// retune only in the gap after the previous burst has AIRED (an LO move while a burst
			// airs would shift it), and only if the gap can hold one at all
			double gap_s = ((double)slot_n - (double)burst_n) / plan_cfg.fs_hz;
			if(have_prev && pos_now < (double)last_air_end) { say("wait_gap"); return false; }
			if(gap_s < plan_cfg.gap_min_s && have_prev) {
				// gap too short for any retune: drop this slot, the next one gets the retune
				// after a whole empty slot
				st.bursts_dropped++;
				have_prev = false;
				next_slot++;
				say("gap_too_short");
				return true;
			}
			bool broken = false;
			long us = radio.retune(sp.f_lo_hz, &broken);
			if(us < 0) { disarm_locked("retune failed"); say("retune_failed"); return false; }
			st.retunes++;
			retune_ms.push_back((double)us / 1000.0);
			update_retune_stats();
			current_lo = sp.f_lo_hz;
			st.freq_lo_hz = current_lo;
			if(broken) {
				// the apply broke the TX timeline: re-anchor the feed session and the slot grid
				// (RF stays up - only the session restarts)
				return rebase("retune broke the timeline", why);
			}
			// re-read the clock: the retune took time; the lead rule decides below
			if(!radio.timing(tm)) { disarm_locked("radio timing lost"); say("timing"); return false; }
			pos_now = (tm.now_tick > tm.t0_ext ? ticks_to_samples(tm.now_tick - tm.t0_ext) : 0.0) + axis_skew;
			sp.baseband_offset_hz = sp.f_ch_hz - current_lo;
			if((double)sp.feed_pos < pos_now + lead_samples) {
				sp.skip = true;
				sp.skip_reason = "retune_late";
			}
		} else if((double)sp.feed_pos < pos_now + lead_samples) {
			sp.skip = true;
			sp.skip_reason = "late";
		}
		st.slot = next_slot;
		st.nonce = lm->nonce_at(next_slot);
		st.fhss_ptr = lm->fhss_ptr_at(next_slot);
		st.channel = sp.channel;
		if(sp.skip) {
			st.bursts_dropped++;
			have_prev = false;
			next_slot++;
			say(sp.skip_reason);
			return true;
		}
		// ---- build the packet ----
		uint8_t nonce = lm->nonce_at(next_slot);
		int ptr = lm->fhss_ptr_at(next_slot);
		int hop = rp->hop_interval;
		bool is_sync = false;
		double t_now_s = (double)sp.feed_pos / plan_cfg.fs_hz;
		if(req.sync_packets && sp.channel == FHSS_SYNC_CHANNEL && (sync_slot / 2) <= (int)(nonce % hop) && (t_now_s - last_sync_t) > sync_interval_s) {
			is_sync = true;
			sync_slot = (sync_slot + 1) % (2 * hop);
			last_sync_t = t_now_s;
		}
		std::array<uint8_t, 8> pkt;
		if(is_sync) {
			int denom = req.tlm_denom > 0 ? req.tlm_denom : rp->tlm_denom_default;
			uint8_t field = 0;
			for(int f = 1; f <= 7; f++) if((1 << (8 - f)) == denom) field = (uint8_t)f;
			pkt = ota4_build_sync(uid, (uint8_t)ptr, nonce, rp->index, field, false, 0);
		} else {
			pkt = ota4_build_rc(uid, rc, nonce, hop, req.tlm_denom > 0 ? req.tlm_denom : rp->tlm_denom_default, 2, false);
		}
		std::vector<uint16_t> syms = codec->encode(pkt.data(), pkt.size(), 0);
		size_t n = synth->synth(syms, sp.baseband_offset_hz, iq);
		if(!radio.write_at(sp.feed_pos, iq.data(), n)) {
			tx_health h;
			radio.health(h);
			st.bursts_dropped++;
			st.last_error = "write_at refused: " + h.last_error;
			have_prev = false;
			next_slot++;
			say("write_refused");
			return true;
		}
		radio.flush(true);
		st.bursts_sent++;
		if(is_sync) st.syncs_sent++;
		last_air_end = sp.feed_pos + n;
		have_prev = true;
		next_slot++;
		say(is_sync ? "sync" : "rc");
		return true;
	}

	tx_start_params start_params() const {
		tx_start_params p;
		p.fs_hz = plan_cfg.fs_hz;
		p.freq_lo_hz = current_lo;
		p.power_dbm = req.power_dbm;
		p.power_cap_dbm = cap_dbm;
		p.antenna = req.antenna;
		p.lpf_bw_mhz = std::max(1, (int)std::lround(plan_cfg.fs_hz / 1e6));
		return p;
	}

	void thread_main() {
		while(!thread_stop.load()) {
			std::string why;
			bool consumed;
			{
				std::lock_guard<std::mutex> l(mtx);
				if(!armed) break;
				consumed = step(&why);
				if(!armed) break;
			}
			if(!consumed) std::this_thread::sleep_for(std::chrono::microseconds(200));
		}
	}
};

elrs_transmitter::elrs_transmitter(radio_tx &radio, const tx_plan_config &plan, int cap_dbm, bool enabled_by_cli)
	: p_(new impl(radio, plan, cap_dbm, enabled_by_cli)) {}

elrs_transmitter::~elrs_transmitter() {
	disarm();
	delete p_;
}

void elrs_transmitter::set_threaded(bool on) { std::lock_guard<std::mutex> l(p_->mtx); p_->threaded = on; }

void elrs_transmitter::set_uid(const uid_t6 &uid) {
	std::lock_guard<std::mutex> l(p_->mtx);
	p_->uid = uid;
	p_->uid_set = true;
	if(p_->lm) p_->lm->set_uid(uid);
}

void elrs_transmitter::set_rc_channels(const std::array<uint16_t, 16> &crsf) {
	std::lock_guard<std::mutex> l(p_->mtx);
	for(size_t i = 0; i < 16; i++) p_->rc[i] = std::max<uint16_t>(172, std::min<uint16_t>(1811, crsf[i]));
}

bool elrs_transmitter::arm(const tx_arm_request &req, std::string &reason, bool start_session) {
	std::lock_guard<std::mutex> l(p_->mtx);
	impl &d = *p_;
	d.st.policy = d.radio.policy();
	if(d.armed) { reason = "already armed"; return false; }
	if(d.st.policy == tx_policy::NONE) { reason = "board TX policy is none (TX_ALLOWED)"; return false; }
	if(!d.enabled) { reason = "tx not enabled on the command line (--tx 1)"; return false; }
	if(!d.uid_set) { reason = "no uid set (binding phrase)"; return false; }
	if(req.power_dbm > d.cap_dbm) { reason = "power_dbm above the cap"; return false; }
	if(d.st.policy == tx_policy::EMBED && req.antenna != "embed_ant" && req.antenna != "embed") { reason = "policy embed: only the embedded antenna is allowed"; return false; }
	const rate_params *rp = rate_by_index(req.rate_index);
	if(!rp) { reason = "unknown rate index"; return false; }
	if(!rp->verified || rp->kind != ota_kind::OTA4) { reason = "rate unsupported (OTA8 / unverified)"; return false; }
	if(req.duration_s < 0) { reason = "bad duration"; return false; }
	// ---- build the timeline ----
	d.release();
	d.rp = rp;
	d.req = req;
	if(d.req.antenna == "embed") d.req.antenna = "embed_ant";
	d.lm = new link_model(d.uid, *rp);
	sync_packet sp0;
	sp0.fhss_index = 0;   // start on the SYNC channel (ptr 0 -> channel 41) with nonce 0 = dwell start
	sp0.nonce = 0;
	sp0.rate_index = rp->index;
	d.lm->lock_from_sync(0, sp0);
	d.planner = new hop_planner(d.plan_cfg, *rp);
	burst_synth_config sc;
	sc.fs_hz = d.plan_cfg.fs_hz;
	sc.iq_invert = iq_inverted_from_uid(d.uid);
	d.synth = new burst_synth(*rp, sc);
	d.codec = new li_codec(*rp);
	d.current_lo = hop_planner::choose_lo(d.lm->sequence(), 0, 8, d.plan_cfg);
	// ---- radio bring-up (the adapter re-checks every gate) ----
	tx_start_params p = d.start_params();
	p.start_session = start_session;
	uint64_t pos0 = 0;
	// the idle tone is set BEFORE the session starts so the feeder never emits a single zero
	// chunk first (cw_only) - and it is cleared explicitly for a burst arm
	d.radio.set_idle_tone(req.cw_only ? req.cw_offset_hz : 0.0, req.cw_only ? req.cw_amplitude : (int16_t)0);
	if(!d.radio.start(p, &pos0)) {
		tx_health h;
		d.radio.health(h);
		reason = "radio refused: " + h.last_error;
		d.st.last_error = reason;
		return false;
	}
	d.slot0 = 0;
	d.next_slot = 0;
	d.slot0_feed_pos = 0;
	d.session = false;
	if(start_session && !d.lay_grid(pos0, 0)) {
		d.radio.stop();
		reason = "radio timing unavailable after start";
		d.st.last_error = reason;
		return false;
	}
	d.have_prev = false;
	d.sync_slot = 0;
	d.last_sync_t = -1e9;
	d.retune_ms.clear();
	d.armed = true;
	d.t_armed = std::chrono::steady_clock::now();
	d.st = tx_status();
	d.st.enabled = d.enabled;
	d.st.armed = true;
	d.st.policy = d.radio.policy();
	d.st.antenna = d.radio.path_applied();
	d.st.power_dbm_requested = req.power_dbm;
	d.st.power_dbm_applied = d.radio.power_applied_dbm();
	d.radio.power_range(&d.st.power_min_dbm, &d.st.power_max_dbm);
	d.st.power_cap_dbm = d.cap_dbm;
	d.st.freq_lo_hz = d.current_lo;
	d.st.window_hz = d.plan_cfg.window_hz;
	d.st.armed_remaining_s = req.duration_s;
	d.st.session = d.session;
	d.st.cw = req.cw_only;
	d.st.cw_offset_hz = req.cw_only ? req.cw_offset_hz : 0.0;
	d.thread_stop = false;
	if(d.threaded) {
		if(d.thread.joinable()) d.thread.join();
		d.thread = std::thread([this]() { p_->thread_main(); });
		d.thread_id = d.thread.get_id();
	}
	reason.clear();
	return true;
}

bool elrs_transmitter::session_start(std::string &reason) {
	std::lock_guard<std::mutex> l(p_->mtx);
	impl &d = *p_;
	if(!d.armed) { reason = "not armed"; return false; }
	if(d.session) { reason = "session already running"; return false; }
	uint64_t pos0 = 0;
	if(!d.radio.session_start(&pos0)) {
		tx_health h;
		d.radio.health(h);
		reason = "radio refused the session: " + h.last_error;
		d.st.last_error = reason;
		return false;
	}
	if(!d.lay_grid(pos0, 0)) { reason = "radio timing unavailable after session start"; d.st.last_error = reason; return false; }
	reason.clear();
	return true;
}

void elrs_transmitter::disarm() {
	impl &d = *p_;
	bool from_thread = d.thread.joinable() && std::this_thread::get_id() == d.thread_id;
	d.thread_stop = true;
	if(!from_thread && d.thread.joinable()) d.thread.join();
	std::lock_guard<std::mutex> l(d.mtx);
	d.disarm_locked("");
}

bool elrs_transmitter::armed() const { std::lock_guard<std::mutex> l(p_->mtx); return p_->armed; }

tx_status elrs_transmitter::status() const {
	std::lock_guard<std::mutex> l(p_->mtx);
	tx_status s = p_->st;
	s.policy = p_->radio.policy();
	s.enabled = p_->enabled;
	s.power_cap_dbm = p_->cap_dbm;
	s.session = p_->session;
	if(p_->armed) {
		s.armed_remaining_s = p_->req.duration_s > 0 ? std::max(0.0, p_->req.duration_s - p_->elapsed_s()) : 0;
		p_->radio.health(s.health);
		s.health.rebases += p_->st.health.rebases;
		s.antenna = p_->radio.path_applied();
		s.power_dbm_applied = p_->radio.power_applied_dbm();
	}
	return s;
}

bool elrs_transmitter::step_once(std::string *why) {
	std::lock_guard<std::mutex> l(p_->mtx);
	return p_->step(why);
}

// =====================================================================================
// self-loop checker (--selfloop-device)
// =====================================================================================

std::array<uint16_t, 16> selfloop_rc_pattern() {
	std::array<uint16_t, 16> ch;
	for(uint32_t i = 0; i < 16; i++) ch[i] = (uint16_t)(172 + lowbias32(0x5e1f0000u + i) % 1640);
	return ch;
}

std::string selfloop_counts_json(const selfloop_counts &c) {
	char b[512];
	snprintf(b, sizeof b,
	         "{\"packets\":%llu,\"undecodable\":%llu,\"crc_ok\":%llu,\"crc_fail\":%llu,\"rc\":%llu,\"sync\":%llu,\"other\":%llu,"
	         "\"rc_bitexact\":%llu,\"sync_bitexact\":%llu,\"mismatch\":%llu,\"ok\":%s}",
	         (unsigned long long)c.packets, (unsigned long long)c.undecodable, (unsigned long long)c.crc_ok,
	         (unsigned long long)c.crc_fail, (unsigned long long)c.rc, (unsigned long long)c.sync, (unsigned long long)c.other,
	         (unsigned long long)c.rc_bitexact, (unsigned long long)c.sync_bitexact, (unsigned long long)c.mismatch,
	         c.ok() ? "true" : "false");
	return b;
}

selfloop_checker::selfloop_checker(const uid_t6 &uid, const rate_params &rp, const std::array<uint16_t, 16> &rc, int tlm_denom)
	: uid_(uid), rp_(rp), tlm_denom_(tlm_denom > 0 ? tlm_denom : rp.tlm_denom_default) {
	// RC: the transmitter builds ota4_build_rc(uid, rc, nonce, hop, tlm, 2, false) - only crcHigh
	// (nonce % hop) and the aux slot (nonce) vary, so the 256 variants collapse to a few
	for(int n = 0; n < 256; n++) {
		std::array<uint8_t, 8> p = ota4_build_rc(uid_, rc, (uint8_t)n, rp_.hop_interval, tlm_denom_, 2, false);
		if(std::find(rc_candidates_.begin(), rc_candidates_.end(), p) == rc_candidates_.end()) rc_candidates_.push_back(p);
	}
	// SYNC: (fhss_index, nonce) pairs that the transmitter's timeline produces (nonce0 = 0, ptr0 = 0
	// at slot 0, via the same link_model the transmitter uses); period = lcm(256, 240 * hop) slots
	sync_pair_ok_.assign(240 * 256, 0);
	link_model lm(uid_, rp_);
	sync_packet sp0;
	sp0.fhss_index = 0;
	sp0.nonce = 0;
	sp0.rate_index = rp_.index;
	lm.lock_from_sync(0, sp0);
	const int64_t period = 256LL * 240 * rp_.hop_interval;   // a multiple of the true period
	for(int64_t s = 0; s < period; s++) {
		int ptr = lm.fhss_ptr_at(s);
		uint8_t nonce = lm.nonce_at(s);
		if(ptr >= 0 && ptr < 240) sync_pair_ok_[(size_t)ptr * 256 + nonce] = 1;
	}
}

bool selfloop_checker::rc_expected(const uint8_t pkt[8]) const {
	for(const auto &c : rc_candidates_) if(std::memcmp(c.data(), pkt, 8) == 0) return true;
	return false;
}

bool selfloop_checker::sync_expected(const uint8_t pkt[8]) const {
	uint8_t ptr = pkt[1], nonce = pkt[2];
	if(ptr >= 240 || !sync_pair_ok_[(size_t)ptr * 256 + nonce]) return false;
	uint8_t field = 0;
	for(int f = 1; f <= 7; f++) if((1 << (8 - f)) == tlm_denom_) field = (uint8_t)f;
	std::array<uint8_t, 8> e = ota4_build_sync(uid_, ptr, nonce, rp_.index, field, false, 0);
	return std::memcmp(e.data(), pkt, 8) == 0;
}

void selfloop_checker::on_packet(const std::string &kind, bool crc_ok, const uint8_t *pkt8) {
	counts_.packets++;
	if(kind == "undecodable" || !pkt8) { counts_.undecodable++; return; }
	if(!crc_ok) { counts_.crc_fail++; return; }
	counts_.crc_ok++;
	if(kind == "rc") {
		counts_.rc++;
		if(rc_expected(pkt8)) counts_.rc_bitexact++; else counts_.mismatch++;
	} else if(kind == "sync") {
		counts_.sync++;
		if(sync_expected(pkt8)) counts_.sync_bitexact++; else counts_.mismatch++;
	} else {
		counts_.other++;
		counts_.mismatch++;   // the self-loop transmitter never sends MSP/TLM
	}
}

bool selfloop_checker::hex_to_bytes(const std::string &hex16, uint8_t out[8]) {
	if(hex16.size() != 16) return false;
	auto nib = [](char c) -> int {
		if(c >= '0' && c <= '9') return c - '0';
		if(c >= 'a' && c <= 'f') return c - 'a' + 10;
		if(c >= 'A' && c <= 'F') return c - 'A' + 10;
		return -1;
	};
	for(int i = 0; i < 8; i++) {
		int h = nib(hex16[2 * i]), l = nib(hex16[2 * i + 1]);
		if(h < 0 || l < 0) return false;
		out[i] = (uint8_t)((h << 4) | l);
	}
	return true;
}

// =====================================================================================
// selftest
// =====================================================================================

// the loopback through B's rx_chain lives in src/tx_selftest.cpp (NEEDS rx_chain/dsp); weak so
// that tx_chain.cpp builds and tests on its own until B lands
__attribute__((weak)) int run_selftest_tx_loopback(selftest_counter &c);

namespace {

// direct correlation demod at the DEVICE rate (no resampler, no FFT): the reference for value s is
// the synthesized symbol itself -> tests the synthesis against its own phase convention and the
// cyclic-shift/argmax invariant independent of B
static int demod_direct(const int16_t *iq, size_t n_start, const rate_params &rp, const burst_synth_config &cfg, double offset_hz, bool down) {
	const int M = 1 << rp.sf;
	const double bw = cfg.bw_hz, T = (double)M / bw, fs = cfg.fs_hz;
	const size_t n = (size_t)std::floor(T * fs);
	const double sgn = (down ? -1.0 : 1.0) * (cfg.iq_invert ? -1.0 : 1.0);
	// y = x * conj(carrier) * conj(base chirp); the symbol-s reference is the base chirp times a
	// tone at s*bw/M (wrapping to s*bw/M - bw after T - shift), so one dechirp serves all M candidates
	std::vector<std::complex<double>> y(n);
	for(size_t i = 0; i < n; i++) {
		double t = (double)(n_start + i) / fs, tau = (double)i / fs;
		std::complex<double> v((double)iq[2 * (n_start + i)], (double)iq[2 * (n_start + i) + 1]);
		y[i] = v * std::polar(1.0, -2.0 * M_PI * offset_hz * t - sgn * upchirp_phase(tau, bw, T));
	}
	double best = -1; int bi = -1;
	for(int s = 0; s < M; s++) {
		double shift = (double)s * T / M;
		size_t n1 = std::min(n, (size_t)std::ceil((T - shift) * fs - 1e-9));
		double f1 = bw * shift / T, c1 = M_PI * bw * shift * shift / T - M_PI * bw * shift;
		double f2 = f1 - bw, c2 = M_PI * bw * (shift - T) * (shift - T) / T - M_PI * bw * (shift - T);
		std::complex<double> acc(0, 0);
		std::complex<double> ph = std::polar(1.0, -sgn * c1), st = std::polar(1.0, -sgn * 2.0 * M_PI * f1 / fs);
		for(size_t i = 0; i < n1; i++) { acc += y[i] * ph; ph *= st; }
		ph = std::polar(1.0, -sgn * (2.0 * M_PI * f2 * (double)n1 / fs + c2));
		st = std::polar(1.0, -sgn * 2.0 * M_PI * f2 / fs);
		for(size_t i = n1; i < n; i++) { acc += y[i] * ph; ph *= st; }
		double m = std::abs(acc);
		if(m > best) { best = m; bi = s; }
	}
	return bi;
}

static void selftest_synth(selftest_counter &c) {
	for(int idx : rate_indices_verified()) {
		const rate_params *rp = rate_by_index(idx);
		burst_synth_config cfg;
		burst_synth bs(*rp, cfg);
		std::vector<uint16_t> syms((size_t)rp->n_sym);
		uint32_t seed = 12345u + (uint32_t)idx;
		for(auto &s : syms) { seed = seed * 1664525u + 1013904223u; s = (uint16_t)((seed >> 8) % (1u << rp->sf)); }
		std::vector<int16_t> iq;
		size_t n = bs.synth(syms, 0.0, iq);
		char name[96];
		snprintf(name, sizeof(name), "synth length idx%d", idx);
		size_t expect = (size_t)std::llround(rp->burst_s() * cfg.fs_hz);
		c.check(n == expect && iq.size() == 2 * n && n == bs.burst_samples(), name, std::to_string(n) + " samples = " + std::to_string(rp->burst_s() * 1e3) + " ms");
		int peak = 0; double p = 0;
		for(size_t i = 0; i < n; i++) { peak = std::max(peak, std::max(std::abs((int)iq[2 * i]), std::abs((int)iq[2 * i + 1]))); p += (double)iq[2 * i] * iq[2 * i] + (double)iq[2 * i + 1] * iq[2 * i + 1]; }
		double rms = std::sqrt(p / (double)n);
		snprintf(name, sizeof(name), "synth amplitude idx%d", idx);
		c.check(peak <= 14000 && peak >= 13990 && std::fabs(rms - 14000.0) < 30.0, name, "peak " + std::to_string(peak) + " rms " + std::to_string((int)rms));
		// instantaneous frequency of the first preamble symbol: linear ramp -bw/2 .. +bw/2
		const double T = (double)(1 << rp->sf) / cfg.bw_hz;
		size_t ns = (size_t)std::floor(T * cfg.fs_hz);
		double f_first = 0, f_last = 0; int cnt_first = 0, cnt_last = 0;
		for(size_t i = 1; i < ns; i++) {
			std::complex<double> a((double)iq[2 * (i - 1)], (double)iq[2 * (i - 1) + 1]), b((double)iq[2 * i], (double)iq[2 * i + 1]);
			double f = std::arg(b * std::conj(a)) * cfg.fs_hz / (2.0 * M_PI);
			if(i < 8) { f_first += f; cnt_first++; }
			if(i >= ns - 8) { f_last += f; cnt_last++; }
		}
		f_first /= cnt_first; f_last /= cnt_last;
		snprintf(name, sizeof(name), "synth preamble chirp ramp idx%d", idx);
		c.check(std::fabs(f_first + cfg.bw_hz / 2) < 0.01 * cfg.bw_hz && std::fabs(f_last - cfg.bw_hz / 2) < 0.01 * cfg.bw_hz, name,
		        "f_start " + std::to_string((int)f_first) + " Hz f_end " + std::to_string((int)f_last) + " Hz");
	}
	// symbol round trip by direct correlation at 25 MSps: preamble 0, sync word 8/16, fine-sync 1, payload
	for(int idx : rate_indices_verified()) {
		for(int inv = 0; inv < 2; inv++) {
			for(double off : {0.0, 3.7e6, -6.4e6}) {
				const rate_params *rp = rate_by_index(idx);
				burst_synth_config cfg;
				cfg.iq_invert = inv != 0;
				burst_synth bs(*rp, cfg);
				std::vector<uint16_t> syms((size_t)rp->n_sym);
				uint32_t seed = 777u + (uint32_t)idx * 31u + (uint32_t)inv;
				for(auto &s : syms) { seed = seed * 1664525u + 1013904223u; s = (uint16_t)((seed >> 8) % (1u << rp->sf)); }
				std::vector<int16_t> iq;
				bs.synth(syms, off, iq);
				const double T = (double)(1 << rp->sf) / cfg.bw_hz;
				auto at = [&](double sym_pos) { return (size_t)std::llround(sym_pos * T * cfg.fs_hz); };
				int bad = 0;
				std::string detail;
				if(demod_direct(iq.data(), at(0), *rp, cfg, off, false) != 0) { bad++; detail += "pre "; }
				if(demod_direct(iq.data(), at(rp->preamble_len), *rp, cfg, off, false) != 8) { bad++; detail += "sync1 "; }
				if(demod_direct(iq.data(), at(rp->preamble_len + 1), *rp, cfg, off, false) != 16) { bad++; detail += "sync2 "; }
				if(demod_direct(iq.data(), at(rp->preamble_len + 2), *rp, cfg, off, true) != 0) { bad++; detail += "sfd "; }
				if(rp->fine_sync && demod_direct(iq.data(), at(rp->preamble_len + 4.25), *rp, cfg, off, false) != 1) { bad++; detail += "fine "; }
				int checked = std::min(rp->n_sym, 6);
				for(int i = 0; i < checked; i++) {
					int got = demod_direct(iq.data(), at(rp->frame_offset_symbols + i), *rp, cfg, off, false);
					if(got != (int)syms[(size_t)i]) { bad++; detail += "pay" + std::to_string(i) + " "; }
				}
				char name[96];
				snprintf(name, sizeof(name), "synth direct-demod idx%d inv%d off%+.1fMHz", idx, inv, off / 1e6);
				c.check(bad == 0, name, bad ? detail : "preamble/sync/sfd/" + std::string(rp->fine_sync ? "fine/" : "") + std::to_string(checked) + " payload symbols");
			}
		}
	}
}

static uid_t6 test_uid() { return uid_t6{0xd2, 0x4c, 0x3e, 0x2e, 0xfb, 0x4d}; }

static void selftest_planner(selftest_counter &c) {
	const rate_params *rp = rate_by_index(6);
	tx_plan_config pc;
	link_model lm(test_uid(), *rp);
	sync_packet sp0; sp0.fhss_index = 0; sp0.nonce = 0; sp0.rate_index = 6;
	lm.lock_from_sync(0, sp0);
	hop_planner hp(pc, *rp);
	c.check(hp.slot_samples() == 100000, "planner slot_samples 250hz", std::to_string(hp.slot_samples()));
	c.check(!hp.config().wide_window, "planner wide_window off");
	// slot grid
	double lo = hop_planner::choose_lo(lm.sequence(), 0, 8, pc);
	bool grid_ok = true, chan_ok = true, retune_ok = true;
	int retunes = 0;
	double cur = lo;
	for(int64_t s = 0; s < 400; s++) {
		slot_plan p = hp.plan_slot(lm, s, cur, 1000, 0);
		if(p.feed_pos != 1000 + (uint64_t)s * 100000) grid_ok = false;
		if(p.channel != lm.channel_at(s) || std::fabs(p.f_ch_hz - fhss_channel_freq_hz(p.channel)) > 400) chan_ok = false;
		if(std::fabs(p.baseband_offset_hz) > pc.window_hz / 2 - pc.guard_hz + 1) retune_ok = false;
		if(p.retune_before) { retunes++; cur = p.f_lo_hz; }
	}
	c.check(grid_ok, "planner slot grid positions");
	c.check(chan_ok, "planner channels follow the link model (SX1280-exact <= 400 Hz off nominal)");
	c.check(retune_ok && retunes > 0 && retunes < 100, "planner offsets inside the window, retunes only at window exits", std::to_string(retunes) + " retunes / 100 dwells");
	// choose_lo keeps the first dwell clear of the LO leak zone
	c.check(std::fabs(fhss_channel_freq_hz_sx1280(lm.channel_at(0)) - lo) >= pc.lo_min_offset_hz, "planner choose_lo avoids the LO leak zone", std::to_string((int)(lo / 1e3)) + " kHz");
}

static void selftest_transmitter(selftest_counter &c) {
	const uint64_t TICK_HZ = 61440000ULL;
	tx_plan_config pc;
	// ---- arming matrix ----
	struct arm_case { tx_policy pol; std::vector<std::string> paths; bool enabled; bool uid; int power; std::string ant; bool expect; const char *name; };
	std::vector<arm_case> cases = {
		{tx_policy::NONE, {"sma_a", "embed_ant"}, true, true, -40, "sma_a", false, "arm refused: policy none"},
		{tx_policy::ALL, {"sma_a"}, false, true, -40, "sma_a", false, "arm refused: --tx 0"},
		{tx_policy::ALL, {"sma_a"}, true, false, -40, "sma_a", false, "arm refused: no uid"},
		{tx_policy::ALL, {"sma_a"}, true, true, -10, "sma_a", false, "arm refused: power above cap"},
		{tx_policy::EMBED, {"sma_a"}, true, true, -40, "embed_ant", false, "arm refused: embed policy without embed_ant path"},
		{tx_policy::EMBED, {"sma_a"}, true, true, -40, "sma_a", false, "arm refused: embed policy with sma antenna"},
		{tx_policy::EMBED, {"sma_a", "embed_ant"}, true, true, -40, "embed_ant", true, "arm ok: embed policy on embed_ant"},
		{tx_policy::ALL, {"sma_a"}, true, true, -40, "sma_a", true, "arm ok: policy all sma_a"},
		{tx_policy::ALL, {"sma_a"}, true, true, -40, "sma_b", true, "arm ok: unknown antenna falls back to preferred (policy all)"},
	};
	for(auto &cs : cases) {
		radio_tx_mock m;
		m.set_policy(cs.pol);
		m.set_paths(cs.paths, "sma_a");
		m.set_fake_now(TICK_HZ);
		elrs_transmitter tx(m, pc, -20, cs.enabled);
		tx.set_threaded(false);
		if(cs.uid) tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = cs.power; r.antenna = cs.ant; r.duration_s = 10; r.rate_index = 6;
		std::string why;
		bool ok = tx.arm(r, why);
		c.check(ok == cs.expect && tx.armed() == cs.expect && m.running() == cs.expect, cs.name, ok ? ("path " + tx.status().antenna + " power " + std::to_string(tx.status().power_dbm_applied)) : why);
		if(ok) {
			tx.disarm();
			c.check(!tx.armed() && !m.running(), "disarm stops the radio");
		}
	}
	// ---- adapter-level gates (defense in depth, no transmitter involved) ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::NONE);
		tx_start_params p; p.power_dbm = -40; p.power_cap_dbm = -20;
		uint64_t pos0 = 0;
		bool started = m.start(p, &pos0);
		tx_health h; m.health(h);
		c.check(!started && h.refusals == 1, "adapter refuses start() with policy none", h.last_error);
		int16_t z[2] = {0, 0};
		c.check(!m.write_at(0, z, 1), "adapter refuses write_at() when not started");
		m.set_policy(tx_policy::ALL);
		p.power_dbm = -10;
		c.check(!m.start(p, &pos0), "adapter refuses start() above the power cap regardless of the caller");
		p.power_dbm = -40;
		m.set_power_range(-60, -30);
		c.check(m.start(p, &pos0) && m.power_applied_dbm() == -40, "adapter clamps power into the range", std::to_string(m.power_applied_dbm()));
		p.power_dbm = -20;
		m.stop();
		c.check(m.start(p, &pos0) && m.power_applied_dbm() == -30, "adapter clamps power to min(range.max, cap)", std::to_string(m.power_applied_dbm()));
		m.stop();
		m.set_policy(tx_policy::EMBED);
		m.set_paths({"sma_a"}, "sma_a");
		p.antenna = "embed_ant";
		started = m.start(p, &pos0);
		m.health(h);
		c.check(!started && h.last_error.find("embed") != std::string::npos, "adapter refuses embed policy without EMBED_ANT (never falls back)", h.last_error);
	}
	// ---- slot loop on the mock: grid, lead rule, retune in the gap, late skip, lookahead cap ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_retune_us(300);
		m.set_fake_now(TICK_HZ);
		elrs_transmitter tx(m, pc, -20, true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -40; r.duration_s = 0; r.rate_index = 6; r.sync_packets = true;
		std::string why;
		c.check(tx.arm(r, why), "arm for the slot loop", why);
		tx_timing tm; m.timing(tm);
		const uint64_t slot_n = 100000;
		auto samples_to_ticks = [&](double s) { return (uint64_t)(s * (double)tm.r_num / (double)tm.r_den); };
		// with the clock standing still, the loop may write at most lookahead_slots bursts
		int consumed = 0;
		for(int i = 0; i < 10; i++) { if(tx.step_once(&why)) consumed++; else break; }
		c.check((int)m.captured().size() >= 1 && (int)m.captured().size() <= pc.lookahead_slots && why == "ahead", "lookahead cap holds the loop", std::to_string(m.captured().size()) + " bursts, then '" + why + "'");
		// advance the fake clock slot by slot and drive the loop
		uint64_t pos_base = m.captured().empty() ? 0 : m.captured()[0].pos;
		int total_slots = 400;
		for(int64_t s = 0; s < total_slots; s++) {
			uint64_t pos_now = pos_base + (uint64_t)s * slot_n;
			m.set_fake_now(tm.t0_ext + samples_to_ticks((double)pos_now) - 1000);
			for(int i = 0; i < 8; i++) if(!tx.step_once(&why)) break;
		}
		tx_status st = tx.status();
		const auto &ev = m.captured();
		bool grid_ok = true, lo_ok = true, order_ok = true;
		uint64_t last_pos = 0;
		std::vector<double> los;
		for(size_t i = 0; i < ev.size(); i++) {
			if((ev[i].pos - pos_base) % slot_n != 0) grid_ok = false;
			if(i && ev[i].pos <= last_pos) order_ok = false;
			last_pos = ev[i].pos;
			if(i && ev[i].freq_lo_hz != ev[i - 1].freq_lo_hz) los.push_back(ev[i].freq_lo_hz);
		}
		// every burst has its LO set BEFORE the write (retune-in-gap rule); the LO used must contain the channel
		link_model lm(test_uid(), *rate_by_index(6));
		sync_packet sp0; sp0.fhss_index = 0; sp0.nonce = 0; sp0.rate_index = 6;
		lm.lock_from_sync(0, sp0);
		for(size_t i = 0; i < ev.size(); i++) {
			int64_t slot = (int64_t)((ev[i].pos - pos_base) / slot_n);
			double fch = fhss_channel_freq_hz_sx1280(lm.channel_at(slot));
			if(std::fabs(fch - ev[i].freq_lo_hz) > pc.window_hz / 2 - pc.guard_hz + 1) lo_ok = false;
		}
		c.check(grid_ok && order_ok, "bursts land on the slot grid, strictly increasing feed positions", std::to_string(ev.size()) + " bursts");
		c.check(lo_ok && st.retunes > 0, "every burst airs inside the LO window set before it (retune in the gap)", std::to_string(st.retunes) + " retunes, LO changes seen " + std::to_string(los.size()));
		c.check(st.bursts_sent == ev.size() && st.bursts_sent + st.bursts_dropped >= (uint64_t)total_slots - 2, "counters: sent + dropped cover the slots", "sent " + std::to_string(st.bursts_sent) + " dropped " + std::to_string(st.bursts_dropped));
		c.check(st.syncs_sent >= 1, "SYNC packets sent on channel 41 visits", std::to_string(st.syncs_sent));
		c.check(st.retune_ms_p50 > 0.29 && st.retune_ms_max < 0.31, "retune p50/p99/max bookkeeping", "p50 " + std::to_string(st.retune_ms_p50) + " p99 " + std::to_string(st.retune_ms_p99) + " max " + std::to_string(st.retune_ms_max));
		// burst content is a real ELRS frame: decode the first burst's payload symbols by direct correlation
		if(!ev.empty()) {
			const rate_params *rp = rate_by_index(6);
			burst_synth_config sc; sc.iq_invert = iq_inverted_from_uid(test_uid());
			li_codec codec(*rp);
			std::vector<uint16_t> syms;
			const double T = (double)(1 << rp->sf) / sc.bw_hz;
			double off = fhss_channel_freq_hz_sx1280(lm.channel_at(0)) - ev[0].freq_lo_hz;
			for(int i = 0; i < rp->n_sym; i++) syms.push_back((uint16_t)demod_direct(ev[0].iq.data(), (size_t)std::llround((rp->frame_offset_symbols + i) * T * sc.fs_hz), *rp, sc, off, false));
			std::vector<uint8_t> payload;
			bool dec = codec.decode(syms.data(), syms.size(), 0, payload, false);
			ota4_decoded d = dec ? ota4_decode(payload.data(), test_uid(), rp->hop_interval, 64) : ota4_decoded();
			c.check(dec && d.crc_ok && d.type == ota_type::SYNC && d.sync.fhss_index == 0 && d.sync.nonce == 0 && d.sync.rate_index == 6,
			        "first burst = SYNC packet (ptr 0, nonce 0) decodable by direct correlation", dec ? ("crc " + std::to_string(d.crc_ok) + " type " + std::to_string((int)d.type)) : "decode failed");
			// the second burst must be an RC packet with the default (disarmed) channels
			if(ev.size() > 1) {
				syms.clear();
				off = fhss_channel_freq_hz_sx1280(lm.channel_at(1)) - ev[1].freq_lo_hz;
				for(int i = 0; i < rp->n_sym; i++) syms.push_back((uint16_t)demod_direct(ev[1].iq.data(), (size_t)std::llround((rp->frame_offset_symbols + i) * T * sc.fs_hz), *rp, sc, off, false));
				dec = codec.decode(syms.data(), syms.size(), 0, payload, false);
				d = dec ? ota4_decode(payload.data(), test_uid(), rp->hop_interval, 64) : ota4_decoded();
				c.check(dec && d.crc_ok && d.type == ota_type::RC && !d.rc.ch4_high && std::abs((int)d.rc.ch[0] - 992) <= 3 && std::find(d.nonce_residues.begin(), d.nonce_residues.end(), 1) != d.nonce_residues.end(),
				        "second burst = RC packet, nonce 1, channels centred, disarmed", dec ? ("ch0 " + std::to_string(d.rc.ch[0]) + " ch4_high " + std::to_string(d.rc.ch4_high)) : "decode failed");
			}
		}
		tx.disarm();
	}
	// ---- late skip: clock already past the lead -> slots dropped, never written, never caught up ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_fake_now(TICK_HZ);
		elrs_transmitter tx(m, pc, -20, true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -40; r.duration_s = 0; r.rate_index = 4;
		std::string why;
		c.check(tx.arm(r, why), "arm 500hz for the late test", why);
		tx_timing tm; m.timing(tm);
		const uint64_t slot_n = 50000;
		auto to_tick = [&](double samples) { return tm.t0_ext + (uint64_t)(samples * (double)tm.r_num / (double)tm.r_den); };
		// walk the clock until the first burst is written (the grid starts a few slots ahead)
		for(int i = 0; i < 20 && m.captured().empty(); i++) { m.set_fake_now(to_tick((double)i * slot_n)); tx.step_once(&why); }
		c.check(m.captured().size() == 1, "first burst written on the grid", std::to_string(m.captured().size()));
		uint64_t first = m.captured().empty() ? 0 : m.captured()[0].pos;
		// jump 3 slots past it: slots first+1..first+4 lie inside the lead and must be dropped
		m.set_fake_now(to_tick((double)(first + 3 * slot_n)));
		int late = 0, written = 0, other = 0;
		for(int i = 0; i < 12; i++) { if(!tx.step_once(&why)) break; if(why == "late") late++; else if(why == "rc" || why == "sync") written++; else other++; }
		tx_status st = tx.status();
		c.check(late >= 2 && st.bursts_dropped >= 3 && written == 1 && st.bursts_sent == 2, "late slots are skipped (never caught up), the next slot in front of the lead is written",
		        "late " + std::to_string(late) + " other skips " + std::to_string(other) + " written " + std::to_string(written) + " dropped " + std::to_string(st.bursts_dropped));
		bool mono = true; uint64_t lp = 0;
		for(auto &e : m.captured()) { if(lp && e.pos <= lp) mono = false; lp = e.pos; }
		double lead_samples = (double)tm.feed_lead_ticks * tm.r_den / tm.r_num + pc.lead_margin_s * pc.fs_hz;
		c.check(mono && m.captured().size() == 2 && (double)m.captured()[1].pos >= (double)(first + 3 * slot_n) + lead_samples, "no burst written behind the cursor or inside the lead");
		tx.disarm();
	}
	// ---- retune too slow for the gap -> retune_late skip, never on the old LO ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_retune_us(20000);   // 20 ms: longer than any slot
		m.set_fake_now(TICK_HZ);
		tx_plan_config pc2 = pc;
		elrs_transmitter tx(m, pc2, -20, true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -40; r.duration_s = 0; r.rate_index = 6;
		std::string why;
		c.check(tx.arm(r, why), "arm for the slow-retune test", why);
		tx_timing tm; m.timing(tm);
		int retune_late = 0, sent = 0;
		uint64_t base = 0;
		for(int64_t s = 0; s < 300; s++) {
			if(s == 1 && !m.captured().empty()) base = m.captured()[0].pos;
			uint64_t pos_now = base + (uint64_t)s * 100000;
			m.set_fake_now(tm.t0_ext + (uint64_t)((double)pos_now * (double)tm.r_num / (double)tm.r_den) - 1000);
			for(int i = 0; i < 8; i++) { if(!tx.step_once(&why)) break; if(why == "retune_late") retune_late++; else if(why == "rc" || why == "sync") sent++; }
		}
		// every written burst must sit inside the LO window that was current at its write
		link_model lm(test_uid(), *rate_by_index(6));
		sync_packet sp0; sp0.fhss_index = 0; sp0.nonce = 0; sp0.rate_index = 6;
		lm.lock_from_sync(0, sp0);
		bool lo_ok = true;
		for(auto &e : m.captured()) {
			int64_t slot = (int64_t)((e.pos - base) / 100000);
			if(std::fabs(fhss_channel_freq_hz_sx1280(lm.channel_at(slot)) - e.freq_lo_hz) > pc.window_hz / 2 - pc.guard_hz + 1) lo_ok = false;
		}
		c.check(retune_late > 0 && lo_ok, "slow retune -> retune_late skips, nothing aired on the old LO", "retune_late " + std::to_string(retune_late) + " sent " + std::to_string(sent));
		tx.disarm();
	}
	// ---- auto-disarm by duration and by max_bursts ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_fake_now(TICK_HZ);
		elrs_transmitter tx(m, pc, -20, true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -40; r.duration_s = 0.05; r.rate_index = 6;
		std::string why;
		c.check(tx.arm(r, why), "arm with 50 ms duration", why);
		std::this_thread::sleep_for(std::chrono::milliseconds(70));
		tx.step_once(&why);
		c.check(!tx.armed() && !m.running() && why == "auto-disarm", "auto-disarm after the duration", why);
		r.duration_s = 0; r.max_bursts = 2;
		c.check(tx.arm(r, why), "re-arm with max_bursts 2", why);
		tx_timing tm; m.timing(tm);
		for(int i = 0; i < 6; i++) {
			m.set_fake_now(tm.t0_ext + (uint64_t)((double)i * 100000.0 * (double)tm.r_num / (double)tm.r_den));
			for(int k = 0; k < 4; k++) if(!tx.step_once(&why)) break;
		}
		c.check(!tx.armed() && tx.status().bursts_sent == 2, "auto-disarm after max_bursts", "sent " + std::to_string(tx.status().bursts_sent));
		c.check(!tx.armed(), "disarm idempotent");
		tx.disarm();
		tx.disarm();
	}
	// ---- threaded loop: arm with the real thread on the mock's wall clock, then disarm from the caller ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		elrs_transmitter tx(m, pc, -20, true);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -40; r.duration_s = 5; r.rate_index = 6;
		std::string why;
		c.check(tx.arm(r, why), "arm threaded on the wall clock", why);
		std::this_thread::sleep_for(std::chrono::milliseconds(120));
		tx_status st = tx.status();
		tx.disarm();
		c.check(st.bursts_sent >= 10 && st.bursts_sent <= 40 && !tx.armed(), "threaded loop paces on the slot grid (120 ms @ 250 Hz)", "sent " + std::to_string(st.bursts_sent) + " dropped " + std::to_string(st.bursts_dropped));
	}
	// ---- board.conf policy parser ----
	{
		const char *path = "/tmp/elrsd_selftest_board.conf";
		FILE *f = fopen(path, "w");
		if(f) { fputs("BOARD=lime\nTX_ALLOWED=embed\n", f); fclose(f); }
		std::string board;
		tx_policy p = tx_policy_from_boardconf(path, &board);
		c.check(p == tx_policy::EMBED && board == "lime", "board.conf TX_ALLOWED=embed", board + "/" + tx_policy_str(p));
		f = fopen(path, "w");
		if(f) { fputs("BOARD=blue\nTX_ALLOWED=1\n", f); fclose(f); }
		c.check(tx_policy_from_boardconf(path, &board) == tx_policy::ALL, "board.conf TX_ALLOWED=1");
		remove(path);
		c.check(tx_policy_from_boardconf("/nonexistent/board.conf", &board) == tx_policy::NONE && board == "unknown", "missing board.conf = policy none");
	}
}

} // namespace

static void selftest_selfloop(selftest_counter &c) {
	const rate_params &rp = *rate_by_index(6);   // 250 Hz, hop 4
	const uid_t6 uid = test_uid();
	std::array<uint16_t, 16> pat = selfloop_rc_pattern();
	bool in_range = true;
	for(uint16_t v : pat) in_range = in_range && v >= 172 && v <= 1811;
	c.check(in_range && pat == selfloop_rc_pattern() && pat[0] != pat[1], "selfloop: rc pattern deterministic, CRSF range",
	        "ch0 " + std::to_string(pat[0]) + " ch1 " + std::to_string(pat[1]));
	selfloop_checker chk(uid, rp, pat, rp.tlm_denom_default);
	// RC byte-equality for every nonce, a bit flip and a different pattern are rejected
	bool all_rc = true;
	for(int n = 0; n < 256; n++) {
		std::array<uint8_t, 8> p = ota4_build_rc(uid, pat, (uint8_t)n, rp.hop_interval, rp.tlm_denom_default, 2, false);
		if(!chk.rc_expected(p.data())) all_rc = false;
	}
	c.check(all_rc, "selfloop: rc pattern packet byte-equal for all 256 nonces");
	{
		std::array<uint8_t, 8> p = ota4_build_rc(uid, pat, 17, rp.hop_interval, rp.tlm_denom_default, 2, false);
		p[3] ^= 0x01;
		std::array<uint16_t, 16> other = pat;
		other[0] = (uint16_t)(pat[0] == 1000 ? 1001 : 1000);
		std::array<uint8_t, 8> q = ota4_build_rc(uid, other, 17, rp.hop_interval, rp.tlm_denom_default, 2, false);
		uid_t6 uid2 = uid; uid2[5] ^= 0x10;
		std::array<uint8_t, 8> r = ota4_build_rc(uid2, pat, 17, rp.hop_interval, rp.tlm_denom_default, 2, false);
		c.check(!chk.rc_expected(p.data()) && !chk.rc_expected(q.data()) && !chk.rc_expected(r.data()),
		        "selfloop: rc bit flip / other pattern / other uid rejected");
	}
	// SYNC: (ptr, nonce) from the transmitter timeline accepted, off-timeline and other uid rejected
	{
		link_model lm(uid, rp);
		sync_packet sp0; sp0.fhss_index = 0; sp0.nonce = 0; sp0.rate_index = rp.index;
		lm.lock_from_sync(0, sp0);
		bool ok = true;
		for(int64_t s : {0LL, 3LL, 4LL, 255LL, 256LL, 959LL, 960LL, 5000LL}) {
			std::array<uint8_t, 8> p = ota4_build_sync(uid, (uint8_t)lm.fhss_ptr_at(s), lm.nonce_at(s), rp.index, 2, false, 0);
			if(!chk.sync_expected(p.data())) ok = false;
		}
		c.check(ok, "selfloop: sync (ptr, nonce) on the tx timeline byte-equal");
		// slot 5: nonce 5, ptr 1 (hop 4); ptr 7 with nonce 5 is not on the timeline
		std::array<uint8_t, 8> bad = ota4_build_sync(uid, 7, 5, rp.index, 2, false, 0);
		uid_t6 uid2 = uid; uid2[4] ^= 0x01;
		std::array<uint8_t, 8> bad2 = ota4_build_sync(uid2, (uint8_t)lm.fhss_ptr_at(5), lm.nonce_at(5), rp.index, 2, false, 0);
		std::array<uint8_t, 8> bad3 = ota4_build_sync(uid, (uint8_t)lm.fhss_ptr_at(5), lm.nonce_at(5), rp.index, 3 /* other tlm field */, false, 0);
		c.check(!chk.sync_expected(bad.data()) && !chk.sync_expected(bad2.data()) && !chk.sync_expected(bad3.data()),
		        "selfloop: sync off-timeline / other uid / other tlm rejected");
	}
	// counting: 3 rc bit-exact, 1 rc mismatch, 1 crc fail, 1 sync ok, 1 undecodable, 1 msp
	{
		selfloop_checker cc(uid, rp, pat, rp.tlm_denom_default);
		for(int n = 0; n < 3; n++) {
			std::array<uint8_t, 8> p = ota4_build_rc(uid, pat, (uint8_t)(n * 7), rp.hop_interval, rp.tlm_denom_default, 2, false);
			cc.on_packet("rc", true, p.data());
		}
		std::array<uint16_t, 16> other = pat; other[2] = (uint16_t)(pat[2] + 1);
		std::array<uint8_t, 8> m = ota4_build_rc(uid, other, 1, rp.hop_interval, rp.tlm_denom_default, 2, false);
		cc.on_packet("rc", true, m.data());
		cc.on_packet("rc", false, m.data());
		std::array<uint8_t, 8> sy = ota4_build_sync(uid, 0, 0, rp.index, 2, false, 0);
		cc.on_packet("sync", true, sy.data());
		cc.on_packet("undecodable", false, nullptr);
		cc.on_packet("msp", true, m.data());
		selfloop_counts k = cc.counts();
		bool ok = k.packets == 8 && k.undecodable == 1 && k.crc_ok == 6 && k.crc_fail == 1 && k.rc == 4 && k.sync == 1 && k.other == 1 &&
		          k.rc_bitexact == 3 && k.sync_bitexact == 1 && k.mismatch == 2 && !k.ok();
		c.check(ok, "selfloop: counting (bit-exact, mismatch, crc fail, undecodable, foreign kind)", selfloop_counts_json(k));
		selfloop_checker cd(uid, rp, pat, rp.tlm_denom_default);
		std::array<uint8_t, 8> p = ota4_build_rc(uid, pat, 9, rp.hop_interval, rp.tlm_denom_default, 2, false);
		cd.on_packet("rc", true, p.data());
		cd.on_packet("sync", true, sy.data());
		cd.on_packet("undecodable", false, nullptr);
		std::string j = selfloop_counts_json(cd.counts());
		c.check(cd.counts().ok() && j.find("\"ok\":true") != std::string::npos && j.find("\"rc_bitexact\":1") != std::string::npos &&
		        j.find("\"sync_bitexact\":1") != std::string::npos && j.find("\"mismatch\":0") != std::string::npos,
		        "selfloop: ok verdict + json", j);
		selfloop_counts empty;
		c.check(!empty.ok(), "selfloop: no packet = not ok");
	}
	// hex round trip (packet_entry.hex is what the daemon feeds in)
	{
		uint8_t b[8];
		bool ok = selfloop_checker::hex_to_bytes("00a1FF7e10203040", b) && b[0] == 0x00 && b[1] == 0xa1 && b[2] == 0xff && b[3] == 0x7e && b[7] == 0x40;
		bool bad = !selfloop_checker::hex_to_bytes("00a1ff7e1020304", b) && !selfloop_checker::hex_to_bytes("00a1ff7e1020304g", b);
		c.check(ok && bad, "selfloop: hex_to_bytes");
	}
	// 50 Hz (hop 2) timeline as well: the pair table must follow the rate's hop interval
	{
		const rate_params &rp9 = *rate_by_index(9);
		selfloop_checker c9(uid, rp9, pat, rp9.tlm_denom_default);
		link_model lm(uid, rp9);
		sync_packet sp0; sp0.fhss_index = 0; sp0.nonce = 0; sp0.rate_index = rp9.index;
		lm.lock_from_sync(0, sp0);
		std::array<uint8_t, 8> good = ota4_build_sync(uid, (uint8_t)lm.fhss_ptr_at(7), lm.nonce_at(7), rp9.index, 4, false, 0);
		std::array<uint8_t, 8> bad = ota4_build_sync(uid, 1, 7, rp9.index, 4, false, 0);   // hop 2: nonce 7 sits at ptr 3
		c.check(c9.sync_expected(good.data()) && !c9.sync_expected(bad.data()), "selfloop: hop-2 timeline pairs");
	}
	// adapter gate: a power_range.min above the cap is refused (mock mirrors tx_rfnm.cpp)
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_power_range(-10, 10);
		tx_start_params p; p.power_dbm = -60; p.power_cap_dbm = -20;
		uint64_t pos0 = 0;
		bool started = m.start(p, &pos0);
		tx_health h; m.health(h);
		c.check(!started && h.refusals == 1 && !m.running(), "adapter refuses start() when power_range.min is above the cap", h.last_error);
	}
}

// --tx-only (): pure transmit, NO RX path. The feed session is anchored IMMEDIATELY in
// arm(start_session=true) (there is no RX apply to move the anchor, so no deferred session_start).
// This mirrors daemon_main.cpp: run_tx_only on the host mock. On the rfnm adapter "samples at the
// DAC" is health.pos_placed; the mock's equivalent is the bursts placed on its feed core (writes /
// captured), so the test asserts those.
static void selftest_tx_only(selftest_counter &c) {
	const uint64_t TICK_HZ = 61440000ULL;
	tx_plan_config pc;
	const rate_params *rp = rate_by_index(6);   // 250 Hz, hop 4
	const uint64_t slot_n = 100000;
	// ---- happy path: every gate passes, arm brings up RF AND the session at once, bursts flow, disarm stops ----
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_fake_now(TICK_HZ);
		elrs_transmitter tx(m, pc, -20, /*enabled*/ true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -100; r.antenna = "sma_a"; r.duration_s = 0; r.rate_index = rp->index; r.sync_packets = true; r.tlm_denom = rp->tlm_denom_default;
		std::string why;
		bool armed = tx.arm(r, why, /*start_session=*/true);
		tx_status s0 = tx.status();
		c.check(armed && tx.armed() && m.running() && s0.session && m.session_running() && m.session_starts() == 1,
		        "tx-only: arm(start_session=true) brings RF up AND the feed session at once (no deferred session_start)",
		        why + " session " + std::to_string(s0.session) + " starts " + std::to_string(m.session_starts()));
		int pmin = 0, pmax = 0; m.power_range(&pmin, &pmax);
		c.check(s0.power_dbm_applied == pmin && s0.power_dbm_applied <= -20,
		        "tx-only: -100 dBm request clamped up to power_range.min, under the cap", std::to_string(s0.power_dbm_applied));
		// drive the scheduler across the slot grid; bursts must land on the feed (the DAC)
		tx_timing tm; m.timing(tm);
		auto samples_to_ticks = [&](double smp) { return (uint64_t)(smp * (double)tm.r_num / (double)tm.r_den); };
		for(int i = 0; i < 10; i++) { if(!tx.step_once(&why)) break; }
		uint64_t pos_base = m.captured().empty() ? 0 : m.captured()[0].pos;
		for(int64_t sl = 0; sl < 200; sl++) {
			uint64_t pos_now = pos_base + (uint64_t)sl * slot_n;
			m.set_fake_now(tm.t0_ext + samples_to_ticks((double)pos_now) - 1000);
			for(int i = 0; i < 8; i++) if(!tx.step_once(&why)) break;
		}
		tx_status st = tx.status();
		c.check(st.bursts_sent > 0 && st.bursts_sent == m.captured().size(),
		        "tx-only: bursts placed on the DAC (mock feed = pos_placed on the device)", "sent " + std::to_string(st.bursts_sent) + " captured " + std::to_string(m.captured().size()));
		c.check(st.syncs_sent >= 1, "tx-only: SYNC packets sent on channel-41 visits", std::to_string(st.syncs_sent));
		c.check(st.health.writes > 0 && st.health.bursts_fed + st.health.bursts_queued > 0,
		        "tx-only: feed health advances (writes / bursts_fed)", "writes " + std::to_string(st.health.writes) + " fed " + std::to_string(st.health.bursts_fed));
		tx.disarm();
		c.check(!tx.armed() && !m.running() && !m.session_running(), "tx-only: disarm stops the radio and the feed session");
	}
	// ---- R = 4/2 (tick_hz = 2*fs) with a re-minted anchor: the observed anchor re-mint ----------------------
	// The device reclock (set_samp_rate at DCS 200 MHz) re-minted the TX anchor so the feed frontier
	// pos0 sat ~2^31 samples above (now - t0_ext)*R while t0_ext stayed put: the scheduler read every
	// burst as "ahead" (tx sent 0, feed fed 0, frontier lead 85 s). With the axis re-basing in
	// lay_grid the scheduler puts real bursts on the frontier axis again.
	{
		radio_tx_mock m;
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		m.set_clock(50000000ULL, 4, 2);              // tick_hz = 2*fs, report R = 4/2 unreduced
		const int64_t skew = (int64_t)1 << 31;       // ~85.9 s at 25 MSps: the measured phantom lead
		m.set_feed_axis_skew(skew);
		m.set_fake_now(50000000ULL);
		elrs_transmitter tx(m, pc, -20, /*enabled*/ true);
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		tx_arm_request r; r.power_dbm = -100; r.antenna = "sma_a"; r.duration_s = 0; r.rate_index = rp->index; r.sync_packets = true; r.tlm_denom = rp->tlm_denom_default;
		std::string why;
		bool armed = tx.arm(r, why, /*start_session=*/true);
		tx_timing tm; m.timing(tm);
		c.check(armed && tm.r_num == 4 && tm.r_den == 2 && tm.tick_hz == 50000000ULL,
		        "tx-only R4/2: armed, device reports tick_hz = 2*fs and R = 4/2", why + " R " + std::to_string(tm.r_num) + "/" + std::to_string(tm.r_den));
		uint64_t frontier0 = m.pos();                // feed frontier at session start, ~2^31 up the axis
		c.check(frontier0 >= (uint64_t)skew, "tx-only R4/2: feed frontier sits on the skewed axis (~2^31)", std::to_string(frontier0));
		// drive air-now forward in slot steps: to advance air-now by S samples, advance the tick clock
		// by S * r_num/r_den ticks (= S*2 here). pos_now must track the frontier, not the tick anchor.
		auto samples_to_ticks = [&](double smp) { return (uint64_t)(smp * (double)tm.r_num / (double)tm.r_den); };
		for(int64_t sl = 0; sl < 200; sl++) {
			m.set_fake_now(tm.t0_ext + samples_to_ticks((double)sl * (double)slot_n));
			for(int i = 0; i < 8; i++) if(!tx.step_once(&why)) break;
		}
		uint64_t pos_base = m.captured().empty() ? 0 : m.captured().front().pos;
		tx_status st = tx.status();
		c.check(st.bursts_sent > 0 && st.bursts_sent == m.captured().size(),
		        "tx-only R4/2: scheduler places real bursts despite the 85 s anchor skew (tx sent > 0)",
		        "sent " + std::to_string(st.bursts_sent) + " captured " + std::to_string(m.captured().size()));
		c.check(st.syncs_sent >= 1, "tx-only R4/2: SYNC packets still placed", std::to_string(st.syncs_sent));
		// the grid sits ON the frontier axis: the first burst is a small, bounded lead above the
		// frontier (pace + a few slots), NOT ~2^31 samples ahead of it (the 85 s runaway).
		c.check(!m.captured().empty() && pos_base > frontier0 && (pos_base - frontier0) < 10 * slot_n,
		        "tx-only R4/2: first burst is pace+a few slots ahead of the frontier, not 85 s",
		        m.captured().empty() ? "no bursts" : ("burst0 " + std::to_string(pos_base) + " - frontier " + std::to_string(frontier0) + " = " + std::to_string(pos_base - frontier0) + " samples"));
		// bursts land on a strict slot grid on the skewed axis
		bool grid_ok = true; uint64_t lp = 0;
		for(auto &e : m.captured()) { if((e.pos - pos_base) % slot_n != 0) grid_ok = false; if(lp && e.pos <= lp) grid_ok = false; lp = e.pos; }
		c.check(grid_ok, "tx-only R4/2: bursts on the slot grid, strictly increasing on the skewed axis", std::to_string(m.captured().size()) + " bursts");
		// the DAC actually receives them: drain the stream from the frontier and confirm fed > 0
		std::vector<int16_t> stream;
		m.drain(stream, (size_t)(m.captured().back().pos + slot_n - frontier0));
		st = tx.status();
		c.check(st.health.bursts_fed > 0 && st.health.bursts_late == 0,
		        "tx-only R4/2: bursts reach the DAC feed (bursts_fed > 0, none late)",
		        "fed " + std::to_string(st.health.bursts_fed) + " late " + std::to_string(st.health.bursts_late));
		tx.disarm();
	}
	// ---- gate refusals before any transmit (run_tx_only's gate order: policy, --tx 1, uid, cap) ----
	{
		struct gcase { tx_policy pol; bool enabled; bool uid; int power; const char *name; };
		std::vector<gcase> cases = {
			{ tx_policy::NONE, true, true, -100, "tx-only refused: board policy none" },
			{ tx_policy::ALL, false, true, -100, "tx-only refused: --tx 0 (not enabled)" },
			{ tx_policy::ALL, true, false, -100, "tx-only refused: no uid" },
			{ tx_policy::ALL, true, true, 10, "tx-only refused: power above the cap" },
		};
		for(auto &g : cases) {
			radio_tx_mock m;
			m.set_policy(g.pol);
			m.set_paths({"sma_a"}, "sma_a");
			m.set_fake_now(TICK_HZ);
			elrs_transmitter tx(m, pc, -20, g.enabled);
			tx.set_threaded(false);
			if(g.uid) tx.set_uid(test_uid());
			tx_arm_request r; r.power_dbm = g.power; r.antenna = "sma_a"; r.duration_s = 0; r.rate_index = rp->index;
			std::string why;
			bool armed = tx.arm(r, why, /*start_session=*/true);
			c.check(!armed && !tx.armed() && !m.running() && !m.session_running(), g.name, why);
		}
	}
}

// the continuous feed: what the DAC receives is a gapless stream with the bursts at their exact
// sample positions and explicit zeros (or the CW tone) everywhere else
static void selftest_feed(selftest_counter &c) {
	// ---- tx_feed core: positions, gaps, chunking, refusals ----
	{
		tx_feed f(1000);
		std::vector<int16_t> a(2 * 700), b(2 * 300);
		for(size_t i = 0; i < 700; i++) { a[2 * i] = (int16_t)(100 + i); a[2 * i + 1] = (int16_t)(-(int)i); }
		for(size_t i = 0; i < 300; i++) { b[2 * i] = 7; b[2 * i + 1] = (int16_t)(i * 3); }
		std::string err;
		c.check(!f.queue_burst(999, a.data(), 700, &err), "feed: burst behind the frontier refused", err);
		err.clear();
		c.check(f.queue_burst(1536, a.data(), 700, &err), "feed: burst at 1536 queued", err);
		c.check(!f.queue_burst(2000, b.data(), 300, &err), "feed: overlapping burst refused", err);
		c.check(f.queue_burst(4096, b.data(), 300, &err) && f.pending() == 2, "feed: second burst at 4096 queued");
		std::vector<int16_t> stream;
		std::vector<int16_t> chunk(2 * 1024);
		while(f.frontier() < 6000) {
			size_t n = f.next(chunk.data(), 1024);
			if(!n) break;
			stream.insert(stream.end(), chunk.begin(), chunk.begin() + 2 * n);
		}
		size_t total = stream.size() / 2;
		auto zero_span = [&](size_t s0, size_t s1) { for(size_t i = s0; i < s1; i++) if(stream[2 * i] || stream[2 * i + 1]) return false; return true; };
		auto eq_span = [&](size_t s0, const std::vector<int16_t> &v) { return memcmp(stream.data() + 2 * s0, v.data(), v.size() * 2) == 0; };
		c.check(total >= 5000 && f.frontier() == 1000 + total, "feed: frontier advances by exactly the emitted samples", std::to_string(total));
		c.check(zero_span(0, 536) && eq_span(536, a) && zero_span(1236, 3096) && eq_span(3096, b) && zero_span(3396, total),
		        "feed: stream = zeros | burst@1536 | zeros | burst@4096 | zeros (sample-exact, across 1024-chunks)");
		tx_feed::stats st = f.get_stats();
		c.check(st.queued == 2 && st.fed == 2 && st.refused_late == 2 && st.burst_samples == 1000 && st.zero_samples == total - 1000 && f.pending() == 0,
		        "feed: stats (queued 2, fed 2, late 2, burst 1000 samples, rest zeros)");
		c.check(!f.queue_burst(5000, b.data(), 300, &err), "feed: position already written (behind the frontier) refused", err);
		c.check(f.queue_burst(f.frontier(), b.data(), 300, &err), "feed: burst exactly at the frontier accepted");
		size_t n = f.next(chunk.data(), 100);
		c.check(n == 100 && chunk[0] == 7 && chunk[1] == 0 && chunk[198] == 7 && chunk[199] == 99 * 3, "feed: a burst is emitted piecewise when the chunk is smaller");
		f.reset(50000);
		c.check(f.pending() == 0 && f.frontier() == 50000, "feed: reset drops pending bursts and moves the frontier");
	}
	// ---- idle tone (--selfloop-cw): amplitude and frequency, phase-continuous across chunks ----
	{
		tx_feed f(0);
		const double fs = 25e6, off = 1e6;
		f.set_idle_tone(off / fs, 14000);
		std::vector<int16_t> chunk(2 * 1024), stream;
		for(int k = 0; k < 4; k++) { size_t n = f.next(chunk.data(), 1000); stream.insert(stream.end(), chunk.begin(), chunk.begin() + 2 * n); }
		bool amp_ok = true, freq_ok = true;
		for(size_t i = 1; i < stream.size() / 2; i++) {
			std::complex<double> z0((double)stream[2 * (i - 1)], (double)stream[2 * (i - 1) + 1]), z1((double)stream[2 * i], (double)stream[2 * i + 1]);
			if(std::fabs(std::abs(z1) - 14000.0) > 2.0) amp_ok = false;
			double fi = std::arg(z1 * std::conj(z0)) * fs / (2.0 * M_PI);
			if(std::fabs(fi - off) > 200.0) freq_ok = false;   // also across the 1000-sample chunk seams
		}
		c.check(amp_ok && freq_ok, "feed: idle tone 14000 @ LO+1 MHz, phase-continuous across chunks");
		f.set_idle_tone(0, 0);
		size_t n = f.next(chunk.data(), 64);
		bool z = true; for(size_t i = 0; i < 2 * n; i++) if(chunk[i]) z = false;
		c.check(n == 64 && z, "feed: idle fill back to zeros");
	}
	tx_plan_config pc;
	auto arm_on_mock = [&](radio_tx_mock &m, elrs_transmitter &tx, tx_arm_request &r, bool start_session, std::string &why) {
		m.set_policy(tx_policy::ALL);
		m.set_paths({"sma_a"}, "sma_a");
		tx.set_threaded(false);
		tx.set_uid(test_uid());
		r.power_dbm = -40; r.duration_s = 30; r.rate_index = 6;
		return tx.arm(r, why, start_session);
	};
	// ---- end to end: scheduler -> mock -> drained "DAC" stream carries the bursts at their feed positions ----
	{
		radio_tx_mock m;
		elrs_transmitter tx(m, pc, -20, true);
		tx_arm_request r;
		std::string why;
		c.check(arm_on_mock(m, tx, r, true, why) && tx.status().session, "feed e2e: armed with a session", why);
		tx_timing tm; m.timing(tm);
		auto to_tick = [&](double samples) { return tm.t0_ext + (uint64_t)(samples * (double)tm.r_num / (double)tm.r_den); };
		// let the clock run slot by slot until two bursts are queued
		uint64_t slot_n = 100000;
		for(int i = 0; i < 40 && m.captured().size() < 2; i++) { m.set_fake_now(to_tick((double)i * slot_n / 4)); tx.step_once(&why); }
		const std::vector<mock_tx_event> ev = m.captured();
		c.check(ev.size() >= 2, "feed e2e: two bursts queued", std::to_string(ev.size()));
		if(ev.size() >= 2) {
			std::vector<int16_t> stream;
			uint64_t base = m.pos();                        // frontier before the drain = first sample of the stream
			uint64_t upto = ev[1].pos + ev[1].iq.size() / 2 + 5000;
			m.drain(stream, (size_t)(upto - base));
			auto at = [&](uint64_t pos) { return (size_t)(pos - base); };
			bool b0 = memcmp(stream.data() + 2 * at(ev[0].pos), ev[0].iq.data(), ev[0].iq.size() * 2) == 0;
			bool b1 = memcmp(stream.data() + 2 * at(ev[1].pos), ev[1].iq.data(), ev[1].iq.size() * 2) == 0;
			bool gap = true;
			for(size_t i = at(ev[0].pos + ev[0].iq.size() / 2); i < at(ev[1].pos); i++) if(stream[2 * i] || stream[2 * i + 1]) gap = false;
			bool head = true;
			for(size_t i = 0; i < at(ev[0].pos); i++) if(stream[2 * i] || stream[2 * i + 1]) head = false;
			tx_status st = tx.status();
			c.check(b0 && b1 && gap && head, "feed e2e: DAC stream = zeros | burst k | zeros | burst k+1 at the planned feed positions",
			        "pos " + std::to_string(ev[0].pos) + " / " + std::to_string(ev[1].pos) + " base " + std::to_string(base));
			c.check(st.health.bursts_queued == 2 && st.health.bursts_fed == 2 && st.health.bursts_late == 0 && st.health.burst_samples == ev[0].iq.size() / 2 + ev[1].iq.size() / 2,
			        "feed e2e: health counts queued/fed/late/burst_samples", "fed " + std::to_string(st.health.bursts_fed) + " late " + std::to_string(st.health.bursts_late));
			// a burst behind the frontier is refused by the adapter and counted as dropped by the scheduler
			std::vector<int16_t> z(2 * 256, 1);
			c.check(!m.write_at(base, z.data(), 256) && tx.status().health.bursts_late == 1, "feed e2e: burst behind the frontier refused + counted late");
		}
		tx.disarm();
	}
	// ---- deferred session (self-loop order: RF up, RX apply, THEN anchor) ----
	{
		radio_tx_mock m;
		elrs_transmitter tx(m, pc, -20, true);
		tx_arm_request r;
		std::string why;
		bool armed = arm_on_mock(m, tx, r, false, why);
		tx_status st = tx.status();
		c.check(armed && st.armed && !st.session && m.running() && !m.session_running(), "feed deferred: armed, RF up, no session", why);
		bool consumed = tx.step_once(&why);
		c.check(!consumed && why == "no session" && m.captured().empty(), "feed deferred: scheduler idles without a session", why);
		c.check(tx.session_start(why) && tx.status().session && m.session_running() && m.session_starts() == 1, "feed deferred: session_start anchors + lays the grid", why);
		c.check(!tx.session_start(why), "feed deferred: second session_start refused", why);
		tx_timing tm; m.timing(tm);
		for(int i = 0; i < 40 && m.captured().empty(); i++) { m.set_fake_now(tm.t0_ext + (uint64_t)((double)i * 25000.0 * (double)tm.r_num / (double)tm.r_den)); tx.step_once(&why); }
		c.check(!m.captured().empty() && m.captured()[0].pos >= m.pos(), "feed deferred: bursts follow on the session's grid", why);
		tx.disarm();
		c.check(!tx.armed() && !m.session_running() && !m.running(), "feed deferred: disarm stops session and RF");
	}
	// ---- anchor moved (device re-mint) -> rebase: new session, grid re-laid, bursts continue ----
	{
		radio_tx_mock m;
		elrs_transmitter tx(m, pc, -20, true);
		tx_arm_request r;
		std::string why;
		c.check(arm_on_mock(m, tx, r, true, why), "feed rebase: armed", why);
		tx_timing tm; m.timing(tm);
		for(int i = 0; i < 40 && m.captured().empty(); i++) { m.set_fake_now(tm.t0_ext + (uint64_t)((double)i * 25000.0 * (double)tm.r_num / (double)tm.r_den)); tx.step_once(&why); }
		size_t before = m.captured().size();
		m.set_anchor_moved(true);
		bool consumed = tx.step_once(&why);
		tx_status st = tx.status();
		c.check(!consumed && why == "rebased" && m.session_starts() == 2 && st.health.rebases == 1 && st.health.anchor_moves == 1 && tx.armed(),
		        "feed rebase: anchor move -> session restarted once, counted, still armed", why + " starts " + std::to_string(m.session_starts()));
		m.timing(tm);
		uint64_t frontier = m.pos();
		for(int i = 0; i < 40 && m.captured().size() == before; i++) { m.set_fake_now(tm.t0_ext + (uint64_t)((double)i * 25000.0 * (double)tm.r_num / (double)tm.r_den)); tx.step_once(&why); }
		c.check(m.captured().size() > before && m.captured().back().pos >= frontier, "feed rebase: bursts continue on the new grid ahead of the new frontier", why);
		tx.disarm();
	}
	// ---- cw_only: no bursts, the tone is the only content, every gate still applies ----
	{
		radio_tx_mock m;
		elrs_transmitter tx(m, pc, -20, true);
		tx_arm_request r;
		r.cw_only = true; r.cw_offset_hz = 1e6; r.cw_amplitude = 14000;
		std::string why;
		c.check(arm_on_mock(m, tx, r, true, why) && tx.status().cw, "feed cw: armed with cw_only", why);
		tx_timing tm; m.timing(tm);
		for(int i = 0; i < 20; i++) { m.set_fake_now(tm.t0_ext + (uint64_t)((double)i * 100000.0 * (double)tm.r_num / (double)tm.r_den)); tx.step_once(&why); }
		std::vector<int16_t> stream;
		m.drain(stream, 4000);
		bool tone = stream.size() == 8000;
		for(size_t i = 1; i < stream.size() / 2 && tone; i++) {
			std::complex<double> z0((double)stream[2 * (i - 1)], (double)stream[2 * (i - 1) + 1]), z1((double)stream[2 * i], (double)stream[2 * i + 1]);
			double fi = std::arg(z1 * std::conj(z0)) * pc.fs_hz / (2.0 * M_PI);
			if(std::fabs(std::abs(z1) - 14000.0) > 2.0 || std::fabs(fi - 1e6) > 200.0) tone = false;
		}
		c.check(why == "cw" && m.captured().empty() && tone, "feed cw: no bursts, drained stream is the 14000 @ +1 MHz tone", why);
		radio_tx_mock m2;
		m2.set_policy(tx_policy::NONE);
		elrs_transmitter tx2(m2, pc, -20, true);
		tx2.set_uid(test_uid());
		c.check(!tx2.arm(r, why), "feed cw: policy none refuses the cw arm too", why);
		tx.disarm();
	}
}

int run_selftest_tx() {
	selftest_counter c;
	selftest_synth(c);
	selftest_planner(c);
	selftest_transmitter(c);
	selftest_selfloop(c);
	selftest_tx_only(c);
	selftest_feed(c);
	if(run_selftest_tx_loopback) {
		run_selftest_tx_loopback(c);   // its checks already count in c
	} else {
		printf("[SKIP] tx loopback through rx_chain: src/tx_selftest.cpp not linked (B's rx_chain missing)\n");
	}
	printf("tx selftest: %d pass, %d fail\n", c.passes, c.fails);
	return c.fails;
}

} // namespace elrsd

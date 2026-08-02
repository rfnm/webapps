#include "ais_phy.h"

#include <cmath>

namespace aisd {

// ---- CRC / coding ----------------------------------------------------------------

uint16_t crc16_x25(const bits &b) {
	uint16_t crc = 0xFFFF;
	for(uint8_t bit : b) {
		uint16_t x = (uint16_t)((crc ^ bit) & 1);
		crc >>= 1;
		if(x) {
			crc ^= 0x8408;   // reflected 0x1021
		}
	}
	return (uint16_t)(crc ^ 0xFFFF);
}

bits nrzi_encode(const bits &data, int init_level) {
	bits out;
	out.reserve(data.size());
	int level = init_level & 1;
	for(uint8_t d : data) {
		if(!d) {
			level ^= 1;   // a data 0 toggles the line level
		}
		out.push_back((uint8_t)level);
	}
	return out;
}

bits nrzi_decode(const bits &line, int init_level) {
	bits out;
	out.reserve(line.size());
	int prev = init_level & 1;
	for(uint8_t l : line) {
		out.push_back((uint8_t)(l == prev ? 1 : 0));   // no transition -> 1
		prev = l;
	}
	return out;
}

bits bit_stuff(const bits &in) {
	bits out;
	out.reserve(in.size() + in.size() / 5 + 1);
	int ones = 0;
	for(uint8_t b : in) {
		out.push_back(b);
		if(b) {
			if(++ones == 5) {
				out.push_back(0);   // stuff a 0 after five 1s
				ones = 0;
			}
		} else {
			ones = 0;
		}
	}
	return out;
}

bits logical_to_wire(const bits &logical) {
	bits out(logical.size());
	for(size_t byte = 0; byte + 8 <= logical.size(); byte += 8) {
		for(int i = 0; i < 8; i++) {
			out[byte + i] = logical[byte + 7 - i];
		}
	}
	// tail that is not a full byte (should not happen for byte-aligned AIS) passes through
	for(size_t i = logical.size() & ~size_t(7); i < logical.size(); i++) {
		out[i] = logical[i];
	}
	return out;
}

static const uint8_t FLAG_BITS[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };   // 0x7E

bits build_frame(const bits &wire_msg, int preamble_bits) {
	bits df = wire_msg;
	uint16_t fcs = crc16_x25(wire_msg);
	for(int i = 0; i < 16; i++) {
		df.push_back((uint8_t)((fcs >> i) & 1));   // FCS appended LSB-first
	}
	bits stuffed = bit_stuff(df);

	bits frame;
	frame.reserve((size_t)preamble_bits + 16 + stuffed.size());
	for(int i = 0; i < preamble_bits; i++) {
		frame.push_back((uint8_t)(i & 1));   // 0,1,0,1,... training sequence
	}
	for(int i = 0; i < 8; i++) {
		frame.push_back(FLAG_BITS[i]);
	}
	frame.insert(frame.end(), stuffed.begin(), stuffed.end());
	for(int i = 0; i < 8; i++) {
		frame.push_back(FLAG_BITS[i]);
	}
	return frame;
}

// ---- GMSK modulator ---------------------------------------------------------------

// Gaussian phase pulse Psi(x), x in symbol units: 0 for x <= -L/2, 1 for x >= +L/2,
// the integral of the unit-area Gaussian frequency pulse in between. Built once.
static const std::vector<float> &gmsk_phase_pulse(int &L_out) {
	static const int L = 4;
	static std::vector<float> psi;
	static const int STEPS = 4096;   // samples across [-L/2, L/2]
	L_out = L;
	if(!psi.empty()) {
		return psi;
	}
	auto Q = [](double z) { return 0.5 * erfc(z / std::sqrt(2.0)); };
	const double k = 2.0 * M_PI * AIS_BT / std::sqrt(std::log(2.0));
	std::vector<double> g(STEPS + 1);
	double area = 0;
	for(int i = 0; i <= STEPS; i++) {
		double x = -L / 2.0 + (double)i * L / STEPS;                  // symbol units
		g[i] = 0.5 * (Q(k * (x - 0.5)) - Q(k * (x + 0.5)));           // unit-area freq pulse
		area += g[i];
	}
	psi.resize(STEPS + 1);
	double acc = 0;
	for(int i = 0; i <= STEPS; i++) {
		acc += g[i];
		psi[i] = (float)(acc / area);                                // normalized 0..1
	}
	return psi;
}

static inline double psi_eval(const std::vector<float> &psi, int L, double x) {
	if(x <= -L / 2.0) {
		return 0.0;
	}
	if(x >= L / 2.0) {
		return 1.0;
	}
	double f = (x + L / 2.0) / L * (psi.size() - 1);
	int i = (int)f;
	double frac = f - i;
	return psi[i] * (1.0 - frac) + psi[i + 1] * frac;
}

std::vector<cf> gmsk_modulate(const bits &line, double fs, double baud) {
	int L = 0;
	const std::vector<float> &psi = gmsk_phase_pulse(L);
	int nsym = (int)line.size();
	if(nsym == 0) {
		return {};
	}
	double sps = fs / baud;
	// prefix[j] = sum of NRZ symbols a[0..j-1]; a symbol fully in the past contributes a
	// completed phase step (Psi -> 1), so its full a[k] must be summed, not skipped -- only
	// the L symbols straddling the current time use the partial phase pulse
	std::vector<int> prefix(nsym + 1, 0);
	for(int k = 0; k < nsym; k++) {
		prefix[k + 1] = prefix[k] + (line[k] ? 1 : -1);
	}
	// run a couple of symbols past the end so the last pulse tail completes
	size_t nout = (size_t)std::llround((nsym + L) * sps);
	std::vector<cf> out(nout);
	for(size_t n = 0; n < nout; n++) {
		double tsym = (double)n / sps;                 // time in symbol units
		int k0 = (int)std::floor(tsym - L / 2.0);
		int k1 = (int)std::ceil(tsym + L / 2.0);
		if(k0 < 0) {
			k0 = 0;
		}
		if(k1 > nsym - 1) {
			k1 = nsym - 1;
		}
		// completed symbols strictly before the window each add a full +-1 step
		double acc = (k0 > 0) ? (double)prefix[k0] : 0.0;
		for(int k = k0; k <= k1; k++) {
			int a = line[k] ? 1 : -1;
			acc += a * psi_eval(psi, L, tsym - k);
		}
		double phase = M_PI * AIS_H * acc;
		out[n] = cf((float)std::cos(phase), (float)std::sin(phase));
	}
	return out;
}

// ---- HDLC deframer ----------------------------------------------------------------

static inline bool is_flag_at(const bits &b, size_t p) {
	for(int i = 0; i < 8; i++) {
		if(b[p + i] != FLAG_BITS[i]) {
			return false;
		}
	}
	return true;
}

// remove a 0 that immediately follows five consecutive 1s
static bits destuff(const bits &content) {
	bits out;
	out.reserve(content.size());
	int ones = 0;
	for(uint8_t b : content) {
		if(ones == 5 && b == 0) {
			ones = 0;
			continue;
		}
		out.push_back(b);
		ones = b ? ones + 1 : 0;
	}
	return out;
}

void hdlc_rx::reset() {
	buf_.clear();
	last_flag_end_ = -1;
}

void hdlc_rx::push_bit(int b) {
	buf_.push_back((uint8_t)(b & 1));
	if(buf_.size() >= 8 && is_flag_at(buf_, buf_.size() - 8)) {
		long flag_start = (long)buf_.size() - 8;
		if(last_flag_end_ >= 0 && flag_start >= last_flag_end_) {
			bits content(buf_.begin() + last_flag_end_, buf_.begin() + flag_start);
			bits data = destuff(content);
			// data = wire_msg + 16-bit FCS; need a byte-aligned message and a matching CRC
			if(data.size() >= 40 && ((data.size() - 16) % 8) == 0) {
				size_t mlen = data.size() - 16;
				bits msg(data.begin(), data.begin() + mlen);
				uint16_t fcs_rx = 0;
				for(size_t i = 0; i < 16; i++) {
					fcs_rx |= (uint16_t)(data[mlen + i] & 1) << i;   // FCS is LSB-first
				}
				if(crc16_x25(msg) == fcs_rx) {
					on_frame_(msg);
				}
			}
		}
		last_flag_end_ = (long)buf_.size();
	}
	// keep the buffer bounded; a frame spans at most ~1100 bits, so 4096 is ample
	if(buf_.size() > 4096) {
		const size_t drop = 2048;
		buf_.erase(buf_.begin(), buf_.begin() + drop);
		last_flag_end_ = last_flag_end_ >= (long)drop ? last_flag_end_ - (long)drop : -1;
	}
}

// ---- AIVDM armor ------------------------------------------------------------------

static inline char armor_char(int v) {
	int c = v + 48;
	if(c > 87) {
		c += 8;
	}
	return (char)c;
}

static inline int dearmor_char(char c) {
	int v = (int)(unsigned char)c - 48;
	if(v > 40) {
		v -= 8;
	}
	return v & 0x3F;
}

std::string aivdm_armor(const bits &logical, int &fill_out) {
	int fill = (int)((6 - (logical.size() % 6)) % 6);
	fill_out = fill;
	std::string out;
	for(size_t i = 0; i < logical.size() + (size_t)fill; i += 6) {
		int v = 0;
		for(int j = 0; j < 6; j++) {
			size_t idx = i + j;
			v = (v << 1) | (idx < logical.size() ? (logical[idx] & 1) : 0);
		}
		out.push_back(armor_char(v));
	}
	return out;
}

bits aivdm_dearmor(const std::string &payload, int fill) {
	bits out;
	out.reserve(payload.size() * 6);
	for(char c : payload) {
		int v = dearmor_char(c);
		for(int j = 5; j >= 0; j--) {
			out.push_back((uint8_t)((v >> j) & 1));
		}
	}
	if(fill > 0 && (size_t)fill <= out.size()) {
		out.resize(out.size() - fill);
	}
	return out;
}

uint8_t nmea_checksum(const std::string &body) {
	uint8_t cs = 0;
	for(char c : body) {
		cs ^= (uint8_t)c;
	}
	return cs;
}

std::vector<std::string> ais_aivdm(const bits &logical, char channel, int seq) {
	int fill = 0;
	std::string payload = aivdm_armor(logical, fill);
	// fragment so each sentence's armored payload stays well within the 82-char NMEA line
	const size_t MAX_CHARS = 60;
	size_t nfrag = (payload.size() + MAX_CHARS - 1) / MAX_CHARS;
	if(nfrag == 0) {
		nfrag = 1;
	}
	std::vector<std::string> out;
	for(size_t fi = 0; fi < nfrag; fi++) {
		std::string frag = payload.substr(fi * MAX_CHARS, MAX_CHARS);
		int frag_fill = (fi == nfrag - 1) ? fill : 0;   // fill bits only on the last fragment
		char head[80];
		if(nfrag > 1) {
			snprintf(head, sizeof(head), "AIVDM,%zu,%zu,%d,%c,", nfrag, fi + 1, seq, channel);
		} else {
			snprintf(head, sizeof(head), "AIVDM,1,1,,%c,", channel);
		}
		std::string body = std::string(head) + frag + "," + std::to_string(frag_fill);
		char cs[8];
		snprintf(cs, sizeof(cs), "*%02X", nmea_checksum(body));
		out.push_back("!" + body + cs);
	}
	return out;
}

// ---- logical-field accessors ------------------------------------------------------

uint64_t bits_get_uint(const bits &b, size_t start, size_t len) {
	uint64_t v = 0;
	for(size_t i = 0; i < len; i++) {
		uint64_t bit = (start + i < b.size()) ? (b[start + i] & 1) : 0;
		v = (v << 1) | bit;
	}
	return v;
}

int64_t bits_get_int(const bits &b, size_t start, size_t len) {
	uint64_t v = bits_get_uint(b, start, len);
	if(len > 0 && (v & (1ull << (len - 1)))) {
		v |= ~((1ull << len) - 1);   // sign-extend
	}
	return (int64_t)v;
}

std::string bits_get_str(const bits &b, size_t start, size_t nchars) {
	std::string s;
	for(size_t c = 0; c < nchars; c++) {
		int v = (int)bits_get_uint(b, start + c * 6, 6);
		char ch = (v < 32) ? (char)(v + 64) : (char)v;   // AIS 6-bit ASCII
		s.push_back(ch);
	}
	// trim trailing @ (AIS null) and spaces
	size_t end = s.find_last_not_of("@ ");
	s = (end == std::string::npos) ? "" : s.substr(0, end + 1);
	// drop any embedded @ used as padding
	for(char &ch : s) {
		if(ch == '@') {
			ch = ' ';
		}
	}
	return s;
}

void bits_put_uint(bits &b, size_t start, size_t len, uint64_t v) {
	for(size_t i = 0; i < len; i++) {
		if(start + i < b.size()) {
			b[start + i] = (uint8_t)((v >> (len - 1 - i)) & 1);
		}
	}
}

} // namespace aisd

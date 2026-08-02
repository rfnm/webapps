// vrtd: daemon core implementation. See core.h.
#include "core.h"

#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace vrtd {

// byte-swap each 16-bit word — CS16 host-LE <-> VRT big-endian. NEON (vrev16) on the i.MX8MP/A53; the
// scalar tail is the exact reference and non-NEON fallback. src/dst must not alias. Runs at the sample
// rate on RX export + TX ingest, so it is the hot conversion in the VITA-49 data path.
static inline void bswap16_stream(const uint8_t *src, uint8_t *dst, size_t nbytes) {
	size_t i = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
	for(; i + 16 <= nbytes; i += 16) {
		vst1q_u8(dst + i, vrev16q_u8(vld1q_u8(src + i)));
	}
#endif
	for(; i + 2 <= nbytes; i += 2) {
		dst[i] = src[i + 1];
		dst[i + 1] = src[i];
	}
}

// v1 per-field ack status words (documented vendor simplification; the standardized
// 49.2 ack error codes can replace these once cross-checked against the spec text)
static constexpr uint32_t ERR_FIELD_NOT_EXECUTABLE = 0x00000001;
static constexpr uint32_t ERR_HW_REJECTED = 0x00000002;
static constexpr uint32_t ERR_TX_FORBIDDEN = 0x00000003;   // board policy refuses TX (see config.tx_allowed)

static double now_s() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

core::core(const config &cfg, radio *r) : cfg_(cfg), rad_(r) {}

core::~core() {
	stop();
}

bool core::start() {
	if(!cfg_.dest_host.empty()) {
		out_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(out_fd_ < 0) {
			return false;
		}
		dest_.sin_family = AF_INET;
		dest_.sin_port = htons(cfg_.dest_port);
		if(inet_pton(AF_INET, cfg_.dest_host.c_str(), &dest_.sin_addr) != 1) {
			fprintf(stderr, "vrtd: bad destination address %s\n", cfg_.dest_host.c_str());
			return false;
		}
		has_dest_ = true;
	}
	if(cfg_.listen_port) {
		in_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(in_fd_ < 0) {
			return false;
		}
		sockaddr_in a{};
		a.sin_family = AF_INET;
		a.sin_port = htons(cfg_.listen_port);
		a.sin_addr.s_addr = htonl(INADDR_ANY);
		if(bind(in_fd_, (sockaddr *)&a, sizeof(a)) < 0) {
			fprintf(stderr, "vrtd: bind %u failed\n", cfg_.listen_port);
			return false;
		}
		struct timeval tv = { 0, 200000 };
		setsockopt(in_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	}
	run_ = true;
	if(has_dest_) {
		stage_.reserve(2 * cfg_.samples_per_pkt);
		emit_context(false);
		next_ctx_ = now_s() + cfg_.context_interval_s;
		if(!rad_->start_rx([this](const int16_t *iq, size_t n) { on_rx(iq, n); })) {
			fprintf(stderr, "vrtd: radio RX start failed\n");
			return false;
		}
	}
	if(in_fd_ >= 0) {
		ingest_thr_ = std::thread([this] { ingest_loop(); });
	}
	return true;
}

void core::stop() {
	if(!run_.exchange(false)) {
		return;
	}
	rad_->stop_rx();
	rad_->stop_tx();
	if(ingest_thr_.joinable()) {
		ingest_thr_.join();
	}
	if(out_fd_ >= 0) {
		close(out_fd_);
		out_fd_ = -1;
	}
	if(in_fd_ >= 0) {
		close(in_fd_);
		in_fd_ = -1;
	}
}

// ---- exporter --------------------------------------------------------------------------
void core::on_rx(const int16_t *iq, size_t n) {
	stage_.insert(stage_.end(), iq, iq + 2 * n);
	const size_t per = cfg_.samples_per_pkt;
	size_t off = 0;
	while(stage_.size() - off >= 2 * per) {
		vrt::data_packet p;
		p.stream_id = cfg_.sid_rx;
		if(cfg_.oui) {
			p.cls.present = true;
			p.cls.oui = cfg_.oui;
			p.cls.info_class = cfg_.info_class;
			p.cls.packet_class = cfg_.packet_class;
		}
		p.ts.tsi = vrt::TSI_UTC;
		p.ts.tsi_sec = (uint32_t)time(nullptr);
		p.ts.tsf = vrt::TSF_SAMPLES;
		p.ts.tsf_frac = sample_clock_;
		p.count = data_count_;
		data_count_ = (data_count_ + 1) & 0xF;
		// payload: CS16 big-endian, I then Q per 32-bit word
		std::vector<uint8_t> payload(per * 4);
		// CS16 -> VRT big-endian: byte-swap each int16 (NEON vrev16). stage_[off..] is [i,q,i,q,...],
		// so the whole per*4-byte payload is one 16-bit-word byte-swap.
		bswap16_stream(reinterpret_cast<const uint8_t *>(&stage_[off]), payload.data(), per * 4);
		p.payload = payload.data();
		p.payload_words = per;
		pktbuf_.clear();
		vrt::pack_data(p, pktbuf_);
		sendto(out_fd_, pktbuf_.data(), pktbuf_.size(), 0, (sockaddr *)&dest_, sizeof(dest_));
		st.data_pkts_out++;
		st.samples_out += per;
		sample_clock_ += per;
		off += 2 * per;
	}
	stage_.erase(stage_.begin(), stage_.begin() + off);

	double t = now_s();
	if(t >= next_ctx_) {
		next_ctx_ = t + cfg_.context_interval_s;
		emit_context(false);
	}
}

void core::emit_context(bool changed) {
	if(!has_dest_) {
		return;
	}
	std::lock_guard<std::mutex> lk(ctl_mtx_);
	radio_state s = rad_->state();
	vrt::context_packet c;
	c.stream_id = cfg_.sid_rx;
	if(cfg_.oui) {
		c.cls.present = true;
		c.cls.oui = cfg_.oui;
		c.cls.info_class = cfg_.info_class;
		c.cls.packet_class = cfg_.packet_class;
	}
	c.ts.tsi = vrt::TSI_UTC;
	c.ts.tsi_sec = (uint32_t)time(nullptr);
	c.count = ctx_count_;
	ctx_count_ = (ctx_count_ + 1) & 0xF;
	c.f.cif0 = vrt::CIF0_RF_FREQ | vrt::CIF0_SAMP_RATE | vrt::CIF0_GAIN | vrt::CIF0_PAYLOAD_FMT | (changed ? (uint32_t)vrt::CIF0_CHANGE : 0u);
	c.f.rf_freq_hz = s.rx_freq_hz;
	c.f.samp_rate_hz = s.samp_rate_hz;
	c.f.gain1_db = s.rx_gain_db;
	c.f.gain2_db = 0;
	c.f.payload_fmt_w0 = vrt::PAYLOAD_FMT_CS16_W0;
	c.f.payload_fmt_w1 = vrt::PAYLOAD_FMT_CS16_W1;
	pktbuf_.clear();
	vrt::pack_context(c, pktbuf_);
	sendto(out_fd_, pktbuf_.data(), pktbuf_.size(), 0, (sockaddr *)&dest_, sizeof(dest_));
	st.ctx_pkts_out++;
}

// ---- ingest + control --------------------------------------------------------------------
void core::ingest_loop() {
	std::vector<uint8_t> buf(65536);
	std::vector<int16_t> txconv;
	while(run_) {
		sockaddr_in from{};
		socklen_t flen = sizeof(from);
		ssize_t n = recvfrom(in_fd_, buf.data(), buf.size(), 0, (sockaddr *)&from, &flen);
		if(n <= 0) {
			continue;
		}
		vrt::parsed p;
		const char *why;
		if(!vrt::unpack(buf.data(), (size_t)n, p, &why)) {
			st.malformed_in++;
			continue;
		}
		if((p.type == vrt::PT_DATA || p.type == vrt::PT_DATA_NOSID) && p.data.stream_id == cfg_.sid_tx) {
			st.data_pkts_in++;
			if(!cfg_.tx_allowed) {   // TX SAFETY layer 1: policy-forbidden TX data never reaches the radio
				st.tx_forbidden++;
				continue;
			}
			if(last_in_count_ >= 0 && ((last_in_count_ + 1) & 0xF) != p.data.count) {
				st.seq_gaps_in++;
			}
			last_in_count_ = p.data.count;
			txconv.resize(2 * p.payload_words);
			// VRT big-endian -> host CS16: byte-swap each int16 (NEON vrev16)
			bswap16_stream(p.payload, reinterpret_cast<uint8_t *>(txconv.data()), p.payload_words * 4);
			if(!rad_->submit_tx(txconv.data(), p.payload_words)) {
				st.tx_drops++;
			} else {
				st.samples_in += p.payload_words;
			}
			continue;
		}
		if(p.type == vrt::PT_COMMAND && !p.cmd.is_ack) {
			st.cmds_in++;
			handle_command(p.cmd, from);
			continue;
		}
		// context or unmatched data: counted, ignored
		st.malformed_in += 0;
	}
}

void core::handle_command(const vrt::command_packet &c, const sockaddr_in &from) {
	// controllee gate: when the sender names a controllee it must be us
	if(c.cam.controllee && cfg_.controllee_id && c.controllee_id != cfg_.controllee_id) {
		st.cmds_rejected++;
		return;
	}
	bool for_rx = c.stream_id == cfg_.sid_rx;
	bool for_tx = c.stream_id == cfg_.sid_tx;

	uint32_t eif0 = 0;
	std::vector<uint32_t> ewords;
	auto fail = [&](uint32_t bit, uint32_t code) {
		eif0 |= bit;
		ewords.push_back(code);
	};

	if(!for_rx && !for_tx) {
		fail(vrt::CIF0_CHANGE, ERR_FIELD_NOT_EXECUTABLE);   // unknown stream: flag on bit 31
	} else if(for_tx && !cfg_.tx_allowed) {
		// TX SAFETY layer 2: an explicit, spec-shaped refusal (dry-runs too) - the remote
		// controller learns WHY instead of getting silence
		st.tx_forbidden++;
		for(int b = 31; b >= 0; b--) {
			if((c.f.cif0 | vrt::CIF0_CHANGE) & (1u << b)) {
				fail(1u << b, ERR_TX_FORBIDDEN);
			}
		}
	} else if(c.is_cancel) {
		// v1: nothing schedulable exists, cancels are trivially "done"
	} else {
		// validate first: every requested field must be one we can execute
		uint32_t exec_ok = vrt::CIF0_CHANGE | vrt::CIF0_RF_FREQ | vrt::CIF0_SAMP_RATE | vrt::CIF0_GAIN;
		uint32_t bad = c.f.cif0 & ~exec_ok;
		for(int b = 31; b >= 0; b--) {
			if(bad & (1u << b)) {
				fail(1u << b, ERR_FIELD_NOT_EXECUTABLE);
			}
		}
		if(c.cam.action == 2 && !eif0) {          // execute (dry-run stops after validation)
			std::lock_guard<std::mutex> lk(ctl_mtx_);
			if(c.f.cif0 & vrt::CIF0_RF_FREQ) {
				bool ok = for_rx ? rad_->set_rx_freq(c.f.rf_freq_hz) : rad_->set_tx_freq(c.f.rf_freq_hz);
				if(!ok) {
					fail(vrt::CIF0_RF_FREQ, ERR_HW_REJECTED);
				}
			}
			if(c.f.cif0 & vrt::CIF0_SAMP_RATE) {
				if(!rad_->set_samp_rate(c.f.samp_rate_hz)) {
					fail(vrt::CIF0_SAMP_RATE, ERR_HW_REJECTED);
				}
			}
			if(c.f.cif0 & vrt::CIF0_GAIN) {
				bool ok = for_rx ? rad_->set_rx_gain(c.f.gain1_db) : rad_->set_tx_power(c.f.gain1_db);
				if(!ok) {
					fail(vrt::CIF0_GAIN, ERR_HW_REJECTED);
				}
			}
		}
	}
	if(eif0) {
		st.cmds_rejected++;
	}

	// acknowledge per the CAM request bits (nack_only suppresses success acks)
	bool want_ack = (c.cam.req_v && c.cam.action != 2) || (c.cam.req_x && c.cam.action == 2) || c.cam.req_s;
	if(c.cam.nack_only && !eif0) {
		want_ack = false;
	}
	if(!want_ack) {
		if(!eif0 && c.cam.action == 2 && (c.f.cif0 & (vrt::CIF0_RF_FREQ | vrt::CIF0_SAMP_RATE | vrt::CIF0_GAIN))) {
			emit_context(true);
		}
		return;
	}

	vrt::command_packet ack;
	ack.is_ack = true;
	ack.stream_id = c.stream_id;
	ack.count = ack_count_;
	ack_count_ = (ack_count_ + 1) & 0xF;
	ack.cam = c.cam;                                // echo, then mark what this ack carries
	ack.cam.controllee = false;                     // ids are not repeated in v1 acks
	ack.cam.controller = false;
	ack.cam.req_warn = false;
	ack.cam.req_err = eif0 != 0;
	ack.message_id = c.message_id;
	ack.eif0 = eif0;
	ack.eif0_words = ewords;
	send_ack(ack, from);

	if(!eif0 && c.cam.action == 2 && (c.f.cif0 & (vrt::CIF0_RF_FREQ | vrt::CIF0_SAMP_RATE | vrt::CIF0_GAIN))) {
		emit_context(true);
	}
}

void core::send_ack(vrt::command_packet ack, const sockaddr_in &to) {
	std::vector<uint8_t> out;
	vrt::pack_command(ack, out);
	sendto(in_fd_, out.data(), out.size(), 0, (const sockaddr *)&to, sizeof(to));
	st.acks_out++;
}

} // namespace vrtd

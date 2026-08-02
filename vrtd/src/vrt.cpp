// vrtd: VITA 49.2 wire layer implementation. See vrt.h for the layout table.
#include "vrt.h"

namespace vrt {

// ---- small append helpers -------------------------------------------------------------
static void put32(std::vector<uint8_t> &out, uint32_t v) {
	size_t n = out.size();
	out.resize(n + 4);
	wr32(&out[n], v);
}
static void put64(std::vector<uint8_t> &out, uint64_t v) {
	put32(out, (uint32_t)(v >> 32));
	put32(out, (uint32_t)v);
}

static void put_prologue_common(std::vector<uint8_t> &out, const class_id &cls, const timestamp &ts) {
	if(cls.present) {
		put32(out, cls.oui & 0x00FFFFFF);
		put32(out, ((uint32_t)cls.info_class << 16) | cls.packet_class);
	}
	if(ts.tsi != TSI_NONE) {
		put32(out, ts.tsi_sec);
	}
	if(ts.tsf != TSF_NONE) {
		put64(out, ts.tsf_frac);
	}
}

static uint32_t header_word(int type, bool cls, uint32_t indicators, const timestamp &ts, uint8_t count, size_t words) {
	return ((uint32_t)type << 28) | (cls ? 1u << 27 : 0) | (indicators << 24) |
	       ((uint32_t)ts.tsi << 22) | ((uint32_t)ts.tsf << 20) | ((uint32_t)(count & 0xF) << 16) | (uint32_t)(words & 0xFFFF);
}

// patch the size field once the packet is fully appended
static size_t finish(std::vector<uint8_t> &out, size_t hdr_at) {
	size_t words = (out.size() - hdr_at) / 4;
	uint32_t h = rd32(&out[hdr_at]);
	wr32(&out[hdr_at], (h & 0xFFFF0000u) | (uint32_t)words);
	return words;
}

// ---- fields sub-codec -------------------------------------------------------------------
size_t pack_fields(const fields &f, std::vector<uint8_t> &out) {
	size_t start = out.size();
	put32(out, f.cif0);
	// field order = descending CIF0 bit position
	if(f.cif0 & CIF0_BANDWIDTH) {
		put64(out, hz_to_q20(f.bandwidth_hz));
	}
	if(f.cif0 & CIF0_IF_FREQ) {
		put64(out, hz_to_q20(f.if_freq_hz));
	}
	if(f.cif0 & CIF0_RF_FREQ) {
		put64(out, hz_to_q20(f.rf_freq_hz));
	}
	if(f.cif0 & CIF0_REF_LEVEL) {
		put32(out, db_to_q7(f.ref_level_dbm));
	}
	if(f.cif0 & CIF0_GAIN) {
		put32(out, ((uint32_t)db_to_q7(f.gain2_db) << 16) | db_to_q7(f.gain1_db));
	}
	if(f.cif0 & CIF0_SAMP_RATE) {
		put64(out, hz_to_q20(f.samp_rate_hz));
	}
	if(f.cif0 & CIF0_DEVICE_ID) {
		put32(out, f.device_oui & 0x00FFFFFF);
		put32(out, f.device_code);
	}
	if(f.cif0 & CIF0_STATE_EVENT) {
		put32(out, f.state_event);
	}
	if(f.cif0 & CIF0_PAYLOAD_FMT) {
		put32(out, f.payload_fmt_w0);
		put32(out, f.payload_fmt_w1);
	}
	return (out.size() - start) / 4;
}

bool unpack_fields(const uint8_t *w, size_t nwords, fields &f, const char **why) {
	if(nwords < 1) {
		*why = "fields: truncated CIF0";
		return false;
	}
	f.cif0 = rd32(w);
	if(f.cif0 & ~CIF0_KNOWN_MASK) {
		*why = "fields: unsupported CIF0 bits";   // includes CIF1/2/3/7 enables - v1 is CIF0-only
		return false;
	}
	size_t i = 1;
	auto need = [&](size_t n) {
		if(i + n > nwords) {
			return false;
		}
		return true;
	};
	auto get64 = [&]() {
		uint64_t v = ((uint64_t)rd32(w + 4 * i) << 32) | rd32(w + 4 * (i + 1));
		i += 2;
		return v;
	};
	if(f.cif0 & CIF0_BANDWIDTH) {
		if(!need(2)) { *why = "fields: truncated bandwidth"; return false; }
		f.bandwidth_hz = q20_to_hz(get64());
	}
	if(f.cif0 & CIF0_IF_FREQ) {
		if(!need(2)) { *why = "fields: truncated if_freq"; return false; }
		f.if_freq_hz = q20_to_hz(get64());
	}
	if(f.cif0 & CIF0_RF_FREQ) {
		if(!need(2)) { *why = "fields: truncated rf_freq"; return false; }
		f.rf_freq_hz = q20_to_hz(get64());
	}
	if(f.cif0 & CIF0_REF_LEVEL) {
		if(!need(1)) { *why = "fields: truncated ref_level"; return false; }
		f.ref_level_dbm = q7_to_db((uint16_t)rd32(w + 4 * i));
		i++;
	}
	if(f.cif0 & CIF0_GAIN) {
		if(!need(1)) { *why = "fields: truncated gain"; return false; }
		uint32_t g = rd32(w + 4 * i);
		i++;
		f.gain2_db = q7_to_db((uint16_t)(g >> 16));
		f.gain1_db = q7_to_db((uint16_t)g);
	}
	if(f.cif0 & CIF0_SAMP_RATE) {
		if(!need(2)) { *why = "fields: truncated samp_rate"; return false; }
		f.samp_rate_hz = q20_to_hz(get64());
	}
	if(f.cif0 & CIF0_DEVICE_ID) {
		if(!need(2)) { *why = "fields: truncated device_id"; return false; }
		f.device_oui = rd32(w + 4 * i) & 0x00FFFFFF;
		f.device_code = (uint16_t)rd32(w + 4 * (i + 1));
		i += 2;
	}
	if(f.cif0 & CIF0_STATE_EVENT) {
		if(!need(1)) { *why = "fields: truncated state_event"; return false; }
		f.state_event = rd32(w + 4 * i);
		i++;
	}
	if(f.cif0 & CIF0_PAYLOAD_FMT) {
		if(!need(2)) { *why = "fields: truncated payload_fmt"; return false; }
		f.payload_fmt_w0 = rd32(w + 4 * i);
		f.payload_fmt_w1 = rd32(w + 4 * (i + 1));
		i += 2;
	}
	if(i != nwords) {
		*why = "fields: trailing words";
		return false;
	}
	return true;
}

// ---- CAM -------------------------------------------------------------------------------
uint32_t cam_word::pack() const {
	uint32_t w = 0;
	w |= controllee ? 1u << 31 : 0;
	w |= controllee_uuid ? 1u << 30 : 0;
	w |= controller ? 1u << 29 : 0;
	w |= controller_uuid ? 1u << 28 : 0;
	w |= partial_ok ? 1u << 27 : 0;
	w |= warnings_ok ? 1u << 26 : 0;
	w |= errors_ok ? 1u << 25 : 0;
	w |= (uint32_t)(action & 3) << 23;
	w |= nack_only ? 1u << 22 : 0;
	w |= req_v ? 1u << 20 : 0;
	w |= req_x ? 1u << 19 : 0;
	w |= req_s ? 1u << 18 : 0;
	w |= req_warn ? 1u << 17 : 0;
	w |= req_err ? 1u << 16 : 0;
	w |= (uint32_t)(timing & 7) << 12;
	w |= partial_taken ? 1u << 11 : 0;
	w |= scheduled ? 1u << 10 : 0;
	return w;
}

cam_word cam_word::unpack(uint32_t w) {
	cam_word c;
	c.controllee = w & (1u << 31);
	c.controllee_uuid = w & (1u << 30);
	c.controller = w & (1u << 29);
	c.controller_uuid = w & (1u << 28);
	c.partial_ok = w & (1u << 27);
	c.warnings_ok = w & (1u << 26);
	c.errors_ok = w & (1u << 25);
	c.action = (uint8_t)((w >> 23) & 3);
	c.nack_only = w & (1u << 22);
	c.req_v = w & (1u << 20);
	c.req_x = w & (1u << 19);
	c.req_s = w & (1u << 18);
	c.req_warn = w & (1u << 17);
	c.req_err = w & (1u << 16);
	c.timing = (uint8_t)((w >> 12) & 7);
	c.partial_taken = w & (1u << 11);
	c.scheduled = w & (1u << 10);
	return c;
}

// ---- packers ----------------------------------------------------------------------------
size_t pack_data(const data_packet &p, std::vector<uint8_t> &out) {
	size_t hdr = out.size();
	uint32_t ind = p.trailer ? 4u : 0;   // bit 26 of header = bit 2 of the 3-bit indicator field
	put32(out, header_word(PT_DATA, p.cls.present, ind, p.ts, p.count, 0));
	put32(out, p.stream_id);
	put_prologue_common(out, p.cls, p.ts);
	size_t n = out.size();
	out.resize(n + p.payload_words * 4);
	memcpy(&out[n], p.payload, p.payload_words * 4);
	if(p.trailer) {
		put32(out, p.trailer_word);
	}
	return finish(out, hdr);
}

size_t pack_context(const context_packet &p, std::vector<uint8_t> &out) {
	size_t hdr = out.size();
	uint32_t ind = p.tsm_coarse ? 1u : 0;   // bit 24
	put32(out, header_word(PT_CONTEXT, p.cls.present, ind, p.ts, p.count, 0));
	put32(out, p.stream_id);
	put_prologue_common(out, p.cls, p.ts);
	pack_fields(p.f, out);
	return finish(out, hdr);
}

size_t pack_command(const command_packet &p, std::vector<uint8_t> &out) {
	size_t hdr = out.size();
	uint32_t ind = (p.is_ack ? 4u : 0) | (p.is_cancel ? 1u : 0);
	put32(out, header_word(PT_COMMAND, p.cls.present, ind, p.ts, p.count, 0));
	put32(out, p.stream_id);
	put_prologue_common(out, p.cls, p.ts);
	put32(out, p.cam.pack());
	put32(out, p.message_id);
	if(p.cam.controllee && !p.cam.controllee_uuid) {
		put32(out, p.controllee_id);
	}
	if(p.cam.controller && !p.cam.controller_uuid) {
		put32(out, p.controller_id);
	}
	if(p.is_ack) {
		// v1 ack payload: WIF0 word if warnings requested, EIF0 word if errors
		// requested, each followed by its per-field status words
		if(p.cam.req_warn) {
			put32(out, p.wif0);
			for(uint32_t w : p.wif0_words) {
				put32(out, w);
			}
		}
		if(p.cam.req_err) {
			put32(out, p.eif0);
			for(uint32_t w : p.eif0_words) {
				put32(out, w);
			}
		}
	} else if(!p.is_cancel) {
		pack_fields(p.f, out);
	}
	return finish(out, hdr);
}

// ---- unpacker -----------------------------------------------------------------------------
bool unpack(const uint8_t *buf, size_t len, parsed &out, const char **why) {
	*why = "";
	if(len < 4 || (len & 3)) {
		*why = "not a whole number of words";
		return false;
	}
	uint32_t h = rd32(buf);
	size_t words = h & 0xFFFF;
	if(words * 4 != len) {
		*why = "size field does not match datagram";
		return false;
	}
	int type = (int)(h >> 28);
	bool has_cls = h & (1u << 27);
	uint32_t ind = (h >> 24) & 7;
	timestamp ts;
	ts.tsi = (tsi_mode)((h >> 22) & 3);
	ts.tsf = (tsf_mode)((h >> 20) & 3);
	uint8_t count = (uint8_t)((h >> 16) & 0xF);

	size_t i = 1;   // word index
	auto need = [&](size_t n) { return i + n <= words; };
	auto get = [&]() { uint32_t v = rd32(buf + 4 * i); i++; return v; };

	uint32_t stream_id = 0;
	if(type != PT_DATA_NOSID && type != PT_EXT_DATA_NOSID) {
		if(!need(1)) { *why = "truncated stream id"; return false; }
		stream_id = get();
	}
	class_id cls;
	if(has_cls) {
		if(!need(2)) { *why = "truncated class id"; return false; }
		cls.present = true;
		cls.oui = get() & 0x00FFFFFF;
		uint32_t w = get();
		cls.info_class = (uint16_t)(w >> 16);
		cls.packet_class = (uint16_t)w;
	}
	if(ts.tsi != TSI_NONE) {
		if(!need(1)) { *why = "truncated tsi"; return false; }
		ts.tsi_sec = get();
	}
	if(ts.tsf != TSF_NONE) {
		if(!need(2)) { *why = "truncated tsf"; return false; }
		uint64_t hi = get();
		ts.tsf_frac = (hi << 32) | get();
	}

	out.type = type;
	switch(type) {
	case PT_DATA_NOSID:
	case PT_DATA: {
		bool trailer = ind & 4;
		size_t tail = trailer ? 1 : 0;
		if(words < i + tail) { *why = "truncated data payload"; return false; }
		out.data.stream_id = stream_id;
		out.data.cls = cls;
		out.data.ts = ts;
		out.data.count = count;
		out.data.trailer = trailer;
		out.payload = buf + 4 * i;
		out.payload_words = words - i - tail;
		out.data.payload = out.payload;
		out.data.payload_words = out.payload_words;
		if(trailer) {
			out.data.trailer_word = rd32(buf + 4 * (words - 1));
		}
		return true;
	}
	case PT_CONTEXT: {
		out.ctx.stream_id = stream_id;
		out.ctx.cls = cls;
		out.ctx.ts = ts;
		out.ctx.count = count;
		out.ctx.tsm_coarse = ind & 1;
		return unpack_fields(buf + 4 * i, words - i, out.ctx.f, why);
	}
	case PT_COMMAND: {
		command_packet &c = out.cmd;
		c.is_ack = ind & 4;
		c.is_cancel = ind & 1;
		c.stream_id = stream_id;
		c.cls = cls;
		c.ts = ts;
		c.count = count;
		if(!need(2)) { *why = "truncated CAM/message id"; return false; }
		c.cam = cam_word::unpack(get());
		c.message_id = get();
		if(c.cam.controllee_uuid || c.cam.controller_uuid) {
			*why = "UUID identifiers unsupported";   // v1: 32-bit ids only
			return false;
		}
		if(c.cam.controllee) {
			if(!need(1)) { *why = "truncated controllee id"; return false; }
			c.controllee_id = get();
		}
		if(c.cam.controller) {
			if(!need(1)) { *why = "truncated controller id"; return false; }
			c.controller_id = get();
		}
		if(c.is_ack) {
			if(c.cam.req_warn) {
				if(!need(1)) { *why = "truncated wif0"; return false; }
				c.wif0 = get();
				for(int b = 31; b >= 0; b--) {
					if(c.wif0 & (1u << b)) {
						if(!need(1)) { *why = "truncated wif0 field"; return false; }
						c.wif0_words.push_back(get());
					}
				}
			}
			if(c.cam.req_err) {
				if(!need(1)) { *why = "truncated eif0"; return false; }
				c.eif0 = get();
				for(int b = 31; b >= 0; b--) {
					if(c.eif0 & (1u << b)) {
						if(!need(1)) { *why = "truncated eif0 field"; return false; }
						c.eif0_words.push_back(get());
					}
				}
			}
			if(i != words) { *why = "ack: trailing words"; return false; }
			return true;
		}
		if(c.is_cancel) {
			if(i != words) { *why = "cancel: trailing words"; return false; }
			return true;
		}
		return unpack_fields(buf + 4 * i, words - i, c.f, why);
	}
	default:
		*why = "unsupported packet type";
		return false;
	}
}

} // namespace vrt

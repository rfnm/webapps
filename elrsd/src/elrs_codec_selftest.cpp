// NEEDS: src/elrs_codec.cpp src/elrs_link.cpp
// elrsd: codec selftests - codec (bit-exact against the embedded vectors, TEST-UID only) and
// link model (slot clock, lock A / lock B). Deterministic, no I/O, no files.
#include "elrs_codec.h"
#include "elrs_link.h"
#include "elrs_vectors_gen.h"
#include "selftest.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

namespace elrsd {

namespace {

std::string hex8(const uint8_t *p, size_t n) {
	char buf[4];
	std::string s;
	for(size_t i = 0; i < n; i++) {
		snprintf(buf, sizeof(buf), "%02x", p[i]);
		s += buf;
	}
	return s;
}

uid_t6 test_uid() {
	uid_t6 u;
	for(int i = 0; i < 6; i++) u[i] = vec::UID[i];
	return u;
}

// deterministic LCG for the selftests (not the ELRS one)
struct test_rng {
	uint64_t s;
	explicit test_rng(uint64_t seed) : s(seed) {}
	uint32_t next() {
		s = s * 6364136223846793005ull + 1442695040888963407ull;
		return (uint32_t)(s >> 33);
	}
	uint32_t below(uint32_t n) { return next() % n; }
};

// ---- codec ---------------------------------------------------------------------------------

void test_uid_and_seeds(selftest_counter &t) {
	uid_t6 u = uid_from_binding_phrase(vec::UID_PHRASE);
	t.check(u == test_uid(), "uid_from_binding_phrase(test phrase)", uid_to_string(u) + " vs " + vec::UID_HEX);
	t.check(uid_to_string(u) == vec::UID_HEX, "uid_to_string");
	uid_t6 p;
	t.check(uid_from_string(vec::UID_HEX, p) && p == u, "uid_from_string(colon)");
	t.check(uid_from_string("D2-4C-3E-2E-FB-4D", p) && p == u, "uid_from_string(dash, upper)");
	t.check(uid_from_string("d24c3e2efb4d", p) && p == u, "uid_from_string(no separator)");
	t.check(!uid_from_string("d2:4c:3e:2e:fb", p), "uid_from_string(short) rejected");
	t.check(!uid_from_string("d2:4c:3e:2e:fb:4g", p), "uid_from_string(bad hex) rejected");
	int ok = 0;
	for(int i = 0; i < vec::N_UID_EXTRA; i++) {
		uid_t6 e = uid_from_binding_phrase(vec::UID_EXTRA[i].phrase);
		bool eq = memcmp(e.data(), vec::UID_EXTRA[i].uid, 6) == 0;
		if(eq) ok++;
	}
	t.check(ok == vec::N_UID_EXTRA, "uid extra phrases (incl. empty, MD5 edge)", std::to_string(ok) + "/" + std::to_string(vec::N_UID_EXTRA));
	t.check(fhss_seed_from_uid(u) == vec::UID_FHSS_SEED, "fhss_seed_from_uid", std::to_string(fhss_seed_from_uid(u)));
	t.check(crc14_init_from_uid(u) == vec::UID_CRC14_INIT, "crc14_init_from_uid", std::to_string(crc14_init_from_uid(u)));
	t.check(iq_inverted_from_uid(u) == vec::UID_IQ_INVERTED, "iq_inverted_from_uid");
	t.check(OTA_VERSION_ID == vec::UID_OTA_VERSION_ID, "OTA_VERSION_ID == 3");
}

void test_fhss(selftest_counter &t) {
	uid_t6 u = test_uid();
	t.check(FHSS_NUM_CHANNELS == vec::FHSS_N_CHANNELS && FHSS_SEQ_LEN == vec::FHSS_SEQ_LEN && FHSS_SYNC_CHANNEL == vec::FHSS_SYNC_CHANNEL,
	        "fhss constants (80 / 240 / 41)");
	elrs_rng rng(vec::UID_FHSS_SEED);
	int ok = 0;
	for(int i = 0; i < 16; i++) ok += rng.next() == vec::LCG_FIRST16[i];
	t.check(ok == 16, "elrs_rng first 16 outputs", std::to_string(ok) + "/16");
	auto seq = fhss_sequence(u);
	int mism = 0;
	for(int i = 0; i < FHSS_SEQ_LEN; i++) mism += seq[i] != vec::FHSS_SEQUENCE[i];
	t.check(mism == 0, "fhss_sequence 240 entries", std::to_string(mism) + " mismatches");
	int n41 = 0;
	bool at0 = true;
	for(int i = 0; i < FHSS_SEQ_LEN; i++) {
		if(seq[i] == FHSS_SYNC_CHANNEL) { n41++; if(i % 80 != 0) at0 = false; }
	}
	t.check(n41 == 3 && at0, "sync channel only at 0/80/160");
	bool perm = true;
	for(int blk = 0; blk < 3; blk++) {
		std::set<int> s;
		for(int i = 0; i < 80; i++) s.insert(seq[blk * 80 + i]);
		if(s.size() != 80) perm = false;
	}
	t.check(perm, "each 80-block is a permutation of 0..79");
	int fok = 0, sok = 0;
	for(int ch = 0; ch < 80; ch++) {
		fok += fabs(fhss_channel_freq_hz(ch) - vec::FHSS_FREQ_NOMINAL_HZ[ch]) < 1e-3;
		sok += fabs(fhss_channel_freq_hz_sx1280(ch) - vec::SX1280_FREQ_ACTUAL_HZ[ch]) < 1e-3;
	}
	t.check(fok == 80, "fhss_channel_freq_hz nominal grid", std::to_string(fok) + "/80");
	t.check(sok == 80, "fhss_channel_freq_hz_sx1280 register-exact", std::to_string(sok) + "/80");
	double maxdev = 0;
	for(int ch = 0; ch < 80; ch++) maxdev = std::max(maxdev, fabs(fhss_channel_freq_hz_sx1280(ch) - fhss_channel_freq_hz(ch)));
	t.check(maxdev <= 327.0, "sx1280 deviation <= 327 Hz", std::to_string(maxdev));
	int cok = 0;
	for(int ch = 0; ch < 80; ch++) {
		cok += fhss_channel_from_freq(fhss_channel_freq_hz(ch)) == ch;
		cok += fhss_channel_from_freq(fhss_channel_freq_hz(ch) + 0.4e6) == ch;
		cok += fhss_channel_from_freq(fhss_channel_freq_hz(ch) - 0.49e6) == ch;
	}
	t.check(cok == 240, "fhss_channel_from_freq nearest", std::to_string(cok) + "/240");
	t.check(fhss_channel_from_freq(2400.4e6 - 0.6e6) == -1 && fhss_channel_from_freq(2479.4e6 + 0.6e6) == -1, "fhss_channel_from_freq out of band -> -1");
}

void test_whitening_gray(selftest_counter &t) {
	const uint8_t *w = whitening_seq();
	int mism = 0;
	for(int i = 0; i < 255; i++) mism += w[i] != vec::WHITENING_SEQ[i];
	t.check(mism == 0, "whitening_seq 255 bytes", std::to_string(mism) + " mismatches");
	uint8_t buf[8] = {0};
	whiten(buf, 8);
	t.check(memcmp(buf, w, 8) == 0, "whiten(zeros) == W");
	whiten(buf, 8);
	bool z = true;
	for(int i = 0; i < 8; i++) z = z && buf[i] == 0;
	t.check(z, "whiten is an involution");
	int gok = 0;
	for(uint32_t v = 0; v < 64; v++) {
		gok += gray_encode(v) == vec::GRAY_ENCODE[v];
		gok += gray_decode(v) == vec::GRAY_DECODE[v];
		gok += gray_decode(gray_encode(v)) == v;
	}
	t.check(gok == 192, "gray encode/decode[64] + inverse", std::to_string(gok) + "/192");
}

// re-derive the header-only RATE_TABLE literals from the formulas and the JSON
void test_rate_table(selftest_counter &t) {
	int ok = 0, n = 0;
	for(int i = 0; i < vec::N_RATES; i++) {
		const vec::rate_vec &r = vec::RATES[i];
		const rate_params *rp = rate_by_index(r.index);
		n++;
		if(!rp) { t.check(false, "rate_by_index", std::to_string(r.index)); continue; }
		bool eq = rp->index == r.index && rp->rate_hz == r.rate_hz && rp->sf == r.sf && rp->cr_bits == r.cr_bits &&
		          rp->payload_bytes == r.payload_bytes && rp->preamble_len == r.preamble_len && rp->interval_us == r.interval_us &&
		          rp->hop_interval == r.hop_interval && rp->tlm_denom_default == r.tlm_denom_default &&
		          fabs(rp->frame_offset_symbols - r.frame_offset_symbols) < 1e-9 && rp->verified == r.verified &&
		          rp->fine_sync == (r.sf < 7) && rp->kind == (r.payload_bytes == 8 ? ota_kind::OTA4 : ota_kind::OTA8) &&
		          rp->n_cw == r.n_cw && rp->n_coded == r.n_coded && rp->n_sym == r.n_sym && rp->n_slots == r.n_slots && rp->n_tail == r.n_tail;
		// formulas
		int n_cw = 2 * r.payload_bytes, n_coded = n_cw * r.cr_bits;
		int n_sym, n_slots;
		if(r.sf >= 7) {
			int head = 8 * (r.sf - 2);
			int rest = std::max(n_coded - head, 0);
			int body = (rest + r.sf - 1) / r.sf;
			n_sym = 8 + body;
			n_slots = head + body * r.sf;
		} else {
			n_sym = (n_coded + r.sf - 1) / r.sf;
			n_slots = n_sym * r.sf;
		}
		double frame = r.preamble_len + 2 + 2.25 + (r.sf < 7 ? 2 : 0);
		bool feq = n_cw == rp->n_cw && n_coded == rp->n_coded && n_sym == rp->n_sym && n_slots == rp->n_slots && (n_slots - n_coded) == rp->n_tail &&
		           fabs(frame - rp->frame_offset_symbols) < 1e-9;
		t.check(eq && feq, ("RATE_TABLE[" + std::to_string(r.index) + "] literals == JSON == derived").c_str(),
		        std::to_string(r.rate_hz) + "hz sf" + std::to_string(r.sf) + " n_sym " + std::to_string(rp->n_sym) + " slots " + std::to_string(rp->n_slots));
		ok += eq && feq;
	}
	t.check(ok == n, "RATE_TABLE all rows", std::to_string(ok) + "/" + std::to_string(n));
	t.check(rate_by_name("250hz") == rate_by_index(6) && rate_by_name("500hz") == rate_by_index(4) && rate_by_name("150hz") == rate_by_index(7) &&
	        rate_by_name("50hz") == rate_by_index(9) && rate_by_name("foo") == nullptr, "rate_by_name");
	t.check(rate_from_sf(5) == rate_by_index(4) && rate_from_sf(6) == rate_by_index(6) && rate_from_sf(7) == rate_by_index(7) &&
	        rate_from_sf(8) == rate_by_index(9) && rate_from_sf(5, false) == rate_by_index(5) && rate_from_sf(7, false) == rate_by_index(8) &&
	        rate_from_sf(9) == nullptr, "rate_from_sf");
	t.check(rate_by_index(3) == nullptr && rate_by_index(10) == nullptr, "rate_by_index out of range");
	std::vector<int> v = rate_indices_verified();
	bool vok = v.size() == 4;
	for(int idx : v) vok = vok && rate_by_index(idx) && rate_by_index(idx)->verified;
	for(int idx : rate_indices()) {
		bool inv = std::find(v.begin(), v.end(), idx) != v.end();
		vok = vok && (inv == rate_by_index(idx)->verified);
	}
	t.check(vok, "rate_indices_verified == {4,6,7,9} == verified flags");
	// air times (3.3.1 TOA table 1507/3300/5871/10798 us, +-2 %)
	const double toa[4] = {1507e-6, 3300e-6, 5871e-6, 10798e-6};
	const int idxs[4] = {4, 6, 7, 9};
	bool tok = true;
	std::string det;
	for(int i = 0; i < 4; i++) {
		double b = rate_by_index(idxs[i])->burst_s();
		tok = tok && fabs(b - toa[i]) / toa[i] < 0.02;
		det += std::to_string(idxs[i]) + ":" + std::to_string(b * 1e3) + "ms ";
	}
	t.check(tok, "burst_s() within 2 % of 3.3.1 TOA", det);
}

void test_li_codec(selftest_counter &t) {
	test_rng rng(20260914);
	for(int i = 0; i < vec::N_RATES; i++) {
		const vec::rate_vec &r = vec::RATES[i];
		const rate_params *rp = rate_by_index(r.index);
		li_codec codec(*rp);
		std::string tag = "rate " + std::to_string(r.index) + " ";
		// tables
		bool wok = (int)codec.symbol_widths().size() == r.n_sym;
		for(int s = 0; wok && s < r.n_sym; s++) wok = codec.symbol_widths()[s] == r.symbol_widths[s];
		t.check(wok, (tag + "symbol_widths").c_str());
		bool hok = true;
		for(int nib = 0; nib < 16; nib++) hok = hok && codec.hamming_table()[nib] == r.hamming_encode[nib];
		t.check(hok, (tag + "hamming_encode[16]").c_str());
		int ncw = 1 << r.cr_bits, cok = 0, xok = 0;
		for(int cw = 0; cw < ncw; cw++) {
			cok += codec.hamming_decode((uint32_t)cw, true) == r.correct_table[cw];
			int exact = -1;
			for(int nib = 0; nib < 16; nib++) if(r.hamming_encode[nib] == cw) exact = nib;
			xok += codec.hamming_decode((uint32_t)cw, false) == exact;
		}
		t.check(cok == ncw, (tag + "hamming correct_table").c_str(), std::to_string(cok) + "/" + std::to_string(ncw));
		t.check(xok == ncw, (tag + "hamming_decode(correct=false) exact only").c_str(), std::to_string(xok) + "/" + std::to_string(ncw));
		bool sok = (int)codec.coded_bit_to_slot().size() == r.n_coded;
		int smis = 0;
		for(int b = 0; sok && b < r.n_coded; b++) smis += codec.coded_bit_to_slot()[b] != r.coded_bit_to_slot[b];
		t.check(sok && smis == 0, (tag + "coded_bit_to_slot").c_str(), std::to_string(smis) + " mismatches");
		bool wcok = (int)codec.whitening_coded_bits().size() == r.n_coded;
		int wmis = 0;
		for(int b = 0; wcok && b < r.n_coded; b++) wmis += codec.whitening_coded_bits()[b] != r.whitening_coded_bits[b];
		t.check(wcok && wmis == 0, (tag + "whitening_coded_bits").c_str(), std::to_string(wmis) + " mismatches");
		bool tok = (int)codec.tail_copies().size() == r.n_tail;
		for(int k = 0; tok && k < r.n_tail; k++) {
			tok = codec.tail_copies()[k].first == r.tail_slots[k] && codec.tail_copies()[k].first == r.tail_copies[2 * k] &&
			      codec.tail_copies()[k].second == r.tail_copies[2 * k + 1];
		}
		t.check(tok, (tag + "tail_slots / tail_copies").c_str(), std::to_string(r.n_tail) + " tail slots");
		// vectors: encode, decode with and without correction
		int eok = 0, dok = 0, dnok = 0, rok = 0;
		for(int v = 0; v < r.n_vectors; v++) {
			const vec::li_vector &lv = r.vectors[v];
			std::vector<uint16_t> sym = codec.encode(lv.payload, (size_t)r.payload_bytes, lv.ref);
			bool e = (int)sym.size() == r.n_sym;
			for(int s = 0; e && s < r.n_sym; s++) e = sym[s] == lv.symbols[s];
			eok += e;
			std::vector<uint8_t> out;
			int fec = -1;
			bool d = codec.decode(lv.symbols, (size_t)r.n_sym, lv.ref, out, true, &fec);
			dok += d && (int)out.size() == r.payload_bytes && memcmp(out.data(), lv.payload, (size_t)r.payload_bytes) == 0 && fec == 0;
			std::vector<uint8_t> out2;
			bool d2 = codec.decode(lv.symbols, (size_t)r.n_sym, lv.ref, out2, false, nullptr);
			dnok += d2 && (int)out2.size() == r.payload_bytes && memcmp(out2.data(), lv.payload, (size_t)r.payload_bytes) == 0;
			// symbol-level helpers are mutual inverses
			bool rt = true;
			for(int s = 0; s < r.n_sym; s++) {
				uint32_t bits = codec.symbol_to_bits(lv.symbols[s], s, lv.ref);
				rt = rt && codec.bits_to_symbol(bits, s, lv.ref) == lv.symbols[s];
			}
			rok += rt;
		}
		t.check(eok == r.n_vectors, (tag + "encode == JSON symbols").c_str(), std::to_string(eok) + "/" + std::to_string(r.n_vectors));
		t.check(dok == r.n_vectors, (tag + "decode(correct) == payload, fec 0").c_str(), std::to_string(dok) + "/" + std::to_string(r.n_vectors));
		t.check(dnok == r.n_vectors, (tag + "decode(correct=false) == payload").c_str(), std::to_string(dnok) + "/" + std::to_string(r.n_vectors));
		t.check(rok == r.n_vectors, (tag + "symbol_to_bits <-> bits_to_symbol").c_str());
		// random round trips with random ref
		int rr = 0;
		const int NR = 200;
		for(int k = 0; k < NR; k++) {
			uint8_t pl[13];
			for(int b = 0; b < r.payload_bytes; b++) pl[b] = (uint8_t)rng.next();
			int ref = (int)rng.below(1u << r.sf);
			std::vector<uint16_t> sym = codec.encode(pl, (size_t)r.payload_bytes, ref);
			std::vector<uint8_t> out;
			rr += codec.decode(sym.data(), sym.size(), ref, out, false) && memcmp(out.data(), pl, (size_t)r.payload_bytes) == 0;
		}
		t.check(rr == NR, (tag + "random encode->decode round trip").c_str(), std::to_string(rr) + "/" + std::to_string(NR));
		// single symbol errors: CR4/8 corrects any single-symbol error (each symbol carries at most
		// one bit of every codeword), CR4/6 detects every single-bit error (dmin 2)
		int se_ok = 0, se_n = 0, se_fec_pos = 0;
		for(int v = 0; v < r.n_vectors; v++) {
			const vec::li_vector &lv = r.vectors[v];
			for(int s = 0; s < r.n_sym; s += 3) {
				std::vector<uint16_t> sym(lv.symbols, lv.symbols + r.n_sym);
				uint16_t bad;
				do { bad = (uint16_t)rng.below(1u << r.sf); } while(bad == lv.symbols[s]);
				sym[s] = bad;
				// does the corruption touch a slot that carries a coded bit (not a tail copy / unused low bit)?
				uint32_t diff = codec.symbol_to_bits(bad, s, lv.ref) ^ codec.symbol_to_bits(lv.symbols[s], s, lv.ref);
				int w = r.symbol_widths[s], off = 0;
				for(int q = 0; q < s; q++) off += r.symbol_widths[q];
				bool touches_used = false;
				for(int b = 0; b < w; b++) {
					if(!((diff >> (w - 1 - b)) & 1)) continue;
					int slot = off + b;
					for(int i = 0; i < r.n_coded; i++) if(r.coded_bit_to_slot[i] == slot) touches_used = true;
				}
				std::vector<uint8_t> out;
				int fec = 0;
				bool d = codec.decode(sym.data(), sym.size(), lv.ref, out, true, &fec);
				se_n++;
				if(r.t_correct >= 1) {
					bool good = d && memcmp(out.data(), lv.payload, (size_t)r.payload_bytes) == 0;
					se_ok += good;
					se_fec_pos += good && ((fec >= 1) == touches_used);
				} else {
					// dmin 2: a corrupted symbol changes >= 1 bit; if it changes the used bits, decode must
					// fail (detected); a change confined to the tail copies or unused low bits is invisible
					bool detected = !d;
					bool unchanged = d && memcmp(out.data(), lv.payload, (size_t)r.payload_bytes) == 0;
					se_ok += detected || unchanged;
				}
			}
		}
		if(r.t_correct >= 1) {
			t.check(se_ok == se_n && se_fec_pos == se_n, (tag + "single-symbol error corrected (fec_corrected counts iff a used slot changed)").c_str(),
			        std::to_string(se_ok) + "/" + std::to_string(se_n));
		} else {
			t.check(se_ok == se_n, (tag + "single-symbol error detected, never mis-decoded (CR4/6)").c_str(), std::to_string(se_ok) + "/" + std::to_string(se_n));
		}
		// two bit errors in one codeword (CR4/8): must not silently mis-decode into a wrong payload
		if(r.cr_bits == 8) {
			int bad_silent = 0, n2 = 0;
			for(int v = 0; v < 4; v++) {
				const vec::li_vector &lv = r.vectors[v];
				for(int c = 0; c < r.n_cw; c += 5) {
					std::vector<uint16_t> sym(lv.symbols, lv.symbols + r.n_sym);
					// flip coded bits b=0 and b=1 of codeword c: locate their slots -> symbols -> flip that slot bit
					for(int b = 0; b < 2; b++) {
						int slot = codec.coded_bit_to_slot()[r.cr_bits * c + b];
						int s = 0, off = 0;
						while(off + r.symbol_widths[s] <= slot) { off += r.symbol_widths[s]; s++; }
						int w = r.symbol_widths[s];
						uint32_t bits = codec.symbol_to_bits(sym[s], s, lv.ref);
						bits ^= 1u << (w - 1 - (slot - off));
						sym[s] = codec.bits_to_symbol(bits, s, lv.ref);
					}
					std::vector<uint8_t> out;
					bool d = codec.decode(sym.data(), sym.size(), lv.ref, out, true);
					n2++;
					if(d && memcmp(out.data(), lv.payload, (size_t)r.payload_bytes) != 0) bad_silent++;
				}
			}
			t.check(bad_silent == 0, (tag + "double-bit codeword error never mis-decodes").c_str(), std::to_string(n2) + " cases");
		}
	}
	// short input rejected
	{
		li_codec codec(*rate_by_index(6));
		std::vector<uint8_t> out;
		t.check(!codec.decode(vec::RATES[2].vectors[0].symbols, 10, 0, out), "decode rejects too few symbols");
	}
}

void test_crc14(selftest_counter &t) {
	const uint16_t *tab = crc14_table();
	int ok = 0;
	for(int i = 0; i < 256; i++) ok += tab[i] == vec::CRC14_TABLE[i];
	t.check(ok == 256, "crc14_table[256]", std::to_string(ok) + "/256");
	int eok = 0;
	for(int i = 0; i < vec::N_CRC14_EXAMPLES; i++) {
		const vec::crc_example &e = vec::CRC14_EXAMPLES[i];
		eok += crc14(e.data, (size_t)e.len, e.init) == e.crc;
	}
	t.check(eok == vec::N_CRC14_EXAMPLES, "crc14 examples", std::to_string(eok) + "/" + std::to_string(vec::N_CRC14_EXAMPLES));
}

void test_crsf(selftest_counter &t) {
	int ok = 0, n = 0;
	for(int i = 0; i < vec::N_CRSF_CRSF_TO_UINT10; i++) { n++; ok += crsf_to_uint10((uint16_t)vec::CRSF_CRSF_TO_UINT10[i].a) == vec::CRSF_CRSF_TO_UINT10[i].b; }
	for(int i = 0; i < vec::N_CRSF_UINT10_TO_CRSF; i++) { n++; ok += uint10_to_crsf((uint16_t)vec::CRSF_UINT10_TO_CRSF[i].a) == vec::CRSF_UINT10_TO_CRSF[i].b; }
	for(int i = 0; i < vec::N_CRSF_CRSF_TO_N128; i++) { n++; ok += crsf_to_n((uint16_t)vec::CRSF_CRSF_TO_N128[i].a, 128) == vec::CRSF_CRSF_TO_N128[i].b; }
	for(int i = 0; i < vec::N_CRSF_CRSF_TO_N64; i++) { n++; ok += crsf_to_n((uint16_t)vec::CRSF_CRSF_TO_N64[i].a, 64) == vec::CRSF_CRSF_TO_N64[i].b; }
	for(int i = 0; i < vec::N_CRSF_N_TO_CRSF127; i++) { n++; ok += n_to_crsf((uint16_t)vec::CRSF_N_TO_CRSF127[i].a, 127) == vec::CRSF_N_TO_CRSF127[i].b; }
	t.check(ok == n, "crsf conversions", std::to_string(ok) + "/" + std::to_string(n));
	t.check(bit_to_crsf(true) == 1792 && bit_to_crsf(false) == 191, "bit_to_crsf");
	int sok = 0;
	for(int i = 0; i < 32; i++) sok += hybrid_wide_switch_index((uint8_t)i) == vec::HYBRID_WIDE_SWITCH_INDEX[i];
	t.check(sok == 32, "hybrid_wide_switch_index[32]", std::to_string(sok) + "/32");
}

void test_ota4(selftest_counter &t) {
	uid_t6 u = test_uid();
	int bok = 0, dok = 0, rok = 0, nok = 0, rej = 0;
	for(int i = 0; i < vec::N_OTA4_RC; i++) {
		const vec::rc_vec &r = vec::OTA4_RC[i];
		std::array<uint16_t, 16> ch;
		for(int k = 0; k < 16; k++) ch[k] = r.channels_crsf[k];
		std::array<uint8_t, 8> pkt = ota4_build_rc(u, ch, (uint8_t)r.nonce, r.hop_interval, r.tlm_denom, 2, false);
		bool b = memcmp(pkt.data(), r.packet, 8) == 0;
		bok += b;
		if(!b && bok == i) t.check(false, "ota4_build_rc first mismatch", "nonce " + std::to_string(r.nonce) + " got " + hex8(pkt.data(), 8) + " want " + hex8(r.packet, 8));
		// decode without nonce (MOD_HOP view)
		ota4_decoded d = ota4_decode(r.packet, u, r.hop_interval, r.tlm_denom);
		bool ok = d.type == ota_type::RC && d.crc_ok && d.crc_high == r.d_crc_high && d.crc_low == r.d_crc_low &&
		          (int)d.nonce_residues.size() == r.n_residues && d.rc.switch_index == -1 && d.rc.aux_channel == -1 &&
		          d.rc.ch4_high == (r.d_ch4_armed != 0);
		for(int k = 0; ok && k < r.n_residues; k++) ok = d.nonce_residues[k] == r.residues[k];
		for(int k = 0; ok && k < 4; k++) ok = d.rc.ch[k] == r.d_ch1_4[k] && d.rc.ch[k] == r.expected_ch1_4[k];
		ok = ok && ((r.n_residues == 1 && r.residues[0] == r.nonce % r.hop_interval) || r.n_residues > 1);
		ok = ok && d.rc.ch[4] == (r.d_ch4_armed ? 1792 : 191);
		dok += ok;
		// decode with nonce (FULL view): switch fields
		ota4_decoded f = ota4_decode(r.packet, u, r.hop_interval, r.tlm_denom, r.nonce);
		bool fo = f.crc_ok && f.rc.switch_index == r.d_switch_index;
		if(r.d_switch_index == 7) {
			fo = fo && f.rc.aux_channel == -1 && f.rc.uplink_tx_power == r.d_tx_power && (int)f.rc.telem_bit == r.d_telem_bit;
		} else {
			fo = fo && f.rc.aux_channel == r.d_aux_channel && f.rc.aux_value_raw == r.d_aux_value_raw && f.rc.ch[r.d_aux_channel] == r.d_aux_crsf &&
			     f.rc.uplink_tx_power == -1;
		}
		nok += fo;
		// resolve later == resolve now
		rc_packet rc = d.rc;
		ota4_resolve_switch(rc, (uint8_t)r.nonce, r.tlm_denom);
		rok += rc.switch_index == f.rc.switch_index && rc.aux_channel == f.rc.aux_channel && rc.aux_value_raw == f.rc.aux_value_raw &&
		       rc.ch == f.rc.ch && rc.uplink_tx_power == f.rc.uplink_tx_power && rc.telem_bit == f.rc.telem_bit;
		// corrupted packet -> crc fails
		uint8_t bad[8];
		memcpy(bad, r.packet, 8);
		bad[3] ^= 0x10;
		ota4_decoded e = ota4_decode(bad, u, r.hop_interval, r.tlm_denom);
		rej += !e.crc_ok && e.nonce_residues.empty();
	}
	t.check(bok == vec::N_OTA4_RC, "ota4_build_rc == JSON packets (hop 4/2, tlm 64/128/16)", std::to_string(bok) + "/" + std::to_string(vec::N_OTA4_RC));
	t.check(dok == vec::N_OTA4_RC, "ota4_decode RC (no nonce): crc residues, ch1..4, ch4 bit", std::to_string(dok) + "/" + std::to_string(vec::N_OTA4_RC));
	t.check(nok == vec::N_OTA4_RC, "ota4_decode RC (with nonce): switch/aux/power fields", std::to_string(nok) + "/" + std::to_string(vec::N_OTA4_RC));
	t.check(rok == vec::N_OTA4_RC, "ota4_resolve_switch == decode with nonce", std::to_string(rok) + "/" + std::to_string(vec::N_OTA4_RC));
	t.check(rej == vec::N_OTA4_RC, "ota4_decode rejects a flipped bit", std::to_string(rej) + "/" + std::to_string(vec::N_OTA4_RC));
	// wrong UID -> no residue matches (for all 54 vectors)
	{
		uid_t6 other = u;
		other[5] ^= 0x40;
		int r0 = 0;
		for(int i = 0; i < vec::N_OTA4_RC; i++) r0 += !ota4_decode(vec::OTA4_RC[i].packet, other, vec::OTA4_RC[i].hop_interval, vec::OTA4_RC[i].tlm_denom).crc_ok;
		t.check(r0 == vec::N_OTA4_RC, "ota4_decode with another UID fails the CRC", std::to_string(r0) + "/" + std::to_string(vec::N_OTA4_RC));
	}
	// sync
	int sb = 0, sd = 0;
	for(int i = 0; i < vec::N_OTA4_SYNC; i++) {
		const vec::sync_vec &s = vec::OTA4_SYNC[i];
		std::array<uint8_t, 8> pkt = ota4_build_sync(u, (uint8_t)s.fhss_index, (uint8_t)s.nonce, s.rate_index, (uint8_t)s.new_tlm_ratio_field, s.switch_enc_mode != 0, 0);
		bool b = memcmp(pkt.data(), s.packet, 8) == 0;
		sb += b;
		if(!b) t.check(false, "ota4_build_sync mismatch", "got " + hex8(pkt.data(), 8) + " want " + hex8(s.packet, 8));
		ota4_decoded d = ota4_decode(s.packet, u, 4, 64);
		bool ok = d.type == ota_type::SYNC && d.crc_ok && d.sync.fhss_index == s.fhss_index && d.sync.nonce == s.nonce &&
		          d.sync.rate_index == s.rate_index && d.sync.new_tlm_ratio == s.new_tlm_ratio_field && d.sync.tlm_denom() == s.tlm_denom &&
		          (int)d.sync.switch_enc_mode == s.switch_enc_mode && d.sync.uid3 == s.uid345[0] && d.sync.uid4 == s.uid345[1] &&
		          d.sync.uid5 == s.uid345[2] && d.sync.uid_match(u);
		sd += ok;
	}
	t.check(sb == vec::N_OTA4_SYNC, "ota4_build_sync == JSON (3.3.1 layout, crcHigh 0)", std::to_string(sb) + "/" + std::to_string(vec::N_OTA4_SYNC));
	t.check(sd == vec::N_OTA4_SYNC, "ota4_decode SYNC fields + uid_match", std::to_string(sd) + "/" + std::to_string(vec::N_OTA4_SYNC));
	{
		// model-match XOR on uid5 keeps the top 2 bits -> uid_match still true; changed uid4 -> false
		std::array<uint8_t, 8> pkt = ota4_build_sync(u, 0, 0, 6, 2, false, 0x2A);
		ota4_decoded d = ota4_decode(pkt.data(), u, 4, 64);
		sync_packet sp = d.sync;
		bool m1 = d.crc_ok && sp.uid_match(u);
		sp.uid4 ^= 1;
		t.check(m1 && !sp.uid_match(u), "sync uid_match ignores modelId bits of UID5, checks UID3/UID4");
		t.check(sync_packet{}.tlm_denom() == 1, "sync tlm_denom(field 0) == 1");
	}
}

void test_freerun(selftest_counter &t) {
	int ok = 0;
	for(int i = 0; i < vec::N_FREERUN_LOWBIAS32; i++) ok += lowbias32(vec::FREERUN_LOWBIAS32[i].a) == vec::FREERUN_LOWBIAS32[i].b;
	t.check(ok == vec::N_FREERUN_LOWBIAS32, "lowbias32 examples", std::to_string(ok) + "/" + std::to_string(vec::N_FREERUN_LOWBIAS32));
	int cok = 0;
	for(int i = 0; i < vec::N_FREERUN_CHANNELS; i++) {
		std::array<uint16_t, 16> ch = freerun_channels(vec::FREERUN_CHANNELS[i].tick);
		bool e = true;
		for(int k = 0; k < 16; k++) e = e && ch[k] == vec::FREERUN_CHANNELS[i].ch[k];
		cok += e;
	}
	t.check(cok == vec::N_FREERUN_CHANNELS, "freerun_channels(tick)", std::to_string(cok) + "/" + std::to_string(vec::N_FREERUN_CHANNELS));
	t.check(FREERUN_TICK_OFFSET_250HZ == -2 + 4 * 50 + 960 * 608, "FREERUN_TICK_OFFSET_250HZ == k - 2 + 4*50 + 960*608");
	// ground-truth packet for a capture slot round-trips through the codec of rate 6
	uid_t6 u = test_uid();
	uint32_t tick = (uint32_t)(100 + FREERUN_TICK_OFFSET_250HZ);
	std::array<uint8_t, 8> pkt = ota4_build_rc(u, freerun_channels(tick), (uint8_t)(tick % 256), 4, 64, 2, false);
	li_codec codec(*rate_by_index(6));
	std::vector<uint16_t> sym = codec.encode(pkt.data(), 8, 5);
	std::vector<uint8_t> out;
	bool rt = codec.decode(sym.data(), sym.size(), 5, out) && memcmp(out.data(), pkt.data(), 8) == 0;
	ota4_decoded d = ota4_decode(out.data(), u, 4, 64, (int)(tick % 256));
	t.check(rt && d.crc_ok && d.nonce_residues.size() == 1 && d.nonce_residues[0] == (int)(tick % 4), "freerun truth packet: build -> LI encode -> decode -> crc ok");
}

// ---- link ------------------------------------------------------------------------------------

void test_slot_clock(selftest_counter &t) {
	// ideal raster
	slot_clock c(4e-3);
	bool ok = true;
	for(int k = 0; k < 100; k++) ok = ok && c.observe(1.0 + k * 4e-3) == k;
	t.check(ok && c.valid() && c.last_slot() == 99, "slot_clock ideal raster");
	// 100 ppm drift + 3 % jitter + dropouts: slots must stay integer-correct
	slot_clock d(4e-3);
	test_rng rng(7);
	bool dok = true;
	int64_t expect = 0;
	double T = 4e-3 * (1 + 100e-6);
	double t0 = 0.5;
	int nobs = 0;
	for(int k = 0; k < 5000; k++) {
		if(k > 0 && rng.below(10) < 3) continue;   // 30 % dropouts
		double jit = ((double)rng.below(1000) / 1000.0 - 0.5) * 0.06 * 4e-3;
		bool off = false;
		int64_t s = d.observe(t0 + k * T + jit, &off);
		if(nobs == 0) expect = k;
		if(s != k - expect || off) dok = false;
		nobs++;
	}
	t.check(dok, "slot_clock 100 ppm drift, 3 % jitter, 30 % dropouts", "interval est " + std::to_string(d.interval_estimate_s() * 1e6) + " us");
	t.check(fabs(d.interval_estimate_s() - T) < 10e-6, "slot_clock interval estimate within 10 us of the true raster (3 % jitter)", std::to_string(d.interval_estimate_s() * 1e6));
	// off-raster burst is flagged, slot still assigned
	slot_clock e(4e-3);
	e.observe(0.0);
	bool off = false;
	int64_t s = e.observe(4e-3 * 3 + 1.5e-3, &off);
	t.check(off && s == 3, "slot_clock off-raster flagged", "slot " + std::to_string(s));
	// prediction
	slot_clock p(2e-3);
	p.observe(1.0);
	p.observe(1.0 + 10 * 2e-3);
	t.check(fabs(p.predict_start_s(20) - (1.0 + 20 * 2e-3)) < 1e-9, "slot_clock predict_start_s");
	p.reset();
	t.check(!p.valid() && p.observe(5.0) == 0, "slot_clock reset");
	// 20 ms raster (50 Hz) with 200 ppm drift stays locked over 60 s
	slot_clock f(20e-3);
	bool fok = true;
	for(int k = 0; k < 3000; k++) fok = fok && f.observe(k * 20e-3 * (1 + 200e-6)) == k;
	t.check(fok, "slot_clock 50 Hz raster, 200 ppm, 60 s");
}

void test_link_model(selftest_counter &t) {
	uid_t6 u = test_uid();
	const rate_params &rp = *rate_by_index(6);
	link_model lm(u, rp);
	t.check(lm.lock().state == lock_state::NONE && lm.next_sync_slot(0) == -1, "link_model starts without lock");
	// lock A from the JSON sync vectors: the packet's own slot has its channel / nonce
	int aok = 0;
	for(int i = 0; i < vec::N_OTA4_SYNC; i++) {
		const vec::sync_vec &s = vec::OTA4_SYNC[i];
		const rate_params &r = *rate_by_index(s.rate_index);
		link_model m(u, r);
		ota4_decoded d = ota4_decode(s.packet, u, r.hop_interval, r.tlm_denom_default);
		int64_t slot = 1000 + 17 * i;
		m.lock_from_sync(slot, d.sync);
		bool ok = m.lock().state == lock_state::FULL && m.nonce_at(slot) == s.nonce && m.fhss_ptr_at(slot) == s.fhss_index &&
		          m.channel_at(slot) == vec::FHSS_SEQUENCE[s.fhss_index] && m.nonce_residue_at(slot) == s.nonce % r.hop_interval &&
		          m.is_dwell_start(slot) == (s.nonce % r.hop_interval == 0);
		// walk forward: pointer advances once per dwell, channel follows the sequence
		int64_t k = slot;
		int ptr = s.fhss_index;
		uint8_t nonce = (uint8_t)s.nonce;
		for(int step = 0; step < 2000 && ok; step++) {
			ok = m.nonce_at(k) == nonce && m.fhss_ptr_at(k) == ptr && m.channel_at(k) == vec::FHSS_SEQUENCE[ptr] &&
			     fabs(m.freq_hz_at(k) - vec::FHSS_FREQ_NOMINAL_HZ[vec::FHSS_SEQUENCE[ptr]]) < 1e-3;
			if((nonce + 1) % r.hop_interval == 0) ptr = (ptr + 1) % 240;
			nonce = (uint8_t)(nonce + 1);
			k++;
		}
		// and backward
		k = slot;
		ptr = s.fhss_index;
		nonce = (uint8_t)s.nonce;
		for(int step = 0; step < 600 && ok; step++) {
			k--;
			nonce = (uint8_t)(nonce - 1);
			if((nonce + 1) % r.hop_interval == 0) ptr = (ptr + 239) % 240;
			ok = m.nonce_at(k) == nonce && m.fhss_ptr_at(k) == ptr;
		}
		aok += ok;
	}
	t.check(aok == vec::N_OTA4_SYNC, "lock A from the JSON sync packets: nonce/ptr/channel walk +2000/-600 slots (wraps)", std::to_string(aok) + "/" + std::to_string(vec::N_OTA4_SYNC));
	// next_sync_slot: from a FULL lock at ptr 5, the next channel-41 dwell is ptr 80 -> 75 dwells later
	{
		link_model m(u, rp);
		sync_packet sp;
		sp.nonce = 0;
		sp.fhss_index = 5;
		m.lock_from_sync(100, sp);
		int64_t ns = m.next_sync_slot(100);
		bool ok = ns == 100 + 75 * 4 && m.is_sync_channel_at(ns) && !m.is_sync_channel_at(ns - 1) && m.is_sync_channel_at(ns + 3) && !m.is_sync_channel_at(ns + 4);
		int64_t ns2 = m.next_sync_slot(ns + 4);
		ok = ok && ns2 == ns + 80 * 4;
		t.check(ok, "next_sync_slot (75 dwells ahead, then +80 dwells)", std::to_string(ns) + " " + std::to_string(ns2));
		// consistency + mismatch counter
		link_model::observation good{ns + 1, 41, {1}};
		link_model::observation bad{ns + 1, 42, {}};
		link_model::observation badres{ns + 1, 41, {0, 2}};
		ok = m.consistent(good) && !m.consistent(bad) && !m.consistent(badres);
		m.add_observation(bad);
		m.add_observation(bad);
		int mm = m.mismatches();
		m.add_observation(good);
		ok = ok && mm == 2 && m.mismatches() == 0;
		t.check(ok, "consistent() and mismatch counter");
		m.drop_lock();
		t.check(m.lock().state == lock_state::NONE && m.next_sync_slot(0) == -1, "drop_lock");
	}
	// lock B: synthetic trace with dropouts, nonce wrap, only a 25-channel window visible
	for(int hop_case = 0; hop_case < 2; hop_case++) {
		const rate_params &r = *rate_by_index(hop_case == 0 ? 6 : 9);
		const int hop = r.hop_interval;
		link_model truth(u, r);
		sync_packet sp;
		sp.nonce = 250;             // wraps within the trace
		sp.fhss_index = 233;        // wraps the sequence too
		truth.lock_from_sync(0, sp);
		link_model fitm(u, r);
		test_rng rng(hop_case + 11);
		int seen = 0;
		bool locked = false;
		int64_t lock_at = -1;
		for(int64_t s = 0; s < 4000 && !locked; s++) {
			int ch = truth.channel_at(s);
			if(ch < 29 || ch > 53) continue;              // outside the 25-MHz window
			if(rng.below(10) < 2) continue;               // 20 % missed bursts
			link_model::observation o{s, ch, {}};
			if(rng.below(2)) o.nonce_residues = {truth.nonce_residue_at(s)};   // CRC residue known half the time
			fitm.add_observation(o);
			seen++;
			if(seen >= 8 && seen % 4 == 0 && fitm.fit(8)) {
				locked = true;
				lock_at = s;
			}
		}
		bool ok = locked && fitm.lock().state == lock_state::MOD_HOP && fitm.lock().candidates == 1;
		int agree = 0;
		for(int64_t s = lock_at; s < lock_at + 3000 && ok; s++) {
			agree += fitm.channel_at(s) == truth.channel_at(s) && fitm.nonce_residue_at(s) == truth.nonce_residue_at(s) &&
			         fitm.fhss_ptr_at(s) == truth.fhss_ptr_at(s);
		}
		ok = ok && agree == 3000;
		t.check(ok, hop_case == 0 ? "lock B (hop 4, window 29..53, 20 % dropouts, nonce+ptr wrap) predicts 3000 slots" :
		                            "lock B (hop 2, 50 Hz, same trace) predicts 3000 slots",
		        "locked after " + std::to_string(seen) + " observations at slot " + std::to_string(lock_at) + ", score " + std::to_string(fitm.lock().score));
		// the fit must NOT lock on random channels
		link_model noise(u, r);
		for(int64_t s = 0; s < 200; s++) noise.add_observation({s, (int)rng.below(80), {}});
		t.check(!noise.fit(8) && noise.lock().state == lock_state::NONE, hop_case == 0 ? "lock B refuses random channels (hop 4)" : "lock B refuses random channels (hop 2)");
		// a FULL lock that the fit confirms stays FULL
		if(hop_case == 0) {
			link_model full(u, r);
			full.lock_from_sync(0, sp);
			for(int64_t s = 0; s < 400; s++) full.add_observation({s, truth.channel_at(s), {}});
			bool f = full.fit(8) && full.lock().state == lock_state::FULL && full.nonce_at(0) == 250;
			t.check(f, "fit() confirms an existing FULL lock (keeps the absolute nonce)");
			// a wrong SYNC lock is overridden by the fit (state -> MOD_HOP)
			link_model wrong(u, r);
			sync_packet bad = sp;
			bad.fhss_index = 7;
			wrong.lock_from_sync(0, bad);
			for(int64_t s = 0; s < 400; s++) wrong.add_observation({s, truth.channel_at(s), {}});
			int mm = wrong.mismatches();
			bool w = wrong.fit(8) && wrong.lock().state == lock_state::MOD_HOP && wrong.channel_at(1234) == truth.channel_at(1234);
			t.check(w && mm >= 4, "fit() replaces a contradicted FULL lock, mismatches counted", "mismatches " + std::to_string(mm));
		}
		(void)hop;
	}
	// set_rate / set_uid reset the lock
	{
		link_model m(u, rp);
		sync_packet sp;
		m.lock_from_sync(0, sp);
		m.set_rate(*rate_by_index(4));
		bool a = m.lock().state == lock_state::NONE && m.rate().index == 4;
		m.lock_from_sync(0, sp);
		uid_t6 v = u;
		v[2] ^= 1;
		m.set_uid(v);
		bool b = m.lock().state == lock_state::NONE && m.sequence() == fhss_sequence(v) && m.sequence() != fhss_sequence(u);
		t.check(a && b, "set_rate / set_uid reset the lock, sequence follows the uid");
	}
}

} // namespace

int run_selftest_codec() {
	selftest_counter t;
	test_uid_and_seeds(t);
	test_fhss(t);
	test_whitening_gray(t);
	test_rate_table(t);
	test_li_codec(t);
	test_crc14(t);
	test_crsf(t);
	test_ota4(t);
	test_freerun(t);
	printf("codec: %d pass, %d fail\n", t.passes, t.fails);
	return t.fails;
}

int run_selftest_link() {
	selftest_counter t;
	test_slot_clock(t);
	test_link_model(t);
	printf("link: %d pass, %d fail\n", t.passes, t.fails);
	return t.fails;
}

} // namespace elrsd

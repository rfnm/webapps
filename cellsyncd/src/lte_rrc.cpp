// cellsync: 36.331 UPER head of SystemInformationBlockType1 - see lte_rrc.h.
//
// The grammar below is transcribed field by field from 36.331 (SystemInformationBlockType1
// and its children) with the X.691 UPER rules applied by hand:
//   - a SEQUENCE starts with one preamble bit per OPTIONAL field, in declaration order;
//   - a CHOICE with n alternatives spends ceil(log2 n) bits on the index;
//   - a constrained INTEGER (lb..ub) spends ceil(log2(ub-lb+1)) bits on value-lb;
//   - a constrained SEQUENCE OF spends the same on count-lb before the elements;
//   - a non-extensible ENUMERATED is a constrained integer over its values;
//   - an extensible ENUMERATED spends 1 extension bit first; extended values follow as a
//     "normally small non-negative whole number" (X.691 10.6: one 0 bit + 6 bits for <64);
//   - fixed-size BIT STRINGs are just their bits, MSB first.
// None of the types parsed here carry an extension marker of their own - SIB1 evolves via
// nonCriticalExtension instead, which trails the message and can therefore be ignored.
//
// Verification story (lesson 2 of applies to grammars too): the selftest pins
// this decoder against the two ON-AIR transport blocks - bytes produced by two different
// real eNodeB encoders, field values confirmed by the external oracle AND by a manual
// bit walk - plus one hand-assembled vector for the branches the air never exercises
// (MCC inheritance, 3-digit MNC, csg-Identity, p-Max, TDD config, extended SIB-Type).

#include "lte_rrc.h"

#include <cstdio>
#include <cstring>

namespace cellsync {

namespace {

// MSB-first bit reader. Overrun poisons the reader instead of faulting: the caller gets
// one error at the end rather than a bounds check per field.
struct bitrd {
	const uint8_t *b;
	size_t nbits, pos;
	bool overrun;

	bitrd(const uint8_t *bytes, size_t nbytes) : b(bytes), nbits(nbytes * 8), pos(0), overrun(false) {}

	uint32_t get(unsigned n) {
		if(pos + n > nbits) {
			overrun = true;
			return 0;
		}
		uint32_t v = 0;
		for(unsigned i = 0; i < n; i++, pos++) {
			v = (v << 1) | ((b[pos >> 3] >> (7 - (pos & 7))) & 1);
		}
		return v;
	}

	// constrained INTEGER lb..ub (range <= 2^32): fixed ceil(log2 range) bits
	int cint(int lb, int ub) {
		uint32_t range = (uint32_t)(ub - lb) + 1;
		unsigned n = 0;
		while(((uint64_t)1 << n) < range) {
			n++;
		}
		return lb + (int)get(n);
	}
};

} // namespace

int sib1_plmn_mcc(const sib1_plmn &p) {
	return p.mcc[0] * 100 + p.mcc[1] * 10 + p.mcc[2];
}

int sib1_plmn_mnc(const sib1_plmn &p) {
	int v = 0;
	for(int i = 0; i < p.mnc_digits; i++) {
		v = v * 10 + p.mnc[i];
	}
	return v;
}

int sib1_head_decode(const uint8_t *tb, size_t nbytes, sib1_head &out) {
	bitrd r(tb, nbytes);
	out = sib1_head{};

	// BCCH-DL-SCH-Message ::= SEQUENCE { message CHOICE { c1 CHOICE {
	//   systemInformation, systemInformationBlockType1 }, messageClassExtension } }
	if(r.get(1) != 0) {
		return -1; // messageClassExtension - not a SIB1
	}
	if(r.get(1) != 1) {
		return -2; // c1 alternative 0 is systemInformation (SIBs 2+), not our message
	}

	// SIB1 root preamble: p-Max, tdd-Config, nonCriticalExtension - in DECLARATION order,
	// which is not transmission order (p-Max sits between the two mandatory groups).
	bool has_pmax = r.get(1);
	out.has_tdd_config = r.get(1);
	r.get(1); // nonCriticalExtension present - trails the message, never read

	// cellAccessRelatedInfo ::= SEQUENCE { plmn-IdentityList, trackingAreaCode,
	//   cellIdentity, cellBarred, intraFreqReselection, csg-Indication,
	//   csg-Identity OPTIONAL }
	out.has_csg_identity = r.get(1);

	int nplmn = (int)r.get(3) + 1; // SIZE (1..6)
	if(nplmn > 6) {
		return -3;
	}
	int prev_mcc[3] = {0, 0, 0};
	bool have_mcc = false;
	for(int i = 0; i < nplmn; i++) {
		sib1_plmn p{};
		// PLMN-Identity ::= SEQUENCE { mcc MCC OPTIONAL, mnc MNC }: an absent MCC means
		// "same as the previous entry" (36.331 field description), so the first entry
		// must carry one.
		if(r.get(1)) {
			for(int k = 0; k < 3; k++) {
				prev_mcc[k] = (int)r.get(4); // MCC-MNC-Digit ::= INTEGER (0..9)
			}
			have_mcc = true;
		} else if(!have_mcc) {
			return -4;
		}
		memcpy(p.mcc, prev_mcc, sizeof(p.mcc));
		p.mnc_digits = 2 + (int)r.get(1); // MNC ::= SEQUENCE (SIZE (2..3)) OF digit
		for(int k = 0; k < p.mnc_digits; k++) {
			p.mnc[k] = (int)r.get(4);
		}
		for(int k = 0; k < 3; k++) {
			if(p.mcc[k] > 9 || (k < p.mnc_digits && p.mnc[k] > 9)) {
				return -5; // 4-bit field, values 10..15 are illegal - a corrupt block
			}
		}
		p.reserved_for_operator = (r.get(1) == 0); // ENUMERATED {reserved, notReserved}
		out.plmn.push_back(p);
	}

	out.tac = (uint16_t)r.get(16);       // BIT STRING (SIZE (16))
	out.cell_identity = r.get(28);       // BIT STRING (SIZE (28))
	out.cell_barred = (r.get(1) == 0);   // ENUMERATED {barred, notBarred}
	out.intra_freq_resel = (r.get(1) == 0); // ENUMERATED {allowed, notAllowed}
	out.csg_indication = r.get(1);       // BOOLEAN
	if(out.has_csg_identity) {
		out.csg_identity = r.get(27);    // BIT STRING (SIZE (27))
	}

	// cellSelectionInfo ::= SEQUENCE { q-RxLevMin INTEGER (-70..-22),
	//   q-RxLevMinOffset INTEGER (1..8) OPTIONAL }
	out.has_q_rxlevmin_offset = r.get(1);
	out.q_rxlevmin = r.cint(-70, -22);
	if(out.has_q_rxlevmin_offset) {
		out.q_rxlevmin_offset = r.cint(1, 8);
	}

	out.has_p_max = has_pmax;
	if(has_pmax) {
		out.p_max = r.cint(-30, 33); // P-Max ::= INTEGER (-30..33)
	}

	out.freq_band = r.cint(1, 64); // FreqBandIndicator ::= INTEGER (1..maxFBI), maxFBI=64

	// SchedulingInfoList ::= SEQUENCE (SIZE (1..maxSI-Message)) OF SchedulingInfo,
	// maxSI-Message = 32. This is the input the SIB16 stage needs.
	int nsched = (int)r.get(5) + 1;
	static const int PERIOD_RF[8] = {8, 16, 32, 64, 128, 256, 512, 0};
	for(int i = 0; i < nsched; i++) {
		sib1_sched s{};
		s.periodicity_rf = PERIOD_RF[r.get(3) & 7]; // ENUMERATED {rf8..rf512}: 7 values
		int nmap = (int)r.get(5);                   // SIB-MappingInfo SIZE (0..maxSIB-1)
		for(int k = 0; k < nmap; k++) {
			// SIB-Type ::= ENUMERATED { sibType3, ..., sibType18, ... }: 16 root values
			// (index + 3 = SIB number), later releases extend past the marker.
			int sib;
			if(r.get(1) == 0) {
				sib = 3 + (int)r.get(4);
			} else {
				// X.691 10.6 normally-small number; a set flag would mean a length-
				// prefixed value >= 64, which no release of the enum comes near.
				if(r.get(1) != 0) {
					return -6;
				}
				sib = 3 + 16 + (int)r.get(6);
			}
			s.sib_types.push_back(sib);
		}
		out.sched.push_back(s);
	}

	if(out.has_tdd_config) {
		// TDD-Config ::= SEQUENCE { subframeAssignment ENUMERATED {sa0..sa6},
		//   specialSubframePatterns ENUMERATED {ssp0..ssp8} }: 3 + 4 bits, values unused
		// on this FDD-only path but they must be consumed to keep the tail aligned.
		r.get(3);
		r.get(4);
	}

	static const int SI_WINDOW_MS[8] = {1, 2, 5, 10, 15, 20, 40, 0};
	out.si_window_ms = SI_WINDOW_MS[r.get(3) & 7]; // ENUMERATED {ms1..ms40}: 7 values
	out.si_value_tag = (int)r.get(5);              // INTEGER (0..31)

	// One overrun check for the whole walk. Trailing bits after this point are the
	// nonCriticalExtension and UPER padding - legal and deliberately unread.
	return r.overrun ? -7 : 0;
}

// ---- selftest --------------------------------------------------------------------------

namespace {

int g_fail = 0;

void check(bool ok, const char *name, const char *detail, bool verbose) {
	if(!ok) {
		g_fail++;
	}
	if(verbose || !ok) {
		printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
		       detail && *detail ? ": " : "", detail ? detail : "");
	}
}

// MSB-first bit assembler for the hand-built vector: the (value,width) pairs in the test
// are the hand-derived artifact; only the trivial packing convention is shared with the
// reader, and that convention is independently pinned by the on-air byte literals.
struct bitwr {
	std::vector<uint8_t> bytes;
	size_t pos = 0;

	void put(uint32_t v, unsigned n) {
		for(unsigned i = 0; i < n; i++, pos++) {
			if((pos & 7) == 0) {
				bytes.push_back(0);
			}
			bytes.back() |= (uint8_t)(((v >> (n - 1 - i)) & 1) << (7 - (pos & 7)));
		}
	}
};

} // namespace

int lte_rrc_selftest(bool verbose) {
	g_fail = 0;
	char d[200];

	// --- the 816.0 MHz reference cell (PCI 274): the on-air transport block, byte-identical
	// off our own chain and the external oracle, all head fields hand-walked against 36.331.
	static const uint8_t TB816[26] = {
		0x48, 0x49, 0x88, 0x03, 0x11, 0xfa, 0x19, 0x6b, 0x10, 0x68, 0x19, 0x30, 0x90,
		0x81, 0x04, 0x58, 0x18, 0x2a, 0x49, 0x50, 0x18, 0x06, 0x05, 0x24, 0x15, 0x60};
	{
		sib1_head h;
		int rc = sib1_head_decode(TB816, sizeof(TB816), h);
		bool ok = rc == 0 && h.plmn.size() == 1 && sib1_plmn_mcc(h.plmn[0]) == 262 &&
		          sib1_plmn_mnc(h.plmn[0]) == 1 && h.plmn[0].mnc_digits == 2 &&
		          !h.plmn[0].reserved_for_operator && h.tac == 0x11FA &&
		          h.cell_identity == 0x196B106 && !h.cell_barred && h.intra_freq_resel &&
		          !h.csg_indication && !h.has_csg_identity && h.q_rxlevmin == -64 &&
		          !h.has_p_max && h.freq_band == 20 && !h.has_tdd_config &&
		          h.si_window_ms == 40;
		snprintf(d, sizeof(d), "rc=%d PLMN %d-%0*d TAC 0x%04X ECI 0x%07X band %d qRxLevMin %d siWin %d ms",
		         rc, h.plmn.empty() ? 0 : sib1_plmn_mcc(h.plmn[0]),
		         h.plmn.empty() ? 1 : h.plmn[0].mnc_digits,
		         h.plmn.empty() ? 0 : sib1_plmn_mnc(h.plmn[0]),
		         h.tac, h.cell_identity, h.freq_band, h.q_rxlevmin, h.si_window_ms);
		check(ok, "sib1_head_816", d, verbose);
		// eNB/sector split per - the arithmetic that was once wrong in the doc
		check(ok && (h.cell_identity >> 8) == 104113 && (h.cell_identity & 0xFF) == 6,
		      "sib1_eci_split_816", "eNB 104113 sector 6", verbose);
	}

	// --- the 796.0 MHz reference cell (PCI 377), same standing as above.
	static const uint8_t TB796[28] = {
		0x68, 0x49, 0x88, 0x07, 0x2a, 0xf8, 0x03, 0x6d, 0x80, 0x18, 0x1b, 0x54, 0xc4, 0x40,
		0x82, 0x06, 0x11, 0x58, 0x60, 0xa9, 0x25, 0x40, 0x60, 0x18, 0x14, 0x90, 0x55, 0x80};
	{
		sib1_head h;
		int rc = sib1_head_decode(TB796, sizeof(TB796), h);
		bool ok = rc == 0 && h.plmn.size() == 1 && sib1_plmn_mcc(h.plmn[0]) == 262 &&
		          sib1_plmn_mnc(h.plmn[0]) == 3 && h.plmn[0].mnc_digits == 2 &&
		          h.tac == 0x2AF8 && h.cell_identity == 0x036D801 && !h.cell_barred &&
		          h.freq_band == 20 && !h.has_tdd_config;
		snprintf(d, sizeof(d), "rc=%d PLMN %d-%0*d TAC 0x%04X ECI 0x%07X band %d (eNB %u sector %u)",
		         rc, h.plmn.empty() ? 0 : sib1_plmn_mcc(h.plmn[0]),
		         h.plmn.empty() ? 1 : h.plmn[0].mnc_digits,
		         h.plmn.empty() ? 0 : sib1_plmn_mnc(h.plmn[0]),
		         h.tac, h.cell_identity, h.freq_band, h.cell_identity >> 8, h.cell_identity & 0xFF);
		check(ok, "sib1_head_796", d, verbose);
		// The SIB16 stage will consume this list: it must parse to SOMETHING plausible
		// (1..32 entries, legal periodicities) on both real cells.
		bool sched_ok = rc == 0 && !h.sched.empty();
		for(auto &s : h.sched) {
			sched_ok = sched_ok && s.periodicity_rf >= 8 && s.periodicity_rf <= 512;
		}
		snprintf(d, sizeof(d), "%zu entries, first rf%d with %zu SIBs", h.sched.size(),
		         h.sched.empty() ? 0 : h.sched[0].periodicity_rf,
		         h.sched.empty() ? 0 : h.sched[0].sib_types.size());
		check(sched_ok, "sib1_sched_796", d, verbose);
	}

	// --- hand-assembled vector for every branch the two FDD macro cells never take:
	// second PLMN inheriting the MCC, 3-digit MNC, reserved flag, csg-Identity, barred,
	// q-RxLevMinOffset, p-Max, extended SIB-Type, TDD config. Field widths and values
	// written straight from the 36.331/X.691 walk in the header comment.
	{
		bitwr w;
		w.put(0, 1);              // c1
		w.put(1, 1);              // systemInformationBlockType1
		w.put(1, 1);              // p-Max present
		w.put(1, 1);              // tdd-Config present
		w.put(0, 1);              // nonCriticalExtension absent
		w.put(1, 1);              // csg-Identity present
		w.put(1, 3);              // 2 PLMNs
		w.put(1, 1);              // PLMN 0: mcc present
		w.put(0, 4); w.put(0, 4); w.put(1, 4); // MCC 001
		w.put(1, 1);              // 3-digit MNC
		w.put(9, 4); w.put(9, 4); w.put(9, 4); // MNC 999
		w.put(0, 1);              // reserved
		w.put(0, 1);              // PLMN 1: mcc ABSENT -> inherits 001
		w.put(0, 1);              // 2-digit MNC
		w.put(4, 4); w.put(2, 4); // MNC 42
		w.put(1, 1);              // notReserved
		w.put(0xBEEF, 16);        // TAC
		w.put(0xABCDEF1, 28);     // cellIdentity
		w.put(0, 1);              // barred
		w.put(1, 1);              // intraFreqReselection notAllowed
		w.put(1, 1);              // csg-Indication true
		w.put(0x5A5A5A5, 27);     // csg-Identity
		w.put(1, 1);              // q-RxLevMinOffset present
		w.put(20, 6);             // q-RxLevMin -50 = -70 + 20
		w.put(4, 3);              // offset 5 = 1 + 4
		w.put(53, 6);             // p-Max 23 = -30 + 53
		w.put(2, 6);              // freqBandIndicator 3 = 1 + 2
		w.put(0, 5);              // 1 scheduling entry
		w.put(6, 3);              // rf512
		w.put(2, 5);              // 2 SIB types
		w.put(0, 1); w.put(13, 4);            // root index 13 = sibType16
		w.put(1, 1); w.put(0, 1); w.put(5, 6); // extended index 5 -> SIB 24
		w.put(3, 3);              // TDD subframeAssignment sa3
		w.put(7, 4);              // TDD specialSubframePatterns ssp7
		w.put(5, 3);              // si-WindowLength ms20
		w.put(17, 5);             // systemInfoValueTag

		sib1_head h;
		int rc = sib1_head_decode(w.bytes.data(), w.bytes.size(), h);
		bool ok = rc == 0 && h.plmn.size() == 2 && sib1_plmn_mcc(h.plmn[0]) == 1 &&
		          sib1_plmn_mnc(h.plmn[0]) == 999 && h.plmn[0].mnc_digits == 3 &&
		          h.plmn[0].reserved_for_operator && sib1_plmn_mcc(h.plmn[1]) == 1 &&
		          sib1_plmn_mnc(h.plmn[1]) == 42 && !h.plmn[1].reserved_for_operator &&
		          h.tac == 0xBEEF && h.cell_identity == 0xABCDEF1 && h.cell_barred &&
		          !h.intra_freq_resel && h.csg_indication && h.has_csg_identity &&
		          h.csg_identity == 0x5A5A5A5 && h.has_q_rxlevmin_offset &&
		          h.q_rxlevmin == -50 && h.q_rxlevmin_offset == 5 && h.has_p_max &&
		          h.p_max == 23 && h.freq_band == 3 && h.sched.size() == 1 &&
		          h.sched[0].periodicity_rf == 512 && h.sched[0].sib_types.size() == 2 &&
		          h.sched[0].sib_types[0] == 16 && h.sched[0].sib_types[1] == 24 &&
		          h.has_tdd_config && h.si_window_ms == 20 && h.si_value_tag == 17;
		snprintf(d, sizeof(d), "rc=%d: MCC inheritance, MNC-999/42, CSG, p-Max, ext SIB-Type, TDD", rc);
		check(ok, "sib1_hand_vector", d, verbose);
	}

	// --- negatives: the decoder must refuse, not misread.
	{
		sib1_head h;
		uint8_t not_sib1[26];
		memcpy(not_sib1, TB816, sizeof(not_sib1));
		not_sib1[0] ^= 0x80; // messageClassExtension
		bool a = sib1_head_decode(not_sib1, sizeof(not_sib1), h) == -1;
		not_sib1[0] = TB816[0] ^ 0x40; // c1 -> systemInformation
		bool b = sib1_head_decode(not_sib1, sizeof(not_sib1), h) == -2;
		bool c = sib1_head_decode(TB816, 8, h) == -7; // truncated mid-message
		snprintf(d, sizeof(d), "ext=%d, systemInformation=%d, truncation=%d", a, b, c);
		check(a && b && c, "sib1_negatives", d, verbose);
	}

	return g_fail;
}

} // namespace cellsync

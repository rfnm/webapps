// cellsync: 36.331 RRC ASN.1 (UPER) - the head of SystemInformationBlockType1.
// Third and last conventions file of the SIB1 chain: lte_seq.h owns 36.211, lte_coding.h
// owns 36.212, this file owns the 36.331 bit grammar. Hand-written on purpose - the only
// alternative is an ASN.1 code generator in the build, for a message whose interesting
// part is ~200 bits.
//
// Scope: BCCH-DL-SCH-Message down to the full SIB1 root SEQUENCE - cellAccessRelatedInfo
// (PLMN list, TAC, cellIdentity), cellSelectionInfo, freqBandIndicator, and the
// schedulingInfoList that the SIB16 stage will need to find its SI window. Everything
// behind nonCriticalExtension is ignored, which UPER makes safe: the extension is the
// last field, so ignoring it cannot shift anything we do read.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cellsync {

struct sib1_plmn {
	// Digits, not a packed BCD word: MNC "01" and "1" are different operators, so the
	// digit count is part of the identity and must survive parsing.
	int mcc[3];
	int mnc[3];
	int mnc_digits;              // 2 or 3
	bool reserved_for_operator;  // cellReservedForOperatorUse == reserved
};

struct sib1_sched {
	int periodicity_rf;          // si-Periodicity in radio frames: 8..512
	// sib-MappingInfo as SIB numbers (3 = sibType3, ... 16 = sibType16). Entry 0 of the
	// list implicitly also carries SIB2 per 36.331 5.2.2.7 - that is a scheduling rule,
	// not a bit-grammar fact, so it is NOT materialised here.
	std::vector<int> sib_types;
};

struct sib1_head {
	std::vector<sib1_plmn> plmn; // 1..6 entries
	uint16_t tac;
	uint32_t cell_identity;      // 28 bits; eNB = id >> 8, sector = id & 0xFF
	bool cell_barred;            // ENUMERATED {barred, notBarred}: bit 0 means barred
	bool intra_freq_resel;       // true = allowed
	bool csg_indication;
	bool has_csg_identity;
	uint32_t csg_identity;       // 27 bits, only when has_csg_identity
	int q_rxlevmin;              // RAW ASN value (-70..-22); dBm is 2x this
	bool has_q_rxlevmin_offset;
	int q_rxlevmin_offset;       // 1..8 (units of 2 dB), only when present
	bool has_p_max;
	int p_max;                   // dBm, -30..33, only when has_p_max
	int freq_band;               // freqBandIndicator 1..64
	std::vector<sib1_sched> sched;
	bool has_tdd_config;         // both reference cells are FDD; parsed but unused
	int si_window_ms;            // si-WindowLength decoded to milliseconds: 1..40
	int si_value_tag;            // systemInfoValueTag 0..31
};

// Parse a CRC-verified BCCH-DL-SCH transport block (the bytes the PDSCH stage outputs).
// Returns 0 on success or a negative stage code naming the first field that failed -
// worth having because a wrong TB from a future cell should say WHERE the grammar and
// the air disagree, not just "no":
//   -1 message is not c1                    -2 c1 is not systemInformationBlockType1
//   -3 plmn-IdentityList size out of range  -4 first PLMN omits the MCC
//   -5 an MCC/MNC digit is not 0..9         -6 extended SIB-Type too large for UPER small number
//   -7 ran out of bits before systemInfoValueTag
int sib1_head_decode(const uint8_t *tb, size_t nbytes, sib1_head &out);

// Convenience for logging: MCC/MNC as plain integers (262, 1) - the digit count for
// rendering "01" vs "1" stays in sib1_plmn::mnc_digits.
int sib1_plmn_mcc(const sib1_plmn &p);
int sib1_plmn_mnc(const sib1_plmn &p);

// Hardware-free selftest, called from cellsyncd --selftest. Returns failed-check count.
int lte_rrc_selftest(bool verbose);

} // namespace cellsync

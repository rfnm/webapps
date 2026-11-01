// cellscan: hardware-free acceptance gate. The load-bearing check the task named: the two
// on-air BCCH transport blocks from as byte literals -> the exact report row a
// person reads, so the display provably shows what was decoded. Around it: the pure human
// derivations (operator from PLMN, band/EARFCN from frequency, eNB/sector from ECI, including
// the 14040-not-224216 arithmetic rule), the persist round-trip and its rejection of a
// truncated file, and the scan state machine driven by a mock burst source.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cmath>
#include <sys/resource.h>   // ru_maxrss: the peak-allocation measurement is a measurement

#include "bandplan.h"
#include "cellfind.h"
#include "decode_carrier.h"
#include "job.h"
#include "plmn.h"
#include "report.h"
#include "scan.h"

namespace cellscan {

static int g_fails = 0;
static void check(bool ok, const std::string &name, const std::string &detail = "") {
	printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : ": ", detail.c_str());
	if(!ok) {
		g_fails++;
	}
	fflush(stdout);
}
static bool contains(const std::string &hay, const std::string &needle) {
	return hay.find(needle) != std::string::npos;
}
static std::vector<uint8_t> hexbytes(const std::string &h) {
	std::vector<uint8_t> b;
	for(size_t i = 0; i + 1 < h.size(); i += 2) {
		b.push_back((uint8_t)strtol(h.substr(i, 2).c_str(), nullptr, 16));
	}
	return b;
}

// ---- pure derivations --------------------------------------------------------------------
static void test_plmn() {
	check(operator_name("262", "01") == "Telekom", "operator 262-01 Telekom");
	check(operator_name("262", "02") == "Vodafone", "operator 262-02 Vodafone");
	check(operator_name("262", "03") == "Telefonica/O2", "operator 262-03 Telefonica");
	check(operator_name("262", "07") == "Telefonica/O2", "operator 262-07 Telefonica");
	check(operator_name("262", "23") == "1&1", "operator 262-23 1&1");
	check(operator_name("262", "42") == "PLMN 262-42", "operator unknown MNC -> bare PLMN");
	check(operator_name("310", "260") == "PLMN 310-260", "operator foreign MCC -> bare PLMN");
}

static void test_band_earfcn() {
	check(band_for_freq_mhz(816.0) == 20, "band 816 MHz -> B20");
	check(band_for_freq_mhz(796.0) == 20, "band 796 MHz -> B20 (not B28, table order)");
	check(band_for_freq_mhz(942.0) == 8, "band 942 MHz -> B8");
	check(band_for_freq_mhz(1815.0) == 3, "band 1815 MHz -> B3");
	check(band_for_freq_mhz(2140.0) == 1, "band 2140 MHz -> B1");
	check(band_for_freq_mhz(2650.0) == 7, "band 2650 MHz -> B7");
	check(band_for_freq_mhz(500.0) == 0, "band 500 MHz -> none");
	check(earfcn_for_freq_mhz(816.0) == 6400, "EARFCN 816 MHz -> 6400",
	      std::to_string(earfcn_for_freq_mhz(816.0)));
	check(earfcn_for_freq_mhz(796.0) == 6200, "EARFCN 796 MHz -> 6200",
	      std::to_string(earfcn_for_freq_mhz(796.0)));
	check(bw_mhz_for_nrb(50) == 10.0, "bw 50 PRB -> 10 MHz");
	check(bw_mhz_for_nrb(100) == 20.0, "bw 100 PRB -> 20 MHz");
	check(bw_mhz_for_nrb(125) == 0.0, "bw impossible N_RB -> 0 (MIB coincidence)");
}

static void test_eci() {
	// : eNB = ECI>>8, sector = ECI&0xFF; the tell for the arithmetic bug is that
	// 0x036D801 is eNB 14040, NOT 224216 (which would be 0x36BD8 and has zero DB records).
	check(eci_enb(0x036D801) == 14040, "ECI 0x036D801 -> eNB 14040",
	      std::to_string(eci_enb(0x036D801)));
	check(eci_enb(0x036D801) != 224216, "ECI 0x036D801 -> NOT eNB 224216 (the pinned bug)");
	check(eci_sector(0x036D801) == 1, "ECI 0x036D801 -> sector 1");
	check(eci_enb(0x196B106) == 104113, "ECI 0x196B106 -> eNB 104113",
	      std::to_string(eci_enb(0x196B106)));
	check(eci_sector(0x196B106) == 6, "ECI 0x196B106 -> sector 6");
}

// ---- the two on-air transport blocks -> the exact report row -------------------------------
static cell_result build_cell(const std::string &tb_hex, int pci, int nid2, double freq_mhz) {
	cell_result c;
	c.pci = pci;
	c.nid1 = pci / 3;
	c.nid2 = nid2;
	c.mib_ok = true;
	c.ports = 2;
	c.nrb = 50;
	c.sfn = 0;
	derive_cell(c);   // bandwidth from N_RB
	std::vector<uint8_t> tb = hexbytes(tb_hex);
	int rc = fill_cell_from_tb(c, tb.data(), tb.size());
	check(rc == 0, "SIB1 head decode rc=0 (PCI " + std::to_string(pci) + ")", std::to_string(rc));
	c.status = c.sib1_ok ? "decoded" : "mib_only";
	// wrap into a carrier and emit exactly as /status would
	(void)freq_mhz;
	return c;
}

// ---- the SIB1 soft-bit accumulation policy -----------------------------------------------
// One synthetic SIB1 occasion of a grant: rate-match, hit with AWGN, hand the LLRs to the
// accumulator the way decode_cell does. `d` is the turbo output of the block, held by the
// caller so every occasion of a trial carries the SAME transport block - which is the whole
// premise being tested.
// `sfn` is the radio frame the occasion sat in, since the accumulator is bounded by the SI
// modification period as well as by the grant. `keep`, when given, receives this occasion's
// own LLRs, so a test can drive the accumulated-then-solo ladder decode_cell runs.
static void accum_occasion(sib1_soft &acc, const cellsync::dci_si &s,
                           const std::vector<uint8_t> d[3], size_t E, int rv, int sfn,
                           float sigma, std::mt19937 &rng, std::vector<float> *keep = nullptr) {
	std::normal_distribution<float> gauss(0.0f, 1.0f);
	std::vector<uint8_t> rm;
	cellsync::turbo_rate_match(d, (size_t)s.tbs + 24, E, rv, rm);
	std::vector<float> llr(rm.size());
	for(size_t i = 0; i < rm.size(); i++) {
		llr[i] = (rm[i] ? -1.0f : 1.0f) + sigma * gauss(rng);
	}
	cellsync::dci_si o = s;
	o.rv = rv;
	sib1_soft_accum(acc, o, sfn, llr);
	if(keep) {
		*keep = llr;
	}
}

static bool accum_decodes(const sib1_soft &acc, const std::vector<uint8_t> &tb) {
	std::vector<uint8_t> dec;
	cellsync::turbo_decode(acc.soft, acc.K, 8, dec);
	return cellsync::crc_check(dec.data(), dec.size(), cellsync::CRC24A) &&
	       std::equal(tb.begin(), tb.end(), dec.begin());
}

static void test_sib1_accum() {
	// The 796 MHz grant, which is the one that never closed on air: RB 0+4 at i_tbs 7 is
	// TBS 224 over E = 960, rate 0.26 - roughly 4 dB tighter than the 816 MHz cell's
	// RB 0+10 at i_tbs 4 (TBS 208 over E = 2320, rate 0.10). Below the crossover a single
	// occasion is hopeless, so what decides the decode is how many occasions combine.
	cellsync::dci_si s796{};
	s796.format = 0x1A;
	s796.localized = true;
	s796.rb_start = 0;
	s796.n_prb = 4;
	s796.i_tbs = 7;
	s796.n_prb_1a = 2;
	s796.tbs = 224;
	const size_t E = 960;
	const float sigma = 1.6f;
	static const int RVCYC[4] = { 0, 2, 3, 1 };   // 36.321 BCCH cycle over one 80 ms period
	const int trials = 24;

	// The RV cycle repeats identically every 80 ms, so occasion j of one period carries the
	// SAME redundancy version as occasion j of the next. Two periods that each offered only
	// ONE occasion is exactly the live PCI 439 case - and it has to decode better than one
	// occasion alone, or the soft bits of the first period were thrown away at the boundary.
	int one = 0, two_same_rv = 0, four_cycle = 0;
	for(int t = 0; t < trials; t++) {
		std::mt19937 rng((unsigned)(20260819 + t));
		std::vector<uint8_t> tb(s796.tbs);
		for(auto &b : tb) {
			b = (uint8_t)(rng() & 1);
		}
		std::vector<uint8_t> blk = tb;
		cellsync::crc_attach(blk, cellsync::CRC24A);
		std::vector<uint8_t> d[3];
		cellsync::turbo_encode(blk, d);

		sib1_soft a1;
		accum_occasion(a1, s796, d, E, RVCYC[0], 0, sigma, rng);
		one += accum_decodes(a1, tb) ? 1 : 0;

		sib1_soft a2;   // period n and period n+1, one occasion each, same RV
		for(int p = 0; p < 2; p++) {
			accum_occasion(a2, s796, d, E, RVCYC[0], 8 * p, sigma, rng);
		}
		two_same_rv += (a2.n == 2 && accum_decodes(a2, tb)) ? 1 : 0;

		sib1_soft a4;   // the full cycle, one occasion per 80 ms period (SFN 0, 8, 16, 24)
		for(int j = 0; j < 4; j++) {
			accum_occasion(a4, s796, d, E, RVCYC[j], 8 * j, sigma, rng);
		}
		four_cycle += (a4.n == 4 && accum_decodes(a4, tb)) ? 1 : 0;
	}
	check(one * 2 <= trials, "accum: one occasion of the 796 grant is not enough",
	      std::to_string(one) + "/" + std::to_string(trials));
	check(two_same_rv >= one + trials / 4,
	      "accum: the SAME RV from two 80 ms periods combines (repetition gain)",
	      std::to_string(one) + "/" + std::to_string(trials) + " -> " +
	      std::to_string(two_same_rv) + "/" + std::to_string(trials));
	check(four_cycle * 4 >= trials * 3, "accum: the full RV cycle closes the 796 grant",
	      std::to_string(four_cycle) + "/" + std::to_string(trials));

	// A changed grant is a VISIBLE content change, and it must clear rather than add. The 816
	// grant is RB 0+10 at i_tbs 4, which is TBS 208 only at n_prb_1a 3 (36.213 Table
	// 7.1.7.2.1-1) - the same tuple `dci_1a_pinned` parses off the air.
	cellsync::dci_si s816 = s796;
	s816.n_prb = 10;
	s816.i_tbs = 4;
	s816.n_prb_1a = 3;
	s816.tbs = 208;
	std::mt19937 rng(4242);
	std::vector<uint8_t> blk(s796.tbs + 24, 0);
	std::vector<uint8_t> d[3];
	cellsync::turbo_encode(blk, d);
	sib1_soft acc;
	accum_occasion(acc, s796, d, E, 0, 0, 0.5f, rng);
	accum_occasion(acc, s796, d, E, 2, 2, 0.5f, rng);
	check(acc.n == 2 && acc.K == 248 && acc.soft.size() == 3 * (248 + 4),
	      "accum: same grant keeps adding", std::to_string(acc.n));
	std::vector<uint8_t> blk8(s816.tbs + 24, 0);
	std::vector<uint8_t> d8[3];
	cellsync::turbo_encode(blk8, d8);
	accum_occasion(acc, s816, d8, 2320, 0, 4, 0.5f, rng);
	check(acc.n == 1 && acc.K == 232 && acc.soft.size() == 3 * (232 + 4),
	      "accum: a different grant clears the buffer", std::to_string(acc.n));

	// ...but the grant is NOT the only content observable: SIB1 content can change at an SI
	// modification-period boundary (36.331 §5.2.1.3) with the grant untouched, and nothing on
	// air announces it. So the buffer is also keyed on sfn/64 - the SHORTEST legal period,
	// modificationPeriodCoeff * defaultPagingCycle = 64 frames - which no cell can undercut.
	sib1_soft mp;
	accum_occasion(mp, s796, d, E, 0, 56, 0.5f, rng);
	accum_occasion(mp, s796, d, E, 2, 62, 0.5f, rng);
	check(mp.n == 2 && mp.mod_period == 0,
	      "accum: occasions inside one modification period keep adding", std::to_string(mp.n));
	accum_occasion(mp, s796, d, E, 3, 64, 0.5f, rng);
	check(mp.n == 1 && mp.mod_period == 1,
	      "accum: the modification-period boundary (sfn/64) clears the buffer",
	      std::to_string(mp.n));
	// The cap is conservative, not tight: SIB1 repeats in SF5 of every even SFN, so 64 frames
	// still holds 8 full 80 ms periods - 8x what the old sfn/8 window could ever gather.
	sib1_soft wide;
	for(int p = 0; p < 8; p++) {
		accum_occasion(wide, s796, d, E, RVCYC[p & 3], 8 * p, 0.5f, rng);
	}
	check(wide.n == 8, "accum: 64 frames still admit 8 SIB1 periods", std::to_string(wide.n));

	// ---- the two-attempt ladder, and what rv_combined means -------------------------------
	// The real 796 transport block, so a decode has to satisfy the ASN.1 head too.
	std::vector<uint8_t> real = hexbytes("684988072af8036d80181b54c4408206115860a92540601814905580");
	std::vector<uint8_t> good(8 * real.size());
	for(size_t i = 0; i < real.size(); i++) {
		for(int j = 0; j < 8; j++) {
			good[8 * i + j] = (uint8_t)((real[i] >> (7 - j)) & 1);
		}
	}
	std::vector<uint8_t> stale(good.size());   // a DIFFERENT block under an identical grant
	for(auto &b : stale) {
		b = (uint8_t)(rng() & 1);
	}
	std::vector<uint8_t> gblk = good, sblk = stale;
	cellsync::crc_attach(gblk, cellsync::CRC24A);
	cellsync::crc_attach(sblk, cellsync::CRC24A);
	std::vector<uint8_t> dg[3], ds[3];
	cellsync::turbo_encode(gblk, dg);
	cellsync::turbo_encode(sblk, ds);
	auto blank_cell = []() {
		cell_result c;
		c.pci = 377;
		c.nid1 = 377 / 3;
		c.nid2 = 2;
		c.mib_ok = true;
		c.ports = 2;
		c.nrb = 50;
		return c;
	};

	// Poison the sum the way an unobserved content change would - two occasions of the old
	// content, then the first of the new - and run the ladder: the accumulated attempt is
	// dominated by the stale block, and only the current occasion alone carries the burst.
	sib1_soft poisoned;
	std::vector<float> llr_now;
	accum_occasion(poisoned, s796, ds, E, 2, 0, 0.2f, rng);
	accum_occasion(poisoned, s796, ds, E, 3, 2, 0.2f, rng);
	accum_occasion(poisoned, s796, dg, E, 0, 4, 0.2f, rng, &llr_now);
	cell_result pc = blank_cell();
	check(poisoned.n == 3 &&
	          sib1_try_decode(poisoned.soft, poisoned.K, (size_t)s796.tbs, pc) != 0,
	      "ladder: a poisoned sum does not decode");
	cellsync::dci_si now = s796;
	now.rv = 0;
	bool combined = true;
	int rc_solo = sib1_decode_occasion(poisoned, now, llr_now, pc, &combined);
	check(rc_solo == 0 && pc.sib1_ok && pc.eci == 0x036D801u && !combined,
	      "ladder: the current occasion alone still decodes (the burst self-heals)",
	      std::to_string(rc_solo));

	// rv_combined means what its name says: >1 DISTINCT redundancy version, not >1 occasion.
	// Both grants' single occasions already carry every mother-code bit (796: K_w 768, 756
	// non-dummy, against E 960), so a repeated RV combines exactly as well - but it is
	// repetition, and the flag must stay clear for it.
	auto ladder_flag = [&](int rv_a, int rv_b) {
		sib1_soft a;
		std::vector<float> last;
		accum_occasion(a, s796, dg, E, rv_a, 0, 0.35f, rng);
		accum_occasion(a, s796, dg, E, rv_b, 8, 0.35f, rng, &last);
		cell_result c = blank_cell();
		cellsync::dci_si o = s796;
		o.rv = rv_b;
		bool f = false;
		return sib1_decode_occasion(a, o, last, c, &f) == 0 ? (f ? 1 : 0) : -1;
	};
	const int f_same = ladder_flag(0, 0), f_cross = ladder_flag(0, 2);
	check(f_same == 0, "rv_combined: the SAME RV twice is repetition, flag clear",
	      std::to_string(f_same));
	check(f_cross == 1, "rv_combined: two DISTINCT RVs set the flag", std::to_string(f_cross));
}

static void test_reference_blocks() {
	// 796 MHz, PCI 377, 2 ports, Telefonica (262-03), TAC 0x2AF8, ECI 0x036D801
	{
		cell_result c = build_cell("684988072af8036d80181b54c4408206115860a92540601814905580",
		                           377, 2, 796.0);
		check(c.sib1_ok, "796: SIB1 decoded");
		check(c.plmns.size() == 1 && c.plmns[0].mcc == "262" && c.plmns[0].mnc == "03",
		      "796: PLMN 262-03");
		check(c.op == "Telefonica/O2", "796: operator Telefonica/O2", c.op);
		check(c.tac == 0x2AF8, "796: TAC 0x2AF8 (11000)", std::to_string(c.tac));
		check(c.eci == 0x036D801u, "796: ECI 0x036D801");
		check(c.enb == 14040 && c.sector == 1, "796: eNB 14040 / sector 1",
		      std::to_string(c.enb) + "/" + std::to_string(c.sector));
		check(c.band_ind == 20, "796: freqBandIndicator 20", std::to_string(c.band_ind));
		check(c.si_window_ms == 20, "796: si-WindowLength 20 ms", std::to_string(c.si_window_ms));
		check(!c.sib16, "796: SIB16 not scheduled");

		carrier_result cr;
		cr.freq_mhz = 796.0;
		cr.band = band_for_freq_mhz(796.0);
		cr.earfcn = earfcn_for_freq_mhz(796.0);
		cr.cells.push_back(c);
		scan_doc d;
		d.carriers.push_back(cr);
		std::string j = emit_scan(d);
		check(contains(j, "\"operator\":\"Telefonica/O2\""), "796: JSON operator row");
		check(contains(j, "\"plmn\":\"262-03\""), "796: JSON plmn 262-03");
		check(contains(j, "\"tac_hex\":\"2AF8\""), "796: JSON tac_hex 2AF8");
		check(contains(j, "\"eci_hex\":\"036D801\""), "796: JSON eci_hex 036D801");
		check(contains(j, "\"enb\":14040"), "796: JSON enb 14040");
		check(contains(j, "\"sector\":1"), "796: JSON sector 1");
		check(contains(j, "\"band\":20") && contains(j, "\"band_ind\":20"),
		      "796: JSON band (freq-derived) and band_ind (SIB1) both 20");
	}
	// 816 MHz, PCI 274, 4 ports, Telekom (262-01), TAC 0x11FA, ECI 0x196B106,
	// q-RxLevMin -64 dBm, si-WindowLength 40 ms, notBarred
	{
		cell_result c;
		c.pci = 274;
		c.nid1 = 91;
		c.nid2 = 1;
		c.mib_ok = true;
		c.ports = 4;
		c.nrb = 50;
		derive_cell(c);
		std::vector<uint8_t> tb = hexbytes("4849880311fa196b1068193090810458182a4950180605241560");
		int rc = fill_cell_from_tb(c, tb.data(), tb.size());
		check(rc == 0, "816: SIB1 head decode rc=0", std::to_string(rc));
		c.status = c.sib1_ok ? "decoded" : "mib_only";
		check(c.plmns.size() == 1 && c.plmns[0].mnc == "01", "816: PLMN 262-01");
		check(c.op == "Telekom", "816: operator Telekom", c.op);
		check(c.tac == 0x11FA, "816: TAC 0x11FA (4602)", std::to_string(c.tac));
		check(c.eci == 0x196B106u, "816: ECI 0x196B106");
		check(c.enb == 104113 && c.sector == 6, "816: eNB 104113 / sector 6",
		      std::to_string(c.enb) + "/" + std::to_string(c.sector));
		check(c.band_ind == 20, "816: freqBandIndicator 20");
		check(c.q_rxlevmin == -64, "816: q-RxLevMin field value -64",
		      std::to_string(c.q_rxlevmin));
		check(c.q_rxlevmin_dbm == -128, "816: q-RxLevMin -128 dBm (2 dB units)",
		      std::to_string(c.q_rxlevmin_dbm));
		check(c.si_window_ms == 40, "816: si-WindowLength 40 ms", std::to_string(c.si_window_ms));
		check(!c.cell_barred, "816: cellBarred notBarred");
		check(!c.sib16, "816: SIB16 not scheduled");

		carrier_result cr;
		cr.freq_mhz = 816.0;
		cr.band = band_for_freq_mhz(816.0);
		cr.earfcn = earfcn_for_freq_mhz(816.0);
		cr.cells.push_back(c);
		scan_doc d;
		d.carriers.push_back(cr);
		std::string j = emit_scan(d);
		check(contains(j, "\"operator\":\"Telekom\""), "816: JSON operator Telekom");
		check(contains(j, "\"eci_hex\":\"196B106\""), "816: JSON eci_hex 196B106");
		check(contains(j, "\"enb\":104113"), "816: JSON enb 104113");
		check(contains(j, "\"sector\":6"), "816: JSON sector 6");
		check(contains(j, "\"q_rxlevmin\":-64"), "816: JSON q_rxlevmin -64");
		check(contains(j, "\"q_rxlevmin_dbm\":-128"), "816: JSON q_rxlevmin_dbm -128");
		check(contains(j, "\"ports\":4"), "816: JSON ports 4");
		check(contains(j, "\"tac_hex\":\"11FA\""), "816: JSON tac_hex 11FA");
	}
}

// ---- the SIB1 failure ladder: a mib_only cell must say WHERE it died ------------------------
static void test_sib1_diag() {
	cell_result c;
	c.pci = 274;
	c.mib_ok = true;
	c.ports = 2;
	c.nrb = 50;
	c.status = "mib_only";
	derive_cell(c);
	c.diag.pcfich_fail = 1;
	c.diag.dci_rejected = 3;
	c.diag.crc_fail = 2;
	c.diag.asn1_fail = 1;
	c.diag.last_asn1_rc = -4;
	c.diag.dcis.push_back({ 0x1A, false, 0, 8, 5, 0 });   // distributed 1A: the v1 gate rejects it
	c.diag.dcis.push_back({ 0x1C, false, 4, 8, 3, 0 });
	carrier_result cr;
	cr.freq_mhz = 816.0;
	cr.status = "mib_only";
	cr.cells.push_back(c);
	scan_doc d;
	d.carriers.push_back(cr);
	std::string j = emit_scan(d);
	check(contains(j, "\"sib1_diag\":{"), "diag: mib_only cell carries sib1_diag");
	check(contains(j, "\"pcfich_fail\":1") && contains(j, "\"dci_rejected\":3") &&
	      contains(j, "\"crc_fail\":2") && contains(j, "\"asn1_fail\":1"),
	      "diag: JSON counters are the ladder as counted");
	check(contains(j, "\"last_asn1_rc\":-4"), "diag: sib1_head_decode rc is kept, not discarded");
	check(contains(j, "\"format\":\"1A\"") && contains(j, "\"format\":\"1C\""),
	      "diag: DCI formats emitted as hex strings 1A/1C");
	check(contains(j, "\"localized\":false") && contains(j, "\"rb_start\":4") &&
	      contains(j, "\"n_prb\":8") && contains(j, "\"mcs\":3"),
	      "diag: the grant that was on air is emitted verbatim");

	// a decoded cell has nothing to explain
	cell_result ok = build_cell("4849880311fa196b1068193090810458182a4950180605241560", 274, 1, 816.0);
	carrier_result cr2;
	cr2.freq_mhz = 816.0;
	cr2.cells.push_back(ok);
	scan_doc d2;
	d2.carriers.push_back(cr2);
	check(!contains(emit_scan(d2), "sib1_diag"), "diag: a decoded cell emits no sib1_diag");

	// and the diagnostics survive the persist round-trip
	const char *path = "./cellscan_selftest_diag.tmp";
	scan_doc back;
	int64_t saved = 0;
	if(persist_save(path, d) && persist_load(path, back, saved) && !back.carriers.empty() &&
	   !back.carriers[0].cells.empty()) {
		const sib1_diag &bd = back.carriers[0].cells[0].diag;
		check(bd.dci_rejected == 3 && bd.last_asn1_rc == -4 && bd.dcis.size() == 2 &&
		      bd.dcis[0].format == 0x1A && bd.dcis[1].n_prb == 8,
		      "diag: persist round-trip keeps the ladder and the grants");
	} else {
		check(false, "diag: persist round-trip keeps the ladder and the grants");
	}
	remove(path);
}

// ---- the PBCH hypothesis grid: one h, and it decides BOTH quarters and grouping -----------
// The naive port of the SIB1 accumulation - slide a window, keep summing - produces a MIB in
// roughly half the bursts of which up to half carry the WRONG SFN: a group straddling a 40 ms
// boundary descrambles perfectly while the payloads underneath differ in the SFN MSBs, the
// disagreeing bits cancel toward zero, the convolutional code fills them back in and CRC-16
// passes. Nothing downstream notices, and the wrong SFN then picks the wrong SIB1 subframes
// on a cell that decodes fine today. So the coupling checked below is the load-bearing part
// of this whole change, and wb_mib_decode_n refuses a straddling group outright.
static void test_mib_hypothesis() {
	// 1. no group ever spans a TTI, and every group is exactly the consecutive quarters of
	//    one payload, at every hypothesis and every window length
	{
		bool bounded = true, complete = true;
		for(int n = 1; n <= 12; n++) {
			std::vector<int> fr;
			for(int f = 0; f < n; f++) {
				fr.push_back(f);
			}
			for(int h = 0; h < 4; h++) {
				std::vector<std::pair<size_t, size_t>> runs;
				mib_group_runs(fr, h, runs);
				size_t seen = 0;
				for(const auto &r : runs) {
					const int q0 = mib_quarter(h, fr[r.first]);
					const int nf = (int)(r.second - r.first);
					bounded = bounded && q0 + nf <= 4;
					for(size_t i = r.first; i < r.second; i++) {
						bounded = bounded &&
						          mib_quarter(h, fr[i]) == q0 + (int)(i - r.first) &&
						          mib_tti(h, fr[i]) == mib_tti(h, fr[r.first]);
					}
					complete = complete && r.first == seen;
					seen = r.second;
				}
				complete = complete && seen == fr.size();
			}
		}
		check(bounded, "mib: no group ever spans a 40 ms TTI boundary");
		check(complete, "mib: the groups cover the window exactly once");
	}

	// 2. a hole (a frame that would not demodulate) splits the group instead of shifting
	//    every quarter behind it by one
	{
		std::vector<int> fr = { 0, 1, 3, 4, 5 };
		std::vector<std::pair<size_t, size_t>> runs;
		mib_group_runs(fr, 0, runs);
		check(runs.size() == 3 && runs[0] == std::make_pair((size_t)0, (size_t)2) &&
		      runs[1] == std::make_pair((size_t)2, (size_t)3) &&
		      runs[2] == std::make_pair((size_t)3, (size_t)5),
		      "mib: a missing frame splits the group, it does not shift it",
		      std::to_string(runs.size()) + " groups");
	}

	// 3. the recovered SFN of the window's first frame is EXACTLY right, whatever offset the
	//    window starts at and whichever group happens to verify. This is the off-by-one that
	//    would kill SIB1 silently: decode_cell selects subframes with 'sf % 10 != 5 ||
	//    (sfn & 1)', so being four frames out is a coin flip on every window.
	{
		bool exact = true;
		int cases = 0;
		for(int sfn_start : { 0, 1, 2, 3, 7, 421, 1021, 1023 }) {
			const int h = sfn_start & 3;   // the window's first frame carries this quarter
			for(int f0 = 0; f0 < 40; f0++) {
				const int sfn_f0 = (sfn_start + f0) % 1024;
				exact = exact && mib_quarter(h, f0) == (sfn_f0 & 3);
				exact = exact && mib_sfn0(sfn_f0, f0) == sfn_start;
				cases++;
			}
		}
		check(exact, "mib: the window's SFN is exact at every start offset",
		      std::to_string(cases) + " cases");
	}

	// 4. and the row says why, for a cell that never got a MIB at all
	{
		cell_result c;
		c.pci = 439;
		c.status = "pss_only";
		c.mdiag.frames = 200;
		c.mdiag.demod_fail = 2;
		c.mdiag.crc_fail = 597;
		c.mdiag.bw_reject = 3;
		c.mdiag.win_short = 1;
		c.mdiag.chest_fail = 4;
		c.mdiag.re_budget = 5;
		c.mdiag.path_metric = 0.8123;
		carrier_result cr;
		cr.freq_mhz = 796.0;
		cr.status = "pss_only";
		cr.cells.push_back(c);
		scan_doc d;
		d.carriers.push_back(cr);
		const std::string j = emit_scan(d);
		check(contains(j, "\"mib_diag\":{"), "mib: a pss_only cell carries mib_diag");
		check(contains(j, "\"frames\":200") && contains(j, "\"crc_fail\":597") &&
		      contains(j, "\"demod_fail\":2") && contains(j, "\"bw_reject\":3") &&
		      contains(j, "\"win_short\":1") && contains(j, "\"chest_fail\":4") &&
		      contains(j, "\"re_budget\":5"),
		      "mib: the ladder is emitted as counted");
		check(contains(j, "\"path_metric\":0.8123"),
		      "mib: the Viterbi metric is kept, not discarded");
		check(!contains(j, "\"sib1_diag\":"), "mib: a cell without a MIB emits no SIB1 ladder");

		// and it survives the persist round-trip, like the SIB1 ladder above
		const char *path = "./cellscan_selftest_mib.tmp";
		scan_doc back;
		int64_t saved = 0;
		if(persist_save(path, d) && persist_load(path, back, saved) && !back.carriers.empty() &&
		   !back.carriers[0].cells.empty()) {
			// ALL eight fields, not the five that used to be checked here: a rung nobody
			// asserts on is a rung that can be dropped from the emitter or the loader
			// without a test noticing, and the three untested ones were the three the
			// counting bugs were in
			const mib_diag &bm = back.carriers[0].cells[0].mdiag;
			check(bm.frames == 200 && bm.crc_fail == 597 && bm.bw_reject == 3 &&
			      bm.demod_fail == 2 && bm.win_short == 1 && bm.chest_fail == 4 &&
			      bm.re_budget == 5 && std::fabs(bm.path_metric - 0.8123) < 1e-6,
			      "mib: persist round-trip keeps the ladder");
		} else {
			check(false, "mib: persist round-trip keeps the ladder");
		}
		remove(path);

		cell_result m = c;
		m.mib_ok = true;
		m.nrb = 50;
		m.status = "mib_only";
		carrier_result cr2;
		cr2.freq_mhz = 796.0;
		cr2.cells.push_back(m);
		scan_doc d2;
		d2.carriers.push_back(cr2);
		check(!contains(emit_scan(d2), "mib_diag"), "mib: a cell with a MIB emits no mib_diag");
	}
}

// ---- the MIB window: how much of the capture is looked at, and what the ladder then says ---
// Two things are pinned here that no test covered. The window GEOMETRY: a deep dwell is four
// times the burst but the window is bounded, so with a single window six of the eight seconds
// an escalated pss_only cell paid for were never examined - while the escalation exists to
// give that cell "a second independent window of TTIs". And the DIAGNOSTIC: win_short used to
// be bumped once for the frame loop reaching the end of the burst, which every finite burst
// does, and once more for the second half hypothesis starting before the buffer, which is
// normal whenever the PSS sits in the first frame - so it read >= 1 on essentially every cell
// and separated nothing. `frames` was assigned inside the half-hypothesis loop while every
// other counter accumulated, so it reported only the last half's observations.
static void test_mib_window() {
	// 1. the windows tile the dwell: consecutive, non-overlapping, and together the whole
	//    capture - 200 frames of a 2 s burst, 800 of the 8 s one
	{
		bool tiled = true;
		for(int wi = 1; wi < MIB_DEEP_WINDOWS; wi++) {
			tiled = tiled && mib_win_begin(wi) == mib_win_end(wi - 1) &&
			        mib_win_end(wi) > mib_win_begin(wi);
		}
		check(tiled && mib_win_begin(0) == 0 && mib_windows(false) == 1 &&
		      mib_windows(false) * MIB_MAX_FRAMES == 200 &&
		      mib_windows(true) * MIB_MAX_FRAMES == 800,
		      "mib: the windows tile the dwell, 2 s in one and 8 s in four",
		      std::to_string(mib_windows(true) * MIB_MAX_FRAMES) + " frames deep");
	}

	// 2. the ladder, off a real (noise) buffer through the production decode_cell. A burst
	//    that holds whole 40 ms TTIs is NOT a short window; one that cannot hold four frames
	//    is, and that is the only thing win_short may mean.
	{
		wb_num g = cellsync::wb_num_for(50);
		fft_plan plan((unsigned)g.fft);
		found_cell fc;
		fc.pci = 274;
		fc.nid1 = 91;
		fc.nid2 = 1;
		fc.half = 0;
		// both half hypotheses inside the buffer, so the second one is exercised and neither
		// starts before its start - the b < 0 that used to count as a short window
		fc.pss_pos = 5 * g.subframe + g.sym_off(6);

		auto noise = [](size_t n) {
			std::vector<cf> x(n);
			uint32_t s = 12345;
			for(size_t i = 0; i < n; i++) {
				s = s * 1664525u + 1013904223u;
				const float a = (float)(((s >> 8) & 0xFFFF) / 65536.0 - 0.5);
				s = s * 1664525u + 1013904223u;
				const float b = (float)(((s >> 8) & 0xFFFF) / 65536.0 - 0.5);
				x[i] = cf(a, b);
			}
			return x;
		};

		std::vector<cf> lng = noise((size_t)(12 * g.frame));
		cell_result cl;
		std::atomic<bool> cancel(false);
		decode_cell(lng, fc, g, plan, false, cl, &cancel);
		check(cl.mdiag.win_short == 0,
		      "mib: a window holding whole TTIs is not reported short",
		      std::to_string(cl.mdiag.win_short));
		check(cl.mdiag.frames > 12,
		      "mib: frames counts both half hypotheses, it is not overwritten",
		      std::to_string(cl.mdiag.frames));

		std::vector<cf> shrt = noise((size_t)(3 * g.frame));
		cell_result cs;
		decode_cell(shrt, fc, g, plan, false, cs, &cancel);
		check(cs.mdiag.win_short > 0, "mib: a burst too short for one TTI IS reported short",
		      std::to_string(cs.mdiag.win_short));
		check(!cl.mib_ok && !cs.mib_ok, "mib: noise yields no MIB on either window");
	}
}

// A cell as the classifier sees it: what it decoded, and how far above the detection floors
// (cellfind.h) the acquisition that produced it actually sat. A cell with a MIB carries the
// quality numbers too - they are simply not what decides its class.
static cell_result mk_cell(bool mib, bool sib1, int nrb, double metric, double margin) {
	cell_result c;
	c.mib_ok = mib;
	c.sib1_ok = sib1;
	c.nrb = nrb;
	c.pss_metric = metric;
	c.sss_margin = margin;
	c.status = sib1 ? "decoded" : mib ? "mib_only" : "pss_only";
	return c;
}
// a detection sitting exactly ON both detection floors: reportable, and nothing more
static cell_result bare_cell() {
	return mk_cell(false, false, 0, PSS_DETECT_METRIC, (double)SSS_DETECT_MARGIN);
}
// a detection clear of both floors: no MIB yet, but worth a second look
static cell_result strong_cell() {
	return mk_cell(false, false, 0, 0.40, 2.20);
}

// ---- the 2 s -> 8 s escalation predicate ---------------------------------------------------
// It was inline in scanner::run, unreachable from --selftest, next to a comment claiming the
// path "has its own coverage". It decides how much radio time - and, since one deep capture is
// a 983 MB allocation on a 3.27 GB board, how much RAM - a sweep spends.
static void test_escalation_gate() {
	// a cell that finished never escalates, whatever its width
	check(!cell_needs_deep(mk_cell(true, true, 50, 0.4, 2.0)), "escalate: a decoded cell does not");
	check(!cell_needs_deep(mk_cell(true, true, 100, 0.4, 2.0)),
	      "escalate: a decoded wide cell does not");
	// MIB-bearing behaviour is EXACTLY what it was before the budget existed: a 50 PRB cell
	// whose SIB1 never closed escalates, on the strength of the MIB and nothing else - even
	// when the acquisition that found it was itself bare-threshold
	check(cell_needs_deep(mk_cell(true, false, 50, 0.4, 2.0)), "escalate: mib_only at 50 PRB does");
	check(cell_needs_deep(mk_cell(true, false, 50, PSS_DETECT_METRIC, SSS_DETECT_MARGIN)),
	      "escalate: a VERIFIED MIB escalates on the MIB, not on its PSS metric");
	check(cell_deep_class(mk_cell(true, false, 50, 0.4, 2.0)) == DEEP_MIB,
	      "escalate: a MIB whose SIB1 is open is the strongest class");
	// and the one that must not: a MIB saying the carrier is wider than this numerology can
	// take is a finished answer, not a stuck one
	check(!cell_needs_deep(mk_cell(true, false, 100, 0.4, 2.0)),
	      "escalate: a too-wide cell is finished, not stuck");

	// (c) a cell with NO MIB has only the two acquisition numbers to offer, and at the bare
	// detection floors those are what a phantom PCI looks like: reportable, not worth 983 MB
	check(!cell_needs_deep(bare_cell()),
	      "escalate: a bare-threshold pss_only cell does NOT earn a 983 MB capture");
	check(cell_deep_class(bare_cell()) == DEEP_WEAK, "escalate: bare threshold is class weak");
	check(cell_needs_deep(strong_cell()),
	      "escalate: a pss_only cell clear of both floors still does - 'not enough energy yet'");
	check(cell_deep_class(strong_cell()) == DEEP_PSS, "escalate: clear of the floors is class pss");
	// both floors, not either: a phantom that fakes one still has to fake the other
	check(cell_deep_class(mk_cell(false, false, 0, 0.40, (double)SSS_DETECT_MARGIN)) == DEEP_WEAK,
	      "escalate: a strong PSS with a floor-level SSS margin is still weak");
	check(cell_deep_class(mk_cell(false, false, 0, PSS_DETECT_METRIC, 2.20)) == DEEP_WEAK,
	      "escalate: a clean SSS on a floor-level PSS peak is still weak");
	// exactly at the escalation thresholds is IN (>=): the boundary is stated once, here
	check(cell_deep_class(mk_cell(false, false, 0, DEEP_PSS_METRIC, DEEP_SSS_MARGIN)) == DEEP_PSS,
	      "escalate: the escalation thresholds are inclusive");
	// and the thresholds really are above the floors, in both signals
	check(DEEP_PSS_METRIC >= 2.0 * PSS_DETECT_METRIC,
	      "escalate: the PSS escalation threshold is >= 2x the detection floor",
	      std::to_string(DEEP_PSS_METRIC) + " vs " + std::to_string(PSS_DETECT_METRIC));
	check(DEEP_SSS_MARGIN > (double)SSS_DETECT_MARGIN,
	      "escalate: the SSS escalation margin is above the detection floor",
	      std::to_string(DEEP_SSS_MARGIN) + " vs " + std::to_string(SSS_DETECT_MARGIN));
	// a cell_result nobody filled in must not escalate by omission
	check(cell_deep_class(cell_result()) == DEEP_WEAK,
	      "escalate: a default cell_result is weak - the fields default BELOW every floor");

	carrier_result empty;
	check(!carrier_needs_deep(empty) && carrier_deep_class(empty) == DEEP_NONE,
	      "escalate: a carrier with no cells never does - empty spectrum stays cheap");
	carrier_result one;
	one.cells.push_back(mk_cell(true, true, 50, 0.4, 2.0));
	check(!carrier_needs_deep(one), "escalate: a fully decoded carrier does not");
	one.cells.push_back(bare_cell());
	check(!carrier_needs_deep(one) && carrier_deep_class(one) == DEEP_WEAK,
	      "escalate: a decoded cell beside a phantom does not escalate for the phantom");
	one.cells.push_back(strong_cell());
	check(carrier_needs_deep(one) && carrier_deep_class(one) == DEEP_PSS,
	      "escalate: one unfinished cell clear of the floors is enough");
	one.cells.push_back(mk_cell(true, false, 50, 0.4, 2.0));
	check(carrier_deep_class(one) == DEEP_MIB,
	      "escalate: a carrier carries the promise of its BEST cell");
}

// ---- (a)+(d) what a capture costs, and that it costs it once -------------------------------
// The two allocations that hung the board: 983 MB asked for with no check that the machine had
// it, and 245 MB still held while it was asked for.
static size_t peak_rss_kb() {
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	return (size_t)ru.ru_maxrss;   // Linux: kB, and a high-water mark - it never walks back
}

static void test_capture_budget() {
	// the arithmetic, spelled out: cf is 8 bytes at 15.36 MSps
	check(sizeof(cf) == 8, "capture: cf is 8 bytes", std::to_string(sizeof(cf)));
	check(capture_samples(2.0) == 30720000u, "capture: a 2 s dwell is 30.72 M samples",
	      std::to_string(capture_samples(2.0)));
	check(capture_bytes(2.0) == 245760000ull, "capture: a 2 s dwell is 245.76 MB",
	      std::to_string(capture_bytes(2.0)));
	check(capture_bytes(8.0) == 983040000ull, "capture: an 8 s deep dwell is 983.04 MB",
	      std::to_string(capture_bytes(8.0)));

	// (d) capture_reset gives the storage BACK before asking for the new one, and asks for
	// exactly what is needed - the old code reserved want + 5% of a second, 6 MB of slack the
	// fill loop could never reach
	{
		std::vector<cf> v;
		capture_reset(v, 1000000);
		check(v.empty() && v.capacity() == 1000000, "capture: reset sizes exactly, no slack",
		      std::to_string(v.capacity()));
		capture_reset(v, 10);
		check(v.empty() && v.capacity() == 10,
		      "capture: reset SHRINKS - the previous buffer was released, not kept",
		      std::to_string(v.capacity()));
	}

	// (a) the budget. MemAvailable is overridden so the verdict is arithmetic, not weather.
	{
		uint64_t need = 0, avail = 0;
		set_mem_available_override(4ull << 30);
		check(mem_available_bytes() == (4ull << 30), "budget: the override is what is read");
		check(capture_fits(8.0, &need, &avail) && need == 983040000ull,
		      "budget: 983 MB fits in half of 4 GiB");
		// the exact edge: fits iff need <= CAPTURE_MEM_FRACTION * MemAvailable
		set_mem_available_override(1966080000ull);   // 2 x 983.04 MB
		check(capture_fits(8.0, nullptr, nullptr), "budget: 2x the need is exactly enough");
		set_mem_available_override(1966079999ull);
		check(!capture_fits(8.0, nullptr, nullptr), "budget: one byte less is refused");
		// the shape that hung the board: ~1.3 GB already in use of the 3.13 GB idle figure,
		// so the deep capture would have taken more than half of what was left
		set_mem_available_override(1800ull << 20);
		check(!capture_fits(8.0, nullptr, nullptr),
		      "budget: the 8 s capture is refused with 1.8 GB available");
		check(capture_fits(2.0, nullptr, nullptr),
		      "budget: the 2 s capture still runs there - the scan is not stopped, only bounded");
		// an idle board: the feature is NOT weakened
		set_mem_available_override(3130ull << 20);   // ~3.13 GB, measured at idle
		check(capture_fits(8.0, nullptr, nullptr),
		      "budget: on an idle board the deep capture is still allowed");
		set_mem_available_override(0);
		check(mem_available_bytes() > 0,
		      "budget: MemAvailable is really readable from /proc/meminfo",
		      std::to_string(mem_available_bytes() >> 20) + " MB");
	}

	// (d) measured, not asserted: ru_maxrss is a high-water mark, so a transient second buffer
	// cannot hide from it. First the real path, then a deliberate double-buffer as the control
	// - without the control this test could pass by simply not being sensitive enough.
	{
		const size_t N = 4u << 20;   // 4 Mi samples = 32 MiB, far above any RSS noise
		std::vector<cf> v;
		capture_reset(v, N);
		v.resize(N, cf(1, 1));   // touch every page: RSS counts what is resident
		const size_t p1 = peak_rss_kb();
		capture_reset(v, N);
		v.resize(N, cf(2, 2));
		const size_t grew = peak_rss_kb() - p1;
		check(grew < 8u * 1024u,
		      "capture: re-sizing a held buffer does NOT add a second one to the peak",
		      std::to_string(grew) + " kB, one buffer is " + std::to_string(N * sizeof(cf) / 1024) + " kB");
		std::vector<cf> keep;
		keep.assign(N, cf(3, 3));
		const size_t p2 = peak_rss_kb();
		std::vector<cf> second;
		second.assign(N, cf(4, 4));   // the old shape: both resident at once
		const size_t ctl = peak_rss_kb() - p2;
		check(ctl > 24u * 1024u,
		      "capture: the control DOES show a second buffer - the measurement can see it",
		      std::to_string(ctl) + " kB");
	}

	// (d) on a real source: the file replay serves a shorter dwell out of a vector that holds
	// a longer one, which is the realloc the board could not afford.
	{
		const char *path = "./cellscan_selftest_burst.tmp";
		FILE *f = fopen(path, "wb");
		check(f != nullptr, "capture: temp burst file opened");
		if(f) {
			std::vector<cf> junk(200000, cf(0.5f, -0.5f));
			fwrite(junk.data(), sizeof(cf), junk.size(), f);
			fclose(f);
			file_burst_source fs(path, false);
			std::atomic<bool> cancel{ false };
			std::vector<cf> out;
			check(fs.capture(0.01, out, cancel, nullptr), "capture: file replay serves 10 ms");
			const size_t big = out.capacity();
			check(out.size() == capture_samples(0.01) && big == out.size(),
			      "capture: 10 ms of file is 153600 samples, sized exactly",
			      std::to_string(out.size()));
			check(fs.capture(0.001, out, cancel, nullptr), "capture: file replay serves 1 ms");
			check(out.size() == capture_samples(0.001) && out.capacity() == out.size() &&
			              out.capacity() < big,
			      "capture: the longer buffer was RELEASED before the shorter one was sized",
			      std::to_string(out.capacity()) + " after " + std::to_string(big));
			remove(path);
		}
	}
}

// ---- (b)+(c) the per-job escalation budget, walked over a full 16-carrier sweep -------------
static void test_deep_budget() {
	set_mem_available_override(8ull << 30);   // this test is about the caps, not the memory
	// 16 phantom carriers - the measured case: a PSS/SSS detection on nearly every carrier,
	// none of them with a MIB. They may spend the sub-cap and not one escalation more.
	{
		deep_budget b;
		int adm = 0;
		for(int i = 0; i < SWEEP_MAX_DECODES; i++) {
			if(!b.admit(DEEP_PSS, 8.0)) {
				adm++;
			}
		}
		check(adm == DEEP_MAX_PSS_PER_JOB,
		      "budget: 16 pss_only carriers spend the pss sub-cap, not the whole budget",
		      std::to_string(adm));
		check(b.used == DEEP_MAX_PSS_PER_JOB && b.used_pss == DEEP_MAX_PSS_PER_JOB,
		      "budget: and both counters agree");
	}
	// 16 MIB-bearing carriers: the full cap, and nothing beyond it
	{
		deep_budget b;
		int adm = 0;
		for(int i = 0; i < SWEEP_MAX_DECODES; i++) {
			if(!b.admit(DEEP_MIB, 8.0)) {
				adm++;
			}
		}
		check(adm == DEEP_MAX_PER_JOB, "budget: 16 MIB carriers spend the job cap exactly",
		      std::to_string(adm));
		check(std::string(b.admit(DEEP_MIB, 8.0)) == "job_cap",
		      "budget: the 17th is refused as job_cap");
	}
	// the ordering property: phantoms first must NOT leave a real cell with nothing. Eight
	// phantoms, then four cells holding a verified MIB - the sweep order a false positive on
	// a strong carrier produces.
	{
		deep_budget b;
		int ph = 0, mib = 0;
		for(int i = 0; i < 8; i++) {
			if(!b.admit(DEEP_PSS, 8.0)) {
				ph++;
			}
		}
		for(int i = 0; i < 4; i++) {
			if(!b.admit(DEEP_MIB, 8.0)) {
				mib++;
			}
		}
		check(ph == DEEP_MAX_PSS_PER_JOB && mib == DEEP_MAX_PER_JOB - DEEP_MAX_PSS_PER_JOB,
		      "budget: phantoms cannot starve a MIB-bearing carrier later in the sweep",
		      std::to_string(ph) + " pss + " + std::to_string(mib) + " mib");
	}
	// (c) at the budget's own door: a weak carrier is refused before any counter moves
	{
		deep_budget b;
		check(std::string(b.admit(DEEP_WEAK, 8.0)) == "weak", "budget: a weak carrier is refused");
		check(b.used == 0 && b.used_pss == 0, "budget: and a refusal costs nothing");
	}
	// (a) inside the budget: no headroom, no escalation - and no counter spent either, so the
	// budget is still there when memory comes back
	{
		deep_budget b;
		set_mem_available_override(1ull << 30);   // 1 GiB: half of it is not 983 MB
		check(std::string(b.admit(DEEP_MIB, 8.0)) == "memory",
		      "budget: a MIB carrier is refused when the 983 MB does not fit");
		check(b.used == 0, "budget: a memory refusal does not consume the job cap");
		set_mem_available_override(8ull << 30);
		check(b.admit(DEEP_MIB, 8.0) == nullptr,
		      "budget: with headroom back, the same carrier is admitted");
	}
	set_mem_available_override(0);
}

// ---- the sweep band plan: plausibility of every row ----------------------------------------
static void test_bandplan() {
	const auto &t = sweep_band_table();
	check(t.size() == 5, "sweep: 5 bands in the plan", std::to_string(t.size()));
	size_t total = 0;
	std::vector<double> all;
	for(const auto &r : t) {
		std::vector<sweep_probe> c = sweep_candidates({ r.band });
		check(!c.empty(), std::string("sweep: ") + r.key + " has candidates");
		bool in_band = true, in_range = true, spaced = true;
		double prev = 0;
		for(size_t i = 0; i < c.size(); i++) {
			// every candidate must land in ITS band per the 36.101 table (plmn.h) - the same
			// map the report uses, so probe band and reported band can never disagree
			in_band = in_band && band_for_freq_mhz(c[i].freq_mhz) == r.band && c[i].band == r.band;
			in_range = in_range && c[i].freq_mhz >= r.first_mhz - 1e-9 &&
			           c[i].freq_mhz <= r.last_mhz + 1e-9;
			if(i > 0) {
				spaced = spaced && std::fabs((c[i].freq_mhz - prev) - r.step_mhz) < 1e-9;
			}
			prev = c[i].freq_mhz;
			all.push_back(c[i].freq_mhz);
		}
		check(in_band, std::string("sweep: ") + r.key + " candidates inside the band");
		check(in_range, std::string("sweep: ") + r.key + " candidates inside first..last");
		check(spaced, std::string("sweep: ") + r.key + " uniform step");
		total += c.size();
	}
	std::sort(all.begin(), all.end());
	bool dup = false;
	for(size_t i = 1; i < all.size(); i++) {
		dup = dup || std::fabs(all[i] - all[i - 1]) < 1e-9;
	}
	check(!dup, "sweep: no duplicate candidate across the whole plan");
	check(sweep_candidates({}).size() == total, "sweep: empty band list = whole plan",
	      std::to_string(sweep_candidates({}).size()));
	// pin the count: editing the table changes the sweep budget and must be a conscious act
	check(total == 47, "sweep: 47 candidates in the full plan", std::to_string(total));
	// the three validated B20 block centres (cells were found on all three - )
	std::vector<sweep_probe> b20 = sweep_candidates({ 20 });
	check(b20.size() == 3 && b20[0].freq_mhz == 796.0 && b20[1].freq_mhz == 806.0 &&
	      b20[2].freq_mhz == 816.0, "sweep: B20 = the validated 796/806/816 block centres");
	check(sweep_band_by_key("b7") && sweep_band_by_key("b7")->band == 7, "sweep: key b7 -> band 7");
	check(sweep_band_by_key("b13") == nullptr, "sweep: unknown key -> null");
	check(sweep_band_by_id(99) == nullptr, "sweep: unknown band id -> null");
}

// ---- the stage-2 selection against a synthetic level list ----------------------------------
static void test_sweep_select() {
	auto probes_for = [](int band, const std::vector<double> &levels) {
		std::vector<sweep_probe> ps = sweep_candidates({ band });
		for(size_t i = 0; i < ps.size() && i < levels.size(); i++) {
			ps[i].level_db = levels[i];
		}
		return ps;
	};
	// B1 (5 MHz grid, 11 candidates): a 10 MHz carrier lights neighbours -> only maxima decode
	{
		std::vector<sweep_probe> ps =
		        probes_for(1, { 3, 8, 20, 12, 3, 9, 9, 3, 30, 2, 5.9 });
		std::vector<size_t> sel = sweep_select(ps, 6.0, 16);
		check(sel.size() == 3, "select: grid band -> 3 maxima", std::to_string(sel.size()));
		check(sel.size() == 3 && sel[0] == 8 && sel[1] == 2 && sel[2] == 5,
		      "select: strongest first (30, 20, 9)");
		// the shoulders of the 20 dB carrier (8 and 12 dB neighbours) must NOT decode
		bool shoulders = false;
		for(size_t s : sel) {
			shoulders = shoulders || s == 1 || s == 3;
		}
		check(!shoulders, "select: carrier shoulders suppressed on the grid");
		// plateau (9,9) collapses to its first probe; 5.9 dB stays under the 6 dB threshold
		bool has6 = false, has10 = false;
		for(size_t s : sel) {
			has6 = has6 || s == 6;
			has10 = has10 || s == 10;
		}
		check(!has6, "select: plateau collapses to one probe");
		check(!has10, "select: below threshold never decodes");
	}
	// B20 (10 MHz blocks): every block is its own carrier - a strong neighbour block must
	// never shadow a weaker one (the real measurement found cells on all three blocks)
	{
		std::vector<sweep_probe> ps = probes_for(20, { 20, 15, 25 });
		std::vector<size_t> sel = sweep_select(ps, 6.0, 16);
		check(sel.size() == 3, "select: B20 blocks never suppress each other",
		      std::to_string(sel.size()));
		check(sel.size() == 3 && sel[0] == 2 && sel[1] == 0 && sel[2] == 1,
		      "select: B20 order 816, 796, 806 by level");
	}
	// cap: only the strongest max_picks survive
	{
		std::vector<sweep_probe> ps = probes_for(20, { 20, 15, 25 });
		std::vector<size_t> sel = sweep_select(ps, 6.0, 2);
		check(sel.size() == 2 && sel[0] == 2 && sel[1] == 0, "select: cap keeps the strongest");
	}
	// a failed capture is not evidence: ok=false never decodes, however loud it claims to be
	{
		std::vector<sweep_probe> ps = probes_for(20, { 40, 3, 3 });
		ps[0].ok = false;
		std::vector<size_t> sel = sweep_select(ps, 6.0, 16);
		check(sel.empty(), "select: failed capture never picked", std::to_string(sel.size()));
	}
}

// ---- the antenna policy: the RF path is a property of the frequency, not of the job --------
// How many times a decode list changes the RF path under a policy - the number `antenna_group`
// exists to hold at 1, because every change costs the settle discard of a retune.
static int ant_switches(const std::vector<double> &freqs, const antenna_policy &ap) {
	int n = 0;
	for(size_t i = 1; i < freqs.size(); i++) {
		n += ap.at(freqs[i]) != ap.at(freqs[i - 1]) ? 1 : 0;
	}
	return n;
}

static void test_antenna_policy() {
	antenna_policy ap;
	check(ap.crossover_mhz == ANTENNA_CROSSOVER_MHZ && ap.low == "embed" && ap.high == "sma_a",
	      "antenna: the shipped policy is embed below 1000 MHz, sma_a above");
	// the measured pairs (README "Operating discipline", one bench board, gain 12, 8 s deep dwell):
	// SIB1 came out of B20 only on embed and out of B1 only on sma_a, so the policy has to
	// reproduce the winning side of each - one antenna cannot serve 796 MHz and 2145 MHz
	check(ap.at(796.0) == "embed", "antenna: 796 MHz (SIB1 decoded on embed) -> embed", ap.at(796.0));
	check(ap.at(816.0) == "embed", "antenna: 816 MHz (SIB1 decoded on embed) -> embed", ap.at(816.0));
	check(ap.at(2145.0) == "sma_a", "antenna: 2145 MHz (SIB1 decoded on sma_a) -> sma_a",
	      ap.at(2145.0));
	check(ap.at(2665.0) == "sma_a", "antenna: 2665 MHz (B7) -> sma_a", ap.at(2665.0));
	// every band of the plan is entirely one antenna's: the crossover must not split a band,
	// or a sweep would switch paths inside it and the grouping could never hold at one change
	bool split = false;
	for(const auto &r : sweep_band_table()) {
		std::vector<sweep_probe> c = sweep_candidates({ r.band });
		for(const auto &pr : c) {
			split = split || ap.at(pr.freq_mhz) != ap.at(c.front().freq_mhz);
		}
		const std::string want = (r.band == 20 || r.band == 8) ? "embed" : "sma_a";
		check(ap.at(c.front().freq_mhz) == want, std::string("antenna: ") + r.key + " -> " + want,
		      ap.at(c.front().freq_mhz));
	}
	check(!split, "antenna: no band in the plan straddles the crossover");
	// an explicit choice pins the whole job - the crossover is not consulted at all
	antenna_policy pin;
	pin.pin = "sma_a";
	check(pin.at(796.0) == "sma_a" && pin.at(2145.0) == "sma_a",
	      "antenna: --antenna sma_a pins every carrier (the pre-policy behaviour)", pin.at(796.0));
	antenna_policy pin_low;
	pin_low.pin = "embed";
	check(pin_low.at(2145.0) == "embed", "antenna: --antenna embed pins every carrier",
	      pin_low.at(2145.0));
	// both names and the crossover are policy, not hard-wired
	antenna_policy alt;
	alt.low = "sma_b";
	alt.high = "sma_c";
	alt.crossover_mhz = 2000.0;
	check(alt.at(1815.0) == "sma_b" && alt.at(2145.0) == "sma_c",
	      "antenna: names and crossover are configurable", alt.at(1815.0) + "/" + alt.at(2145.0));

	// grouping: an interleaved list costs one change, and the order inside a group survives
	std::vector<double> mixed = { 2145.0, 796.0, 2665.0, 816.0 };
	check(ant_switches(mixed, ap) == 3, "antenna: the ungrouped list would switch 3 times",
	      std::to_string(ant_switches(mixed, ap)));
	antenna_group(mixed, ap);
	check(mixed.size() == 4 && mixed[0] == 2145.0 && mixed[1] == 2665.0 && mixed[2] == 796.0 &&
	              mixed[3] == 816.0,
	      "antenna: grouping keeps the strongest carrier first, and its group with it");
	check(ant_switches(mixed, ap) == 1, "antenna: a grouped list switches exactly once",
	      std::to_string(ant_switches(mixed, ap)));
	std::vector<double> one_band = { 816.0, 796.0 };
	antenna_group(one_band, ap);
	check(ant_switches(one_band, ap) == 0 && one_band[0] == 816.0,
	      "antenna: a single-band list never switches and is not reordered");
	std::vector<double> pinned = { 2145.0, 796.0, 2665.0 };
	antenna_group(pinned, pin);
	check(pinned[0] == 2145.0 && pinned[1] == 796.0 && pinned[2] == 2665.0,
	      "antenna: a pinned job is never reordered");
	std::vector<double> none;
	antenna_group(none, ap);   // must not walk off the front of an empty list
	check(none.empty(), "antenna: grouping an empty list is a no-op");
}

// ---- the stage-1 metric against synthetic spectra (incl. THE DC trap) ----------------------
static void test_probe_metric() {
	using cellsync::fft_plan;
	const double fs = 15.36e6;
	fft_plan plan(1024);
	uint32_t seed = 1;
	auto frnd = [&]() {
		seed = seed * 1664525u + 1013904223u;
		return (float)(((seed >> 8) & 0xFFFF) / 65536.0 - 0.5);
	};
	auto noise_buf = [&](size_t n, float amp) {
		std::vector<cf> x(n);
		for(auto &v : x) {
			v = cf(amp * frnd(), amp * frnd());
		}
		return x;
	};
	const size_t N = 32768;
	// silence and white noise are both "nothing here": ~0 dB
	{
		std::vector<cf> z(N, cf(0, 0));
		double l = sweep_probe_level(z, plan, fs);
		check(std::fabs(l) < 0.5, "probe: silence -> 0 dB", std::to_string(l));
	}
	{
		std::vector<cf> x = noise_buf(N, 0.1f);
		double l = sweep_probe_level(x, plan, fs);
		check(std::fabs(l) < 1.5, "probe: white noise -> ~0 dB", std::to_string(l));
	}
	// THE trap: a huge DC spike over a flat floor must NOT read as a carrier
	{
		std::vector<cf> x = noise_buf(N, 0.01f);
		for(auto &v : x) {
			v += cf(10.0f, 0.0f);   // 60 dB of DC over the noise
		}
		double l = sweep_probe_level(x, plan, fs);
		check(std::fabs(l) < 1.5, "probe: DC spike excluded from the metric (the trap)",
		      std::to_string(l));
	}
	// a real in-band occupant reads strongly above threshold
	{
		std::vector<cf> x = noise_buf(N, 0.01f);
		const int bins[4] = { 67, 134, 1024 - 67, 1024 - 134 };   // +-1.005 / +-2.01 MHz
		for(size_t i = 0; i < x.size(); i++) {
			for(int b : bins) {
				double a = 2.0 * M_PI * b * (double)(i % 1024) / 1024.0;
				x[i] += cf((float)(0.5 * cos(a)), (float)(0.5 * sin(a)));
			}
		}
		double l = sweep_probe_level(x, plan, fs);
		check(l > 10.0, "probe: in-band tones -> well above threshold", std::to_string(l));
	}
	// an out-of-band blocker raises the floor, never the in-band verdict
	{
		std::vector<cf> x = noise_buf(N, 0.01f);
		for(size_t i = 0; i < x.size(); i++) {
			double a = 2.0 * M_PI * 420 * (double)(i % 1024) / 1024.0;   // +6.3 MHz
			x[i] += cf((float)(0.5 * cos(a)), (float)(0.5 * sin(a)));
		}
		double l = sweep_probe_level(x, plan, fs);
		check(l < 1.5, "probe: out-of-band blocker never reads as a carrier", std::to_string(l));
	}
}

// ---- persistence -------------------------------------------------------------------------
static void test_persist() {
	const char *path = "./cellscan_selftest_persist.tmp";
	scan_doc d;
	d.job = 7;
	d.state = "done";
	d.freqs_mhz = { 796.0, 816.0 };
	d.dwell_s = 2;
	d.cells_found = 1;
	d.sib1_decoded = 1;
	carrier_result cr;
	cr.freq_mhz = 816.0;
	cr.band = 20;
	cr.earfcn = 6400;
	cr.status = "decoded";
	cell_result c;
	c.pci = 274;
	c.nid1 = 91;
	c.nid2 = 1;
	c.mib_ok = true;
	c.ports = 4;
	c.nrb = 50;
	c.sib1_ok = true;
	c.op = "Telekom";
	c.tac = 0x11FA;
	c.eci = 0x196B106;
	c.enb = 104113;
	c.sector = 6;
	c.band_ind = 20;
	c.si_window_ms = 40;
	c.status = "decoded";
	c.plmns.push_back({ "262", "01", "262-01", "Telekom", false });
	c.sibs = { 2, 3 };
	cr.cells.push_back(c);
	d.carriers.push_back(cr);

	check(persist_save(path, d), "persist save");
	scan_doc back;
	int64_t saved = 0;
	bool ok = persist_load(path, back, saved);
	check(ok, "persist load");
	check(ok && back.job == 7 && back.state == "done", "persist job/state round-trip");
	check(ok && back.carriers.size() == 1 && back.carriers[0].cells.size() == 1,
	      "persist carrier/cell round-trip");
	if(ok && back.carriers.size() == 1 && back.carriers[0].cells.size() == 1) {
		const cell_result &bc = back.carriers[0].cells[0];
		check(bc.pci == 274 && bc.enb == 104113 && bc.sector == 6 && bc.tac == 0x11FA,
		      "persist cell fields round-trip");
		check(bc.plmns.size() == 1 && bc.plmns[0].plmn == "262-01" && bc.op == "Telekom",
		      "persist PLMN/operator round-trip");
	}

	// truncated file must be rejected (jparse requires the whole document to close)
	FILE *f = fopen(path, "rb");
	std::string body;
	if(f) {
		char buf[4096];
		size_t n;
		while((n = fread(buf, 1, sizeof(buf), f)) > 0) {
			body.append(buf, n);
		}
		fclose(f);
	}
	const char *tpath = "./cellscan_selftest_trunc.tmp";
	FILE *tf = fopen(tpath, "wb");
	if(tf) {
		fwrite(body.data(), 1, body.size() / 2, tf);   // half the bytes
		fclose(tf);
	}
	scan_doc junk;
	int64_t js = 0;
	check(!persist_load(tpath, junk, js), "persist rejects truncated file");
	check(!persist_load("./cellscan_selftest_missing.tmp", junk, js), "persist missing file -> false");
	remove(path);
	remove(tpath);
}

// ---- scan state machine with a mock burst source ------------------------------------------
// The mock returns an empty burst (no PSS) so every carrier resolves to no_signal without any
// DSP - enough to exercise the state machine: happy path, per-carrier progression, cancel and
// the persisted-result round trip.
class mock_source : public burst_source {
public:
	explicit mock_source(bool block) : block_(block) {}
	bool open(double, int, bool, const std::string &) override { return true; }
	bool retune(double, const std::string &) override { return true; }
	bool capture(double, std::vector<cf> &out, std::atomic<bool> &cancel, uint64_t *gap) override {
		if(gap) {
			*gap = 0;
		}
		if(block_) {
			for(int i = 0; i < 200 && !cancel.load(); i++) {
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
			return false;   // cancelled mid-capture
		}
		// a captured-but-empty-of-cells burst: a real, non-empty buffer too short to hold a
		// PSS train, so cellfind returns no cells -> no_signal (empty() would read as a capture
		// failure, which is a different, correct code path).
		out.assign(4096, cf(0, 0));
		return true;
	}

private:
	bool block_;
};

// A source that is LOUD in band on every frequency: the probe metric reads far above threshold,
// so a sweep picks the full SWEEP_MAX_DECODES and stage 2 really runs. That is the shape that
// exposed the progress bar walking backwards on the first board sweep, so the regression test
// needs it - the quiet mock_source above picks nothing and never crosses the stage boundary.
class loud_source : public burst_source {
public:
	bool open(double, int, bool, const std::string &) override { return true; }
	bool retune(double, const std::string &) override { return true; }
	bool capture(double, std::vector<cf> &out, std::atomic<bool> &, uint64_t *gap) override {
		if(gap) {
			*gap = 0;
		}
		// noise floor + an in-band tone at ~2 MHz: above threshold, but no PSS -> no_signal
		out.resize(32768);
		uint32_t s = 7;
		for(size_t i = 0; i < out.size(); i++) {
			s = s * 1664525u + 1013904223u;
			float n = (float)(((s >> 8) & 0xFFFF) / 65536.0 - 0.5) * 0.01f;
			double a = 2.0 * M_PI * 134 * (double)(i % 1024) / 1024.0;
			out[i] = cf(n + (float)(0.5 * cos(a)), n + (float)(0.5 * sin(a)));
		}
		return true;
	}
};

static void test_state_machine() {
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<mock_source>(false);
		});
		job_params p;
		p.freqs_mhz = { 796.0, 806.0, 816.0 };
		std::string e;
		uint64_t id = sc.start(p, e);
		check(id == 1, "state: first job id is 1", e);
		// a second start while running is refused (409-shaped: id 0, err set)
		std::string e2;
		uint64_t id2 = sc.start(p, e2);
		check(id2 == 0 && !e2.empty(), "state: concurrent start refused", e2);
		for(int i = 0; i < 200 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		scan_doc d = sc.snapshot();
		check(d.state == "done", "state: happy path ends done", d.state);
		check(d.carriers.size() == 3, "state: all 3 carriers visited",
		      std::to_string(d.carriers.size()));
		bool all_ns = true;
		for(const auto &c : d.carriers) {
			all_ns = all_ns && c.status == "no_signal";
		}
		check(all_ns, "state: empty burst -> no_signal per carrier");
		check(d.progress.pct == 100 && d.progress.phase == "done", "state: progress completes");
	}
	// validation: out-of-range frequency and empty list are rejected before any capture
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<mock_source>(false);
		});
		job_params p;
		p.freqs_mhz = { 100.0 };
		std::string e;
		check(sc.start(p, e) == 0 && !e.empty(), "state: 100 MHz rejected (400)", e);
		job_params p2;
		std::string e2;
		check(sc.start(p2, e2) == 0, "state: empty freq list rejected");
		job_params p3;
		p3.freqs_mhz = { 796.0 };
		p3.dwell_s = 5;
		std::string e3;
		check(sc.start(p3, e3) == 0 && !e3.empty(), "state: dwell 5 rejected", e3);
	}
	// cancel: a blocking capture is aborted and the job ends cancelled
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<mock_source>(true);
		});
		job_params p;
		p.freqs_mhz = { 796.0 };
		std::string e;
		uint64_t id = sc.start(p, e);
		check(id != 0, "state: cancel-test job started");
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
		sc.cancel();
		for(int i = 0; i < 400 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		scan_doc d = sc.snapshot();
		check(!sc.running(), "state: cancel stops the worker");
		check(d.state == "cancelled", "state: cancelled end state", d.state);
	}
	// persistence across a fresh scanner: a saved scan reloads with provenance
	{
		const char *pp = "./cellscan_selftest_sm.tmp";
		{
			scanner sc(pp);
			sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
				return std::make_unique<mock_source>(false);
			});
			job_params p;
			p.freqs_mhz = { 796.0, 816.0 };
			std::string e;
			sc.start(p, e);
			for(int i = 0; i < 300 && sc.running(); i++) {
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}
		scanner sc2(pp);
		bool loaded = sc2.load_persist();
		check(loaded && sc2.persist_loaded(), "state: persisted scan reloads");
		scan_doc d = sc2.snapshot();
		check(d.carriers.size() == 2, "state: reloaded carriers preserved",
		      std::to_string(d.carriers.size()));
		remove(pp);
	}
}

// ---- the sweep as a job: empty air ends done with every probe measured and zero decodes ----
static void test_sweep_state_machine() {
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<mock_source>(false);
		});
		job_params p;
		p.sweep = true;   // no bands = the whole plan
		std::string e;
		uint64_t id = sc.start(p, e);
		check(id != 0, "sweep: job starts without a frequency list", e);
		for(int i = 0; i < 400 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		scan_doc d = sc.snapshot();
		check(d.state == "done", "sweep: empty air ends done", d.state);
		check(d.mode == "sweep", "sweep: document mode is sweep", d.mode);
		check(d.probes.size() == sweep_candidates({}).size(),
		      "sweep: every candidate probed", std::to_string(d.probes.size()));
		bool all_quiet = !d.probes.empty();
		for(const auto &pr : d.probes) {
			all_quiet = all_quiet && pr.ok && !pr.above && !pr.picked &&
			            std::fabs(pr.level_db) < 1.5;
		}
		check(all_quiet, "sweep: empty bands honestly ~0 dB, nothing above threshold");
		check(d.carriers.empty() && d.freqs_mhz.empty(),
		      "sweep: nothing above threshold -> zero decodes");
		check(d.progress.pct == 100 && d.progress.phase == "done", "sweep: progress completes");
		check(d.sweep_thresh_db == SWEEP_THRESH_DB, "sweep: threshold recorded in the doc");
		// the /status JSON carries the probes (the panel's bar view feeds on this)
		std::string j = emit_scan(d);
		check(contains(j, "\"mode\":\"sweep\"") && contains(j, "\"probes\":["),
		      "sweep: JSON carries mode and probes");
		// restricting to one band probes only that band
		job_params p2;
		p2.sweep = true;
		p2.bands = { 20 };
		std::string e2;
		check(sc.start(p2, e2) != 0, "sweep: band-restricted job starts", e2);
		for(int i = 0; i < 400 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		scan_doc d2 = sc.snapshot();
		check(d2.probes.size() == 3, "sweep: bands=[b20] probes exactly 3 block centres",
		      std::to_string(d2.probes.size()));
	}
	// REGRESSION (first board sweep): a loud sky picks a full decode list, and the progress bar
	// must not walk backwards when stage 1 hands over to stage 2. Stage 1 is budgeted against
	// SWEEP_MAX_DECODES, so its share can only be too pessimistic, never too optimistic.
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<loud_source>();
		});
		job_params p;
		p.sweep = true;
		p.deep = false;   // keep the test quick; the escalation predicate is pinned on its
		                  // own in test_escalation_gate (it was not, when this said so)
		std::string e;
		check(sc.start(p, e) != 0, "sweep: loud-sky job starts", e);
		int last = 0, worst = 0, samples = 0;
		std::string last_phase;
		while(sc.running()) {
			scan_doc s = sc.snapshot();
			if(s.progress.pct < last) {
				worst = std::min(worst, s.progress.pct - last);
			}
			last = s.progress.pct;
			last_phase = s.progress.phase;
			samples++;
		}
		scan_doc d = sc.snapshot();
		check(samples > 0 && worst == 0, "sweep: progress never walks backwards across stages",
		      std::to_string(worst));
		// A sky that is equally loud everywhere is a plateau on every grid: the 5 MHz bands
		// (B8/B3/B1/B7) each collapse to their first probe, while B20's 10 MHz blocks are
		// independent carriers and all three survive -> 3 + 1 + 1 + 1 + 1. The suppression is
		// what keeps a wall of energy from spending the whole decode budget on one band.
		check(d.freqs_mhz.size() == 7, "sweep: a uniform loud sky collapses to 7 picks (3x B20 "
		      "+ one per 5 MHz grid band)", std::to_string(d.freqs_mhz.size()));
		check(d.carriers.size() == d.freqs_mhz.size(), "sweep: every picked carrier decoded",
		      std::to_string(d.carriers.size()));
		check(d.progress.pct == 100 && d.state == "done", "sweep: loud sky ends done at 100%",
		      d.state);
		// the picks are the strongest first, and every one of them was above the threshold
		bool ordered = true, all_above = true;
		double prev = 1e9;
		for(const auto &pr : d.probes) {
			if(pr.picked) {
				all_above = all_above && pr.level_db >= d.sweep_thresh_db;
			}
		}
		for(double f : d.freqs_mhz) {
			for(const auto &pr : d.probes) {
				if(pr.freq_mhz == f) {
					ordered = ordered && pr.level_db <= prev + 1e-9;
					prev = pr.level_db;
				}
			}
		}
		check(all_above, "sweep: every picked probe is above the threshold");
		check(ordered, "sweep: stage-2 list is ordered strongest first");
	}
	// an unknown band id is rejected before any capture
	{
		scanner sc("");
		sc.set_source_factory([]() -> std::unique_ptr<burst_source> {
			return std::make_unique<mock_source>(false);
		});
		job_params p;
		p.sweep = true;
		p.bands = { 99 };
		std::string e;
		check(sc.start(p, e) == 0 && !e.empty(), "sweep: unknown band rejected (400)", e);
	}
}

// ---- the policy on the capture path --------------------------------------------------------
// A source that RECORDS what the job asked for - (frequency, antenna) per open/retune. The
// policy is only worth something if it reaches the capture path, and the grouping is only worth
// something if the job pays for at most one path change.
class ant_log_source : public mock_source {
public:
	explicit ant_log_source(std::vector<std::pair<double, std::string>> *log)
	    : mock_source(false), log_(log) {}
	bool open(double hz, int gain, bool agc, const std::string &ant) override {
		log_->push_back({ hz / 1e6, ant });
		return mock_source::open(hz, gain, agc, ant);
	}
	bool retune(double hz, const std::string &ant) override {
		log_->push_back({ hz / 1e6, ant });
		return mock_source::retune(hz, ant);
	}

private:
	std::vector<std::pair<double, std::string>> *log_;
};

static void test_antenna_job() {
	const antenna_policy ap;
	auto run_job = [](std::vector<std::pair<double, std::string>> &log, const job_params &p) {
		scanner sc("");
		sc.set_source_factory([&log]() -> std::unique_ptr<burst_source> {
			return std::make_unique<ant_log_source>(&log);
		});
		std::string e;
		sc.start(p, e);
		for(int i = 0; i < 600 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		return sc.snapshot();
	};
	auto switches = [](const std::vector<std::pair<double, std::string>> &log) {
		int n = 0;
		for(size_t i = 1; i < log.size(); i++) {
			n += log[i].second != log[i - 1].second ? 1 : 0;
		}
		return n;
	};

	// a carrier list that interleaves the bands: the job groups it into ONE path change, and
	// every carrier is captured on - and reports - the antenna its frequency asks for
	{
		job_params p;
		p.freqs_mhz = { 2145.0, 796.0, 2665.0, 816.0 };
		std::vector<std::pair<double, std::string>> log;
		scan_doc d = run_job(log, p);
		check(d.state == "done" && d.carriers.size() == 4, "antenna: policy scan ran", d.state);
		check(log.size() == 4, "antenna: one tune per carrier, no reopen",
		      std::to_string(log.size()));
		bool used_policy = log.size() == 4;
		for(const auto &e : log) {
			used_policy = used_policy && e.second == ap.at(e.first);
		}
		check(used_policy, "antenna: every capture used the path its frequency asks for");
		check(switches(log) == 1, "antenna: the interleaved list costs exactly one path change",
		      std::to_string(switches(log)));
		std::vector<double> want = { 2145.0, 2665.0, 796.0, 816.0 };
		check(d.freqs_mhz == want, "antenna: the published list is the grouped one");
		bool rows = d.carriers.size() == 4;
		for(const auto &c : d.carriers) {
			rows = rows && c.antenna == ap.at(c.freq_mhz);
		}
		check(rows, "antenna: every carrier row names the path it was received on");
		check(d.antenna == "auto", "antenna: an unpinned job reports antenna auto", d.antenna);
		std::string j = emit_scan(d);
		check(contains(j, "\"antenna\":\"sma_a\"") && contains(j, "\"antenna\":\"embed\""),
		      "antenna: the JSON carriers carry the path that produced them");
		const char *path = "./cellscan_selftest_ant.tmp";
		scan_doc back;
		int64_t saved = 0;
		check(persist_save(path, d) && persist_load(path, back, saved) &&
		              back.carriers.size() == 4 && back.carriers[0].antenna == "sma_a" &&
		              back.carriers[3].antenna == "embed",
		      "antenna: the per-carrier path survives the persist round-trip");
		remove(path);
	}
	// an explicit --antenna is exactly the pre-policy job: no reordering, no path change, and
	// the pinned name on every row - whatever the crossover would have said
	{
		job_params p;
		p.freqs_mhz = { 2145.0, 796.0, 2665.0, 816.0 };
		p.antenna.pin = "sma_a";
		std::vector<std::pair<double, std::string>> log;
		scan_doc d = run_job(log, p);
		bool all_pinned = log.size() == 4;
		for(const auto &e : log) {
			all_pinned = all_pinned && e.second == "sma_a";
		}
		check(all_pinned, "antenna: an explicit --antenna pins every capture");
		check(switches(log) == 0, "antenna: a pinned job never changes the path",
		      std::to_string(switches(log)));
		check(d.freqs_mhz == p.freqs_mhz, "antenna: a pinned scan keeps the caller's order");
		bool rows = d.carriers.size() == 4;
		for(const auto &c : d.carriers) {
			rows = rows && c.antenna == "sma_a";
		}
		check(rows, "antenna: a pinned job reports the pin on every carrier row");
		check(d.antenna == "sma_a", "antenna: the doc names the pinned path", d.antenna);
	}
	// stage 1 obeys the policy too: probing B20 through the high-band path would attenuate
	// exactly the carriers that earn a decode. The plan runs low->high, so the pass costs one
	// change (b20 = 3 candidates on embed, b1 = 11 on sma_a).
	{
		job_params p;
		p.sweep = true;
		p.bands = { 20, 1 };
		std::vector<std::pair<double, std::string>> log;
		scan_doc d = run_job(log, p);
		check(d.probes.size() == 14, "antenna: sweep b20+b1 probed every candidate",
		      std::to_string(d.probes.size()));
		bool used_policy = log.size() == 14;
		for(const auto &e : log) {
			used_policy = used_policy && e.second == ap.at(e.first);
		}
		check(used_policy, "antenna: every stage-1 probe used its band's path");
		check(switches(log) == 1, "antenna: the probe pass crosses the crossover once",
		      std::to_string(switches(log)));
	}
}

// ---- the escalation on the real job path ----------------------------------------------------
// Every decision the budget makes is made ON a carrier_result, and the only honest way to
// produce one is to receive a cell - a hardware-free test would need a 245 MB synthetic LTE
// burst per case. The seam (scanner::decode_burst) supplies the carrier_result the decoder
// would have produced and leaves everything around it - the classification, the budget, the
// buffer handling, the published document - as the shipped code.
struct cap_call {
	double dwell;
	size_t size_in;   // out.size() as the source received it
	size_t cap_in;    // out.capacity() as the source received it: (d) is decided here
};

class rec_source : public burst_source {
public:
	explicit rec_source(std::vector<cap_call> *log) : log_(log) {}
	bool open(double, int, bool, const std::string &) override { return true; }
	bool retune(double, const std::string &) override { return true; }
	bool capture(double dwell, std::vector<cf> &out, std::atomic<bool> &, uint64_t *gap) override {
		if(gap) {
			*gap = 0;
		}
		log_->push_back(cap_call{ dwell, out.size(), out.capacity() });
		capture_reset(out, 4096);
		out.resize(4096, cf(0, 0));   // a real, non-empty burst; the seam gives it its meaning
		return true;
	}

private:
	std::vector<cap_call> *log_;
};

class seam_scanner : public scanner {
public:
	explicit seam_scanner(std::function<carrier_result(double, int)> m)
	    : scanner(""), make_(std::move(m)) {}

protected:
	carrier_result decode_burst(std::vector<cf> &, double freq_mhz, int dwell,
	                            std::atomic<bool> &) override {
		carrier_result cr = make_(freq_mhz, dwell);
		cr.freq_mhz = freq_mhz;
		cr.dwell_s = dwell;
		return cr;
	}

private:
	std::function<carrier_result(double, int)> make_;
};

static scan_doc run_seam_job(std::function<carrier_result(double, int)> make,
                             std::vector<cap_call> &log, const std::vector<double> &freqs) {
	seam_scanner sc(std::move(make));
	sc.set_source_factory([&log]() -> std::unique_ptr<burst_source> {
		return std::make_unique<rec_source>(&log);
	});
	job_params p;
	p.freqs_mhz = freqs;
	std::string e;
	sc.start(p, e);
	for(int i = 0; i < 1000 && sc.running(); i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return sc.snapshot();
}

static int count_deep(const std::vector<cap_call> &log) {
	int n = 0;
	for(const auto &c : log) {
		if(c.dwell > 2.0) {
			n++;
		}
	}
	return n;
}

static void test_deep_job() {
	// the carrier every case below starts from: a verified MIB at 50 PRB whose SIB1 did not
	// close on the 2 s pass, and which the 8 s pass would have decoded
	auto mib_then_decoded = [](double, int dwell) {
		carrier_result cr;
		if(dwell > 2) {
			cr.status = "decoded";
			cr.cells.push_back(mk_cell(true, true, 50, 0.4, 2.0));
		} else {
			cr.status = "mib_only";
			cr.cells.push_back(mk_cell(true, false, 50, 0.4, 2.0));
		}
		return cr;
	};

	// (a) no headroom: the escalation is refused, the 2 s answer survives, and the row says so
	{
		set_mem_available_override(1ull << 30);   // 1 GiB - half of it is not 983 MB
		std::vector<cap_call> log;
		scan_doc d = run_seam_job(mib_then_decoded, log, { 816.0 });
		check(d.state == "done" && d.carriers.size() == 1,
		      "deep job: the low-memory job still completes", d.state);
		check(count_deep(log) == 0, "deep job: no 8 s capture was ever asked for",
		      std::to_string(log.size()) + " captures");
		check(d.carriers[0].status == "mib_only" && d.carriers[0].dwell_s == 2,
		      "deep job: the 2 s result is what gets reported", d.carriers[0].status);
		check(d.carriers[0].deep_skip == "memory",
		      "deep job: and the row says WHY - not a silent 'it did not decode'",
		      d.carriers[0].deep_skip);
		set_mem_available_override(0);
	}
	// with headroom the same carrier escalates and the deep answer wins: the feature the
	// PBCH TTI combining needs is intact, it is only bounded
	{
		set_mem_available_override(8ull << 30);
		std::vector<cap_call> log;
		scan_doc d = run_seam_job(mib_then_decoded, log, { 816.0 });
		check(count_deep(log) == 1, "deep job: with headroom the escalation happens",
		      std::to_string(count_deep(log)));
		check(d.carriers[0].status == "decoded" && d.carriers[0].dwell_s == 8,
		      "deep job: the 8 s result replaces the 2 s one", d.carriers[0].status);
		check(d.carriers[0].deep_skip.empty(),
		      "deep job: an escalation that happened reports nothing");
		// (d) the deep capture is handed a buffer with NO storage: the 245 MB base burst was
		// released before the 983 MB one was asked for, so the peak is ONE capture (983.04 MB)
		// and not base + deep (1.23 GB).
		bool saw = false, freed = false;
		for(const auto &c : log) {
			if(c.dwell > 2.0) {
				saw = true;
				freed = (c.cap_in == 0 && c.size_in == 0);
			}
		}
		check(saw && freed,
		      "deep job: the 8 s capture is handed an EMPTY, capacity-0 buffer - peak is one");
		set_mem_available_override(0);
	}
	// (b) the cap: 16 carriers - a full sweep - that ALL hold a MIB whose SIB1 is open
	{
		set_mem_available_override(8ull << 30);
		std::vector<double> f;
		for(int i = 0; i < SWEEP_MAX_DECODES; i++) {
			f.push_back(800.0 + 10.0 * i);
		}
		std::vector<cap_call> log;
		scan_doc d = run_seam_job(mib_then_decoded, log, f);
		check((int)d.carriers.size() == SWEEP_MAX_DECODES, "deep job: all 16 carriers visited",
		      std::to_string(d.carriers.size()));
		check(count_deep(log) == DEEP_MAX_PER_JOB,
		      "deep job: a full 16-carrier sweep escalates at most 4 times",
		      std::to_string(count_deep(log)));
		int capped = 0, decoded = 0;
		for(const auto &c : d.carriers) {
			if(c.deep_skip == "job_cap") {
				capped++;
			}
			if(c.status == "decoded") {
				decoded++;
			}
		}
		check(decoded == DEEP_MAX_PER_JOB && capped == SWEEP_MAX_DECODES - DEEP_MAX_PER_JOB,
		      "deep job: the other 12 carriers report job_cap, not a silent miss",
		      std::to_string(decoded) + " decoded, " + std::to_string(capped) + " capped");
		set_mem_available_override(0);
	}
	// (b) and the budget goes to the promising: eight phantom carriers first, then four with a
	// verified MIB. This is the measured shape - a phantom PCI on most carriers of a sweep.
	{
		set_mem_available_override(8ull << 30);
		auto mixed = [](double fmhz, int dwell) {
			carrier_result cr;
			if(fmhz < 880.0) {
				cr.status = "pss_only";   // clear of the floors, but never a MIB
				cr.cells.push_back(strong_cell());
			} else if(dwell > 2) {
				cr.status = "decoded";
				cr.cells.push_back(mk_cell(true, true, 50, 0.4, 2.0));
			} else {
				cr.status = "mib_only";
				cr.cells.push_back(mk_cell(true, false, 50, 0.4, 2.0));
			}
			return cr;
		};
		std::vector<double> f;
		for(int i = 0; i < 12; i++) {
			f.push_back(800.0 + 10.0 * i);   // 800..870 phantom, 880..910 with a MIB
		}
		std::vector<cap_call> log;
		scan_doc d = run_seam_job(mixed, log, f);
		int decoded = 0;
		for(const auto &c : d.carriers) {
			if(c.status == "decoded") {
				decoded++;
			}
		}
		check(count_deep(log) == DEEP_MAX_PER_JOB,
		      "deep job: the mixed sweep still escalates exactly the job cap",
		      std::to_string(count_deep(log)));
		check(decoded == DEEP_MAX_PER_JOB - DEEP_MAX_PSS_PER_JOB,
		      "deep job: eight phantoms first cannot starve the real cells behind them",
		      std::to_string(decoded) + " real cells decoded");
		set_mem_available_override(0);
	}
	// (c) a bare-threshold detection does not earn 983 MB; a MIB-bearing carrier still does
	{
		set_mem_available_override(8ull << 30);
		auto bare = [](double, int) {
			carrier_result cr;
			cr.status = "pss_only";
			cr.cells.push_back(bare_cell());
			return cr;
		};
		std::vector<cap_call> log;
		scan_doc d = run_seam_job(bare, log, { 816.0 });
		check(count_deep(log) == 0,
		      "deep job: a cell sitting on the detection floors gets no 8 s capture");
		check(d.carriers[0].deep_skip == "weak", "deep job: and the row names it weak",
		      d.carriers[0].deep_skip);
		std::vector<cap_call> log2;
		scan_doc d2 = run_seam_job(mib_then_decoded, log2, { 816.0 });
		check(count_deep(log2) == 1 && d2.carriers[0].status == "decoded",
		      "deep job: the same job shape WITH a MIB escalates exactly as before");
		set_mem_available_override(0);
	}
	// and the base capture is budgeted too: a job started at dwell 8 asks for the same 983 MB
	// on every carrier, with no escalation involved
	{
		set_mem_available_override(1ull << 30);
		std::vector<cap_call> log;
		seam_scanner sc(mib_then_decoded);
		sc.set_source_factory([&log]() -> std::unique_ptr<burst_source> {
			return std::make_unique<rec_source>(&log);
		});
		job_params p;
		p.freqs_mhz = { 816.0 };
		p.dwell_s = 8;
		std::string e;
		sc.start(p, e);
		for(int i = 0; i < 1000 && sc.running(); i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		scan_doc d = sc.snapshot();
		check(log.empty(), "deep job: a dwell-8 job never even calls capture without headroom",
		      std::to_string(log.size()));
		check(d.carriers.size() == 1 && d.carriers[0].status == "capture_error" &&
		              d.carriers[0].deep_skip == "memory",
		      "deep job: and the carrier reads capture_error with a reason, never 'no signal'",
		      d.carriers.empty() ? "" : d.carriers[0].status + "/" + d.carriers[0].deep_skip);
		set_mem_available_override(0);
	}
}

int run_selftest(bool verbose) {
	(void)verbose;
	g_fails = 0;
	test_plmn();
	test_band_earfcn();
	test_eci();
	test_bandplan();
	test_sweep_select();
	test_antenna_policy();
	test_probe_metric();
	test_sib1_accum();
	test_reference_blocks();
	test_sib1_diag();
	test_mib_hypothesis();
	test_mib_window();
	test_escalation_gate();
	test_capture_budget();
	test_deep_budget();
	test_deep_job();
	test_persist();
	test_state_machine();
	test_sweep_state_machine();
	test_antenna_job();
	printf(g_fails ? "SELFTEST FAILED (%d)\n" : "SELFTEST OK\n", g_fails);
	return g_fails ? 1 : 0;
}

} // namespace cellscan

// cellscan: the human-readable derivations that turn a decoded SIB1 into the row a person
// wants to read - operator name from the PLMN, band/EARFCN from the frequency, eNB/sector
// from the cellIdentity, bandwidth from N_RB. Header-only, no deps, no state: every function
// is a pure map pinned in --selftest. The operator table and the ECI split are the two the
// task calls out by name; the "compute the band from the frequency AND show the SIB1 band"
// pair is 's standing licence-block sanity check, so both live here.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cellscan {

// ---- operator name from the PLMN (MCC-MNC) ------------------------------------------------
// German MCC 262 is all the reference bench needs by name; the task lists these five. Anything
// else is rendered as its bare PLMN so the row is still truthful ("PLMN 262-42"), never blank.
inline std::string operator_name(const std::string &mcc, const std::string &mnc) {
	// The four German MNOs the task names, and only those - an MNC we cannot name is rendered
	// as its bare PLMN so the row stays truthful ("PLMN 262-42"), never a guessed brand.
	if(mcc == "262") {
		if(mnc == "01") { return "Telekom"; }
		if(mnc == "02") { return "Vodafone"; }
		if(mnc == "03" || mnc == "07") { return "Telefonica/O2"; }
		if(mnc == "23") { return "1&1"; }
	}
	return "PLMN " + mcc + "-" + mnc;
}

// ---- bandwidth (MHz) from the MIB's N_RB (36.104 Table 5.6-1) ----------------------------
inline double bw_mhz_for_nrb(int nrb) {
	switch(nrb) {
		case 6:   return 1.4;
		case 15:  return 3.0;
		case 25:  return 5.0;
		case 50:  return 10.0;
		case 75:  return 15.0;
		case 100: return 20.0;
		default:  return 0.0;   // an impossible N_RB is a MIB coincidence
	}
}

// ---- EARFCN + operating band from the downlink centre frequency (36.101 Table 5.7.3-1) ----
// One row per band the bench visits. EARFCN_DL = N_offs + round((F_MHz - F_low) / 0.1).
// Overlapping ranges (B20/B28 both cover 791-803) are resolved by table order: the licensed
// German 800 MHz block is B20, so it is listed first and wins. Both the frequency-derived band
// (here) and the SIB1 freqBandIndicator are surfaced side by side - a mismatch is the tell.
struct band_row {
	int band;
	double f_low_mhz;   // FDL_low
	double f_high_mhz;  // FDL_high
	int n_offs;         // N_Offs-DL
};
inline const std::vector<band_row> &band_table() {
	static const std::vector<band_row> t = {
		{ 20,  791.0,  821.0, 6150 },   // 800 MHz digital dividend (listed before B28 on purpose)
		{ 28,  758.0,  803.0, 9210 },
		{ 8,   925.0,  960.0, 3450 },
		{ 3,  1805.0, 1880.0, 1200 },
		{ 1,  2110.0, 2170.0,    0 },
		{ 7,  2620.0, 2690.0, 2750 },
		{ 32, 1452.0, 1496.0, 9920 },   // SDL
		{ 38, 2570.0, 2620.0, 37750 },  // TDD
		{ 40, 2300.0, 2400.0, 38650 },  // TDD
	};
	return t;
}

// Band number for a centre frequency, or 0 if it falls in no tabled band.
inline int band_for_freq_mhz(double f_mhz) {
	for(const auto &r : band_table()) {
		if(f_mhz >= r.f_low_mhz - 1e-6 && f_mhz <= r.f_high_mhz + 1e-6) {
			return r.band;
		}
	}
	return 0;
}

// EARFCN for a centre frequency, or -1 if it falls in no tabled band.
inline int earfcn_for_freq_mhz(double f_mhz) {
	for(const auto &r : band_table()) {
		if(f_mhz >= r.f_low_mhz - 1e-6 && f_mhz <= r.f_high_mhz + 1e-6) {
			return r.n_offs + (int)(( f_mhz - r.f_low_mhz) / 0.1 + 0.5);
		}
	}
	return -1;
}

// ---- eNB-ID and sector from the 28-bit cellIdentity --------------------------
// eNB = ECI >> 8, sector = ECI & 0xFF. The file records that an earlier draft had 224216 for
// 0x036D801 (wrong arithmetic - that would be 0x36BD8) and 14040 is the real answer, cross-
// checked against the operator database. That regression is pinned in --selftest.
inline uint32_t eci_enb(uint32_t eci)   { return eci >> 8; }
inline int      eci_sector(uint32_t eci) { return (int)(eci & 0xFF); }

} // namespace cellscan

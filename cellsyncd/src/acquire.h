// cellsync auto-acquire: candidate frequency set + last-good persistence.
// A single default frequency cannot be region-safe: no LTE carrier is deployed on
// both sides of the Atlantic. With no --freq the daemon surveys this ordered set on
// ONE rx session (retune per candidate), commits early when the persisted last-good
// re-confirms near its remembered strength (startup fast path - the survey is the
// whole startup cost at a settled site), otherwise runs on the strongest confirmed
// cell of the full pass. --candidates overrides the set for site-specific lists.
// Entries are high-yield fixed centers, not guarantees - a dead candidate costs one
// short visit. US 700: B13 751.0 (one 10 MHz block US-wide), B14 FirstNet 763.0,
// B12 739.0 (the common lower-700 10 MHz center). EU 800: B20 796/806/816 (791-821
// DL = 3 x 10 MHz, the EU-wide coverage layer). US B5 889.0 + PCS 1977.5: bench-site
// receipts 2026-07-21 (rtsa sweep + --scan; the only LTE confirms above 850 MHz
// there - the strong 600 MHz B71 block is 5G NR, invisible to an LTE tracker).
// Variable-raster bands (EU B3/B1/B7) have no universal centers: run a site --scan
// and pin the finds via --candidates.
#pragma once
#include <cmath>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <vector>

namespace cellsync {

static const double k_acquire_mhz[] = { 751.0, 763.0, 739.0, 796.0, 806.0, 816.0, 889.0, 1977.5 };
static const size_t k_acquire_n = sizeof(k_acquire_mhz) / sizeof(k_acquire_mhz[0]);
inline const char k_last_good_path[] = "/rfnm/apps/cellsync/last_good_mhz";

struct last_good {
	double mhz = 0;             // 0 = nothing persisted
	double snr_db = 0;
	bool have_snr = false;      // legacy single-field files carry no snr
};

// candidate order: persisted last-good first (when sane), then the table minus its
// duplicate (100 kHz raster: anything within 50 kHz is the same carrier)
inline std::vector<double> acquire_order(double last_mhz, const double *tbl, size_t n) {
	std::vector<double> out;
	bool last_ok = last_mhz > 400.0 && last_mhz < 4000.0;
	if(last_ok) {
		out.push_back(last_mhz);
	}
	for(size_t i = 0; i < n; i++) {
		if(last_ok && std::fabs(tbl[i] - last_mhz) < 0.05) {
			continue;
		}
		out.push_back(tbl[i]);
	}
	return out;
}

inline last_good last_good_load(const char *path) {
	last_good lg;
	FILE *f = fopen(path, "r");
	if(!f) {
		return lg;
	}
	double mhz = 0, snr = 0;
	int n = fscanf(f, "%lf %lf", &mhz, &snr);
	fclose(f);
	if(n < 1 || !(mhz > 400.0 && mhz < 4000.0)) {
		return lg;
	}
	lg.mhz = mhz;
	if(n == 2) {
		lg.snr_db = snr;
		lg.have_snr = true;
	}
	return lg;
}

inline bool last_good_store(const char *path, double mhz, double snr_db) {
	std::string tmp = std::string(path) + ".tmp";
	FILE *f = fopen(tmp.c_str(), "w");
	if(!f) {
		return false;
	}
	fprintf(f, "%.1f %.1f\n", mhz, snr_db);
	fclose(f);
	if(rename(tmp.c_str(), path) != 0) {
		unlink(tmp.c_str());
		return false;
	}
	return true;
}

// survey fast path: commit to the last-good candidate without visiting the rest of
// the set when it re-confirms near its remembered strength - a cell within 6 dB of
// its stored snr is the same cell breathing, not a downgrade worth a full re-rank
// (the giveup->resurvey fallback still re-ranks whenever the pick goes dark). Legacy
// files without a stored snr demand a solid absolute floor instead.
inline bool survey_commit_early(const last_good &lg, double mhz, double snr_db) {
	if(lg.mhz <= 0 || std::fabs(mhz - lg.mhz) >= 0.05) {
		return false;
	}
	return lg.have_snr ? snr_db >= lg.snr_db - 6.0 : snr_db >= -6.0;
}

} // namespace cellsync

// cellscan: the data model of a survey - one scan job over a frequency list, each carrier a
// dwell + offline decode, each carrier zero or more cells. These structs ARE the /status
// "scan" object; report.cpp emits them to JSON and re-derives nothing the decoder did not
// prove. Kept header-only and dependency-free so the selftest can build a cell_result from a
// SIB1 transport block and assert the exact row a person reads.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cellscan {

// One PLMN of a cell's plmn-IdentityList, already rendered for humans.
struct plmn_entry {
	std::string mcc;       // "262"
	std::string mnc;       // "01" (digit count preserved: "01" and "1" are different operators)
	std::string plmn;      // "262-01"
	std::string op;        // "Telekom" | "PLMN 262-42"
	bool reserved = false; // cellReservedForOperatorUse
};

// One SI-RNTI DCI the common search space actually returned, kept verbatim. A cell that stops
// at mib_only usually IS reachable - the grant was on air and the chain below refused it - so
// the grant itself is the evidence, not a derived verdict.
struct si_dci_seen {
	int format = 0;             // 0x1A | 0x1C, emitted as the hex string "1A" / "1C"
	bool localized = false;     // false = distributed VRB (the usual choice for SIB1)
	int rb_start = 0;
	int n_prb = 0;
	int mcs = 0;                // the DCI MCS field - on SI-RNTI this IS the TBS row index
	int rv = 0;
};
constexpr size_t SIB1_DIAG_MAX_DCIS = 8;

// Where the SIB1 chain died, counted at every point decode_cell() silently gives up. Only
// meaningful - and only emitted - while mib_ok && !sib1_ok: the counters are the ladder in the
// order the decoder walks it, so the first non-zero one names the stage that needs work.
struct sib1_diag {
	uint32_t pcfich_fail = 0;   // no CFI on an SF5 that should carry SIB1
	uint32_t no_si_dci = 0;     // CFI ok, but no PDCCH candidate parsed as an SI grant
	uint32_t dci_rejected = 0;  // SI grant parsed, but 1C / distributed - no PDSCH path in v1
	uint32_t pdsch_fail = 0;    // grant accepted, LLR extraction failed
	uint32_t crc_fail = 0;      // turbo output did not pass CRC24A (also the normal RV-1 case)
	uint32_t asn1_fail = 0;     // CRC passed, the SIB1 grammar rejected the head
	int last_asn1_rc = 0;       // sib1_head_decode's return code from that last rejection
	std::vector<si_dci_seen> dcis;   // capped at SIB1_DIAG_MAX_DCIS, identical tuples deduped
};

// Where the MIB chain died, counted at every point decode_cell() gives up on a frame or on a
// hypothesis. Only meaningful - and only emitted - while !mib_ok, where a cell used to be a
// bare pss_only row saying nothing at all. The counters are the ladder in the order the
// decoder walks it, and what they answer is which KIND of failure this is: a cell with
// frames > 0 and nothing but crc_fail has its PBCH in the buffer and is merely too weak,
// which is the cell more integration can still win; one that never got past demod/chest was
// never received at all.
struct mib_diag {
	uint32_t win_short = 0;    // a window could not offer four frames, i.e. one whole 40 ms
	                           // TTI. NOT bumped when a window merely ends - every finite
	                           // burst does that - nor for a half hypothesis that starts
	                           // before the buffer, which is normal whenever the PSS sits in
	                           // the first frame; counting those made this read >= 1 on
	                           // essentially every cell and mean nothing
	uint32_t demod_fail = 0;   // a PBCH symbol would not demodulate
	uint32_t chest_fail = 0;   // no channel estimate on that frame
	uint32_t re_budget = 0;    // wb_pbch_extract did not return the 240 REs of normal CP
	uint32_t bw_reject = 0;    // decoded, but all-zero or a dl-Bandwidth outside the table
	uint32_t crc_fail = 0;     // decoded and plausible, CRC-16 said no - the NORMAL weak case
	int frames = 0;            // observations that reached the decoder, over every window and
	                           // both half hypotheses, like every counter above
	double path_metric = -1.0; // how close the best hypothesis came: the MEAN over its groups
	                           // of each group's best Viterbi path metric, normalised so 1.0
	                           // means the surviving path agreed with the sign of every soft
	                           // bit. A mean and not a maximum on purpose (wb_mib_metric): the
	                           // maximum grew with the window on PURE NOISE and barely moved
	                           // with SNR, so empty spectrum scanned long outranked a real
	                           // nearly-decodable cell - it pointed at the wrong carriers
};

// One decoded (or partially decoded) cell on a carrier.
struct cell_result {
	int pci = -1;
	int nid1 = -1, nid2 = -1;

	// from the MIB (only meaningful once mib_ok)
	int ports = 0;
	int nrb = 0;
	double bw_mhz = 0.0;
	std::string cp = "normal";
	std::string duplex = "fdd";
	int sfn = -1;

	// signal quality (uncalibrated - see report.cpp)
	double snr_db = 0.0;
	double power_db = 0.0;
	double cfo_hz = 0.0;
	// The two raw acquisition numbers snr_db is only a rendering of. Kept because the
	// escalation gate (job.h) is a statement about THEM, and because a reader looking at a
	// doubtful PCI needs to see how far above the detection floors it actually sat. Both
	// default to 0, i.e. below every floor: a cell_result nobody filled in must not be able
	// to earn a 983 MB deep capture by omission.
	double pss_metric = 0.0;    // normalised PSS correlation peak (cellfind.h)
	double sss_margin = 0.0;    // SSS decision margin (cellfind.h)

	bool mib_ok = false;
	bool sib1_ok = false;
	bool rv_combined = false;   // >1 DISTINCT redundancy version was in the decode

	// from SIB1 (only meaningful once sib1_ok)
	std::vector<plmn_entry> plmns;
	std::string op;             // operator of the first PLMN, the headline row
	int tac = -1;
	uint32_t eci = 0;           // 28-bit cellIdentity
	uint32_t enb = 0;           // eci >> 8
	int sector = -1;            // eci & 0xFF
	int band_ind = 0;           // freqBandIndicator from SIB1
	int q_rxlevmin = 0;         // raw ASN value (-70..-22), the number prints (-64)
	int q_rxlevmin_dbm = 0;     // actual dBm = 2 x raw (-128): units of 2 dB, 36.331
	int si_window_ms = 0;
	std::vector<int> sibs;      // scheduled SIB numbers (2/3/5/...)
	bool sib16 = false;
	bool cell_barred = false;

	// decoded | mib_only | pss_only  (carrier-level statuses live on carrier_result)
	std::string status = "pss_only";

	// why SIB1 did not come out (report.cpp emits it only while mib_ok && !sib1_ok)
	sib1_diag diag;
	// why the MIB did not come out (report.cpp emits it only while !mib_ok)
	mib_diag mdiag;
};

// One carrier of the scan: a frequency, its dwell, and what it yielded.
struct carrier_result {
	double freq_mhz = 0.0;
	int earfcn = -1;
	int band = 0;               // derived from the frequency (cross-checks band_ind, )
	int dwell_s = 2;
	std::string antenna;        // the RF path this carrier was actually received on

	// decoded | mib_only | pss_only | no_signal | too_wide | capture_error
	std::string status = "no_signal";
	double power_db = 0.0;
	// Why the 2 s -> 8 s escalation this carrier ASKED for did not happen: "" = it happened,
	// or was never wanted. weak | job_cap | memory (job.h deep_budget). Without it a refused
	// escalation is indistinguishable from a cell that simply did not decode - the row reads
	// "pss_only" either way - and the operator would tune the wrong knob.
	std::string deep_skip;
	std::vector<cell_result> cells;
};

// One stage-1 sweep probe: a candidate frequency and its measured in-band elevation over the
// out-of-band floor (dB, DC excluded - bandplan.h). The probes array IS the sweep's live view:
// it grows during stage 1, and `picked` marks the probes that went on to the full decode.
struct sweep_probe {
	double freq_mhz = 0.0;
	int band = 0;
	double level_db = 0.0;
	bool ok = true;             // capture succeeded (false = the frequency was NOT measured)
	bool above = false;         // level_db >= the sweep threshold
	bool picked = false;        // selected for stage 2
};

// Progress of the running (or finished) job.
struct scan_progress {
	int index = 0;              // carrier currently worked, 0-based
	int total = 0;
	double freq_mhz = 0.0;
	std::string phase = "idle"; // probe|tune|capture|decode|capture_deep|decode_deep|done
	int pct = 0;
};

// The whole scan document (running or last-finished).
struct scan_doc {
	uint64_t job = 0;
	std::string state = "idle"; // running | done | cancelled | error
	std::string error;          // populated only in the error state
	std::string mode = "scan";  // scan (explicit list) | sweep (band plan, two-stage)
	int64_t started = 0;        // unix seconds
	int64_t finished = 0;
	// sweep only: stage-1 results (grows live) and the threshold they were judged against.
	// For a sweep, freqs_mhz below is the stage-2 decode list, filled AFTER selection.
	double sweep_thresh_db = 0.0;
	std::vector<sweep_probe> probes;
	std::vector<double> freqs_mhz;
	int dwell_s = 2;
	bool deep = true;
	int gain = 12;
	std::string antenna = "auto";   // the job's RF path: a pinned name, or "auto" = the
	                                // per-carrier policy (each carrier below names its own)
	scan_progress progress;
	int cells_found = 0;
	int sib1_decoded = 0;
	std::vector<carrier_result> carriers;
};

} // namespace cellscan

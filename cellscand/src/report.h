// cellscan: turn a scan_doc into the JSON the panel renders, and persist/reload the last one.
// The emitter is the single place the operator name, band, eNB/sector and bandwidth are put
// on the wire - derived SERVER-side (plmn.h) so every client, not just our panel, gets the
// human row. Persistence is atomic (tmp + rename) and rejects a truncated file, because appd
// idle-stops this daemon routinely and a half-written last
// scan must never resurrect as a plausible-looking lie.
#pragma once

#include <cstdint>
#include <string>

#include "scan.h"

namespace cellscan {

// The "scan" object of /status (a JSON object literal, no trailing newline).
std::string emit_scan(const scan_doc &d);

// Fill the derived, human-readable fields of a cell from what the decoder proved: operator
// name(s) from the PLMNs, eNB/sector from the ECI, bandwidth from N_RB. Idempotent.
void derive_cell(cell_result &c);

// Atomic save of the last completed scan to `path` ({"saved_at":<unix>,"scan":{...}}). Returns
// false on any I/O error (already the caller's cue to log, not crash).
bool persist_save(const std::string &path, const scan_doc &d);

// Load a persisted scan. Returns false if the file is missing, unparseable or truncated
// (a scan object without its closing structure). On success fills `out` and `saved_at`.
bool persist_load(const std::string &path, scan_doc &out, int64_t &saved_at);

} // namespace cellscan

# cellscand — LTE cell-survey app for the RFNM

`cellscand` walks a list of downlink frequencies, captures a wideband burst on each, and
decodes it offline into a table of the cells it found: **PCI**, **operator** (derived from the
PLMN), **TAC**, **cellIdentity** with its **eNB-ID / sector**, **bandwidth**, **antenna-port
count**, **frequency/EARFCN/band** and **receive power**. It is the survey front end for the
SIB1 chain proven in [`../cellsyncd`](../cellsyncd) — the same `lte_wb` / `lte_coding` /
`lte_rrc` sources, linked unchanged, never forked (see the Makefile).

RX-only. A scan is a **one-shot auftrag**, not a running mode: the daemon starts idle and
opens no radio; `POST /scan` runs one pass and releases the device when the job finishes.
Because it is a normal `radio:"rx"` app under appd's single-owner lease, it can never run at
the same time as `clocksync`.

## Signal chain (per carrier, strictly sequential)

```
tune (rx_session.retune + settle discard)
  → capture   2 s @ 15.36 MSps as a burst in RAM — NO DSP runs during capture
  → decode    (offline) PSS/SSS acquisition → per PCI:
                MIB (PBCH) → PCFICH → PDCCH (SI-RNTI, common search space)
                → PDSCH → turbo (redundancy-version accumulation in the 80 ms window)
                → SIB1 ASN.1 head → cell record
  → capture_deep / decode_deep   only if the carrier can still be won AND the job can afford
                it: one 8 s burst, decode again (: "2 s, then 8 s, then gain, then
                a fresh window" — the app automates exactly the 2 s → 8 s step, no further)
```

The escalation is bounded three ways (`src/job.h`), because one deep burst is a single 983 MB
allocation on a 3.27 GB board with no swap: a cell that never produced a MIB must sit **clear**
of the detection floors — phantom PCIs sit *on* them — a job escalates at most **4** times, at
most **2** of those on cells without a MIB, and the allocation must fit **half of
MemAvailable** at the moment it is made. A cell holding a verified MIB escalates exactly as it
always did. A refused escalation is logged loudly and carried in the carrier row as
`deep_skip` (`weak` | `job_cap` | `memory`) — it is never quietly reported as "did not decode".

The verified chain does the hard part; this app is the capture buffer, the PSS/SSS acquisition
that produces the PCI list, the per-PCI sequencing (a thin glue over the production
`wb_*` functions), and the human derivations: operator from the PLMN, band/EARFCN from the
frequency, eNB/sector from the 28-bit cellIdentity, bandwidth from N_RB. Those derivations are
done **server-side** so every client — not just this GUI — gets the readable row.

**Operator table** (German MCC 262): `01` Telekom, `02` Vodafone, `03`/`07` Telefónica/O2,
`23` 1&1; anything else renders as its bare PLMN (`PLMN 262-42`), never a guessed brand.

**Standing sanity check:** the band computed from the frequency and the SIB1
`freqBandIndicator` are both reported. Two views that should agree — a mismatch, or an eNB-ID
with no database records, is the tell that something is wrong no matter how clean the CRC
looked. `eNB = ECI >> 8`, `sector = ECI & 0xFF` (0x036D801 → eNB 14040 / sector 1, **not**
224216 — the arithmetic bug pinned in `--selftest`).

## Scope / limits (honest, v1)

- **50 PRB (10 MHz) at 15.36 MSps.** A cell whose MIB reports N_RB > 50 does not fit this
  numerology and is reported as `too_wide` with its MIB fields, not decoded (a 30.72 MSps path
  is deliberately not v1 — N_RB is only known after the MIB, ).
- **FDD, normal CP.** TDD cells surface as `pss_only`.
- **One cell per NID2 per carrier.** The survey reports the dominant cell of each sync colour;
  it does not try to separate co-NID2 neighbours the sync window cannot cleanly resolve.
- `power` and `snr` are **uncalibrated** (carrier-over-floor style) — no
  invented absolute RSRP dBm.

## Build / test

Host build (no librfnm; runs the selftest and the offline `--file` path):

```sh
make                 # builds ./cellscand (WITH_RFNM=0)
./cellscand --selftest
```

Offline decode of a capture (the acceptance path — the two reference captures pass here before
any board is touched):

```sh
./cellscand --file capture_816.cf32 --freq 816.0      # host-TCP captures are conjugated by
./cellscand --file capture_796.cf32 --freq 796.0      #   default; add --no-conj for board-side
```

Board build: cross-compile aarch64 against a board sysroot, `WITH_RFNM=1` (same pattern as the
rest of the bundle; or `./rfnm-sdk build apps`). `-fcx-limited-range` is required for the same
reason as in cellsyncd/lorad (it inlines every `std::complex<float>` multiply).

## Selftest coverage (`--selftest`, hardware-free, deterministic)

- The operator table, band/EARFCN derivation, bandwidth-from-N_RB and the ECI→eNB/sector split
  (including the pinned 14040-not-224216 rule).
- **The two on-air BCCH transport blocks from as byte literals** → through the
  production `sib1_head_decode` → the complete emitted cell JSON, asserting the exact row a
  person reads: 262-01/Telekom/eNB 104113/sector 6/TAC 0x11FA/band 20 for the 816 cell, and
  262-03/Telefónica/eNB 14040/sector 1/TAC 0x2AF8/band 20 for the 796 cell. This proves the
  display shows what was decoded.
- Persist round-trip (save → load), and rejection of a truncated file.
- The scan state machine driven by a mock burst source: happy path, per-carrier progression,
  parameter validation (frequency/dwell/empty-list rejection), cancel, and the persisted-result
  reload.
- The antenna policy: the crossover picks the measured-good path per band and splits no band,
  an explicit `--antenna` pins every carrier and reorders nothing, and a recording burst source
  proves the job actually tunes each carrier on its own path — with exactly one path change for
  an interleaved list, and one for the sweep's probe pass.
- **What the 2 s → 8 s escalation is allowed to cost.** The capture arithmetic (245.76 MB /
  983.04 MB), the memory budget including its exact edge (`need ≤ ½·MemAvailable`) and a
  simulated low-memory board that refuses the escalation and still reports the 2 s result; the
  per-job cap walked over a full 16-carrier sweep, including eight phantom carriers ahead of
  four real ones (the phantoms cannot starve the real cells); a bare-threshold cell that does
  not escalate beside a MIB-bearing one that does; and the peak, **measured** — `ru_maxrss` is
  a high-water mark, so a transient second buffer cannot hide from it, and the test carries a
  deliberate double-buffer as its own control to prove the measurement can see one.

The DSP itself is regressed by cellsyncd's 152 checks (the shared chain) and by the `--file`
acceptance run against the two reference captures.

## Run / HTTP API

```
cellscand [--port 7692] [--preset b20|b20+b8|eu-macro|last]
          [--gain 12] [--antenna auto|embed|sma_a] [--agc 0|1]
          [--file <cf32> [--freq MHz] [--no-conj]]   # offline one-shot, then exit
```

HTTP on `127.0.0.1:7692`, published by the on-board nginx as `/api/app/cellscan/`:

- **`GET /status`** — the one document the panel renders:
  `{state:"idle"|"scanning", uptime, persist:{loaded,saved_at}, scan:{…}}`. The `scan` object
  carries `state` (running/done/cancelled/error), `progress` (index/total/freq/phase/pct), and
  `carriers[]` each with `cells[]` — every field above per cell, plus the `antenna` that
  carrier was received on (the job-level `antenna` is `auto` or the pinned name).
- **`POST /scan`** — body `{"freqs_mhz":[796.0,806.0,816.0],"dwell_s":2,"deep":true,
  "antenna":"auto"}` (all optional; without `freqs_mhz` the manifest preset applies).
  Validation: 1–32 freqs, 400–4000 MHz, dwell ∈ {2,8}. `200 {"job":N,"state":"running"}`;
  `409 {"err":"scan läuft"}` if a job is active; `400 {"err":…}` on a bad body. The list
  arrives in the POST body precisely because a manifest config key is limited to a fixed
  `choices[]` menu and a body is not.
- **`POST /sweep`** — the one-button survey: no body sweeps the whole band plan
  (`src/bandplan.h`: B20 block centres 796/806/816, B8/B3/B1/B7 on a 5 MHz grid, 47 candidates);
  `{"bands":["b20","b3"],"antenna":"auto"}` restricts it. Two stages: a 0.35 s power probe per
  candidate (in-band elevation over the out-of-band floor, DC region excluded — the
  trap), then the full decode path only for probes ≥ 6 dB, strongest first, grid maxima only,
  capped at 16.
  Same job type as `/scan`: `/status` and `/cancel` apply unchanged, and during stage 1 the
  `scan` object grows a `probes[]` array (freq/band/level_db/above/picked) the panel renders as
  live bars. `200 {"job":N,"state":"running","mode":"sweep","candidates":47}`.

  Measured on the board (sma_a, gain 12, suburban indoor antenna): the full plan runs in
  **330–380 s** — 47 probes in **33 s** (stage 1 is retune-bound: 0.35 s settling + 0.35 s
  capture each), then **14–15 decodes** at ~15 s apiece, which is where the time actually goes.
  Stage 2 is decode-bound, not capture-bound, and the progress estimate says so
  (`SWEEP_DECODE_S`). Stage 1 is budgeted against the `SWEEP_MAX_DECODES` worst case so the bar
  can only be pessimistic and never walks backwards when stage 2 takes over. The reference run
  recovered the known B20 cells — 816 MHz PCI 274, 50 PRB, 4 ports, Telekom, TAC 4602, and
  796 MHz PCI 377 — and reported everything else honestly: 806 MHz at −9 dB is empty air, while
  the loud-but-undecodable probes in B1/B7 (up to +27 dB, UMTS/NR occupancy, no LTE PSS) come
  back `no_signal`. The probe metric is a power detector, not an LTE detector: energy above the
  threshold buys a decode attempt, nothing more.
- **`POST /cancel`** — `200 {"job":N,"state":"cancelled"}`, idempotent (`{"state":"idle"}` when
  nothing is running).

`daemon/rfnm-cellscan.service` runs it under systemd; appd generates the nginx route from the
`cellscan` manifest and starts the app when it is granted the radio lease.

## Radio, memory, persistence — the three design decisions

- **Radio lease.** The `radio:"rx"` manifest under appd's exclusive lease is what guarantees no
  overlap with clocksync. The daemon opens librfnm only for the duration of a job (one
  `rx_session`, opened once, `retune()` per carrier — open/close per frequency cost ~5 s), and
  `idle_stop_s:180` lets appd reclaim the device when no browser polls. Viewing results costs
  no lease.
- **Capture.** Per carrier, one 15.36 MSps burst into RAM; the capture callback does only
  memcpy + timestamp-gap bookkeeping, no DSP. Decode runs offline afterward, converting only
  the radio frames it needs from the burst, so the working set stays small even though the raw
  burst is large: `dwell × 15.36e6 × 8 B` = **245.76 MB** for 2 s and **983.04 MB** for the 8 s
  escalation, out of 3.27 GB total / ~3.13 GB available at idle.
- **Peak = one burst, and it is checked before it is asked for.** Both allocations that could
  hurt are bounded (`src/burst.h`). *One buffer:* a capture frees what its destination held
  **before** reserving the new size, and the 2 s burst is released the moment the carrier
  result exists — so an escalation peaks at 983.04 MB and never at 245.76 + 983.04 MB, which
  a `reserve()` over a live buffer would have made transient and invisible. *A budget:* every
  capture, escalation or not, must fit `CAPTURE_MEM_FRACTION = 0.5` of `MemAvailable`
  (`/proc/meminfo`). Half, because the capture is never the only thing live and MemAvailable
  is an estimate, not a promise. On an idle board the deep capture still fits (983 MB vs a
  1.56 GB ceiling); it is refused once something else holds ~1.2 GB — and refusing is logged,
  not silent. Worst case for a full 16-carrier sweep is therefore **983.04 MB** resident at any
  instant, not 16 × 983 MB and not 1.23 GB.
- **Persistence.** The last completed scan is written to `/rfnm/apps/cellscan/last_scan.json`
  (atomic tmp+rename, a few KB). appd idle-stops this daemon routinely, so without persistence
  the "scan, walk away, look later" workflow would lose its result on every idle-stop or
  lease hand-back. The IQ bursts are **not** persisted (flash wear, no reader for them).

## Operating discipline (from )

- **Gain 10–14, not 30.** The front end compresses long before the ADC clips; a too-high gain
  finds nothing and reports it as "no cells", which reads like a dead antenna.
- **The antenna is picked per carrier, not per job.** The SMA-A path has an ~800 MHz high-pass
  filter that attenuates the sub-800 MHz macro cells, and the embedded antenna is the low-band
  path — so no single antenna can decode a sweep from B20 to B7. `src/bandplan.h` holds the
  policy: below `ANTENNA_CROSSOVER_MHZ` (1000 MHz, above B8 and below B3, so no band is split)
  the carrier is received on `embed`, above it on `sma_a`. The measurement it is built on
  (one bench board, Hamburg, gain 12, 8 s deep dwell, fresh boot per run):

  | antenna | 796 MHz PCI 439 | 816 MHz PCI 274 | 2145 MHz PCI 190 | 2665 MHz |
  |---|---|---|---|---|
  | `embed` | **SIB1 decoded** | **SIB1 decoded** | no_signal | no_signal |
  | `sma_a` | mib_only (never decoded) | pss_only | **SIB1 decoded** (twice) | — |

  `--antenna embed|sma_a` (or `"antenna"` in the scan/sweep body, or the `antenna` config key)
  still pins one path for the whole job, exactly as before the policy existed; `auto` is the
  policy and is what ships. Each carrier row in `/status` names the path it was received on.
  Changing the path is one extra field in the `retune()` apply on the open session — no reopen
  — but it re-arms the same ~0.35 s settle discard, so the job groups its carrier list by
  antenna and switches at most once (the band plan runs low→high, so the sweep's probe pass
  crosses over exactly once as well).
- Receive sensitivity on the bench decays over hours and a reboot restores it; treat each
  session as a window and throw the first RX away.

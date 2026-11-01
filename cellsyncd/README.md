# cellsyncd — LTE-disciplined oscillator daemon for the RFNM

Disciplines the RFNM Blue's Si5510 reference to the LTE downlink of surrounding macro
cells: a GPSDO that needs no sky view. RX-only, no SIM, no attach — pure sync-layer
measurement (PSS/SSS acquisition + CRS phase tracking) feeding a slow steering loop that
pulls the Si5510 master DCO in 0.1 ppb steps.

## Why this works

- 3GPP mandates ±50 ppb carrier accuracy at wide-area eNB antennas; real macro cells are
  GNSS-disciplined (TDD sites must hold ±1.5 µs phase), so their long-term frequency truth
  is ~1 ppb — effectively GPS-traceable, and receivable indoors.
- Home/local BS classes are allowed ±100/250 ppb → the loop tracks several cells and takes
  a weighted median; a lone femtocell can't steer us.
- Both the LO and the ADC clock derive from the same Si5510 reference, so the measured
  carrier CFO and the measured symbol-timing drift are two views of the same reference
  error (CFO/f_LO = −timing-slope). The tracker measures both and uses the identity as a
  self-consistency check.

## Signal chain

    LA9310 RX @ 3.84 MSps, LO = f_cell + δ (δ default +607.5 kHz parks the zero-IF DC
      spike OUTSIDE the ±540 kHz sync band after the remix, mid-subcarrier so it never
      owns a bin)
    → NCO −δ (phasor recurrence) → polyphase 1/2 resampler → 1.92 MSps complex float
      (the capture layer detects stream gaps off the ns timestamps: small gaps are
      zero-filled at true length, big ones flush acquisition — a glued stream poisons
      the acquisition accumulators)
    → acquisition: PSS (ZC-63, 3×NID2) correlation on 7 frequency variants (±45 kHz);
      peaks cluster into 5 ms trains → each (NID2, variant, residue) candidate class
      persists across 0.2 s-strided windows and accumulates, per 2.5 kHz CFO hypothesis,
      a noncoherent |corr|² alignment profile plus PSS-equalized SSS metrics for all 336
      (NID1, half). Decision on the accumulated metric excess over the class's own
      336-row median, margined against the m-family-disjoint runner-up; tiers strong /
      weak / desperate. All accepts are provisional — CRS descrambling coherence confirms
      or kills, and a killed PCI is blacklisted 5 min so re-acquisition explores the next
      candidate
    → per-cell tracker (up to 4 PCIs on one carrier): PSS timing follow (parabolic
      sub-sample), CRS extraction each slot (l=0,4 port 0), wide/narrow occupancy
      (N_RB itself is not observable from the sync window), fine CFO from CRS
      cross-symbol phase slope
    → steering: weighted-median cell offset (ppb) → single-integrator loop
      (τ default 120 s) → absolute DCO step register write

## Si5510 DCO actuator

- One step = the clock plan's FINC/FDEC step = **0.100000039438 ppb**; the offset is
  absolute (0 = nominal, each write replaces the last), usable range ±100 000 steps
  (±10 ppm).
- The kernel exposes it via sysfs: `rfnm_set_dco_steps` (raw absolute steps, full 0.1 ppb
  resolution — what the daemon writes) and `rfnm_set_dco_ppb` (human-facing integer ppb).
- The actuator transfer was bench-measured at the chip level (`tools/dco_cal.cpp`):
  +100000 steps → +10.00 ppm, sign +1, unity gain — exactly the plan value. The same gain
  reproduces on a live RF signal through both observables (CFO and timing).
- The master reference pulls every Si5510 output together (LO reference and ADC clock) —
  that is what closes the loop on itself.
- The pull sign is deterministic (+1), so the startup sign probe defaults OFF; opt in with
  `--sign-probe` for unknown-hardware bring-up, or override with `--dco-sign`.

## Sign conventions (single source of truth, used by code and selftest)

Let ε = fractional error of our reference (ε>0 = our clock runs fast).
- LO = f_LO·(1+ε) → measured baseband CFO ≈ **−ε·f_LO**  (positive CFO ⇒ we run slow)
- PSS spacing measured in our samples ≈ 9600·(1+ε) → timing slope **+ε**
- Reported per-cell `offset_ppb` = ε̂·1e9 = −CFO_hz/f_LO·1e9
- Correction: DCO steps ← steps − round((Kp·e)/step_ppb), e = ε̂ in ppb

## What is proven without a board (`--selftest`, `--sim`)

ZC/PSS correlation properties, SSS encode→decode across NID1/half-frame, Gold/CRS
generator self-consistency, resampler tone/image behavior, CP + integer-bin + CRS CFO
estimator accuracy against synthesized downlink with injected CFO/SCO/AWGN/multipath
(including the CFO↔timing-slope identity), N_RB hypothesis detection, multi-cell tracking,
and a closed-loop simulation of the steering loop (convergence from ppm-scale error,
steady-state residual, holdover) with the exact 0.100000039438 ppb step plant. 36.211
sequence conventions are isolated in `src/lte_seq.cpp`: if on-air interop ever shows a
convention slip, the fix is local.

On real cells the closed loop has been proven end-to-end: measured DCO gain on live RF
matches the chip-level calibration, and the loop drives a ~75 ppb initial error
monotonically to a held residual of ~1–2 ppb with no overshoot. CFO is the default
steering observable (near-instant response); `--observable timing` also converges but
carries the drift-window smoothing lag.

## Not in v1 (deliberate)

FDD normal-CP cells only (extended-CP acquisition fallback exists, tracking is normal-CP;
TDD later). Frequency only — no time/phase output (1PPS from frame timing is a later
project). One EARFCN at a time (multi-cell on that carrier); EARFCN round-robin later.
PBCH/MIB not decoded (N_RB comes from the CRS hypothesis test).

Taking it further — decoding SIB1 for the cell's PLMN, `cellIdentity` and TAC, and
SIB16 for a UTC-traceable 1PPS — is a planned extension, together with a
verified off-air reference decode to regress against and the measured constraints
that shape the implementation (full carrier bandwidth is mandatory; SIB1 needs
several dB more than the MIB; the IQ convention depends on the transport).

## Build / run

```sh
make                # host build, no librfnm needed
./cellsyncd --selftest
./cellsyncd --sim   # closed-loop steering simulation report
# board build: cross-compile aarch64, WITH_RFNM=1 (see the apps bundle README);
# or ./rfnm-sdk build apps from github.com/rfnm/sdk
cellsyncd [--freq 751.0] [--rate 3840000] [--gain 30] [--agc 0|1] [--offset 607500]
          [--tau 120] [--dco-path /sys/...] [--dco-sign +1|-1] [--port 7686]
          [--scan 731.5,751.0,882.5]        # PSS-power survey, then exit
          [--candidates 889.0,1977.5,...]   # replace the built-in survey set
```

No `--freq` = auto-acquire: survey the built-in candidate set (`src/acquire.h` — US
B13/B14/B12, EU B20, US B5/PCS; no single frequency is deployed on both continents) plus
the persisted last-good (`/rfnm/apps/cellsync/last_good_mhz`, tried first next start). The
whole pass rides one RX session with per-candidate retunes (~1.5 s/candidate). A last-good
that re-confirms within 6 dB of its stored SNR commits immediately; otherwise every
candidate is visited before choosing — best of the set, never first-to-confirm. If the
committed frequency then confirms nothing for 90 s of stream time, the daemon drops back
to the survey instead of camping on a dead band. Variable-raster bands (EU B3/B1/B7) have
no universal centers: run a site `--scan` and pin finds via `--candidates` (systemd: add
it to `CELLSYNC_ARGS` in `cellsync.conf`).

HTTP on `127.0.0.1:7686`, published as `/api/app/clocksync/`: `/status` JSON
`{mode, freq, rate, uptime, samples, duty, discont, gap_max_us, dco_steps, dco_ppb,
residual_ppb, sigma_ppb, cells:[{pci, nrb, snr_db, cfo_hz, offset_ppb, weight, age}]}`
(`duty` < 0.95 means the DSP is not keeping real time).

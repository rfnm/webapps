# aisd — RFNM AIS receiver

`aisd` is an RX-only marine **AIS** (Automatic Identification System) receiver daemon for
the RFNM Blue, in the same shape as [`lorad`](../lorad): one wideband capture on the
librfnm local transport, per-channel DDCs, a NEON-optimized DSP front end, and a small
HTTP/JSON API for the on-board web GUI. It decodes both AIS VHF channels and emits
standard **NMEA AIVDM** sentences plus decoded ship info.

## What AIS is (ITU-R M.1371 / IEC 62320)

- Two 25 kHz channels: **A = 161.975 MHz** (ch 87B) and **B = 162.025 MHz** (ch 88B).
  A single capture centred at 162.0 MHz covers both.
- **GMSK**, BT = 0.4, modulation index h = 0.5, **9600 bps**.
- HDLC link layer: 24-bit training preamble, `0x7E` start flag, bit-stuffed (data + FCS),
  `0x7E` end flag, **NRZI** line coding. FCS = **CRC-16/X.25**.
- The application message (big-endian fields, e.g. type-1 position reports, type-5
  static/voyage data) is 6-bit-ASCII "armored" into the AIVDM payload; long messages span
  multiple `!AIVDM` fragments.

## Signal chain

```
RFNM local RX @ ~2 MSps  ->  DDC per channel (NCO + FIR decimate to ~100 kHz, NEON)
   ->  FM discriminator  ->  bank of K=16 fixed-phase integrate-and-dump decoders
   ->  NRZI decode  ->  HDLC deframe (destuff + CRC-16/X.25)  ->  AIS message
   ->  ais_info() + AIVDM sentence(s)  ->  HTTP ring + stdout
```

**Why a phase bank instead of a timing loop.** AIS bursts are short (~256 symbols, 27 ms),
so TX/RX clock drift across a burst is far under one symbol. Rather than a feedback timing
loop (which, for non-coherent GMSK on a near-square discriminator, is prone to slow
acquisition and half-symbol / boundary false-lock that flips chaotically with the exact
samples-per-symbol), `aisd` runs **K parallel feed-forward decoders at fixed sub-symbol
phases** and lets the link-layer CRC select the aligned one. This is deterministic and
robust across every sample rate and the full ±500 Hz AIS carrier-offset spec, with no
bistability. Duplicate decodes from adjacent phases are suppressed. See
`src/receiver.{h,cpp}`.

The decision is an **integrate-and-dump** of the FM discriminator over each symbol (the
phase change across a symbol is ±π·h regardless of per-sample noise), which is inherently
tolerant of the in-spec carrier offset. No carrier-offset tracking is needed within
±500 Hz; tune the RFNM centre accurately so the residual offset stays in spec.

## Build / test

Host build (no librfnm; runs the selftest and the no-hardware bench):

```sh
make                # builds ./aisd (WITH_RFNM=0)
make selftest       # ./aisd --selftest  — deterministic, fixed-seed, hardware-free
./aisd --bench 5    # CPU-cost proxy: synthesizes noise through the full DDC+receiver
```

Board build: cross-compile aarch64 against a board sysroot. librfnm v2 **>= 0.2.1** is
required (`rx_disable_stale_channels`); deploy it together with the app:

```sh
make clean
make WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 \
     ASIO_INC='-I/path/to/asio/include -I<sysroot>/usr/local/include' \
     LDFLAGS='-L<sysroot>/usr/local/lib -Wl,-rpath,/usr/local/lib'
```

Easy path: `./rfnm-sdk build apps` from [github.com/rfnm/sdk](https://github.com/rfnm/sdk).

`-fcx-limited-range` is required (it inlines every `std::complex<float>` multiply in the
DDC; without it GCC emits a libgcc `__mulsc3` call per multiply). Full `-ffast-math` is
avoided so the near-threshold selftest cases don't shift.

## Selftest coverage (`--selftest`, all deterministic)

CRC-16/X.25 KAT (`"123456789" -> 0x906E`); NRZI and bit-stuff round-trips; HDLC frame
build/deframe round-trip; AIVDM armor charset + NMEA checksum against a **real captured
sentence** (`13aEOK?P00PD2wVMdLDRhgvL289?`, Type 1, MMSI 244670316) and a
build-then-decode field round-trip; full GMSK → receiver decode at clean / noisy / ±480 Hz
CFO on both channels and both demod rates; full 2 MSps → DDC → decode end-to-end on both
channels; a 424-bit Type 5 through RF producing a 2-fragment AIVDM; two back-to-back
bursts; a CRC-corrupted burst rejected; a **statistical sensitivity** pass (≥98/100
decoded over 100 deterministic bursts at ~17 dB SNR spanning both channels/rates and the
full ±480 Hz spec); and a message-parse check.

Measured off-board (no hardware): decode is essentially error-free to ~13–14 dB SNR and
degrades through the non-coherent-GMSK threshold below ~11 dB (nearby ships are 20+ dB).
Zero false frames over 2000 pure-noise streams (CRC + length gating). Host bench: ~26×
realtime for both channels at 2 MSps.

## Run / HTTP API

```
aisd [--center HZ] [--rate HZ] [--channel A|B|both] [--gain dB] [--agc 0|1] [--port n]
```

Defaults: centre 162.0 MHz, rate 2 MSps (→ demod 100 kHz), both channels, port 7686.
HTTP on `127.0.0.1:7686`, published by the on-board nginx as `/api/app/ais/`:

- `GET /status` — `{center,rate,out_rate,channels,samples,uptime,queue_drops,total,chans:[{ch,freq,messages,drops,queue,last_level}]}`
- `GET /messages` — `{total, messages:[{time,ch,type,mmsi,level,info,nmea}]}` (ring of the last 500)

Each decode is also printed to stdout with its AIVDM sentence(s). `daemon/rfnm-ais.service`
runs it under systemd (`Restart=on-failure`); appd generates the nginx route from the
`ais` manifest and starts the app when it holds the radio lease.

Sanity notes for real-antenna runs: only one radio app owns the device at a time (let appd
do the switching); with an antenna near a waterway, cross-check a sentence against
`gpsdecode` or any online AIVDM decoder; `out_rate = rate/round(rate/100000)` — the phase
bank handles any rate, but avoid needlessly odd `--rate` values.

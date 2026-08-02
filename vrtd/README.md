# vrtd — VITA 49.2 Ethernet bridge for the RFNM

Makes the RFNM speak ANSI/VITA 49.2 on the network, both directions plus control:

- **RX export** — radio RX → VRT signal-data packets + IF context packets → UDP
  destination (`--dest host:port`). Any VRT-consuming toolchain can ingest the stream.
- **TX ingest** — VRT signal-data packets sent to `--listen` with stream id `--sid-tx`
  are converted and played out the TX chain (RFNM as a remote VRT exciter).
- **49.2 control** — command packets (execute / dry-run) tune the radio; acks are
  returned per the CAM request bits, with EIF0 error fields on rejects.

## Wire profile (v1)

- UDP, one VRT packet per datagram, 32-bit big-endian words.
- Data: packet type 1 (signal data + stream id), payload 16-bit complex cartesian
  signed fixed point ("CS16", I then Q per word), declared in context via the data
  payload format field (0x200003CF). Default 360 samples/packet (fits 1500 MTU);
  `--samples-per-pkt` for jumbo. TSI = UTC, TSF = free-running sample count, mod-16
  packet count per stream.
- Context: CIF0 fields RF frequency, sample rate, gain, payload format; emitted at
  start, every 1 s, and with the CHANGE bit set after any executed control.
- Control: CIF0 fields RF_FREQ (routes to RX or TX by stream id), SAMP_RATE, GAIN
  (stage 1 → RX gain or TX power). Other CIF0 fields are answered with an EIF0 ack
  (field-not-executable). Dry-run (CAM action 1) validates without applying.
  Controllee id gate via `--cid` (0 = accept any). UUID identifiers, CIF1/2/3/7,
  and scheduled (timestamped) execution are rejected, not ignored.
- Acks: v1 packs WIF0/EIF0 + one status word per flagged field, presence signaled by
  the CAM warn/err bits (matches the vita49-rs reading of 49.2). Status words are a
  documented simplification: 1 = field not executable, 2 = hardware rejected.

Layout provenance: every bit position lives in `src/vrt.h/.cpp` only, cross-checked
against voyager-tech-inc/vita49-rs (49.2 header, CIF0, CAM, command order, trailer) and
ember91/libvrt (payload format, trailer). The selftest pins them with hand-computed
reference words.

## Selftest (`--selftest`, 47 checks incl. the TX-policy gate matrix)

Reference wire vectors; 400 random pack→unpack round-trips over data/context/command;
~4800 truncation parses (no crash, clean errors); full UDP loopback on 127.0.0.1 with
the mock radio: sample integrity (ramp pattern), mod-16 sequence, context declaration;
49.2 control end-to-end: execute→state change→ACK, dry-run→no state change,
unsupported field→EIF0+untouched state, out-of-range value→EIF0, change-context
re-emit; TX ingest: VRT in→samples swallowed, zero sequence gaps. Green on x86 and
under `qemu-aarch64`.

## Build / run

```sh
make                     # host build (mock + selftest, no librfnm needed)
./vrtd --selftest
./vrtd --mock --dest 127.0.0.1:4991 --listen 4991    # loopback playground

# board build: cross-compile aarch64 (see the apps bundle README), or
# ./rfnm-sdk build apps from github.com/rfnm/sdk
make clean
make WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 \
     ASIO_INC='-I/path/to/asio/include -I<sysroot>/usr/local/include'

vrtd --freq 751.0 --rate 2400000 --gain 30 [--antenna embed]
     --dest 192.168.1.20:4991 --listen 4991 [--sid-rx 1] [--sid-tx 2]
     [--cid 0xRFNM] [--oui 0xNNNNNN] [--samples-per-pkt N] [--port 7689]
```

Status JSON on `127.0.0.1:7689` (`/status` + `/api` self-description), published by the
on-board nginx as `/api/app/vrt/`; the UDP data port (default 4991) is declared in the
app manifest and mirrored in the appd state document. TX starts are gated by the board
TX policy in `/rfnm/board.conf`. Live sample-rate changes are rejected in v1 (a rate
change is a multi-second reclock of the baseband processor); set the rate at start.

Smoke check: run against a VRT consumer (or the selftest's decode helpers), tune via a
49.2 command, then feed TX ingest into a dummy load.

## Not in v1 (deliberate)

Timestamped/scheduled execution (the 49.2 timing control field is parsed but execution is
immediate). Multi-channel (ch0 only). CIF1+ fields. UUID controllee/controller ids. VRL
framing (bare VRT only). TCP transport. Warning (WIF) generation — we either execute or
error.

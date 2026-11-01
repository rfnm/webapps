# apps — RFNM Blue on-board application daemons

The bundle repo of the radio applications that ship on the RFNM Blue, plus the shared
layer they are built from. Every app follows the same shape: one directory with a README,
a Makefile, C++17 sources, a hardware-free `--selftest`, and a small HTTP/JSON API served
on loopback and published by the on-board nginx as `/api/app/<id>/` (routes are generated
by `appd` from each app's manifest — see the `webgui` repo's `API.md` for the contract and
`rfnmctl` for the reference client).

## What's here

| dir | daemon | what it does |
|---|---|---|
| `common/` | — | The shared daemon layer: `rfnm_httpd` (embedded HTTP server), `rfnm_json` (JSON reader/writer), `rfnm_boardconf` (`/rfnm/board.conf` parsing, TX policy). Apps compile these in directly. |
| `aisd/` | `aisd` | RX-only marine AIS receiver: one wideband capture over the librfnm local transport, per-channel DDCs, NEON GMSK demod with a fixed-phase decoder bank, HDLC/CRC link layer → standard NMEA **AIVDM** sentences + decoded ship info. |
| `cellsyncd/` | `cellsyncd` | LTE-disciplined oscillator: acquires and tracks surrounding macro cells (PSS/SSS + CRS phase, RX-only, no SIM) and steers the board's Si5510 reference DCO in 0.1 ppb steps — GPSDO-grade frequency without a sky view. |
| `cellscand/` | `cellscand` | One-button LTE cell survey: sweep, PSS/SSS, MIB and SIB1 decode per carrier, reporting operator, PCI, TAC, cell identity and bandwidth. RX-only, no SIM. FDD, up to 10 MHz cells. Builds the receive chain out of `cellsyncd/src`. |
| `clockgend/` | `clockgend` | Si5510 OUT9 signal generator on the REF CLK OUT connector, driven through the kernel's `rfnm_set_siggen_freq` sysfs knob (1 MHz–200.099 MHz on a 1 kHz grid). Also reports external-reference / reference-lock status. |
| `iod/` | `iod` | Board I/O header GPIO daemon: drives the motherboard bottom-edge header pins through the kernel GPIO chardev uapi v2 (no libgpiod), including runtime pinmux between a pad's primary function and GPIO. |
| `lorad/` | `lorad` | RX-only LoRa sniffer: LoRaWAN uplink channels or the Meshtastic LongFast preset (eu868/us915 band plans), multi-channel channelizer + LoRa PHY demod, LoRaWAN/Meshtastic frame parsing (incl. AES where keys are known). |
| `vrtd/` | `vrtd` | VITA 49.2 Ethernet bridge: RX export as VRT signal-data + context packets over UDP, TX ingest (the RFNM as a remote VRT exciter), and 49.2 command/ack control. |

Each app's own README documents its signal chain, API and selftest coverage.

## Build

Host build first — every app builds and selftests on plain x86 with no hardware and no
librfnm:

```sh
make -C <app>            # e.g. make -C lorad
./<app>/<app> --selftest
```

Board binaries are cross-compiled aarch64 (Cortex-A53) against a board root filesystem
used as the sysroot; apps that touch the radio add `WITH_RFNM=1` and link librfnm v2
(>= 0.2.1):

```sh
make -C <app> clean
make -C <app> WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53   # + your cross CXX/sysroot flags
```

Run cross-built selftests under `qemu-aarch64` before deploying. The easy path:
[github.com/rfnm/sdk](https://github.com/rfnm/sdk) sets up the toolchain, sysroot and
librfnm and builds everything — `./rfnm-sdk build apps`.

## Deploy / run

Daemons go to `/rfnm/apps/<id>/` on the board together with their `app.json` manifest and
systemd unit. Units are **started, never enabled**: `appd` (from the `webgui` repo) is the
exclusive app switcher — it owns the single-owner radio lease, starts/stops apps, and
regenerates the nginx routes on `POST /api/v1/rescan`.

Conventions all radio apps follow:

- Clamp gain to the device's advertised range; validation lives device-side.
- Apply masks name only the channels the app configured.
- Per-app antenna/gain come from `/rfnm/apps/rf.conf`, written by appd's `/rf` endpoint.
- Stream health is judged by the librfnm drop counters, never by log lines.

## License

MPL-2.0, see `LICENSE`.

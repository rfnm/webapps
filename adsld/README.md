# adsld — ADS-L 4 SRD-860 transceiver with live map for the RFNM

`adsld` receives EASA **ADS-L 4 SRD-860 Issue 2** M-band frames on 868.2 and 868.4 MHz, keeps a
table of the aircraft it hears, and serves that table as JSON and as a live map. It can also
transmit an ADS-L beacon. Transmit is off by default and needs the gates listed below.

All signal processing runs on the A53 cores. Nothing runs on the VSPA. The host build uses a
deterministic mock radio, so the daemon, the API, the map and the selftest work without a board.

## Signal chain

    LA9310 RX, CS16 @ 2 MSps, LO 867.9 MHz
    -> one capture fans out to two worker threads, one per channel
    -> DDC: NCO +300 kHz / +500 kHz, FIR decimator 5:1, 400 kHz = 4 samples per chip
    -> FM discriminator
    -> 16 fixed-phase integrate-and-dump chip decoders (sub-chip timing hypotheses)
    -> 64-chip sync correlator, up to 4 chip errors accepted
    -> block capture, differential Manchester bit decisions on the soft chip integrals
    -> LEN and CRC-24 pick the frame length, single-bit syndrome correction
    -> XXTEA descrambling (all-zero key, 6 rounds; key index 3 = off)
    -> Traffic (payload type 2) and Status (type 3) field decode -> traffic table

Both channels are demodulated at the same time from one capture. The receiver does not retune.

Transmit renders Gaussian 2-GFSK bursts at 100 kchip/s, ±50 kHz deviation, to CS16 at the channel
offset. Traffic and Status each alternate channels on every frame of their kind (C.2.5).
Traffic goes out at 1 Hz airborne and 0.1 Hz on ground, Status every 15 s by default (G.2.11).

Frames with any other payload type are not parsed. They are counted in `unknown_rx` and listed in
`GET /rx` with `"unknown":true` and their `payload_type`.

## Build and selftest

```sh
make -C adsld -j8
./adsld/adsld --selftest          # hardware-free, deterministic
./adsld/adsld --bench 5           # DSP real-time factor on noise
./adsld/adsld --demo --lat 47.26 --lon 11.35    # synthetic aircraft for the map
```

Then open <http://127.0.0.1:7693/>.

The selftest runs 111 checks: packet layout and field coding, CRC-24 and XXTEA known answers
(the XXTEA sample code of E.2 and the bit-order example of B.3), Manchester and frame geometry,
modulator and channeliser, receiver sensitivity to timing, noise and carrier offset, the
scheduler under a fake clock (intervals, alternation, backoff bounds, forced frames, quiet time,
Toff, stale fixes, jammed channels, manual beacons), TX gating of the mock radio, and the
conformity notes.

## Board build

Cross-compile against a sysroot taken from the board. The two `-L` paths are required: without
them the cross linker takes glibc from the host's cross libraries, not the sysroot.

```sh
export RFNM_SYSROOT=<rfnm-sdk>/sysroot/<board>
. <rfnm-sdk>/cross/env.sh
make -C adsld clean
make -C adsld WITH_RFNM=1 ARCHFLAGS=-mcpu=cortex-a53 CXX="$CXX" \
     LDFLAGS="-L$RFNM_SYSROOT/lib/aarch64-linux-gnu -L$RFNM_SYSROOT/usr/lib/aarch64-linux-gnu $CROSS_LDFLAGS" -j8
aarch64-linux-gnu-objdump -T adsld/adsld | grep -oE 'GLIBC_[0-9.]+' | sort -Vu | tail -1
```

The last command must print a glibc version the board has.

## Install

`app.json` registers the app with appd, which generates the nginx route `/api/app/adsl/` and owns
the radio lease. `app_adsl.js` is the web GUI panel. It frames the daemon's own map, which uses
relative URLs and works behind the route unchanged.

| File | Goes to |
|---|---|
| `adsld/adsld`, `adsld/app.json`, `adsld/www/` | `/rfnm/apps/adsl/` |
| `adsld/daemon/rfnm-adsl.service` | `/etc/systemd/system/` |
| `adsld/daemon/app_adsl.js` | `/rfnm/spectrum/www/`, plus a `<script>` tag in `index.html` |

Then run `systemctl daemon-reload && systemctl restart rfnm-appd`. Do not `systemctl enable` the
unit: appd starts it through the lease.

Antenna and gain come from `rf.conf` (appd's top-bar control). Everything else goes into
`ADSL_ARGS` in `/rfnm/apps/adsl/adsl.conf`.

## HTTP API

Port 7693, loopback by default, published as `/api/app/adsl/`.

| Route | Purpose |
|---|---|
| `GET /api` | self-description |
| `GET /status` | radio, per-channel RSSI and noise, drops, packet counters, scheduler state, `conformity`, `clock` |
| `GET /traffic` | traffic table: `amt`, `addr`, `lat`, `lon`, `alt_m`, `speed_mps`, `vrate_mps`, `track_deg`, `cat`, `fs`, `emg`, `ts_qs`, `rssi`, `ch`, `count`, `age_s`, `ts`. Fields marked unavailable in the frame are omitted |
| `GET /rx` | last 200 received frames: `time`, `t_ms`, `ch`, `freq`, `rssi`, `corrected`, `hex`, `utc_ms`, and the decoded fields, `status`, or `unknown` + `payload_type` |
| `GET /config` | current configuration |
| `POST /config` | JSON object with any of `lat`, `lon`, `alt`, `flightstate`, `addr`, `amt`, `category`, `emergency`, `interval_airborne_s`, `interval_ground_s`, `tx`. Unknown keys are ignored, bad values clamped |
| `POST /tx` | queue one manual beacon, optional `{"channel":0\|1}`. 409 unless TX is allowed and on |
| `GET /` | map web UI |

The map tiles come from OpenStreetMap. Without internet the map falls back to a plain grid.

## Own ship

The position is static: `--lat`, `--lon`, `--alt` or `POST /config`. There is no GNSS source.
Without a position the beacon sends the no-fix code. `--addr`, `--amt`, `--category` and
`--emergency` set the other beacon fields. AMT 0 without `--addr` is privacy mode (F.2.4): the
address is drawn at random at every start.

## Transmit gates

| Gate | Default | What it does |
|---|---|---|
| `TX_ALLOWED` in `/rfnm/board.conf` | absent = none | `0`, absent or unknown refuses TX. `embed` forces the embedded antenna. `1` allows any path. The backend enforces this at `start_tx()` and on every burst |
| `--tx 1` (or `ADSLD_TX=1`) | 0 | Without it the effective policy is none, whatever the board allows |
| Scheduler | always on | Listen-before-talk, 100 ms Toff per channel, 2 s quiet time after a forced frame, and no frame from a fix older than 500 ms |

`POST /config {"tx":false}` stops the transmitter. `{"tx":true}` restarts it only if the daemon
was started with TX allowed.

There is no timed arm: once the gates are open, the beacon runs until it is stopped. `--power`
defaults to 10 dBm and is capped at compile time by `ADSLD_TX_HARD_CAP_DBM`, 14 dBm, then by the
device to the channel's range. Output power is not calibrated, so the cap is a setting and does not
prove the 14 dBm ERP of the M-band.

**Transmitting ADS-L is not a toy.** Injecting fabricated aircraft into a live conspicuity
network is a flight-safety matter. Use a conducted setup with attenuation, or check your local
rules first.

## Evidence

On an RFNM Blue:

- 44 frames in 46 s with 0 CRC failures off air.
- Zero drops over 51.9 G samples at 2 MSps, at about one A53 core.
- Radiated TX was a sanity check only. There is no range figure.

## Limits

This is not a type-approved or conformance-tested device. `GET /status` lists under `conformity` what
a running station deviates from, with the clause. With TX on, that list is never empty:

- **Listen-before-talk is not effective on the RFNM.** The scheduler implements it, but only the
  selftest against the mock radio exercises it. At 2 MSps the driver refuses a TX plan while an
  RX stream is open, so `--tx 1` needs `--rx 0`, and then the channel is never sensed. The daemon
  reports this as `no_lbt`. With `--rx 1 --tx 1` the transmitter fails to start and the daemon
  stays RX-only.
- The busy threshold is relative (noise floor + 6 dB). It is not calibrated to the -79 dBm of the
  M-band. RSSI is in dBFS, not dBm.
- Output power is not calibrated.
- No GNSS: no measured fix time, flight state or quality indicators. SIL, SDA, NIC, NACp, GVA and
  NACv go out as zero. The Timestamp comes from the system clock, so its accuracy is that of the
  clock's synchronisation.
- Status declares M-band reception as full with RX on, and none with `--rx 0`.
- Only Traffic and Status are parsed. There is no O-band, no HDR, no Reed-Solomon error control
  mode, no relaying, and no Traffic or FIS-B uplink.
- Only 24- and 16-byte LEN values are captured. Other lengths never reach the decoder.
- Interoperability with other vendors' devices has not been systematically tested.

## License

MPL-2.0. `www/vendor/` holds Leaflet 1.9.4 under BSD-2-Clause, see `www/vendor/LICENSE-vendor`.

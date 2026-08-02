# clockgend — Si5510 OUT9 signal generator (REF CLK OUT connector)

Thin HTTP daemon over the kernel's `rfnm_set_siggen_freq` sysfs knob on the RFNM Blue
(Si5510 OUT9 NA-branch signal generator: 1 MHz–200.099 MHz on a 1 kHz grid; retuning is
safe during radio streams). Runs always as an enabled unit — it is `radio:"none"` and
never takes the radio lease; the thing the user enables/disables is the OUT9 OUTPUT, not
the daemon.

## HTTP API

On `127.0.0.1:7690`, published by the on-board nginx as `/api/app/clockgen/`:

- `GET /status` → `{"enabled","freq_khz","freq_min_khz","freq_max_khz","ext_ref":0|1|null,"ref_locked":0|1|null,"sysfs_ok"}`
- `POST /set` → `{"freq_khz":N}` (kHz, 1000–200099) tunes+enables; `{"freq_khz":0}` or
  `{"enabled":false}` disables.

`ext_ref` = `rfnm_ext_ref_in` (1 = a good external 10 MHz on REF CLK IN is disciplining
the board; selection is automatic + auto-revertive, external input over the on-board
TCXO); `ref_locked` = `rfnm_ref_locked` (reference-PLL lock). Both are TTL-cached (2.5 s —
each read is ~6 ms of i2c) and read null on kernels without the attributes.
`rfnm_input_status` is never read here: its LOS/OOF flags are latch-and-clear-on-read, so
polling it would destroy the diagnostic history for every other reader.

## Build / test

```sh
make
./clockgend --selftest
```

Selftest is hardware-free (sysfs redirected to temp files via `RFNM_SIGGEN_SYSFS` /
`RFNM_EXTREF_SYSFS` / `RFNM_REFLOCK_SYSFS`): 15 checks — range validation,
write/readback, JSON parse, status shape, ref-attr present/absent/garbage + cache TTL.
Run it on x86 and under `qemu-aarch64` for the cross build. Easy path:
`./rfnm-sdk build apps` from [github.com/rfnm/sdk](https://github.com/rfnm/sdk).

Reboot default is OUT9 OFF (driver behavior); the daemon does not persist frequency.

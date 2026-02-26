# Certainty Firmware

Low-jitter clock + clock-synced modulation firmware for the Uncertainty module.

## Features
- Stable transport at default `90 BPM`.
- 8 outputs with per-channel ratio, mode, ASR shape, and loop probability.
- Two-timer architecture:
  - Gate scheduler for trig edges.
  - PWM sample engine for ASR/LFO outputs.
- PWM sample rate: `8 kHz`.
- I2C follower interface for runtime control from Teletype.

## Default Configuration
- BPM: `90`
- I2C address: `0x55` (`85`)
- I2C pins: `SDA=GPIO6`, `SCL=GPIO7`
- Input CV is ignored.

Default output layout:
- OUT1: trig loop `/2`
- OUT2: ASR loop `/2`
- OUT3: trig loop `1x`
- OUT4: ASR loop `1x`
- OUT5: trig loop `2x`
- OUT6: ASR loop `2x`
- OUT7: trig loop `4x`
- OUT8: ASR loop `4x`

Default ASR shape: `A/S/R = 1/0/1` (triangle-like).
Default loop probability on all channels: `100%`.

## I2C Protocol
Each message starts with a command byte.

### `0x00` Set BPM
- Payload: `[bpm8]` or `[bpm_hi, bpm_lo]`
- Range: `20..320`

### `0x01` Set Ratio
- Payload: `[out, num, den]`
- `out`: `1..8` (compat also accepts `0..7`)
- `num`, `den`: `1..255`

### `0x02` Set Mode
- Payload: `[out, shape, run]`
- `shape`: `0=trig`, `1=asr`
- `run`: `0=one-shot`, `1=loop`

Mode aliases:
- `clk` = trig + loop (`0,1`)
- `trig` = trig + one-shot (`0,0`)
- `lfo` = asr + loop (`1,1`)
- `env` = asr + one-shot (`1,0`)

### `0x03` Set ASR Controls
- Payload: `[out, sus, skew]`
- `sus`: `0..10`
- `skew`: `0..10`

### `0x04` Trigger
- Single-output payload: `[out]`
- Legacy mask payload (still supported): `[255, mask]`

### `0x05` Set Loop Probability
- Payload: `[out, prob]`
- `prob`: `0..100`
- Applies to looping channels only (`clk`, `lfo`).
- One-shot channels (`trig`, `env`) ignore probability.

## Timing Semantics
- BPM change resets global transport phase.
- Ratio and mode changes are committed on the next beat boundary.
- Loop channels stay phase-locked to the global transport.
- Trigger commands affect one-shot channels; loop channels run from transport.

## Teletype Raw Examples
```text
IIA 85
IIS1 0 120           # BPM
IISB3 1 7 4 1        # ratio: out7 = 4/1
IISB3 2 2 1 1        # mode: out2 = lfo
IISB3 3 2 8 2        # ASR: out2 sus=8 skew=2
IISB1 4 3            # trigger out3
IISB2 5 2 70         # probability: out2 = 70%
```

Request/response is currently disabled (write-only follower behavior).

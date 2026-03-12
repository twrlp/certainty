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

Default output layout (all trig loop):
- OUT1: trig loop `/2`
- OUT2: trig loop `/2`
- OUT3: trig loop `1x`
- OUT4: trig loop `1x`
- OUT5: trig loop `2x`
- OUT6: trig loop `2x`
- OUT7: trig loop `4x`
- OUT8: trig loop `4x`

Default loop probability on all channels: `100%`.

## Teletype Ops

Device address: `0x55` (`85`). All ops are write-only.

| Op | Params | Description |
|----|--------|-------------|
| `CT.BPM bpm` | `bpm`: 20–320 | Set global BPM. Resets transport phase. |
| `CT.RAT out num den` | `out`: 1–8, `num`/`den`: 1–255 | Set output ratio as a fraction. |
| `CT.DIV out div` | `out`: 1–8, `div`: 1–255 | Convenience: set ratio to `1/div`. |
| `CT.MUL out mul` | `out`: 1–8, `mul`: 1–255 | Convenience: set ratio to `mul/1`. |
| `CT.MD.CLK out` | `out`: 1–8 | Set output to trig + loop (clock mode). |
| `CT.MD.TRIG out` | `out`: 1–8 | Set output to trig + one-shot. |
| `CT.MD.LFO out` | `out`: 1–8 | Set output to ASR + loop (LFO mode). |
| `CT.MD.ENV out` | `out`: 1–8 | Set output to ASR + one-shot (envelope). |
| `CT.ASR out sus skew` | `out`: 1–8, `sus`/`skew`: 0–10 | Set ASR sustain and skew parameters. |
| `CT.TR out` | `out`: 1–8 | Fire trigger on a one-shot output. |
| `CT.PROB out prob` | `out`: 1–8, `prob`: 0–100 | Set loop fire probability (%). |

Aliases: `CT.RATIO` = `CT.RAT`, `CT.MODE.CLK/TRIG/LFO/ENV` = `CT.MD.*`, `CT.PRB` = `CT.PROB`.

Teletype examples:
```text
CT.BPM 120              # set BPM to 120
CT.RAT 7 4 1            # out7 = 4/1
CT.DIV 1 3              # out1 = 1/3
CT.MUL 5 2              # out5 = 2/1
CT.MD.LFO 2             # out2 = ASR loop
CT.ASR 2 8 2            # out2 sus=8 skew=2
CT.MD.ENV 3             # out3 = ASR one-shot
CT.TR 3                 # fire out3
CT.PROB 4 70            # out4 loops at 70%
```

## I2C Raw Protocol

Each message starts with a command byte. Device address: `0x55`.

### `0x00` Set BPM
- Payload: `[bpm_hi, bpm_lo]`
- Range: `20..320`

### `0x01` Set Ratio
- Payload: `[out, num, den]`
- `out`: `1..8`
- `num`, `den`: `1..255`

### `0x02` Set Mode
- Payload: `[out, shape, run]`
- `shape`: `0=trig`, `1=asr`
- `run`: `0=one-shot`, `1=loop`

Mode combos:
- `clk` = trig + loop → `0, 1`
- `trig` = trig + one-shot → `0, 0`
- `lfo` = asr + loop → `1, 1`
- `env` = asr + one-shot → `1, 0`

### `0x03` Set ASR Controls
- Payload: `[out, sus, skew]`
- `sus`: `0..10`
- `skew`: `0..10`

### `0x04` Trigger
- Payload: `[out]`
- `out`: `1..8`

### `0x05` Set Loop Probability
- Payload: `[out, prob]`
- `prob`: `0..100`
- Applies to loop channels (`clk`, `lfo`); ignored on one-shot channels.

Raw Teletype IIA/IIS examples:
```text
IIA 85
IIS1 0 120           # BPM = 120 (single-byte shorthand)
IISB3 1 7 4 1        # ratio: out7 = 4/1
IISB3 2 2 1 1        # mode: out2 = lfo (shape=1, run=1)
IISB3 3 2 8 2        # ASR: out2 sus=8 skew=2
IISB1 4 3            # trigger out3
IISB2 5 2 70         # probability: out2 = 70%
```

## Timing Semantics
- BPM change resets global transport phase.
- Ratio and mode changes are committed on the next beat boundary.
- Loop channels stay phase-locked to the global transport.
- Trigger commands affect one-shot channels; loop channels run from transport.

Request/response is currently disabled (write-only follower behavior).

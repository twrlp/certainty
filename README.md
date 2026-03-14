# Certainty Firmware

Low-jitter clock + clock-synced modulation firmware for the Uncertainty module.

## Features
- Stable transport at default `90 BPM`.
- 8 outputs with per-channel ratio, mode, and loop probability.
- Clock synchronization: external clock follower with phase-lock.
- Gate scheduler for trig edges.
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
| `CT.MD.CLK out` | `out`: 1–8 | Set output to loop mode. |
| `CT.MD.TR out` | `out`: 1–8 | Set output to one-shot mode. |
| `CT.TR.P out` | `out`: 1–8 | Fire trigger on a one-shot output. |
| `CT.PROB out prob` | `out`: 1–8, `prob`: 0–100 | Set loop fire probability (%). |
| `CT.PPQN ppqn` | `ppqn`: 1–96 | Set clock PPQN (pulses per quarter-note). |

Aliases: `CT.RATIO` = `CT.RAT`, `CT.MODE.CLK/TRIG` = `CT.MD.*`, `CT.PRB` = `CT.PROB`.

Teletype examples:
```text
CT.BPM 120              # set BPM to 120
CT.RAT 7 4 1            # out7 = 4/1
CT.DIV 1 3              # out1 = 1/3
CT.MUL 5 2              # out5 = 2/1
CT.MD.CLK 2             # out2 = loop mode
CT.MD.TR 3            # out3 = one-shot mode
CT.TR.P 3                 # fire out3
CT.PROB 4 70            # out4 loops at 70%
CT.PPQN 24              # set clock to 24 PPQN (MIDI clock)
CT.PPQN 1               # set clock to 1 PPQN (one pulse per beat)
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
- Payload: `[out, run]`
- `run`: `0=one-shot`, `1=loop`

Mode values:
- `0` = one-shot (trig one-shot)
- `1` = loop (trig loop)

### `0x04` Trigger
- Payload: `[out]`
- `out`: `1..8`

### `0x05` Set Loop Probability
- Payload: `[out, prob]`
- `prob`: `0..100`
- Applies to loop channels; ignored on one-shot channels.

### `0x06` Set Clock PPQN
- Payload: `[ppqn]`
- `ppqn`: `1..96`
- Sets the expected clock resolution (pulses per quarter-note).
- Common values: `1` (1PPQ), `4` (Eurorack clock), `24` (MIDI clock), `96` (MIDI high-res).
- Changing PPQN while locked drops to re-detect state; re-locks on next 3 consistent pulses.

Raw Teletype IIA/IIS examples:
```text
IIA 85
IIS1 0 120           # BPM = 120 (single-byte shorthand)
IISB3 1 7 4 1        # ratio: out7 = 4/1
IISB2 2 2 1          # mode: out2 = loop (run=1)
IISB1 4 3            # trigger out3
IISB2 5 2 70         # probability: out2 = 70%
IISB1 6 24           # PPQN: set to 24 (MIDI clock)
IISB1 6 1            # PPQN: set to 1 (1PPQ)
```

## Timing Semantics
- BPM change resets global transport phase.
- Ratio and mode changes are committed on the next beat boundary.
- Loop channels stay phase-locked to the global transport.
- Trigger commands affect one-shot channels; loop channels run from transport.
- PPQN change: if clock is locked, drops to detect state and re-locks on next 3 consistent pulses at the new PPQN resolution. If not locked, takes effect immediately.

Request/response is currently disabled (write-only follower behavior).

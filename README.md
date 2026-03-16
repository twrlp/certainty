# Certainty

Clock and clock-synced gate firmware for the Uncertainty module. Runs on the Seeeduino XIAO RP2040.

Eight outputs, each with a configurable ratio, mode, and loop probability. Controlled over I2C from Teletype or any I2C controller. MIDI clock + transport input via the CV jack.

---

## Credits

The MIDI clock phase-lock algorithm is adapted from Emilie Gillet's [Tides](https://github.com/pichenettes/eurorack) firmware (`tides2/ramp/ramp_extractor.cc`). The per-tick warp approach — computing expected phase from a tick counter and nudging output frequency to converge — is directly inspired by her work. Thanks Emilie.

---

## Modes

### Internal clock

At boot (or after MIDI disconnects), the module runs its own clock at `90 BPM`. All loop outputs fire at their configured ratio relative to the beat. Ratio and mode changes from I2C apply on the next beat boundary.

BPM can be set over I2C. The transport resets phase on BPM change.

### MIDI clock

Connect a MIDI source to the CV jack. The firmware decodes MIDI clock via a software UART running on Core 1 (ADC at 500kHz, DMA ring buffer).

Once a MIDI clock stream is detected, the module locks to it. All loop outputs phase-lock immediately using a per-tick warp — each incoming 0xF8 nudges every output's frequency so its phase tracks the expected position derived from the tick count. There's no warm-up period; if clocks were running before you hit Play, outputs are phase-locked from the first beat.

Transport behavior:
- **Play (0xFA):** all outputs reset phase to zero and start from beat one.
- **Stop (0xFC):** all gates go low immediately. Output 8 (MIDI reset mode) fires a 10ms pulse.
- **Continue (0xFB):** resumes from the current tick position, no phase reset.

If MIDI clocks stop arriving for more than 2 seconds, the module falls back to internal clock at the last known BPM.

---

## Default output layout

| Output | Mode | Ratio |
|--------|------|-------|
| OUT1 | Loop | 1/2 |
| OUT2 | Loop | 1/2 |
| OUT3 | Loop | 1x |
| OUT4 | Loop | 1x |
| OUT5 | Loop | 2x |
| OUT6 | Loop | 2x |
| OUT7 | Loop | 4x |
| OUT8 | MIDI reset | 4x (internal clock) |

Default loop probability: 100% on all channels.

### Output modes

- **Loop:** fires a gate on each beat cycle at the configured ratio. Phase-locked to the global transport.
- **One-shot:** fires a single 10ms gate pulse each time it's triggered via I2C. Ignores the transport.
- **MIDI reset:** in internal clock mode, acts like loop at the configured ratio. In MIDI mode, skips phase accumulation and fires a single 10ms pulse on MIDI Stop (0xFC). Useful for resetting downstream CV sequencers that don't have MIDI input.

---

## I2C

Device address: `0x55` (85). Write-only.

| Cmd | Payload | Description |
|-----|---------|-------------|
| `0x00` | `[bpm_hi, bpm_lo]` or `[bpm8]` | Set BPM (20–320). Resets transport phase. |
| `0x01` | `[out, num, den]` | Set output ratio. Applied on next beat boundary. |
| `0x02` | `[out, run]` | Set output mode. `0`=one-shot, `1`=loop, `2`=MIDI reset. |
| `0x04` | `[out]` or `[0xFF, mask]` | Fire trigger on one-shot output(s). Fires immediately. |
| `0x05` | `[out, prob]` | Set loop fire probability (0–100%). |

Output index is 1-indexed (`1..8`).

---

## Teletype ops

| Op | Description |
|----|-------------|
| `CT.BPM bpm` | Set BPM (20–320). |
| `CT.RAT out num den` | Set ratio as a fraction (`num/den`). |
| `CT.DIV out div` | Shorthand: set ratio to `1/div`. |
| `CT.MUL out mul` | Shorthand: set ratio to `mul/1`. |
| `CT.MD.CLK out` | Set output to loop mode. |
| `CT.MD.TR out` | Set output to one-shot mode. |
| `CT.TR.P out` | Fire trigger on a one-shot output. |
| `CT.PROB out prob` | Set loop fire probability (%). |

Aliases: `CT.RATIO` = `CT.RAT`, `CT.MODE.CLK` = `CT.MD.CLK`, `CT.MODE.TRIG` = `CT.MD.TR`, `CT.PRB` = `CT.PROB`.

```
CT.BPM 120          // set BPM to 120
CT.RAT 1 1 2        // out1 = half speed (1/2)
CT.MUL 5 4          // out5 = 4x speed
CT.DIV 3 3          // out3 = 1/3 speed
CT.MD.TR 3          // out3 = one-shot mode
CT.TR.P 3           // fire out3
CT.MD.CLK 3         // out3 = loop mode
CT.PROB 4 70        // out4 loops at 70% probability
```

Raw IIA/IIS:
```
IIA 85
IIS1 0 120          // BPM = 120 (single byte)
IISB3 1 3 1 2       // out3 ratio = 1/2
IISB2 2 3 0         // out3 = one-shot
IISB1 4 3           // trigger out3
IISB2 5 4 70        // out4 probability = 70%
```

---

## Hardware

- **CV jack (GPIO26):** MIDI clock input. Connect any standard MIDI source via optocoupler or direct 3.3V-level signal.
- **I2C (GPIO6/7):** SDA/SCL. Pulled up internally. Address `0x55`.
- **Outputs (GPIO0–4, 27–29):** 3.3V gate outputs.

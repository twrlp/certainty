# Implementation Plan: External Clock Following + ASR Removal

## Overview

This plan refactors the firmware to remove all ASR (attack-sustain-release) PWM output machinery and replace the Bresenham-style gate scheduler loop with a unified 8 kHz ISR using a Tides-style float phase accumulator. External clock following is added via a new `clock_follower.cpp` module. The result is a simpler, more coherent codebase where every output is always a gate/trig output and inter-channel phase lock is structurally guaranteed.

---

## Step 1: Delete `src/pwm_engine.cpp` and `src/pwm_engine.h`

**Action:** Delete both files entirely.

These files contain: `pwmSampleCallback`, `configureOutputPinsForModes`, `configurePinAsPwmOutput`, `refreshAsrTimingLocked`, `deriveAsrWeights`, `computeAsrSegments`, `asrLevelFromElapsed`, `asrLevelFromTime`, `usesPwm`. All of these are specific to the ASR PWM path. After this step the firmware will not compile until all referencing files are updated. That is expected; the remaining steps resolve each reference.

---

## Step 2: Update `src/config.h`

**Remove** the three PWM hardware constants that are now unused (they were needed to configure the PWM slice clock divider and wrap value):
- `PWM_WRAP`
- `PWM_CLKDIV_INT`
- `PWM_CLKDIV_FRAC`

**Retain** `PWM_SAMPLE_RATE_HZ` and `PWM_SAMPLE_PERIOD_US`. These now describe the main ISR tick rate. No rename is required; changing the names would be pure churn with no functional benefit.

**Add** the following new constants after the existing I2C block:

```cpp
static const uint8_t  CLK_IN_PIN              = 26;
static const bool     ENABLE_CLK_FOLLOW       = true;
static const uint16_t CLK_ADC_THRESHOLD_HIGH  = 2867;
static const uint16_t CLK_ADC_THRESHOLD_LOW   = 2457;
static const uint64_t CLK_MIN_PERIOD_US       = 20000;
static const uint64_t CLK_MAX_PERIOD_US       = 3000000;
static const uint32_t CLK_HOLDOVER_MULT       = 3;
static const uint32_t CLK_DROPOUT_MULT        = 8;
```

`CLK_ADC_THRESHOLD_HIGH` and `CLK_ADC_THRESHOLD_LOW` implement hysteresis on the 12-bit ADC reading (range 0–4095) to avoid spurious edge detection from noise near the threshold.

---

## Step 3: Update `src/types.h`

### 3a. Remove `OutputShape` enum

Delete the entire `enum OutputShape : uint8_t` block (lines 9–12 in the current file). All outputs are gate outputs from this point on; no shape discrimination is needed anywhere.

### 3b. Remove `I2C_EVENT_SET_ASR` from `I2cEventType`

In the `enum I2cEventType : uint8_t` block, delete the line:
```cpp
I2C_EVENT_SET_ASR = 3,
```
The remaining values (`I2C_EVENT_TRIGGER = 4`, `I2C_EVENT_SET_PROB = 5`) can stay at their current numeric values to preserve wire compatibility with existing controllers, or be renumbered; that is a protocol decision. The plan is to leave them at 4 and 5.

### 3c. Replace `OutputState` struct

Replace the entire `struct OutputState` with:

```cpp
struct OutputState {
  uint8_t  pin;
  Ratio    ratio;
  OutputRun run;

  uint8_t  loopProbPercent;

  bool     ratioPending;
  uint64_t pendingApplyUs;
  Ratio    pendingRatio;
  bool     pendingRunPending;
  uint64_t pendingRunApplyUs;
  OutputRun pendingRun;
  uint16_t pendingTriggerCount;

  uint64_t nextFallUs;
  bool     gateHigh;
  uint32_t gateRises;
  uint32_t gateFalls;

  float    phase;
  float    freq;
};
```

Fields removed vs. the current struct:
- `OutputShape shape` — no shapes anymore
- `uint64_t periodUs` — replaced by `freq`
- `uint64_t lfoAnchorUs`, `uint64_t loopCycleIndex`, `bool loopCycleActive` — LFO tracking, ASR only
- `uint8_t asrSus`, `asrSkew`, `asrA`, `asrS`, `asrR` — ASR parameters
- `bool behaviorPending`, `uint64_t behaviorPendingApplyUs`, `OutputShape pendingShape` — shape-change pending logic
- `uint16_t pendingTriggerCount` is kept; `pendingRun` is kept
- `bool asrOneShotActive`, `uint64_t asrOneShotStartUs/DurationUs/AttackUs/SustainUs/ReleaseUs` — ASR one-shot
- `uint64_t asrAttackUs`, `asrSustainUs`, `asrReleaseUs` — ASR segment timing
- `uint64_t asrOneShotAttackUs`, `asrOneShotSustainUs`, `asrOneShotReleaseUs`
- `uint64_t nextRiseUs` — ISR handles rises via phase accumulator; no scheduled rise needed
- `uint8_t pwmSlice`, `pwmChannel` — PWM hardware config
- `uint16_t lastPwmLevel` — PWM output level cache
- `uint32_t lfoUpdates` — ASR update counter

Fields added:
- `float phase` — current phase in [0.0, 1.0); incremented every ISR tick
- `float freq` — phase advance per ISR tick; derived from ratio and beat period

Note: `behaviorPending` is renamed to `pendingRunPending` and narrowed to run-mode changes only (ONE_SHOT ↔ LOOP). The corresponding timestamp is `pendingRunApplyUs`.

### 3d. Add `ClockFollowMode` enum and `ClockFollowerState` struct

Add after `TransportState`:

```cpp
enum ClockFollowMode : uint8_t {
  CLK_FOLLOW_INACTIVE  = 0,
  CLK_FOLLOW_DETECTING = 1,
  CLK_FOLLOW_LOCKED    = 2,
  CLK_FOLLOW_HOLDOVER  = 3,
};

struct ClockFollowerState {
  ClockFollowMode mode;
  bool            adcHigh;
  uint64_t        lastEdgeUs;
  uint64_t        measuredPeriodUs;
  uint32_t        fallbackBpm;
  uint64_t        inputEdgeCount;
  uint32_t        totalEdges;
};
```

Field notes:
- `adcHigh` is the hysteresis state; `true` means the ADC reading was last seen above `CLK_ADC_THRESHOLD_HIGH`, so the next event fires when it drops below `CLK_ADC_THRESHOLD_LOW`, and vice versa. This prevents multiple edges from a single slow-rising input.
- `lastEdgeUs` is the `time_us_64()` value at the most recent detected rising edge.
- `measuredPeriodUs` is the inter-edge interval computed on the DETECTING→LOCKED and LOCKED→LOCKED transitions.
- `fallbackBpm` is the BPM to restore when dropout occurs. Kept in sync with the last I2C BPM command received while not locked.
- `inputEdgeCount` counts edges since the last lock-on; used for diagnostics.
- `totalEdges` is a lifetime edge counter.

---

## Step 4: Update `src/module_state.h`

### Remove from `ModuleState`:
- `repeating_timer pwmTimer`
- `volatile uint32_t pwmRuns`
- `uint32_t pwmSliceMask`
- `bool pwmTimerRunning`
- `volatile uint32_t i2cAsrAppliedCount`

### Add to `ModuleState`:
```cpp
ClockFollowerState clockFollower;
repeating_timer    mainTimer;
bool               mainTimerRunning;
```

The `#include "pico/time.h"` is already present, so `repeating_timer` is available.

---

## Step 5: Create `src/clock_follower.h`

New file:

```cpp
#pragma once
#include <Arduino.h>
#include "pico/time.h"

namespace certainty {
void initClockFollower();
bool mainCallback(repeating_timer *rt);
}
```

`initClockFollower` configures the ADC. `mainCallback` is the 8 kHz ISR registered with `add_repeating_timer_us`.

---

## Step 6: Create `src/clock_follower.cpp`

This is the most substantial new file. It contains:

### 6a. Includes

```cpp
#include "clock_follower.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"
#include "rng.h"
#include "transport.h"
```

### 6b. `initClockFollower()`

Initializes ADC hardware for the clock input pin and zeroes the `ClockFollowerState`:

```cpp
void initClockFollower() {
  adc_init();
  adc_gpio_init(CLK_IN_PIN);
  ClockFollowerState &cf = g_module.clockFollower;
  cf.mode            = CLK_FOLLOW_INACTIVE;
  cf.adcHigh         = false;
  cf.lastEdgeUs      = 0;
  cf.measuredPeriodUs = 0;
  cf.fallbackBpm     = DEFAULT_BPM;
  cf.inputEdgeCount  = 0;
  cf.totalEdges      = 0;
}
```

Note: `adc_select_input()` must be called before each `adc_read()` in the ISR; it is not set here because the ISR always selects the input channel explicitly.

### 6c. `applyExternalClockLocked(uint64_t measuredPeriodUs, uint64_t nowUs)`

Updates the transport and all output `freq` values to reflect a newly measured input period. Does NOT reset phases (phases run freely while locked; they are only reset on lock-on and dropout).

```
measuredPeriodTicks = measuredPeriodUs / PWM_SAMPLE_PERIOD_US
transport.beatPeriodUs = measuredPeriodUs
transport.bpm = clampBpm(60000000 / measuredPeriodUs)
transport.anchorUs = nowUs
for each output i:
    out.freq = float(out.ratio.num) / (float(out.ratio.den) * float(measuredPeriodTicks))
```

`measuredPeriodTicks` is a `float` derived from the integer microsecond period. The division converts the real-world period into ISR-tick units, which is the unit of `freq`.

This function is called both on DETECTING→LOCKED (where phases are reset before this call) and on every LOCKED→LOCKED edge (where phases are not reset).

### 6d. `onEdgeDetected(ClockFollowerState &cf, uint64_t nowUs)`

Implements the state machine transitions triggered by a detected rising edge on the clock input.

```
switch cf.mode:

  case CLK_FOLLOW_INACTIVE:
    cf.lastEdgeUs = nowUs
    cf.mode = CLK_FOLLOW_DETECTING
    reset all out.phase = 0
    // No freq/transport change yet; need a second edge to measure period

  case CLK_FOLLOW_DETECTING:
    interval = nowUs - cf.lastEdgeUs
    if interval < CLK_MIN_PERIOD_US or interval > CLK_MAX_PERIOD_US:
      // Out of range; stay in DETECTING, reset anchor
      cf.lastEdgeUs = nowUs
    else:
      cf.measuredPeriodUs = interval
      cf.inputEdgeCount = 0
      cf.totalEdges++
      // Reset phases, then compute freq
      for each output: out.phase = 0
      applyExternalClockLocked(cf.measuredPeriodUs, nowUs)
      cf.lastEdgeUs = nowUs
      cf.mode = CLK_FOLLOW_LOCKED

  case CLK_FOLLOW_LOCKED:
    interval = nowUs - cf.lastEdgeUs
    // Accept any interval; use it to update freq (allows tempo changes)
    cf.measuredPeriodUs = interval
    cf.lastEdgeUs = nowUs
    cf.inputEdgeCount++
    cf.totalEdges++
    applyExternalClockLocked(cf.measuredPeriodUs, nowUs)
    // mode stays LOCKED

  case CLK_FOLLOW_HOLDOVER:
    // Re-entering DETECTING; do not compute freq yet
    cf.lastEdgeUs = nowUs
    cf.measuredPeriodUs = 0
    cf.mode = CLK_FOLLOW_DETECTING
```

### 6e. `mainCallback(repeating_timer *rt)` — the 8 kHz ISR

This is the single repeating timer callback. It must return `true` to keep firing.

```
1. ADC EDGE DETECTION
   adc_select_input(channel for CLK_IN_PIN)
   raw = adc_read()
   rising edge detected if !cf.adcHigh and raw >= CLK_ADC_THRESHOLD_HIGH:
     cf.adcHigh = true
     nowUs = time_us_64()
     irqState = save_and_disable_interrupts()
     onEdgeDetected(cf, nowUs)
     restore_interrupts(irqState)
   falling hysteresis: if cf.adcHigh and raw < CLK_ADC_THRESHOLD_LOW:
     cf.adcHigh = false
   (No action on falling edge of clock input; only rising edges drive the state machine)

2. PHASE ACCUMULATION AND GATE FIRING
   nowUs = time_us_64()  (re-read after potential ISR work above)
   needsReschedule = false
   irqState = save_and_disable_interrupts()
   for each output i where out.run == OUTPUT_RUN_LOOP:
     out.phase += out.freq
     if out.phase >= 1.0f:
       out.phase -= 1.0f
       // Safety: extreme ratios could overshoot past 2.0 in one tick
       if out.phase >= 1.0f:
         out.phase = fmodf(out.phase, 1.0f)
       if !out.gateHigh and sampleLoopRetrigger(out.loopProbPercent):
         gpio_put(out.pin, 1)
         out.gateHigh = true
         out.gateRises++
         out.nextFallUs = nowUs + GATE_PULSE_US
         needsReschedule = true
   if needsReschedule:
     rescheduleGateAlarmLocked()
   restore_interrupts(irqState)

3. HOLDOVER / DROPOUT CHECK
   if cf.mode == CLK_FOLLOW_LOCKED or cf.mode == CLK_FOLLOW_HOLDOVER:
     timeSinceEdge = nowUs - cf.lastEdgeUs
     if cf.mode == CLK_FOLLOW_LOCKED and
        timeSinceEdge > CLK_HOLDOVER_MULT * cf.measuredPeriodUs:
       cf.mode = CLK_FOLLOW_HOLDOVER
       // Transport keeps running; freq is frozen (no update)
     else if cf.mode == CLK_FOLLOW_HOLDOVER and
             timeSinceEdge > CLK_DROPOUT_MULT * cf.measuredPeriodUs:
       cf.mode = CLK_FOLLOW_INACTIVE
       irqState = save_and_disable_interrupts()
       applyBpmAndReset(cf.fallbackBpm)  // resets phases, recomputes freq
       restore_interrupts(irqState)

4. return true
```

**Interrupt safety note:** The gate firing block (step 2) runs inside `save_and_disable_interrupts` to prevent the gate alarm ISR from observing partially written `gateHigh`/`nextFallUs`. The ADC read and holdover check (steps 1 and 3) do not need the lock because they only write to the `ClockFollowerState` which is not read by the gate alarm.

**ADC channel selection:** GPIO 26 is ADC input 0. The call is `adc_select_input(0)` before `adc_read()`.

---

## Step 7: Update `src/transport.cpp`

### 7a. Remove includes

Remove:
```cpp
#include "hardware/pwm.h"
#include "pwm_engine.h"
```

### 7b. Simplify `resetTransportLocked`

The new implementation:

```cpp
void resetTransportLocked(uint32_t bpm, uint64_t nowUs) {
  if (g_module.gateAlarmId >= 0) {
    cancel_alarm(g_module.gateAlarmId);
    g_module.gateAlarmId = -1;
  }

  g_module.transport.bpm = clampBpm(bpm);
  g_module.transport.beatPeriodUs = 60000000ull / g_module.transport.bpm;
  g_module.transport.anchorUs = nowUs + START_DELAY_US;
  g_module.transport.resetCount++;

  const float beatPeriodTicks =
      float(g_module.transport.beatPeriodUs) / float(PWM_SAMPLE_PERIOD_US);

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.phase           = 0.0f;
    out.freq            = float(out.ratio.num) / (float(out.ratio.den) * beatPeriodTicks);
    out.ratioPending    = false;
    out.pendingApplyUs  = g_module.transport.anchorUs;
    out.pendingRatio    = out.ratio;
    out.pendingRunPending    = false;
    out.pendingRunApplyUs    = g_module.transport.anchorUs;
    out.pendingRun      = out.run;
    out.pendingTriggerCount = 0;
    out.nextFallUs      = g_module.transport.anchorUs + GATE_PULSE_US;
    out.gateHigh        = false;
    out.gateRises       = 0;
    out.gateFalls       = 0;
    gpio_put(out.pin, 0);
  }

  scheduleNextAlarmLocked();
}
```

Removed vs. current: all ASR field assignments (`lfoAnchorUs`, `loopCycleIndex`, `loopCycleActive`, `asrOneShotActive`, `asrOneShotStartUs`, `asrOneShotDurationUs`, all `asr*Us` fields), `nextRiseUs`, `lfoUpdates`, `lastPwmLevel`, `periodUs`, the `pwm_set_gpio_level` branch, and the `refreshAsrTimingLocked` call.

Added: `out.phase = 0.0f`, `out.freq = ...` computation.

### 7c. Keep `periodFromRatio` or remove it

`periodFromRatio` is called in `processDueConfigChanges` in `commands.cpp` (current code). After the refactor, ratio changes must update `out.freq` instead. If no other caller remains after Step 9, delete `periodFromRatio` from both `transport.cpp` and `transport.h`. If the function is useful for the gate scheduler's fall duration calculation (it is not — `GATE_PULSE_US` is a fixed constant), it can be removed.

Decision: remove `periodFromRatio` entirely. Its only uses are inside the ASR/trig period update paths that are being replaced.

### 7d. Add `applyExternalClockLocked` declaration to `transport.h`

Add to `transport.h`:
```cpp
void applyExternalClockLocked(uint64_t measuredPeriodUs, uint64_t nowUs);
```

The implementation lives in `clock_follower.cpp` (not `transport.cpp`) because it is tightly coupled to the clock follower state. The declaration in `transport.h` makes it available to `commands.cpp` and `app.cpp` without needing to include `clock_follower.h` everywhere.

Alternatively, declare it in `clock_follower.h` and include that where needed. Either works; putting it in `clock_follower.h` is cleaner because that is where the implementation lives. Include `clock_follower.h` in `commands.cpp` and `app.cpp`.

### 7e. Keep `alignToGlobalPhaseGridAtOrAfterLocked` and `nextBeatBoundaryAfterLocked`

These are still used by the one-shot scheduling path in `commands.cpp` (ratio pending applies at the next beat boundary).

---

## Step 8: Update `src/gate_scheduler.cpp`

### 8a. Remove include

Remove:
```cpp
#include "pwm_engine.h"
```

### 8b. Simplify `scheduleNextAlarmLocked`

Remove the shape check (`out.shape == OUTPUT_SHAPE_TRIG`) — all outputs are gate outputs now. Remove the `nextRiseUs` check — the ISR handles loop rises. The new logic:

```cpp
void scheduleNextAlarmLocked() {
  const uint64_t nowUs = time_us_64();
  uint64_t nextDueUs = UINT64_MAX;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const OutputState &out = g_module.outputs[i];
    // Fall for any currently-high gate
    if (out.gateHigh && out.nextFallUs < nextDueUs) {
      nextDueUs = out.nextFallUs;
    }
    // Immediate one-shot rise if pending
    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT &&
        out.pendingTriggerCount > 0 && nowUs < nextDueUs) {
      nextDueUs = nowUs;
    }
  }

  if (nextDueUs == UINT64_MAX) {
    g_module.gateAlarmId = -1;
    return;
  }

  g_module.gateAlarmId = add_alarm_at(absoluteFromUs(nextDueUs), gateAlarmCallback, nullptr, true);
  if (g_module.gateAlarmId < 0) {
    g_module.gateSchedulerMisses++;
  }
}
```

### 8c. Simplify `gateAlarmCallback`

Remove:
- The `behaviorPending`/`pendingShape == OUTPUT_SHAPE_ASR` block (lines 96–104 in current file) — no shapes, no pending shape transitions
- The `OUTPUT_RUN_LOOP` fall → advance rise block (lines 119–147 in current file) — the ISR now owns loop rise scheduling

The new callback body:

```cpp
int64_t gateAlarmCallback(alarm_id_t id, void *user_data) {
  (void)id;
  (void)user_data;

  g_module.gateSchedulerRuns++;
  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];

    // One-shot rise
    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0) {
      gpio_put(out.pin, 1);
      out.gateHigh = true;
      out.gateRises++;
      out.nextFallUs = nowUs + GATE_PULSE_US;
      out.pendingTriggerCount--;
    }

    // Fall for any output whose gate is high and fall time has arrived
    if (out.gateHigh && (int64_t)(nowUs - out.nextFallUs) >= 0) {
      gpio_put(out.pin, 0);
      out.gateHigh = false;
      out.gateFalls++;
      // No rescheduling of loop rises here; the ISR handles those
    }
  }

  scheduleNextAlarmLocked();
  return 0;
}
```

### 8d. Simplify `ensureGateSchedulerRunning`

Remove the `hasTrigOutput` check — all outputs are gate outputs, so the condition is always true. Simplify:

```cpp
void ensureGateSchedulerRunning() {
  const uint32_t irqState = save_and_disable_interrupts();
  if (g_module.gateAlarmId < 0) {
    scheduleNextAlarmLocked();
  }
  restore_interrupts(irqState);
}
```

---

## Step 9: Update `src/i2c_ingress.cpp` and I2C protocol

### I2C wire protocol changes

The protocol uses fixed command bytes. After this refactor the command table is:

| cmd byte | name | wire format | description |
|----------|------|-------------|-------------|
| `0x00` | CT.BPM | `[0x00, bpm]` or `[0x00, bpm_hi, bpm_lo]` | Set BPM. While LOCKED/HOLDOVER: stored as fallbackBpm only. |
| `0x01` | CT.DIV | `[0x01, out, num, den]` | Set output ratio (e.g. num=1 den=4 for ÷4). |
| `0x02` | CT.OS / CT.CLK | `[0x02, out, run]` | Set run mode: `run=0` → one-shot (CT.OS), `run=1` → loop/clock (CT.CLK). **3 bytes, not 4.** Shape byte removed. |
| `0x03` | *(removed)* | — | Was SET_ASR. Now rejected silently (decoder returns false). |
| `0x04` | CT.TR | `[0x04, out]` or `[0x04, 0xFF, mask]` | Trigger one-shot output(s). |
| `0x05` | CT.PROB | `[0x05, out, prob]` | Set loop retrigger probability (0–100). |

**cmd 0x02 wire format change:** was `[0x02, out, shape, run]` (4 bytes). Now `[0x02, out, run]` (3 bytes). The shape byte is gone because all outputs are always gate outputs.

**cmd 0x03 removed:** The decoder must return `false` for any frame beginning with `0x03`. The `I2C_EVENT_SET_ASR` enum value is removed. Any controller sending cmd 0x03 will have it silently ignored (decoded as error, increments `i2cErrorCount`).

### 9a-i2c. Update `decodeI2cEvent` in `src/i2c_ingress.cpp`

Replace the cmd 0x02 block:
```cpp
// cmd 2 = set run mode: [2, out, run]  (CT.OS=0, CT.CLK=1)
if (raw[0] == 0x02 && len >= 3) {
  uint8_t outIndex = 0;
  if (!decodeOutIndex(raw[1], &outIndex)) return false;
  const uint8_t runRaw = raw[2];
  if (runRaw > (uint8_t)OUTPUT_RUN_LOOP) return false;
  event->type  = I2C_EVENT_SET_MODE;
  event->out   = outIndex;
  event->a     = 0;
  event->b     = runRaw;
  event->mask  = 0;
  event->count = 0;
  return true;
}
```

Delete the entire cmd 0x03 block (SET_ASR). Any frame with `raw[0] == 0x03` will fall through to `return false`, incrementing `i2cErrorCount`.

---

## Step 10: Update `src/commands.cpp`

### 10a. Remove includes

Remove:
```cpp
#include "hardware/pwm.h"
#include "pwm_engine.h"
```

Add:
```cpp
#include "clock_follower.h"
```

### 9b. Remove `I2C_EVENT_SET_ASR` case

Delete the entire `case I2C_EVENT_SET_ASR:` block (lines 123–148 in current file), including the call to `deriveAsrWeights` and `refreshAsrTimingLocked`.

### 9c. Update `I2C_EVENT_SET_BPM` case

Replace current unconditional `applyBpmAndReset` with the LOCKED/HOLDOVER guard:

```cpp
case I2C_EVENT_SET_BPM: {
  const uint32_t bpm = event.count;
  const ClockFollowMode cfMode = g_module.clockFollower.mode;
  if (cfMode == CLK_FOLLOW_LOCKED || cfMode == CLK_FOLLOW_HOLDOVER) {
    g_module.clockFollower.fallbackBpm = clampBpm(bpm);
  } else {
    if (bpm != g_module.transport.bpm) {
      applyBpmAndReset(bpm);
      g_module.clockFollower.fallbackBpm = clampBpm(bpm);
      g_module.i2cAppliedCount++;
    }
  }
  break;
}
```

### 9d. Update `I2C_EVENT_SET_MODE` case

Remove all shape-related handling. The only valid parameter is `run` (ONE_SHOT or LOOP). Remove the `shape` parameter from the I2C frame parsing (or ignore it). Rename `behaviorPending` → `pendingRunPending`, `behaviorPendingApplyUs` → `pendingRunApplyUs`.

New case:

```cpp
case I2C_EVENT_SET_MODE: {
  const uint8_t outIndex = event.out;
  const OutputRun run = (OutputRun)event.b;
  if (outIndex >= NUM_OUTPUTS ||
      (run != OUTPUT_RUN_ONE_SHOT && run != OUTPUT_RUN_LOOP)) {
    g_module.i2cErrorCount++;
    break;
  }
  const uint64_t nowUs = time_us_64();
  const uint32_t irqState = save_and_disable_interrupts();
  OutputState &out = g_module.outputs[outIndex];
  out.pendingRun = run;
  out.pendingRunApplyUs = nextBeatBoundaryAfterLocked(nowUs);
  out.pendingRunPending = true;
  restore_interrupts(irqState);
  g_module.i2cModeAppliedCount++;
  break;
}
```

### 9e. Update `I2C_EVENT_SET_RATIO` case

After setting `out.ratioPending`, also immediately update `out.freq` if the new ratio can be computed now (before the pending apply time). The `freq` update at the pendingApplyUs boundary happens in `processDueConfigChanges`, so leave the existing deferred apply mechanism in place. However, remove the `refreshAsrTimingLocked` call that currently happens in `processDueConfigChanges`:

In `processDueConfigChanges`, the `ratioDue` block currently calls `refreshAsrTimingLocked` and updates `periodUs`. Replace with:

```cpp
if (ratioDue) {
  out.ratio = out.pendingRatio;
  // Recompute freq from current transport or measured external period
  const ClockFollowMode cfMode = g_module.clockFollower.mode;
  float basePeriodTicks;
  if (cfMode == CLK_FOLLOW_LOCKED || cfMode == CLK_FOLLOW_HOLDOVER) {
    basePeriodTicks = float(g_module.clockFollower.measuredPeriodUs)
                      / float(PWM_SAMPLE_PERIOD_US);
  } else {
    basePeriodTicks = float(g_module.transport.beatPeriodUs)
                      / float(PWM_SAMPLE_PERIOD_US);
  }
  out.freq = float(out.ratio.num) / (float(out.ratio.den) * basePeriodTicks);
  out.ratioPending = false;
  out.gateHigh = false;
  gpio_put(out.pin, 0);
  channelAffectsGateSchedule = true;
}
```

Note: when `out.run == OUTPUT_RUN_LOOP`, do not reset `out.phase` here. The phase continues freely; the new `freq` takes effect immediately. This means the output may briefly fire slightly off from the new ideal grid, but it will self-correct within one cycle. This is consistent with the design principle that phases only reset on transport reset or lock-on.

### 9f. Update `processDueConfigChanges` for `pendingRunPending`

Replace the `behaviorDue` check and its body with a `runDue` check:

```cpp
const bool runDue = out.pendingRunPending && (int64_t)(nowUs - out.pendingRunApplyUs) >= 0;
```

When `runDue`:
- Set `out.run = out.pendingRun`
- Set `out.pendingRunPending = false`
- If transitioning to ONE_SHOT: ensure `out.gateHigh = false`, `gpio_put(out.pin, 0)`, set `channelAffectsGateSchedule = true`
- If transitioning to LOOP: clear `out.pendingTriggerCount = 0`, set `channelAffectsGateSchedule = true`

Remove the entire `oldShape`/`newShape` branch from the current `behaviorDue` block — it is all ASR-related.

### 9g. Update `applyTriggerEvent`

Remove the `out.shape == OUTPUT_SHAPE_TRIG` / `out.pendingShape == OUTPUT_SHAPE_TRIG` guards — all outputs are gate outputs. Remove the `out.behaviorPending && out.pendingRun == OUTPUT_RUN_ONE_SHOT` branch. Simplify:

```cpp
static void applyTriggerEvent(uint8_t mask, uint16_t count) {
  if (mask == 0 || count == 0) return;
  uint32_t appliedCount = 0;
  bool gateScheduleChanged = false;
  const uint32_t irqState = save_and_disable_interrupts();
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if ((mask & (1u << i)) == 0) continue;
    OutputState &out = g_module.outputs[i];
    if (out.run == OUTPUT_RUN_ONE_SHOT) {
      const uint32_t sum = (uint32_t)out.pendingTriggerCount + (uint32_t)count;
      out.pendingTriggerCount = (sum > 0xFFFFu) ? 0xFFFFu : (uint16_t)sum;
      if (sum > 0xFFFFu) g_module.i2cTriggerSaturatedCount++;
      appliedCount += count;
      gateScheduleChanged = true;
    }
  }
  if (gateScheduleChanged) rescheduleGateAlarmLocked();
  restore_interrupts(irqState);
  g_module.i2cTriggerAppliedCount += appliedCount;
}
```

---

## Step 10: Update `src/app.cpp`

### 10a. Update includes

Remove:
```cpp
#include "pwm_engine.h"
```

Add:
```cpp
#include "clock_follower.h"
#include "hardware/adc.h"
```

### 10b. Remove `DEFAULT_SHAPES` array

Delete:
```cpp
static const OutputShape DEFAULT_SHAPES[NUM_OUTPUTS] = { ... };
```

### 10c. Simplify `appSetup()` output initialization loop

Replace the current loop body with:

```cpp
for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
  OutputState &out = g_module.outputs[i];
  out.pin               = OUTPUT_PINS[i];
  out.ratio             = DEFAULT_RATIOS[i];
  out.run               = DEFAULT_RUNS[i];
  out.loopProbPercent   = 100;
  out.ratioPending      = false;
  out.pendingApplyUs    = 0;
  out.pendingRatio      = {1, 1};
  out.pendingRunPending = false;
  out.pendingRunApplyUs = 0;
  out.pendingRun        = out.run;
  out.pendingTriggerCount = 0;
  out.nextFallUs        = 0;
  out.gateHigh          = false;
  out.gateRises         = 0;
  out.gateFalls         = 0;
  out.phase             = 0.0f;
  out.freq              = 0.0f;
  configurePinAsGateOutput(out.pin);
}
```

This replaces the current loop (which initializes 25+ fields including all ASR and PWM fields) and the subsequent `configureOutputPinsForModes()` call (which is deleted).

### 10d. Replace PWM timer registration with main timer

Remove:
```cpp
g_module.pwmTimerRunning =
    add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, pwmSampleCallback, nullptr, &g_module.pwmTimer);
```

Replace `applyBpmAndReset(DEFAULT_BPM)` section with:

```cpp
applyBpmAndReset(DEFAULT_BPM);

initClockFollower();
g_module.clockFollower.fallbackBpm = DEFAULT_BPM;
g_module.mainTimerRunning =
    add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, mainCallback, nullptr, &g_module.mainTimer);
```

The negative sign on the period instructs the Pico SDK to use the interval as a delay from the end of the previous callback rather than from the start, which provides more uniform timing under load.

---

## Step 11: Update `src/pwm_engine.h` references in remaining files

After deleting `pwm_engine.h`, verify no other files include it. Files known to include it based on current state:
- `gate_scheduler.cpp` — removed in Step 8a
- `transport.cpp` — removed in Step 7a
- `commands.cpp` — removed in Step 9a
- `app.cpp` — removed in Step 10a

Run a search for any remaining `#include "pwm_engine.h"` references before building.

Similarly, search for remaining uses of removed symbols: `OUTPUT_SHAPE_ASR`, `OUTPUT_SHAPE_TRIG` (now implicit), `periodUs`, `nextRiseUs`, `behaviorPending`, `refreshAsrTimingLocked`, `pwmSampleCallback`, `configureOutputPinsForModes`.

---

## File Change Summary

| File | Action |
|------|--------|
| `src/pwm_engine.cpp` | Delete |
| `src/pwm_engine.h` | Delete |
| `src/clock_follower.h` | Create (new) |
| `src/clock_follower.cpp` | Create (new) |
| `src/config.h` | Remove `PWM_WRAP`, `PWM_CLKDIV_INT`, `PWM_CLKDIV_FRAC`; add 8 CLK constants |
| `src/types.h` | Remove `OutputShape` enum; remove `I2C_EVENT_SET_ASR`; replace `OutputState`; add `ClockFollowMode` + `ClockFollowerState` |
| `src/module_state.h` | Remove `pwmTimer`, `pwmRuns`, `pwmSliceMask`, `pwmTimerRunning`, `i2cAsrAppliedCount`; add `clockFollower`, `mainTimer`, `mainTimerRunning` |
| `src/transport.h` | Add `applyExternalClockLocked` declaration (or put in `clock_follower.h`); remove `periodFromRatio`; remove `refreshAsrTimingLocked` |
| `src/transport.cpp` | Remove `hardware/pwm.h` + `pwm_engine.h` includes; rewrite `resetTransportLocked`; remove `periodFromRatio`; keep `alignToGlobalPhaseGridAtOrAfterLocked` + `nextBeatBoundaryAfterLocked` |
| `src/gate_scheduler.cpp` | Remove `pwm_engine.h` include; rewrite `scheduleNextAlarmLocked` (no `nextRiseUs`); rewrite `gateAlarmCallback` (no loop rise block, no shape checks); simplify `ensureGateSchedulerRunning` |
| `src/commands.cpp` | Remove `hardware/pwm.h` + `pwm_engine.h`; add `clock_follower.h`; remove `I2C_EVENT_SET_ASR`; guard `I2C_EVENT_SET_BPM`; simplify `I2C_EVENT_SET_MODE`; update `processDueConfigChanges` to use `pendingRunPending` and recompute `freq` on ratio change; simplify `applyTriggerEvent` |
| `src/app.cpp` | Remove `pwm_engine.h`; add `clock_follower.h` + `hardware/adc.h`; remove `DEFAULT_SHAPES`; simplify output init loop; remove `configureOutputPinsForModes` call; replace PWM timer with `mainCallback` registration; call `initClockFollower` |

---

## Design Principles (Reference)

- **No per-edge phase correction.** Float accumulator drift is below 10^-8 after hours; correction logic caused missed beats for division outputs and is omitted.
- **Freq updated simultaneously for all outputs on each edge.** This is the sole mechanism that guarantees inter-channel phase lock. All `freq[i]` are derived from the same `measuredPeriodUs`, so their proportional relationship is exact and permanent.
- **Phases reset to 0 only on:** `applyBpmAndReset` (transport reset, internal mode tempo change, external clock dropout) and external clock lock-on (DETECTING→LOCKED transition). Not on ratio changes, not on run mode changes, not on LOCKED→LOCKED edges.
- **Gate scheduler owns falls and one-shot rises; ISR owns loop rises.** There is no overlap and no race because only the ISR writes `gateHigh = true` for loop outputs, and it does so inside `save_and_disable_interrupts`.
- **Interrupt safety:** The ISR wraps gate state mutations in `save_and_disable_interrupts` to prevent the gate alarm callback from observing torn `gateHigh`/`nextFallUs` pairs.
- **Single timer.** There is exactly one `add_repeating_timer_us` call in the entire firmware after this refactor. The `mainCallback` at 8 kHz replaces both `pwmSampleCallback` and the (previously planned separate) clock follower timer.

---

## Build Verification

```bash
arduino-cli compile --fqbn "rp2040:rp2040:seeed_xiao_rp2040" .
```

Expected: zero errors, zero warnings about unused variables or missing symbols. The binary size should decrease relative to the current build due to the removal of the PWM hardware driver and ASR envelope math.

---

## Manual Bench Tests

1. **Internal clock at 90 BPM:** All 8 outputs fire at correct rates; phases are locked relative to each other from the moment of the first beat.
2. **External clock at 120 BPM (24 ppqn source):** Outputs track with jitter ≤ 125 µs (one ISR tick).
3. **÷4 output:** Fires on every 4th input pulse, exactly coincident with the pulse (within ±125 µs).
4. **×4 output:** Fires 4 times per input pulse, evenly distributed within the period.
5. **Tempo change on external source:** Outputs retrack within 1 pulse; no phase slip between channels.
6. **Cable unplug:** Module stays in holdover for approximately `CLK_HOLDOVER_MULT` beat periods, then transitions to dropout at `CLK_DROPOUT_MULT` beat periods; snaps back to `fallbackBpm`.
7. **Cable replug:** Re-locks within 2 input pulses (DETECTING requires exactly 2 valid edges).
8. **I2C BPM command while LOCKED:** Command is stored as `fallbackBpm` only; transport is not disturbed.
9. **Ratio change while LOCKED:** `freq` is recomputed immediately at the next beat boundary; output smoothly adopts new rate without phase slip relative to other channels.
10. **I2C trigger (one-shot mode):** Gate fires within one gate scheduler alarm cycle (≤ 1 ms typical latency).

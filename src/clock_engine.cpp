#include "clock_engine.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "commands.h"
#include "i2c_ingress.h"
#include "midi_uart.h"
#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"
#include "rng.h"
#include "transport.h"

namespace certainty {

static void computeHumanizerOffset(OutputState &out, uint8_t index) {
  HumanizerState &h = out.humanizer;

  // Fraction of beat for this output's ratio (cycle period = den/num beats)
  const float frac = float(out.ratio.den) / float(out.ratio.num);
  const float scale = cbrtf(frac);

  // Clock error (pink noise, long-range correlated)
  const float timingError = pinkNoiseSample(h.pink) * h.alpha * scale;

  // Motor error (white noise, uncorrelated)
  const float motorError = gaussianWhiteNoise() * h.motorStd;

  // Listening correction: how much this output reacts to others' timing
  float correction = 0.0f;
  float totalInfluence = 0.0f;

  for (uint8_t j = 0; j < NUM_OUTPUTS; j++) {
    if (j == index) continue;
    const OutputState &other = g_module.outputs[j];
    if (other.run == OUTPUT_RUN_ONE_SHOT) continue;

    const float diff = other.humanizer.accumulator - h.accumulator;
    correction += diff * other.humanizer.influence;
    totalInfluence += other.humanizer.influence;
  }

  if (totalInfluence > 0.0f) {
    correction = (correction / totalInfluence) * h.listening;
  } else {
    correction = 0.0f;
  }

  // Accumulate and clamp
  h.accumulator += timingError + motorError + correction;
  if (h.accumulator > HUMANIZE_MAX_OFFSET_MS) h.accumulator = HUMANIZE_MAX_OFFSET_MS;
  if (h.accumulator < -HUMANIZE_MAX_OFFSET_MS) h.accumulator = -HUMANIZE_MAX_OFFSET_MS;
}

bool mainCallback(repeating_timer *rt) {
  (void)rt;
  const uint64_t nowUs = time_us_64();
  g_module.midi.dbgTickCounter++;

  // 1. Decoded MIDI bytes from Core 1 SPSC ring buffer
  if (ENABLE_MIDI_CLOCK) {
    drainMidiMessages(nowUs);
  }

  // 2. I2C events and scheduled config changes
  if (ENABLE_I2C_BPM) {
    processI2cEvents(nowUs);
    processDueConfigChanges(nowUs);
  }

  // 3. Phase accumulation and gate firing (only while playing, or in master clock mode)
  const bool playing = !ENABLE_MIDI_CLOCK ||
      g_module.midi.playState == MIDI_PLAYING ||
      g_module.clockFollower.mode == CLK_FOLLOW_INACTIVE;

  bool needsReschedule = false;
  // Phase computation runs without disabling interrupts — mainCallback and
  // gateAlarmCallback share TIMER_IRQ_3 (non-reentrant), and the I2C ISR
  // only touches the ring buffer, not phase/gate state.  Keeping interrupts
  // enabled here avoids blocking I2C reception during the fmodf-heavy loop.
  if (playing) {
    const bool isInternal =
        g_module.clockFollower.mode == CLK_FOLLOW_INACTIVE;
    TransportState &tr = g_module.transport;

    // Advance shared master phase (both internal and MIDI modes).
    // In internal mode masterFreq is fixed; in MIDI mode it is warped
    // on each 0xF8 to track the external clock.
    if (tr.masterFreq > 0.0f) {
      tr.masterPhase += tr.masterFreq;
      if (tr.masterPhase >= 1.0f) {
        tr.masterPhase -= 1.0f;
        tr.masterBeatCount++;
      }
    }

    // Derive each output's phase from the shared master accumulator.
    // beatPos spans [0, den) over den consecutive beats, ensuring
    // sub-beat ratios (e.g. 1/2) wrap correctly at their true cycle
    // boundary, not at every beat boundary.
    for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
      OutputState &out = g_module.outputs[i];
      const bool isLoopLike = (out.run == OUTPUT_RUN_LOOP) ||
          (out.run == OUTPUT_RUN_MIDI_RESET && isInternal);
      if (!isLoopLike) continue;
      if (tr.masterFreq <= 0.0f) continue;

      const float beatPos =
          float(tr.masterBeatCount % out.ratio.den) + tr.masterPhase;
      const float rawPhase = fmodf(
          beatPos * float(out.ratio.num) / float(out.ratio.den), 1.0f);

      float newPhase = rawPhase;
      if (g_module.humanizeEnabled) {
        const float beatPeriodMs = float(tr.beatPeriodUs) / 1000.0f;
        const float offsetPhase = out.humanizer.accumulator * float(out.ratio.num)
                                  / (beatPeriodMs * float(out.ratio.den));
        newPhase = rawPhase - offsetPhase;
        if (newPhase < 0.0f) newPhase += 1.0f;
        if (newPhase >= 1.0f) newPhase -= 1.0f;
      }

      const bool fired = (newPhase < out.phase);
      out.phase = newPhase;

      if (fired && !out.gateHigh && sampleLoopRetrigger(out.loopProbPercent)) {
        if (g_module.humanizeEnabled) {
          computeHumanizerOffset(out, i);
        }
        gpio_put(out.pin, 1);
        out.gateHigh    = true;
        out.gateRises++;
        out.nextFallUs  = nowUs + GATE_PULSE_US;
        needsReschedule = true;
      }
    }
  }
  if (needsReschedule) {
    const uint32_t irq = save_and_disable_interrupts();
    rescheduleGateAlarmLocked();
    restore_interrupts(irq);
  }

  const uint64_t nowUsTimeout = time_us_64();

  // 4. MIDI timeout → fall back to internal clock
  if (ENABLE_MIDI_CLOCK && g_module.midi.lastClockUs != 0) {
    const uint64_t lastClockUs = g_module.midi.lastClockUs;
    uint64_t gapUs64 = 0;
    if (nowUsTimeout >= lastClockUs) {
      gapUs64 = nowUsTimeout - lastClockUs;
    }
    const uint32_t gapUs32 = (gapUs64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)gapUs64;
    if (gapUs32 > g_module.midi.dbgMaxClockGapUs) {
      g_module.midi.dbgMaxClockGapUs = gapUs32;
    }
    if (gapUs64 > MIDI_TIMEOUT_US) {
      const uint64_t gapMs64 = gapUs64 / 1000;
      g_module.midi.dbgTimeoutGapMs =
          (gapMs64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)gapMs64;
      g_module.midi.totalTimeouts++;
      g_module.midi.dbgStopSource = 2;
      g_module.midi.lastClockUs = 0;
      g_module.midi.smoothedPeriodUs = 0.0f;
      g_module.midi.playState = MIDI_STOPPED;
      g_module.clockFollower.mode = CLK_FOLLOW_INACTIVE;
      const uint32_t irq2 = save_and_disable_interrupts();
      applyBpmAndReset(g_module.clockFollower.fallbackBpm);
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        OutputState &out = g_module.outputs[i];
        if (out.run == OUTPUT_RUN_LOOP || out.run == OUTPUT_RUN_MIDI_RESET) out.phase = 1.0f;
      }
      restore_interrupts(irq2);
    }
  }

  return true;
}

}  // namespace certainty

#include "clock_follower.h"

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

static void applyBeatPeriodLocked(uint64_t beatPeriodUs, uint64_t nowUs) {
  if (beatPeriodUs == 0) return;
  const float beatPeriodTicks = float(beatPeriodUs) / float(PWM_SAMPLE_PERIOD_US);
  if (beatPeriodTicks < 1.0f) return;

  g_module.transport.beatPeriodUs = beatPeriodUs;
  g_module.transport.bpm = clampBpm((uint32_t)(60000000ull / beatPeriodUs));
  g_module.transport.anchorUs = nowUs;
  // beatAnchorUs is updated in onMidiMessage when isBeatBoundary fires.

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.freq = float(out.ratio.num) /
               (float(out.ratio.den) * beatPeriodTicks);
  }
}

void initMidiClock() {
  MidiClockState &midi = g_module.midi;
  midi.playState          = MIDI_STOPPED;
  midi.startPending       = false;
  midi.lastClockUs        = 0;
  midi.smoothedPeriodUs   = 0.0f;
  midi.clockCount         = 0;
  midi.totalClocks        = 0;
  midi.totalStarts        = 0;
  midi.totalStops         = 0;
  midi.totalTimeouts      = 0;
  midi.dbgStopSource      = 0;
  midi.dbgTimeoutGapMs    = 0;
  midi.dbgMaxClockGapUs   = 0;
  midi.beatCount           = 0;
  midi.dbgBeatBoundaries   = 0;
  ClockFollowerState &cf = g_module.clockFollower;
  cf.mode        = CLK_FOLLOW_INACTIVE;
  cf.fallbackBpm = DEFAULT_BPM;
}

void onMidiMessage(uint8_t msg, uint64_t nowUs) {
  MidiClockState &midi = g_module.midi;
  ClockFollowerState &cf = g_module.clockFollower;

  switch (msg) {
    case 0xF8: {  // Clock
      midi.totalClocks++;

      // Absolute minimum interval: reject physically impossible pulses (spurious noise).
      // 60000000 / (MAX_BPM * MIDI_RT_PPQN) = 60000000 / (320 * 24) ≈ 7812µs
      static const uint64_t MIN_CLOCK_INTERVAL_US =
          60000000ULL / ((uint64_t)MAX_BPM * MIDI_RT_PPQN);

      // Always track BPM, even while stopped.
      // Capture pre-IIR smoothed period and interval for dropped-tick compensation below.
      uint64_t tickInterval = 0;
      float prevSmoothedUs = 0.0f;
      if (midi.lastClockUs != 0) {
        const uint64_t interval = nowUs - midi.lastClockUs;

        // Spurious pulse: interval shorter than physically possible — discard entirely.
        if (interval < MIN_CLOCK_INTERVAL_US) {
          break;
        }

        // Extreme gap: clock was absent long enough that the tick-count reference
        // is no longer trustworthy. Reset IIR and clockCount so the warp starts fresh.
        if (midi.smoothedPeriodUs > 0.0f && interval > (uint64_t)(midi.smoothedPeriodUs * 4.0f)) {
          midi.smoothedPeriodUs = 0.0f;
          midi.clockCount = 0;
          midi.lastClockUs = nowUs;
          break;
        }

        prevSmoothedUs = midi.smoothedPeriodUs;
        tickInterval   = interval;

        // Normal IIR update — warp handles per-tick jitter, no outlier skipping needed.
        const float iv = (float)interval;
        midi.dbgLastRawInterval = (int32_t)interval;
        if (midi.smoothedPeriodUs == 0.0f) {
          midi.smoothedPeriodUs = iv;
        } else {
          midi.smoothedPeriodUs += 0.25f * (iv - midi.smoothedPeriodUs);
        }
        applyBeatPeriodLocked((uint64_t)(midi.smoothedPeriodUs * (float)MIDI_RT_PPQN), nowUs);
        // Transition to locked as soon as a valid clock source is detected,
        // even before Play is received. This stops MIDI_RESET from looping
        // immediately when a clock is connected, rather than waiting for 0xFA.
        if (cf.mode == CLK_FOLLOW_INACTIVE) cf.mode = CLK_FOLLOW_LOCKED;
      }
      midi.lastClockUs = nowUs;

      // First clock after Start: reset counters, set phases to beat-start position.
      // applyBeatPeriodLocked above already set output frequencies from warm smth.
      if (midi.playState == MIDI_PLAYING && midi.startPending) {
        midi.startPending = false;
        midi.clockCount   = 0;
        midi.beatCount    = 0;
        for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
          OutputState &out = g_module.outputs[i];
          if (out.run == OUTPUT_RUN_LOOP) out.phase = 1.0f;
        }
        break;
      }

      // Advance tick counter, compensating for dropped ticks.
      // If the interval is ≥1.5× the expected period, at least one tick was silently
      // dropped by the soft UART. Advancing clockCount by the estimated true tick count
      // keeps the warp reference aligned with the MIDI source and prevents permanent
      // phase drift accumulation. Uses pre-IIR smoothed period as reference to avoid
      // false positives being masked by the IIR already absorbing the long interval.
      uint32_t ticks = 1;
      if (prevSmoothedUs > 0.0f && tickInterval > 0) {
        const float ratio = float(tickInterval) / prevSmoothedUs;
        if (ratio >= 1.5f) ticks = 2;
        if (ratio >= 2.5f) ticks = 3;
        if (ratio >= 3.5f) ticks = 4;
      }
      const uint32_t prevCount = midi.clockCount;
      midi.clockCount += ticks;
      // Check for beat boundary crossing using pre-increment remainder.
      // A post-increment modulo check would miss the boundary when ticks > 1
      // causes clockCount to jump over a multiple of MIDI_RT_PPQN.
      const bool isBeatBoundary =
          (prevCount % MIDI_RT_PPQN) + ticks >= MIDI_RT_PPQN;
      if (isBeatBoundary) {
        midi.beatCount++;
        midi.dbgBeatBoundaries++;
        g_module.transport.beatAnchorUs = nowUs;
        midi.dbgPhaseAtBeat = g_module.outputs[0].phase;
        midi.dbgTicksBetweenBeats = midi.dbgTickCounter - midi.dbgLastBeatTick;
        midi.dbgLastBeatTick = midi.dbgTickCounter;
      }

      // Per-tick phase warp (tides2-style) — only during PLAY.
      // On each clock, compute each output's expected normalized phase from
      // clockCount and nudge its frequency so it converges. This keeps all
      // outputs collectively in phase and handles dropped clocks automatically:
      // a missed tick means expected advances without a matching phase increment,
      // so warp > 1 and the output speeds up to compensate.
      if (midi.playState == MIDI_PLAYING) {
        for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
          OutputState &out = g_module.outputs[i];
          if (out.run != OUTPUT_RUN_LOOP) continue;

          const float expected = fmodf(
              float(midi.clockCount) * float(out.ratio.num) /
              (float(MIDI_RT_PPQN) * float(out.ratio.den)),
              1.0f);

          float err = expected - out.phase;
          if (err >  0.5f) err -= 1.0f;  // wrap to [-0.5, 0.5]
          if (err < -0.5f) err += 1.0f;

          // warp ∈ [0.5, 1.5] — safe, converges within a few ticks
          out.freq *= (1.0f + err);
        }
      }
      break;
    }

    case 0xFA:  // Start
      midi.totalStarts++;
      midi.dbgStopSource      = 0;
      midi.playState          = MIDI_PLAYING;
      midi.startPending       = true;
      midi.clockCount         = 0;
      // lastClockUs intentionally preserved — extreme-gap guard handles
      // stale values if clocks paused; keeps timeout detection active.
      // smoothedPeriodUs preserved — tick-level IIR stays warm so output
      // frequencies lock immediately via smth*PPQN on first post-Start clock.
      midi.beatCount          = 0;
      cf.mode = CLK_FOLLOW_LOCKED;
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        OutputState &out = g_module.outputs[i];
        out.phase = 0.0f;
        if (out.gateHigh) {
          gpio_put(out.pin, 0);
          out.gateHigh = false;
        }
      }
      break;

    case 0xFB:  // Continue
      midi.playState = MIDI_PLAYING;
      cf.mode = CLK_FOLLOW_LOCKED;
      break;

    case 0xFC: {  // Stop
      midi.totalStops++;
      midi.dbgStopSource = 1;
      midi.playState = MIDI_STOPPED;
      cf.mode = CLK_FOLLOW_LOCKED;
      // clockCount NOT reset — Continue resumes at correct tick reference.
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        OutputState &out = g_module.outputs[i];
        if (out.gateHigh) {
          gpio_put(out.pin, 0);
          out.gateHigh = false;
        }
      }
      // Queue a reset pulse on any MIDI_RESET outputs.
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        if (g_module.outputs[i].run == OUTPUT_RUN_MIDI_RESET) {
          g_module.outputs[i].pendingTriggerCount++;
        }
      }
      rescheduleGateAlarmLocked();
      break;
    }
  }
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
  const uint32_t irq = save_and_disable_interrupts();
  if (playing) {
    const ClockFollowerState &cfLoop = g_module.clockFollower;
    for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
      OutputState &out = g_module.outputs[i];
      const bool isLoopLike = (out.run == OUTPUT_RUN_LOOP) ||
          (out.run == OUTPUT_RUN_MIDI_RESET && cfLoop.mode == CLK_FOLLOW_INACTIVE);
      if (!isLoopLike) continue;
      if (out.freq <= 0.0f) continue;
      out.phase += out.freq;
      if (out.phase >= 1.0f) {
        out.phase -= 1.0f;
        if (out.phase >= 1.0f) out.phase = fmodf(out.phase, 1.0f);
        if (!out.gateHigh && sampleLoopRetrigger(out.loopProbPercent)) {
          gpio_put(out.pin, 1);
          out.gateHigh    = true;
          out.gateRises++;
          out.nextFallUs  = nowUs + GATE_PULSE_US;
          needsReschedule = true;
        }
      }
    }
  }
  if (needsReschedule) rescheduleGateAlarmLocked();
  restore_interrupts(irq);

  const uint64_t nowUsTimeout = time_us_64();

  // 3. MIDI timeout → fall back to internal clock
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

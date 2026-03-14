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

static void applyExternalClockLocked(uint64_t measuredPeriodUs, uint64_t nowUs) {
  if (measuredPeriodUs == 0) return;
  const uint32_t ppqn            = MIDI_RT_PPQN;
  const uint64_t beatPeriodUs    = measuredPeriodUs * ppqn;
  const float    beatPeriodTicks = float(beatPeriodUs) / float(PWM_SAMPLE_PERIOD_US);
  if (beatPeriodTicks < 1.0f) return;

  g_module.transport.beatPeriodUs = beatPeriodUs;
  g_module.transport.bpm = clampBpm((uint32_t)(60000000ull / beatPeriodUs));
  g_module.transport.anchorUs = nowUs;
  if ((g_module.clockFollower.inputEdgeCount % MIDI_RT_PPQN) == 0) {
    g_module.transport.beatAnchorUs = nowUs;
  }

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.freq = float(out.ratio.num) /
               (float(out.ratio.den) * beatPeriodTicks);
  }
}

void initMidiClock() {
  MidiClockState &midi = g_module.midi;
  midi.playState        = MIDI_STOPPED;
  midi.startPending     = false;
  midi.lastClockUs      = 0;
  midi.smoothedPeriodUs = 0;
  midi.clockCount       = 0;
  midi.totalClocks      = 0;
  ClockFollowerState &cf = g_module.clockFollower;
  cf.mode           = CLK_FOLLOW_INACTIVE;
  cf.fallbackBpm    = DEFAULT_BPM;
  cf.inputEdgeCount = 0;
}

void onMidiMessage(uint8_t msg, uint64_t nowUs) {
  MidiClockState &midi = g_module.midi;
  ClockFollowerState &cf = g_module.clockFollower;

  switch (msg) {
    case 0xF8: {  // Clock
      midi.totalClocks++;

      // Always track BPM, even while stopped
      if (midi.lastClockUs != 0) {
        const uint64_t interval = nowUs - midi.lastClockUs;
        if (midi.smoothedPeriodUs == 0) {
          midi.smoothedPeriodUs = interval;
        } else {
          midi.smoothedPeriodUs = (midi.smoothedPeriodUs * 3 + interval) / 4;
        }
        if (midi.smoothedPeriodUs > 0) {
          applyExternalClockLocked(midi.smoothedPeriodUs, nowUs);
        }
      }
      midi.lastClockUs = nowUs;  // AFTER measuring interval

      if (midi.playState != MIDI_PLAYING) break;

      // First clock after Start: beat 1
      if (midi.startPending) {
        midi.startPending = false;
        midi.clockCount = 0;
        cf.inputEdgeCount = 0;
        for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
          OutputState &out = g_module.outputs[i];
          if (out.run == OUTPUT_RUN_LOOP) out.phase = 1.0f;  // phase ≥ 1.0 triggers a gate on the next accumulation tick
        }
        break;
      }

      midi.clockCount++;
      cf.inputEdgeCount = midi.clockCount;

      const uint64_t beatCount = midi.clockCount / MIDI_RT_PPQN;
      const bool isBeatBoundary = (midi.clockCount % MIDI_RT_PPQN) == 0;

      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        OutputState &out = g_module.outputs[i];
        const uint64_t cyclePhaseNum =
            (beatCount * (uint64_t)out.ratio.num) % out.ratio.den;
        if (cyclePhaseNum != 0) {
          out.phase = float(cyclePhaseNum) / float(out.ratio.den);
        } else if (isBeatBoundary) {
          out.phase = 1.0f;  // phase ≥ 1.0 triggers a gate on the next accumulation tick
        }
      }

      if (isBeatBoundary) {
        g_module.transport.beatAnchorUs = nowUs;
      }
      break;
    }

    case 0xFA:  // Start
      midi.playState = MIDI_PLAYING;
      midi.startPending = true;
      midi.clockCount = 0;
      midi.lastClockUs = 0;
      midi.smoothedPeriodUs = 0;
      cf.inputEdgeCount = 0;
      cf.mode = CLK_FOLLOW_LOCKED;
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i)
        g_module.outputs[i].phase = 0.0f;
      break;

    case 0xFB:  // Continue
      midi.playState = MIDI_PLAYING;
      cf.mode = CLK_FOLLOW_LOCKED;
      break;

    case 0xFC: {  // Stop
      midi.playState = MIDI_STOPPED;
      for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
        OutputState &out = g_module.outputs[i];
        if (out.gateHigh) {
          gpio_put(out.pin, 0);
          out.gateHigh = false;
        }
      }
      break;
    }
  }
}

bool mainCallback(repeating_timer *rt) {
  (void)rt;
  const uint64_t nowUs = time_us_64();

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
    for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
      OutputState &out = g_module.outputs[i];
      if (out.run != OUTPUT_RUN_LOOP) continue;
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

  // 3. MIDI timeout → fall back to internal clock
  if (ENABLE_MIDI_CLOCK && g_module.midi.lastClockUs != 0) {
    if (nowUs - g_module.midi.lastClockUs > MIDI_TIMEOUT_US) {
      g_module.midi.lastClockUs = 0;
      g_module.midi.smoothedPeriodUs = 0;
      g_module.midi.playState = MIDI_STOPPED;
      g_module.clockFollower.mode = CLK_FOLLOW_INACTIVE;
      const uint32_t irq2 = save_and_disable_interrupts();
      applyBpmAndReset(g_module.clockFollower.fallbackBpm);
      restore_interrupts(irq2);
    }
  }

  return true;
}


}  // namespace certainty

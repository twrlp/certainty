#include "commands.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "clock_follower.h"
#include "config.h"
#include "gate_scheduler.h"
#include "transport.h"

namespace certainty {

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
      if (sum > 0xFFFFu) {
        out.pendingTriggerCount = 0xFFFFu;
        g_module.i2cTriggerSaturatedCount++;
      } else {
        out.pendingTriggerCount = (uint16_t)sum;
      }
      appliedCount += count;
      gateScheduleChanged = true;
    }
  }
  if (gateScheduleChanged) {
    rescheduleGateAlarmLocked();
  }
  restore_interrupts(irqState);
  g_module.i2cTriggerAppliedCount += appliedCount;
}

void applyI2cEvent(const I2cEvent &event, uint64_t nowUs) {
  switch (event.type) {
    case I2C_EVENT_SET_BPM: {
      const uint32_t bpm = event.count;
      const uint32_t irqState = save_and_disable_interrupts();
      const ClockFollowMode cfMode = g_module.clockFollower.mode;
      if (cfMode == CLK_FOLLOW_LOCKED) {
        g_module.clockFollower.fallbackBpm = clampBpm(bpm);
      } else {
        if (bpm != g_module.transport.bpm) {
          applyBpmAndReset(bpm);
          g_module.clockFollower.fallbackBpm = clampBpm(bpm);
          g_module.i2cAppliedCount++;
        }
      }
      restore_interrupts(irqState);
      break;
    }

    case I2C_EVENT_SET_RATIO: {
      const uint8_t outIndex = event.out;
      const uint8_t num = event.a;
      const uint8_t den = event.b;
      if (outIndex >= NUM_OUTPUTS || num == 0 || den == 0) {
        g_module.i2cErrorCount++;
        break;
      }
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_module.outputs[outIndex];
      out.pendingRatio   = {(uint16_t)num, (uint16_t)den};
      out.pendingApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.ratioPending   = true;
      restore_interrupts(irqState);
      g_module.i2cRatioAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_MODE: {
      const uint8_t outIndex = event.out;
      const OutputRun run = (OutputRun)event.b;
      if (outIndex >= NUM_OUTPUTS ||
          (run != OUTPUT_RUN_ONE_SHOT && run != OUTPUT_RUN_LOOP)) {
        g_module.i2cErrorCount++;
        break;
      }
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_module.outputs[outIndex];
      out.pendingRun        = run;
      out.pendingRunApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.pendingRunPending = true;
      restore_interrupts(irqState);
      g_module.i2cModeAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_PROB: {
      const uint8_t outIndex = event.out;
      const uint8_t prob = event.a;
      if (outIndex >= NUM_OUTPUTS || prob > 100) {
        g_module.i2cErrorCount++;
        break;
      }
      const uint32_t irqState = save_and_disable_interrupts();
      g_module.outputs[outIndex].loopProbPercent = prob;
      restore_interrupts(irqState);
      break;
    }

    case I2C_EVENT_TRIGGER:
      applyTriggerEvent(event.mask, event.count);
      break;
  }
}

void processDueConfigChanges(uint64_t nowUs) {
  bool gateScheduleChanged = false;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const uint32_t irqState = save_and_disable_interrupts();
    OutputState &out = g_module.outputs[i];

    const bool ratioDue = out.ratioPending &&
                          (int64_t)(nowUs - out.pendingApplyUs) >= 0;
    const bool runDue   = out.pendingRunPending &&
                          (int64_t)(nowUs - out.pendingRunApplyUs) >= 0;

    if (!ratioDue && !runDue) {
      restore_interrupts(irqState);
      continue;
    }

    if (ratioDue) {
      out.ratio = out.pendingRatio;
      const float basePeriodTicks =
          float(g_module.transport.beatPeriodUs) / float(PWM_SAMPLE_PERIOD_US);
      if (basePeriodTicks < 1.0f) {
        out.freq = 0.0f;
      } else {
        out.freq = float(out.ratio.num) /
                   (float(out.ratio.den) * basePeriodTicks);
      }
      // Align phase to the beat grid so this output is immediately coherent
      // with any other output already running at the same ratio.
      const ClockFollowerState &cf = g_module.clockFollower;
      if (cf.mode == CLK_FOLLOW_LOCKED) {
        // Use absolute edge count — anchorUs tracks PPQN edges, not beat boundaries.
        const uint64_t beatCount = cf.inputEdgeCount / MIDI_RT_PPQN;
        out.phase = float((beatCount * (uint64_t)out.ratio.num) % out.ratio.den)
                    / float(out.ratio.den);
      } else {
        const uint64_t beatUs   = g_module.transport.beatPeriodUs;
        const uint64_t anchorUs = g_module.transport.anchorUs;
        if (beatUs > 0 && (int64_t)(nowUs - anchorUs) >= 0) {
          const uint64_t beatsElapsed = (nowUs - anchorUs) / beatUs;
          out.phase = float((beatsElapsed * (uint64_t)out.ratio.num) % out.ratio.den)
                      / float(out.ratio.den);
        } else {
          out.phase = 0.0f;
        }
      }

      out.ratioPending = false;
      out.gateHigh     = false;
      gpio_put(out.pin, 0);
      gateScheduleChanged = true;
    }

    if (runDue) {
      out.run               = out.pendingRun;
      out.pendingRunPending = false;
      if (out.run == OUTPUT_RUN_ONE_SHOT) {
        out.gateHigh = false;
        gpio_put(out.pin, 0);
      } else {
        out.pendingTriggerCount = 0;
      }
      gateScheduleChanged = true;
    }

    restore_interrupts(irqState);
  }

  if (gateScheduleChanged) {
    const uint32_t irqState = save_and_disable_interrupts();
    rescheduleGateAlarmLocked();
    restore_interrupts(irqState);
  }
}

}  // namespace certainty

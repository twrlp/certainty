#include "commands.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"
#include "pwm_engine.h"
#include "transport.h"

namespace certainty {

static void applyTriggerEvent(uint8_t mask, uint16_t count) {
  if (mask == 0 || count == 0) {
    return;
  }

  uint32_t appliedCount = 0;
  bool gateScheduleChanged = false;

  const uint32_t irqState = save_and_disable_interrupts();
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if ((mask & (1u << i)) == 0) {
      continue;
    }

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
      if (out.shape == OUTPUT_SHAPE_TRIG) {
        gateScheduleChanged = true;
      }
      continue;
    }

    // If a behavior change to one-shot is pending, accumulate triggers now and
    // consume after the mode transitions.
    if (out.behaviorPending && out.pendingRun == OUTPUT_RUN_ONE_SHOT) {
      const uint32_t sum = (uint32_t)out.pendingTriggerCount + (uint32_t)count;
      if (sum > 0xFFFFu) {
        out.pendingTriggerCount = 0xFFFFu;
        g_module.i2cTriggerSaturatedCount++;
      } else {
        out.pendingTriggerCount = (uint16_t)sum;
      }
      appliedCount += count;
      if (out.pendingShape == OUTPUT_SHAPE_TRIG) {
        gateScheduleChanged = true;
      }
    }
  }
  if (gateScheduleChanged) {
    rescheduleGateAlarmLocked();
  }
  restore_interrupts(irqState);

  g_module.i2cTriggerAppliedCount += appliedCount;
}

void applyI2cEvent(const I2cEvent &event) {
  switch (event.type) {
    case I2C_EVENT_SET_BPM: {
      const uint32_t bpm = event.count;
      if (bpm != g_module.transport.bpm) {
        applyBpmAndReset(bpm);
        g_module.i2cAppliedCount++;
      }
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

      const uint64_t nowUs = time_us_64();
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_module.outputs[outIndex];
      out.pendingRatio = {(uint16_t)num, (uint16_t)den};
      out.pendingApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.ratioPending = true;
      restore_interrupts(irqState);
      g_module.i2cRatioAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_MODE: {
      const uint8_t outIndex = event.out;
      const OutputShape shape = (OutputShape)event.a;
      const OutputRun run = (OutputRun)event.b;
      if (outIndex >= NUM_OUTPUTS || (shape != OUTPUT_SHAPE_TRIG && shape != OUTPUT_SHAPE_ASR) ||
          (run != OUTPUT_RUN_ONE_SHOT && run != OUTPUT_RUN_LOOP)) {
        g_module.i2cErrorCount++;
        break;
      }

      const uint64_t nowUs = time_us_64();
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_module.outputs[outIndex];
      out.pendingShape = shape;
      out.pendingRun = run;
      out.behaviorPendingApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.behaviorPending = true;
      restore_interrupts(irqState);
      g_module.i2cModeAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_ASR: {
      const uint8_t outIndex = event.out;
      const uint8_t sus = event.a;
      const uint8_t skew = event.b;
      if (outIndex >= NUM_OUTPUTS || sus > 10 || skew > 10) {
        g_module.i2cErrorCount++;
        break;
      }

      uint8_t a = 0;
      uint8_t s = 0;
      uint8_t r = 0;
      deriveAsrWeights(sus, skew, &a, &s, &r);

      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_module.outputs[outIndex];
      out.asrSus = sus;
      out.asrSkew = skew;
      out.asrA = a;
      out.asrS = s;
      out.asrR = r;
      refreshAsrTimingLocked(out);
      restore_interrupts(irqState);
      g_module.i2cAsrAppliedCount++;
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
      OutputState &out = g_module.outputs[outIndex];
      out.loopProbPercent = prob;
      restore_interrupts(irqState);
      break;
    }

    case I2C_EVENT_TRIGGER:
      applyTriggerEvent(event.mask, event.count);
      break;
  }
}

void processDueConfigChanges() {
  const uint64_t nowUs = time_us_64();
  bool gateScheduleChanged = false;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    bool channelAffectsGateSchedule = false;
    const uint32_t irqState = save_and_disable_interrupts();
    OutputState &out = g_module.outputs[i];
    const bool ratioDue = out.ratioPending && (int64_t)(nowUs - out.pendingApplyUs) >= 0;
    const bool behaviorDue = out.behaviorPending && (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0;

    if (!ratioDue && !behaviorDue) {
      restore_interrupts(irqState);
      continue;
    }

    if (ratioDue) {
      out.ratio = out.pendingRatio;
      out.periodUs = periodFromRatio(g_module.transport.beatPeriodUs, out.ratio);
      refreshAsrTimingLocked(out);
      out.ratioPending = false;

      if (out.shape == OUTPUT_SHAPE_TRIG && out.run == OUTPUT_RUN_LOOP) {
        out.nextRiseUs = alignToGlobalPhaseGridAtOrAfterLocked(out.pendingApplyUs, out.periodUs);
        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
        out.gateHigh = false;
        gpio_put(out.pin, 0);
        channelAffectsGateSchedule = true;
      } else if (out.shape == OUTPUT_SHAPE_ASR && out.run == OUTPUT_RUN_LOOP) {
        out.lfoAnchorUs = g_module.transport.anchorUs;
        out.loopCycleIndex = (uint64_t)-1;
        out.loopCycleActive = true;
      }
    }

    if (behaviorDue) {
      const OutputShape oldShape = out.shape;
      const OutputShape newShape = out.pendingShape;
      const OutputRun newRun = out.pendingRun;
      const uint64_t applyUs = out.behaviorPendingApplyUs;

      if (oldShape != newShape) {
        if (oldShape == OUTPUT_SHAPE_TRIG) {
          if (out.gateHigh) {
            gpio_put(out.pin, 0);
            out.gateHigh = false;
            out.gateFalls++;
          }
        } else {
          pwm_set_gpio_level(out.pin, 0);
          out.lastPwmLevel = 0;
        }

        if (newShape == OUTPUT_SHAPE_TRIG) {
          configurePinAsGateOutput(out.pin);
        } else {
          configurePinAsPwmOutput(out);
          out.lastPwmLevel = 0;
        }
      }

      out.shape = newShape;
      out.run = newRun;
      out.periodUs = periodFromRatio(g_module.transport.beatPeriodUs, out.ratio);
      refreshAsrTimingLocked(out);

      if (newShape == OUTPUT_SHAPE_TRIG) {
        out.asrOneShotActive = false;
        out.gateHigh = false;
        gpio_put(out.pin, 0);

        if (newRun == OUTPUT_RUN_LOOP) {
          out.nextRiseUs = alignToGlobalPhaseGridAtOrAfterLocked(applyUs, out.periodUs);
          out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
          out.pendingTriggerCount = 0;
        }

        channelAffectsGateSchedule = true;
      } else {
        if (newRun == OUTPUT_RUN_LOOP) {
          out.lfoAnchorUs = g_module.transport.anchorUs;
          out.loopCycleIndex = (uint64_t)-1;
          out.loopCycleActive = true;
          out.asrOneShotActive = false;
          out.pendingTriggerCount = 0;
        } else {
          out.asrOneShotActive = false;
          pwm_set_gpio_level(out.pin, 0);
          out.lastPwmLevel = 0;

          if (out.pendingTriggerCount > 0) {
            out.asrOneShotActive = true;
            out.asrOneShotStartUs = applyUs;
            out.asrOneShotDurationUs = out.periodUs;
            out.asrOneShotAttackUs = out.asrAttackUs;
            out.asrOneShotSustainUs = out.asrSustainUs;
            out.asrOneShotReleaseUs = out.asrReleaseUs;
            out.pendingTriggerCount--;
          }
        }

        if (oldShape == OUTPUT_SHAPE_TRIG) {
          channelAffectsGateSchedule = true;
        }
      }

      out.behaviorPending = false;
    }

    restore_interrupts(irqState);
    if (!channelAffectsGateSchedule) {
      continue;
    }
    gateScheduleChanged = true;
  }

  if (gateScheduleChanged) {
    const uint32_t irqState = save_and_disable_interrupts();
    rescheduleGateAlarmLocked();
    restore_interrupts(irqState);
  }
}

}  // namespace certainty

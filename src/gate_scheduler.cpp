#include "gate_scheduler.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"
#include "pwm_engine.h"
#include "rng.h"
#include "transport.h"

namespace certainty {

void configurePinAsGateOutput(uint8_t pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, 0);
}

void scheduleNextAlarmLocked() {
  const uint64_t nowUs = time_us_64();
  uint64_t nextDueUs = UINT64_MAX;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const OutputState &out = g_module.outputs[i];
    uint64_t dueUs = UINT64_MAX;
    if (out.shape == OUTPUT_SHAPE_TRIG) {
      if (out.gateHigh && out.nextFallUs < dueUs) {
        dueUs = out.nextFallUs;
      } else if (out.run == OUTPUT_RUN_LOOP && out.nextRiseUs < dueUs) {
        dueUs = out.nextRiseUs;
      } else if (out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0 && nowUs < dueUs) {
        dueUs = nowUs;
      }
    }

    if (dueUs < nextDueUs) {
      nextDueUs = dueUs;
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

void rescheduleGateAlarmLocked() {
  if (g_module.gateAlarmId >= 0) {
    cancel_alarm(g_module.gateAlarmId);
    g_module.gateAlarmId = -1;
  }
  scheduleNextAlarmLocked();
}

void ensureGateSchedulerRunning() {
  const uint32_t irqState = save_and_disable_interrupts();
  if (g_module.gateAlarmId >= 0) {
    restore_interrupts(irqState);
    return;
  }

  bool hasTrigOutput = false;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if (g_module.outputs[i].shape == OUTPUT_SHAPE_TRIG) {
      hasTrigOutput = true;
      break;
    }
  }

  if (hasTrigOutput) {
    scheduleNextAlarmLocked();
  }
  restore_interrupts(irqState);
}

int64_t gateAlarmCallback(alarm_id_t id, void *user_data) {
  (void)id;
  (void)user_data;

  g_module.gateSchedulerRuns++;
  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    if (out.shape != OUTPUT_SHAPE_TRIG) {
      continue;
    }

    // Config commits are handled in loop(); once the transition point is due,
    // stop producing trig edges until the commit completes.
    if (out.behaviorPending && out.pendingShape == OUTPUT_SHAPE_ASR &&
        (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0) {
      if (out.gateHigh) {
        gpio_put(out.pin, 0);
        out.gateHigh = false;
        out.gateFalls++;
      }
      continue;
    }

    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0) {
      gpio_put(out.pin, 1);
      out.gateHigh = true;
      out.gateRises++;
      out.nextFallUs = nowUs + GATE_PULSE_US;
      out.pendingTriggerCount--;
    }

    if (out.gateHigh && (int64_t)(nowUs - out.nextFallUs) >= 0) {
      gpio_put(out.pin, 0);
      out.gateHigh = false;
      out.gateFalls++;

      if (out.run == OUTPUT_RUN_LOOP) {
        do {
          out.nextRiseUs += out.periodUs;
          if ((int64_t)(nowUs - out.nextRiseUs) >= 0) {
            g_module.gateSchedulerMisses++;
          }
        } while ((int64_t)(nowUs - out.nextRiseUs) >= 0);

        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
      }
    }

    if (out.run == OUTPUT_RUN_LOOP && !out.gateHigh && (int64_t)(nowUs - out.nextRiseUs) >= 0) {
      if (sampleLoopRetrigger(out.loopProbPercent)) {
        gpio_put(out.pin, 1);
        out.gateHigh = true;
        out.gateRises++;
        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
      } else {
        do {
          out.nextRiseUs += out.periodUs;
          if ((int64_t)(nowUs - out.nextRiseUs) >= 0) {
            g_module.gateSchedulerMisses++;
          }
        } while ((int64_t)(nowUs - out.nextRiseUs) >= 0);

        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
      }
    }
  }

  scheduleNextAlarmLocked();
  return 0;
}

}  // namespace certainty

#include "gate_scheduler.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"
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
    if (out.gateHigh && out.nextFallUs < nextDueUs) {
      nextDueUs = out.nextFallUs;
    }
    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT &&
        out.pendingTriggerCount > 0) {
      nextDueUs = nowUs;
    }
  }

  if (nextDueUs == UINT64_MAX) {
    g_module.gateAlarmId = -1;
    return;
  }

  g_module.gateAlarmId =
      add_alarm_at(absoluteFromUs(nextDueUs), gateAlarmCallback, nullptr, true);
  // fire_if_past=true means a negative return does NOT indicate a lost trigger —
  // the SDK reschedules for now+1µs. gateSchedulerMisses is a low-water-mark for
  // alarm pool pressure, not a drop counter.
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
  if (g_module.gateAlarmId < 0) {
    scheduleNextAlarmLocked();
  }
  restore_interrupts(irqState);
}

int64_t gateAlarmCallback(alarm_id_t id, void *user_data) {
  (void)id;
  (void)user_data;

  g_module.gateSchedulerRuns++;
  const uint64_t nowUs = time_us_64();

  // All alarm and repeating-timer callbacks share TIMER_IRQ_3 (same alarm pool).
  // They cannot preempt each other, so no additional interrupt lock is needed here.
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];

    // One-shot rise
    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT &&
        out.pendingTriggerCount > 0) {
      gpio_put(out.pin, 1);
      out.gateHigh   = true;
      out.gateRises++;
      out.nextFallUs = nowUs + GATE_PULSE_US;
      out.pendingTriggerCount--;
    }

    // Fall
    if (out.gateHigh && (int64_t)(nowUs - out.nextFallUs) >= 0) {
      gpio_put(out.pin, 0);
      out.gateHigh = false;
      out.gateFalls++;
    }
  }

  scheduleNextAlarmLocked();
  return 0;
}

}  // namespace certainty

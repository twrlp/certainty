#include "transport.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"

namespace certainty {

absolute_time_t absoluteFromUs(uint64_t usSinceBoot) {
  absolute_time_t t;
  update_us_since_boot(&t, usSinceBoot);
  return t;
}

uint32_t clampBpm(uint32_t bpm) {
  if (bpm < MIN_BPM) {
    return MIN_BPM;
  }
  if (bpm > MAX_BPM) {
    return MAX_BPM;
  }
  return bpm;
}

uint64_t nextBeatBoundaryAfterLocked(uint64_t nowUs) {
  const ClockFollowerState &cf = g_module.clockFollower;
  const uint64_t anchorUs = (cf.mode == CLK_FOLLOW_LOCKED)
      ? g_module.transport.beatAnchorUs
      : g_module.transport.anchorUs;
  const uint64_t beatUs = g_module.transport.beatPeriodUs;
  if (beatUs == 0) {
    return nowUs + 1;
  }
  if ((int64_t)(nowUs - anchorUs) < 0) {
    return anchorUs;
  }
  const uint64_t elapsedUs = nowUs - anchorUs;
  const uint64_t beatsElapsed = elapsedUs / beatUs;
  return anchorUs + ((beatsElapsed + 1u) * beatUs);
}

void resetTransportLocked(uint32_t bpm, uint64_t nowUs) {
  if (g_module.gateAlarmId >= 0) {
    cancel_alarm(g_module.gateAlarmId);
    g_module.gateAlarmId = -1;
  }

  g_module.transport.bpm          = clampBpm(bpm);
  g_module.transport.beatPeriodUs = 60000000ull / g_module.transport.bpm;
  g_module.transport.anchorUs     = nowUs + START_DELAY_US;
  g_module.transport.beatAnchorUs = nowUs + START_DELAY_US;
  g_module.transport.resetCount++;

  const float beatPeriodTicks =
      float(g_module.transport.beatPeriodUs) / float(PWM_SAMPLE_PERIOD_US);

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.phase              = 0.0f;
    out.freq               = float(out.ratio.num) /
                             (float(out.ratio.den) * beatPeriodTicks);
    out.ratioPending       = false;
    out.pendingApplyUs     = g_module.transport.anchorUs;
    out.pendingRatio       = out.ratio;
    out.pendingRunPending  = false;
    out.pendingRunApplyUs  = g_module.transport.anchorUs;
    out.pendingRun         = out.run;
    out.pendingTriggerCount = 0;
    out.nextFallUs         = g_module.transport.anchorUs + GATE_PULSE_US;
    out.gateHigh           = false;
    out.gateRises          = 0;
    out.gateFalls          = 0;
    gpio_put(out.pin, 0);
  }

  scheduleNextAlarmLocked();
}

void applyBpmAndReset(uint32_t bpm) {
  const uint32_t irqState = save_and_disable_interrupts();
  resetTransportLocked(bpm, time_us_64());
  restore_interrupts(irqState);
}

}  // namespace certainty

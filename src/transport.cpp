#include "transport.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"
#include "pwm_engine.h"

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

uint64_t periodFromRatio(uint64_t beatPeriodUs, Ratio ratio) {
  uint64_t period = (beatPeriodUs * ratio.den + (ratio.num / 2u)) / ratio.num;
  const uint64_t minPeriod = (uint64_t)GATE_PULSE_US + 1000u;
  if (period < minPeriod) {
    return minPeriod;
  }
  return period;
}

uint64_t nextBeatBoundaryAfterLocked(uint64_t nowUs) {
  const uint64_t anchorUs = g_module.transport.anchorUs;
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

uint64_t alignToGlobalPhaseGridAtOrAfterLocked(uint64_t atLeastUs, uint64_t periodUs) {
  const uint64_t anchorUs = g_module.transport.anchorUs;
  if (periodUs == 0) {
    return atLeastUs;
  }
  if ((int64_t)(atLeastUs - anchorUs) <= 0) {
    return anchorUs;
  }
  const uint64_t elapsedUs = atLeastUs - anchorUs;
  const uint64_t steps = (elapsedUs + periodUs - 1u) / periodUs;
  return anchorUs + (steps * periodUs);
}

void resetTransportLocked(uint32_t bpm, uint64_t nowUs) {
  if (g_module.gateAlarmId >= 0) {
    cancel_alarm(g_module.gateAlarmId);
    g_module.gateAlarmId = -1;
  }

  g_module.transport.bpm = clampBpm(bpm);
  g_module.transport.beatPeriodUs = 60000000ull / g_module.transport.bpm;
  g_module.transport.anchorUs = nowUs + START_DELAY_US;
  g_module.transport.resetCount++;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.periodUs = periodFromRatio(g_module.transport.beatPeriodUs, out.ratio);
    refreshAsrTimingLocked(out);
    out.lfoAnchorUs = g_module.transport.anchorUs;
    out.loopCycleIndex = (uint64_t)-1;
    out.loopCycleActive = true;
    out.ratioPending = false;
    out.pendingApplyUs = g_module.transport.anchorUs;
    out.pendingRatio = out.ratio;
    out.behaviorPending = false;
    out.behaviorPendingApplyUs = g_module.transport.anchorUs;
    out.pendingShape = out.shape;
    out.pendingRun = out.run;
    out.pendingTriggerCount = 0;
    out.asrOneShotActive = false;
    out.asrOneShotStartUs = g_module.transport.anchorUs;
    out.asrOneShotDurationUs = out.periodUs;
    out.asrOneShotAttackUs = out.asrAttackUs;
    out.asrOneShotSustainUs = out.asrSustainUs;
    out.asrOneShotReleaseUs = out.asrReleaseUs;
    out.nextRiseUs = g_module.transport.anchorUs;
    out.nextFallUs = g_module.transport.anchorUs + GATE_PULSE_US;
    out.gateHigh = false;
    out.gateRises = 0;
    out.gateFalls = 0;
    out.lfoUpdates = 0;
    out.lastPwmLevel = 0;

    if (out.shape != OUTPUT_SHAPE_ASR) {
      gpio_put(out.pin, 0);
    } else {
      pwm_set_gpio_level(out.pin, 0);
    }
  }

  scheduleNextAlarmLocked();
}

void applyBpmAndReset(uint32_t bpm) {
  const uint32_t irqState = save_and_disable_interrupts();
  resetTransportLocked(bpm, time_us_64());
  restore_interrupts(irqState);
}

}  // namespace certainty

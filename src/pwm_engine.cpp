#include "pwm_engine.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "module_state.h"
#include "config.h"
#include "gate_scheduler.h"

namespace certainty {

static void computeAsrSegments(uint64_t durationUs,
                               uint8_t a,
                               uint8_t s,
                               uint8_t r,
                               uint64_t *attackUs,
                               uint64_t *sustainUs,
                               uint64_t *releaseUs) {
  if (durationUs < 2) {
    *attackUs = 0;
    *sustainUs = 0;
    *releaseUs = 0;
    return;
  }

  const uint32_t sum = (uint32_t)a + (uint32_t)s + (uint32_t)r;
  if (sum == 0) {
    *attackUs = 0;
    *sustainUs = 0;
    *releaseUs = 0;
    return;
  }

  uint64_t attack = (durationUs * (uint64_t)a) / sum;
  uint64_t sustain = (durationUs * (uint64_t)s) / sum;

  // Keep non-zero segments non-zero when possible.
  if (a > 0 && attack == 0 && durationUs >= 3) {
    attack = 1;
  }
  if (s > 0 && sustain == 0 && durationUs >= 3) {
    sustain = 1;
  }

  if (attack + sustain >= durationUs) {
    if (sustain > 0) {
      sustain = durationUs - 1;
    } else if (attack > 0) {
      attack = durationUs - 1;
    }
  }

  *attackUs = attack;
  *sustainUs = sustain;
  *releaseUs = durationUs - attack - sustain;
}

static uint16_t asrLevelFromElapsed(uint64_t elapsedUs,
                                    uint64_t durationUs,
                                    uint64_t attackUs,
                                    uint64_t sustainUs,
                                    uint64_t releaseUs) {
  if (durationUs < 2) {
    return 0;
  }

  uint64_t t = elapsedUs;
  if (attackUs > 0) {
    if (t < attackUs) {
      uint64_t level = (t * PWM_WRAP + (attackUs / 2u)) / attackUs;
      if (level > PWM_WRAP) {
        level = PWM_WRAP;
      }
      return (uint16_t)level;
    }
    t -= attackUs;
  }

  if (sustainUs > 0) {
    if (t < sustainUs) {
      return PWM_WRAP;
    }
    t -= sustainUs;
  }

  if (releaseUs > 0 && t < releaseUs) {
    const uint64_t remaining = releaseUs - t;
    uint64_t level = (remaining * PWM_WRAP + (releaseUs / 2u)) / releaseUs;
    if (level > PWM_WRAP) {
      level = PWM_WRAP;
    }
    return (uint16_t)level;
  }

  return 0;
}

static uint16_t asrLevelFromTime(uint64_t nowUs,
                                 uint64_t startUs,
                                 uint64_t durationUs,
                                 uint64_t attackUs,
                                 uint64_t sustainUs,
                                 uint64_t releaseUs,
                                 bool looping) {
  if (durationUs < 2 || (int64_t)(nowUs - startUs) < 0) {
    return 0;
  }
  uint64_t elapsedUs = nowUs - startUs;
  if (looping) {
    elapsedUs %= durationUs;
  } else if (elapsedUs >= durationUs) {
    return 0;
  }
  return asrLevelFromElapsed(elapsedUs, durationUs, attackUs, sustainUs, releaseUs);
}

bool usesPwm(OutputShape shape) {
  return shape == OUTPUT_SHAPE_ASR;
}

void configurePinAsPwmOutput(OutputState &out) {
  gpio_set_function(out.pin, GPIO_FUNC_PWM);
  out.pwmSlice = (uint8_t)pwm_gpio_to_slice_num(out.pin);
  out.pwmChannel = (uint8_t)pwm_gpio_to_channel(out.pin);
  pwm_set_clkdiv_int_frac(out.pwmSlice, PWM_CLKDIV_INT, PWM_CLKDIV_FRAC);
  pwm_set_wrap(out.pwmSlice, PWM_WRAP);
  pwm_set_enabled(out.pwmSlice, true);
  pwm_set_gpio_level(out.pin, 0);
}

void configureOutputPinsForModes() {
  g_module.pwmSliceMask = 0;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    if (!usesPwm(out.shape)) {
      configurePinAsGateOutput(out.pin);
      continue;
    }

    configurePinAsPwmOutput(out);
    g_module.pwmSliceMask |= (1u << out.pwmSlice);
  }

  for (uint8_t slice = 0; slice < NUM_PWM_SLICES; ++slice) {
    if ((g_module.pwmSliceMask & (1u << slice)) == 0) {
      continue;
    }
    pwm_set_clkdiv_int_frac(slice, PWM_CLKDIV_INT, PWM_CLKDIV_FRAC);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
  }
}

void deriveAsrWeights(uint8_t sus, uint8_t skew, uint8_t *a, uint8_t *s, uint8_t *r) {
  // 0..10 controls from TT:
  // sus=0 -> no sustain (triangle-ish), sus=10 -> full sustain (square-ish)
  // skew=0 -> instant attack / long release, skew=10 -> long attack / instant release
  if (sus > 10) {
    sus = 10;
  }
  if (skew > 10) {
    skew = 10;
  }

  const uint8_t rem = (uint8_t)(10 - sus);
  const uint8_t attack = (uint8_t)((rem * skew + 5u) / 10u);
  const uint8_t release = (uint8_t)(rem - attack);
  *a = attack;
  *s = sus;
  *r = release;
}

void refreshAsrTimingLocked(OutputState &out) {
  computeAsrSegments(out.periodUs, out.asrA, out.asrS, out.asrR, &out.asrAttackUs, &out.asrSustainUs,
                     &out.asrReleaseUs);
}

bool pwmSampleCallback(repeating_timer *rt) {
  (void)rt;
  g_module.pwmRuns++;

  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    if (out.shape != OUTPUT_SHAPE_ASR) {
      continue;
    }

    // Config commits are handled in loop(); once the transition point is due,
    // keep ASR output at zero until the commit completes.
    if (out.behaviorPending && out.pendingShape == OUTPUT_SHAPE_TRIG &&
        (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0) {
      pwm_set_gpio_level(out.pin, 0);
      out.lastPwmLevel = 0;
      continue;
    }

    if (out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0) {
      out.asrOneShotActive = true;
      out.asrOneShotStartUs = nowUs;
      out.asrOneShotDurationUs = out.periodUs;
      out.asrOneShotAttackUs = out.asrAttackUs;
      out.asrOneShotSustainUs = out.asrSustainUs;
      out.asrOneShotReleaseUs = out.asrReleaseUs;
      out.pendingTriggerCount--;
    }

    uint16_t level = 0;
    if (out.run == OUTPUT_RUN_LOOP) {
      level = asrLevelFromTime(nowUs,
                               out.lfoAnchorUs,
                               out.periodUs,
                               out.asrAttackUs,
                               out.asrSustainUs,
                               out.asrReleaseUs,
                               true);
    } else if (out.asrOneShotActive) {
      level = asrLevelFromTime(nowUs,
                               out.asrOneShotStartUs,
                               out.asrOneShotDurationUs,
                               out.asrOneShotAttackUs,
                               out.asrOneShotSustainUs,
                               out.asrOneShotReleaseUs,
                               false);
      if ((int64_t)(nowUs - (out.asrOneShotStartUs + out.asrOneShotDurationUs)) >= 0) {
        out.asrOneShotActive = false;
      }
    }

    pwm_set_gpio_level(out.pin, level);
    out.lastPwmLevel = level;
    out.lfoUpdates++;
  }

  return true;
}

}  // namespace certainty

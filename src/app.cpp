#include "app.h"

#include <Arduino.h>

#include "hardware/sync.h"

#include "module_state.h"
#include "commands.h"
#include "config.h"
#include "debug_led.h"
#include "gate_scheduler.h"
#include "i2c_ingress.h"
#include "pwm_engine.h"
#include "transport.h"

namespace certainty {

static const Ratio DEFAULT_RATIOS[NUM_OUTPUTS] = {
    {1, 2}, {1, 2}, {1, 1}, {1, 1}, {2, 1}, {2, 1}, {4, 1}, {4, 1}};

static const OutputShape DEFAULT_SHAPES[NUM_OUTPUTS] = {
    OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG,
    OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_TRIG};

static const OutputRun DEFAULT_RUNS[NUM_OUTPUTS] = {
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP,
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP};

void appSetup() {
  debugLedInit();
  debugLedSet(false);

  g_module.gateAlarmId = -1;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.pin = OUTPUT_PINS[i];
    out.ratio = DEFAULT_RATIOS[i];
    out.shape = DEFAULT_SHAPES[i];
    out.run = DEFAULT_RUNS[i];

    out.periodUs = 0;
    out.lfoAnchorUs = 0;
    out.loopProbPercent = 100;
    out.loopCycleIndex = (uint64_t)-1;
    out.loopCycleActive = true;
    out.asrSus = 0;
    out.asrSkew = 5;
    out.asrA = 1;
    out.asrS = 0;
    out.asrR = 1;
    out.ratioPending = false;
    out.pendingApplyUs = 0;
    out.pendingRatio = {1, 1};
    out.behaviorPending = false;
    out.behaviorPendingApplyUs = 0;
    out.pendingShape = out.shape;
    out.pendingRun = out.run;
    out.pendingTriggerCount = 0;
    out.asrOneShotActive = false;
    out.asrOneShotStartUs = 0;
    out.asrOneShotDurationUs = 0;
    out.asrAttackUs = 0;
    out.asrSustainUs = 0;
    out.asrReleaseUs = 0;
    out.asrOneShotAttackUs = 0;
    out.asrOneShotSustainUs = 0;
    out.asrOneShotReleaseUs = 0;
    out.nextRiseUs = 0;
    out.nextFallUs = 0;
    out.gateHigh = false;
    out.gateRises = 0;
    out.gateFalls = 0;
    out.pwmSlice = 0;
    out.pwmChannel = 0;
    out.lastPwmLevel = 0;
    out.lfoUpdates = 0;
  }

  configureOutputPinsForModes();

  g_module.pwmTimerRunning =
      add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, pwmSampleCallback, nullptr, &g_module.pwmTimer);

  applyBpmAndReset(DEFAULT_BPM);

  if (ENABLE_I2C_BPM) {
    // Prepare bus pins as pulled-up inputs; Wire init is retried from loop if
    // the bus is not idle yet.
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    if (digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH) {
      tryInitI2cBpmReceiver();
    }
  }
}

void appLoop() {
  static uint32_t lastLedToggleMs = 0;
  static uint32_t lastI2cRetryMs = 0;
  static uint32_t errorLedUntilMs = 0;
  static bool ledOn = false;
  static uint32_t lastErrorCount = 0;
  static uint32_t lastDropCount = 0;
  static uint32_t lastDropTrigCount = 0;
  static uint32_t lastSatCount = 0;

  const uint32_t now = millis();
  if (ENABLE_I2C_BPM && !g_module.i2cEnabled && (now - lastI2cRetryMs >= I2C_RETRY_MS)) {
    lastI2cRetryMs = now;
    const bool sdaHigh = (digitalRead(I2C_SDA_PIN) == HIGH);
    const bool sclHigh = (digitalRead(I2C_SCL_PIN) == HIGH);
    if (sdaHigh && sclHigh) {
      tryInitI2cBpmReceiver();
    }
  }

  processDueConfigChanges();
  ensureGateSchedulerRunning();

  if (ENABLE_I2C_BPM) {
    processI2cEvents();
    processDueConfigChanges();
    ensureGateSchedulerRunning();

    uint32_t err = 0;
    uint32_t dropAll = 0;
    uint32_t dropTrig = 0;
    uint32_t sat = 0;
    const uint32_t irqState = save_and_disable_interrupts();
    err = g_module.i2cErrorCount;
    dropAll = g_module.i2cEventQueueDropCount;
    dropTrig = g_module.i2cEventQueueDropTriggerCount;
    sat = g_module.i2cTriggerSaturatedCount;
    restore_interrupts(irqState);

    if (err != lastErrorCount || dropAll != lastDropCount || dropTrig != lastDropTrigCount ||
        sat != lastSatCount) {
      errorLedUntilMs = now + I2C_ERROR_LATCH_MS;
      lastErrorCount = err;
      lastDropCount = dropAll;
      lastDropTrigCount = dropTrig;
      lastSatCount = sat;
    }
  }

  if ((int32_t)(now - errorLedUntilMs) < 0) {
    debugLedSet(true);
  } else if (ENABLE_I2C_ACTIVITY_LED) {
    processI2cActivityLed(now);
  } else if (now - lastLedToggleMs >= HEARTBEAT_MS) {
    lastLedToggleMs = now;
    ledOn = !ledOn;
    debugLedSet(ledOn);
  }
}

}  // namespace certainty

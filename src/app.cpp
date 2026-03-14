#include "app.h"

#include <Arduino.h>

#include "hardware/sync.h"

#include "module_state.h"
#include "clock_follower.h"
#include "config.h"
#include "gate_scheduler.h"
#include "i2c_ingress.h"
#include "midi_uart.h"
#include "transport.h"

namespace certainty {

static const Ratio DEFAULT_RATIOS[NUM_OUTPUTS] = {
    {1, 2}, {1, 2}, {1, 1}, {1, 1}, {2, 1}, {2, 1}, {4, 1}, {4, 1}};

static const OutputRun DEFAULT_RUNS[NUM_OUTPUTS] = {
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP,
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP};

void appSetup() {
  Serial.begin(115200);
  g_module.gateAlarmId = -1;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_module.outputs[i];
    out.pin               = OUTPUT_PINS[i];
    out.ratio             = DEFAULT_RATIOS[i];
    out.run               = DEFAULT_RUNS[i];
    out.loopProbPercent   = 100;
    out.ratioPending      = false;
    out.pendingApplyUs    = 0;
    out.pendingRatio      = {1, 1};
    out.pendingRunPending = false;
    out.pendingRunApplyUs = 0;
    out.pendingRun        = out.run;
    out.pendingTriggerCount = 0;
    out.nextFallUs        = 0;
    out.gateHigh          = false;
    out.gateRises         = 0;
    out.gateFalls         = 0;
    out.phase             = 0.0f;
    out.freq              = 0.0f;
    configurePinAsGateOutput(out.pin);
  }

  applyBpmAndReset(DEFAULT_BPM);

  initMidiClock();
  if (ENABLE_MIDI_CLOCK) {
    initMidiDma();
  }
  g_module.mainTimerRunning =
      add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, mainCallback, nullptr, &g_module.mainTimer);

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
  static uint32_t lastI2cRetryMs = 0;
  static uint32_t lastMidiPrintMs = 0;

  const uint32_t now = millis();

  if (ENABLE_MIDI_CLOCK && (now - lastMidiPrintMs >= 500)) {
    lastMidiPrintMs = now;
    MidiUartState &uart = g_module.midiUart;
    const uint16_t mid = (uint16_t(uart.dbgAdcMin) + uint16_t(uart.dbgAdcMax)) / 2;
    uint8_t startThr = uart.adcStartThreshold;
    if (uart.dbgAdcMax >= uart.dbgAdcMin) {
      const uint16_t range = uint16_t(uart.dbgAdcMax) - uint16_t(uart.dbgAdcMin);
      startThr = uint8_t(uint16_t(uart.dbgAdcMax) - (range / 4));  // ~75% of range
    }
    Serial.print("adc="); Serial.print(uart.dbgAdcMin); Serial.print(".."); Serial.print(uart.dbgAdcMax);
    Serial.print(" low="); Serial.print(uart.dbgAdcAtLow);
    Serial.print(" thr="); Serial.print(uart.adcThreshold);
    Serial.print(" sthr="); Serial.print(startThr);
    Serial.print(" mid="); Serial.print(mid);
    Serial.print(" starts="); Serial.print(uart.dbgStartBits);
    Serial.print(" false="); Serial.print(uart.dbgFalseStarts);
    Serial.print(" stopFail="); Serial.print(uart.dbgStopFails);
    Serial.print(" byteFail="); Serial.print(uart.dbgByteFails);
    Serial.print(" ok="); Serial.print(uart.dbgBytesDecoded);
    Serial.print(" rtF8="); Serial.print(uart.dbgRtClock);
    Serial.print(" rtFA="); Serial.print(uart.dbgRtStart);
    Serial.print(" rtFB="); Serial.print(uart.dbgRtContinue);
    Serial.print(" rtFC="); Serial.print(uart.dbgRtStop);
    Serial.print(" rtFE="); Serial.print(uart.dbgRtActiveSense);
    Serial.print(" ss="); Serial.print(uart.dbgStartSample);
    Serial.print(" ts="); Serial.print(uart.dbgStopSample);
    Serial.print(" bits=");
    for (uint8_t i = 0; i < 8; ++i) {
      if (i) Serial.print(",");
      Serial.print(uart.dbgBitSamples[i]);
    }
    Serial.print(" drops="); Serial.print(uart.dbgMsgDrops);
    Serial.print(" clocks="); Serial.print(g_module.midi.totalClocks);
    Serial.print(" last=0x"); Serial.print(uart.dbgLastByte, HEX);
    Serial.print(" "); Serial.println(g_module.midi.playState == MIDI_PLAYING ? "PLAY" : "STOP");
    uart.adcThreshold = (uint8_t)mid;
    uart.adcStartThreshold = startThr;
    uart.dbgAdcMin = 0xFF;
    uart.dbgAdcMax = 0x00;
    uart.dbgAdcAtLow = 0xFF;
    uart.dbgStartBits = 0;
    uart.dbgFalseStarts = 0;
    uart.dbgStopFails = 0;
    uart.dbgByteFails = 0;
    uart.dbgBytesDecoded = 0;
    uart.dbgRtClock = 0;
    uart.dbgRtStart = 0;
    uart.dbgRtContinue = 0;
    uart.dbgRtStop = 0;
    uart.dbgRtActiveSense = 0;
    uart.dbgMsgDrops = 0;
  }
  if (ENABLE_I2C_BPM && !g_module.i2cEnabled && (now - lastI2cRetryMs >= I2C_RETRY_MS)) {
    lastI2cRetryMs = now;
    const bool sdaHigh = (digitalRead(I2C_SDA_PIN) == HIGH);
    const bool sclHigh = (digitalRead(I2C_SCL_PIN) == HIGH);
    if (sdaHigh && sclHigh) {
      tryInitI2cBpmReceiver();
    }
  }

  // Safety net: reschedule gate alarm if it was missed
  ensureGateSchedulerRunning();

}

}  // namespace certainty

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
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_MIDI_RESET};

void appSetup() {
  Serial.begin(115200);
  g_module.gateAlarmId = -1;

  // Onboard RGB LED (active-low on XIAO RP2040)
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);  // off
  digitalWrite(LED_G_PIN, HIGH);  // off
  digitalWrite(LED_B_PIN, HIGH);  // off

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
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if (g_module.outputs[i].run == OUTPUT_RUN_LOOP ||
        g_module.outputs[i].run == OUTPUT_RUN_MIDI_RESET)
      g_module.outputs[i].phase = 1.0f;
  }

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

    // Snapshot all volatile counters atomically (relative to each other) to avoid
    // race where Core 1 increments between read and reset.
    const uint8_t  sAdcMin       = uart.dbgAdcMin;
    const uint8_t  sAdcMax       = uart.dbgAdcMax;
    const uint8_t  sAdcAtLow     = uart.dbgAdcAtLow;
    const uint32_t sStartBits    = uart.dbgStartBits;
    const uint32_t sFalseStarts  = uart.dbgFalseStarts;
    const uint32_t sStopFails    = uart.dbgStopFails;
    const uint32_t sByteFails    = uart.dbgByteFails;
    const uint32_t sNonRtByte    = uart.dbgNonRtByte;
    const uint32_t sBytesDecoded = uart.dbgBytesDecoded;
    const uint32_t sRtClock      = uart.dbgRtClock;
    const uint32_t sRtStart      = uart.dbgRtStart;
    const uint32_t sRtContinue   = uart.dbgRtContinue;
    const uint32_t sRtStop       = uart.dbgRtStop;
    const uint32_t sRtActiveSense = uart.dbgRtActiveSense;
    const uint32_t sMsgDrops     = uart.dbgMsgDrops;
    const uint32_t sSlopeAssists = uart.dbgSlopeAssists;
    const uint8_t  sStartSample  = uart.dbgStartSample;
    const uint8_t  sStopSample   = uart.dbgStopSample;
    uint8_t sBitSamples[8];
    for (uint8_t i = 0; i < 8; ++i) sBitSamples[i] = uart.dbgBitSamples[i];
    const uint8_t  sLastSlope     = uart.dbgLastSlope;
    const uint8_t  sLastSlopeBit  = uart.dbgLastSlopeAssistBit;
    const uint8_t  sLastSlopeExt  = uart.dbgLastSlopeExtrap;
    const uint8_t  sLastByte      = uart.dbgLastByte;
    const uint32_t sBeatBoundaries = g_module.midi.dbgBeatBoundaries;

    // Reset counters immediately after snapshot
    uart.dbgAdcMin = 0xFF;
    uart.dbgAdcMax = 0x00;
    uart.dbgAdcAtLow = 0xFF;
    uart.dbgStartBits = 0;
    uart.dbgFalseStarts = 0;
    uart.dbgStopFails = 0;
    uart.dbgByteFails = 0;
    uart.dbgNonRtByte = 0;
    uart.dbgBytesDecoded = 0;
    uart.dbgRtClock = 0;
    uart.dbgRtStart = 0;
    uart.dbgRtContinue = 0;
    uart.dbgRtStop = 0;
    uart.dbgRtActiveSense = 0;
    uart.dbgMsgDrops = 0;
    uart.dbgSlopeAssists = 0;
    g_module.midi.dbgBeatBoundaries = 0;

    const uint16_t mid = (uint16_t(sAdcMin) + uint16_t(sAdcMax)) / 2;
    Serial.print("adc="); Serial.print(sAdcMin); Serial.print(".."); Serial.print(sAdcMax);
    Serial.print(" low="); Serial.print(sAdcAtLow);
    Serial.print(" thr="); Serial.print(uart.adcThreshold);
    Serial.print(" sthr="); Serial.print(uart.adcStartThreshold);
    Serial.print(" mid="); Serial.print(mid);
    Serial.print(" starts="); Serial.print(sStartBits);
    Serial.print(" false="); Serial.print(sFalseStarts);
    Serial.print(" stopFail="); Serial.print(sStopFails);
    Serial.print(" byteFail="); Serial.print(sByteFails);
    Serial.print(" nonRt="); Serial.print(sNonRtByte);
    Serial.print(" ok="); Serial.print(sBytesDecoded);
    Serial.print(" rtF8="); Serial.print(sRtClock);
    Serial.print(" rtFA="); Serial.print(sRtStart);
    Serial.print(" rtFB="); Serial.print(sRtContinue);
    Serial.print(" rtFC="); Serial.print(sRtStop);
    Serial.print(" rtFE="); Serial.print(sRtActiveSense);
    Serial.print(" ss="); Serial.print(sStartSample);
    Serial.print(" ts="); Serial.print(sStopSample);
    Serial.print(" bits=");
    for (uint8_t i = 0; i < 8; ++i) {
      if (i) Serial.print(",");
      Serial.print(sBitSamples[i]);
    }
    Serial.print(" sa="); Serial.print(sSlopeAssists);
    Serial.print(" slo="); Serial.print(sLastSlope);
    Serial.print(" sabit="); Serial.print(sLastSlopeBit);
    Serial.print(" ext="); Serial.print(sLastSlopeExt);
    Serial.print(" drops="); Serial.print(sMsgDrops);
    Serial.print(" clocks="); Serial.print(g_module.midi.totalClocks);
    Serial.print(" last=0x"); Serial.print(sLastByte, HEX);
    {
      const MidiClockState &midi = g_module.midi;
      const uint64_t beatPeriodUs = (uint64_t)(midi.smoothedPeriodUs * (float)MIDI_RT_PPQN);
      const uint32_t bpm = beatPeriodUs > 0
          ? (uint32_t)(60000000ULL / beatPeriodUs) : 0;
      Serial.print(" bpm="); Serial.print(bpm);
      Serial.print(" bcnt="); Serial.print((uint32_t)midi.beatCount);
      Serial.print(" bdry="); Serial.print(sBeatBoundaries);
      // Phase-drift diagnostics
      Serial.print(" smth="); Serial.print((int32_t)midi.smoothedPeriodUs);
      Serial.print(" raw="); Serial.print(midi.dbgLastRawInterval);
      Serial.print(" ph0="); Serial.print(midi.dbgPhaseAtBeat, 4);
      Serial.print(" tks="); Serial.print(midi.dbgTicksBetweenBeats);
      Serial.print(" tFA="); Serial.print(midi.totalStarts);
      Serial.print(" tFC="); Serial.print(midi.totalStops);
      Serial.print(" tTO="); Serial.print(midi.totalTimeouts);
      Serial.print(" stp="); Serial.print(midi.dbgStopSource);
      Serial.print(" togMs="); Serial.print(midi.dbgTimeoutGapMs);
      Serial.print(" maxG="); Serial.print(midi.dbgMaxClockGapUs);
    }
    Serial.print(" "); Serial.println(g_module.midi.playState == MIDI_PLAYING ? "PLAY" : "STOP");

    // I2C diagnostics
    Serial.print("i2c rx="); Serial.print(g_module.i2cRxCount);
    Serial.print(" ok="); Serial.print(g_module.i2cAppliedCount);
    Serial.print(" rat="); Serial.print(g_module.i2cRatioAppliedCount);
    Serial.print(" mode="); Serial.print(g_module.i2cModeAppliedCount);
    Serial.print(" trig="); Serial.print(g_module.i2cTriggerAppliedCount);
    Serial.print(" tsat="); Serial.print(g_module.i2cTriggerSaturatedCount);
    Serial.print(" err="); Serial.print(g_module.i2cErrorCount);
    Serial.print(" drop="); Serial.print(g_module.i2cEventQueueDropCount);
    Serial.print(" dropT="); Serial.print(g_module.i2cEventQueueDropTriggerCount);
    Serial.print(" dropC="); Serial.print(g_module.i2cEventQueueDropConfigCount);
    Serial.print(" en="); Serial.print(g_module.i2cEnabled ? 1 : 0);
    // Last received I2C frame
    const uint8_t fLen = g_module.dbgLastI2cLen;
    Serial.print(" last["); Serial.print(fLen); Serial.print("]=");
    for (uint8_t i = 0; i < fLen && i < I2C_RX_MAX_BYTES; ++i) {
      if (i) Serial.print(",");
      Serial.print("0x"); Serial.print(g_module.dbgLastI2cData[i], HEX);
    }
    Serial.print(g_module.dbgLastI2cDecodeOk ? " OK" : " FAIL");
    Serial.println();
  }
  if (ENABLE_I2C_BPM && !g_module.i2cEnabled && (now - lastI2cRetryMs >= I2C_RETRY_MS)) {
    lastI2cRetryMs = now;
    const bool sdaHigh = (digitalRead(I2C_SDA_PIN) == HIGH);
    const bool sclHigh = (digitalRead(I2C_SCL_PIN) == HIGH);
    if (sdaHigh && sclHigh) {
      tryInitI2cBpmReceiver();
    }
  }

  // LED flash on I2C: green = good decode, red = bad decode.
  // Error takes priority. Flag set from ISR; GPIO driven here.
  if (g_module.ledPendingErr) {
    g_module.ledPendingErr = false;
    g_module.ledPendingOk  = false;
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_R_PIN, LOW);
    g_module.ledOffMs = now + 60;
  } else if (g_module.ledPendingOk) {
    g_module.ledPendingOk = false;
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, LOW);
    g_module.ledOffMs = now + 60;
  }
  if (g_module.ledOffMs != 0 && now >= g_module.ledOffMs) {
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    g_module.ledOffMs = 0;
  }

  // Safety net: reschedule gate alarm if it was missed
  ensureGateSchedulerRunning();

}

}  // namespace certainty

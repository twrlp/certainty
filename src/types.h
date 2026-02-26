#pragma once

#include <Arduino.h>

#include "config.h"

namespace certainty {

enum OutputShape : uint8_t {
  OUTPUT_SHAPE_TRIG = 0,
  OUTPUT_SHAPE_ASR = 1,
};

enum OutputRun : uint8_t {
  OUTPUT_RUN_ONE_SHOT = 0,
  OUTPUT_RUN_LOOP = 1,
};

struct Ratio {
  uint16_t num;
  uint16_t den;
};

struct OutputState {
  uint8_t pin;
  Ratio ratio;
  OutputShape shape;
  OutputRun run;

  uint64_t periodUs;
  uint64_t lfoAnchorUs;
  uint8_t loopProbPercent;
  uint64_t loopCycleIndex;
  bool loopCycleActive;
  uint8_t asrSus;
  uint8_t asrSkew;
  uint8_t asrA;
  uint8_t asrS;
  uint8_t asrR;
  bool ratioPending;
  uint64_t pendingApplyUs;
  Ratio pendingRatio;
  bool behaviorPending;
  uint64_t behaviorPendingApplyUs;
  OutputShape pendingShape;
  OutputRun pendingRun;
  uint16_t pendingTriggerCount;
  bool asrOneShotActive;
  uint64_t asrOneShotStartUs;
  uint64_t asrOneShotDurationUs;
  uint64_t asrAttackUs;
  uint64_t asrSustainUs;
  uint64_t asrReleaseUs;
  uint64_t asrOneShotAttackUs;
  uint64_t asrOneShotSustainUs;
  uint64_t asrOneShotReleaseUs;
  uint64_t nextRiseUs;
  uint64_t nextFallUs;
  bool gateHigh;
  uint32_t gateRises;
  uint32_t gateFalls;

  uint8_t pwmSlice;
  uint8_t pwmChannel;
  uint16_t lastPwmLevel;
  uint32_t lfoUpdates;
};

struct TransportState {
  uint32_t bpm;
  uint64_t beatPeriodUs;
  uint64_t anchorUs;
  uint32_t resetCount;
};

enum I2cEventType : uint8_t {
  I2C_EVENT_SET_BPM = 0,
  I2C_EVENT_SET_RATIO = 1,
  I2C_EVENT_SET_MODE = 2,
  I2C_EVENT_SET_ASR = 3,
  I2C_EVENT_TRIGGER = 4,
  I2C_EVENT_SET_PROB = 5,
};

struct I2cEvent {
  I2cEventType type;
  uint8_t out;
  uint8_t a;
  uint8_t b;
  uint8_t mask;
  uint16_t count;
};

struct I2cRxFrame {
  uint8_t len;
  uint8_t data[I2C_RX_MAX_BYTES];
};

}  // namespace certainty

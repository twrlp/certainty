#pragma once

#include <Arduino.h>

namespace certainty {

static const uint8_t NUM_OUTPUTS = 8;
static const uint8_t OUTPUT_PINS[NUM_OUTPUTS] = {27, 28, 29, 0, 3, 4, 2, 1};

static const uint32_t DEFAULT_BPM = 90;
static const uint32_t MIN_BPM = 20;
static const uint32_t MAX_BPM = 320;
static const uint32_t GATE_PULSE_US = 10000;
static const uint32_t START_DELAY_US = 0;
static const uint32_t HEARTBEAT_MS = 500;
static const bool ENABLE_I2C_ACTIVITY_LED = true;
static const uint32_t I2C_ACTIVITY_PULSE_MS = 20;

static const uint16_t PWM_WRAP = 255;
static const uint8_t PWM_CLKDIV_INT = 12;
static const uint8_t PWM_CLKDIV_FRAC = 3;
static const uint32_t PWM_SAMPLE_RATE_HZ = 4000;
static const int64_t PWM_SAMPLE_PERIOD_US = 1000000 / PWM_SAMPLE_RATE_HZ;

static const uint8_t I2C_ADDRESS = 0x55;
static const uint8_t I2C_SDA_PIN = 6;
static const uint8_t I2C_SCL_PIN = 7;
static const bool ENABLE_I2C_BPM = true;
static const uint32_t I2C_RETRY_MS = 250;
static const uint8_t I2C_IRQ_PRIORITY = 0x40;
static const uint8_t TIMER_IRQ_PRIORITY = 0x80;

static const uint8_t I2C_RX_MAX_BYTES = 6;
static const uint8_t I2C_RX_QUEUE_LEN = 128;
static const uint32_t I2C_ERROR_LATCH_MS = 180;

}  // namespace certainty

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
static const uint32_t PWM_SAMPLE_RATE_HZ = 8000;
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

static const uint8_t  CLK_IN_PIN             = 26;

static const bool     ENABLE_MIDI_CLOCK      = true;
static const uint8_t  MIDI_ADC_THRESHOLD     = 160;   // 8-bit: low threshold to handle slow optocoupler recovery
static const uint32_t MIDI_TIMEOUT_US        = 2000000;
static const uint8_t  MIDI_ADC_BUF_LOG2     = 8;      // 2^8 = 256 bytes
static const uint32_t MIDI_ADC_BUF_SIZE     = (1u << MIDI_ADC_BUF_LOG2);
static const uint16_t MIDI_ADC_CLKDIV       = 0;      // CLKDIV=0 → no extra division; ADC takes 96 cycles at 48MHz → 500kHz
static const uint8_t  MIDI_SAMPLES_PER_BIT  = 16;    // 500kHz / 31250 baud
static const uint8_t  MIDI_RT_PPQN          = 24;    // MIDI Real-Time clock pulses per quarter note

}  // namespace certainty

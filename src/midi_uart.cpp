#include "midi_uart.h"

#include "hardware/adc.h"
#include "hardware/dma.h"

#include "module_state.h"
#include "config.h"

namespace certainty {

static uint8_t midi_adc_buf[MIDI_ADC_BUF_SIZE]
    __attribute__((aligned(MIDI_ADC_BUF_SIZE)));

// Reload value for DMA channel chaining — must be in memory for control channel to read
static uint32_t dma_reload_count = 0xFFFFFFFF;

void initMidiDma() {
  adc_init();
  adc_gpio_init(CLK_IN_PIN);
  adc_select_input(0);

  // 8-bit samples, FIFO enabled, DMA DREQ enabled
  adc_fifo_setup(true, true, 1, false, true);
  adc_set_clkdiv(MIDI_ADC_CLKDIV);  // 500kHz

  // --- Data channel: ADC FIFO → ring buffer ---
  int dataCh = dma_claim_unused_channel(true);
  int ctrlCh = dma_claim_unused_channel(true);
  g_module.midiUart.dataChannel = dataCh;
  g_module.midiUart.ctrlChannel = ctrlCh;

  dma_channel_config dataCfg = dma_channel_get_default_config(dataCh);
  channel_config_set_transfer_data_size(&dataCfg, DMA_SIZE_8);
  channel_config_set_read_increment(&dataCfg, false);
  channel_config_set_write_increment(&dataCfg, true);
  channel_config_set_ring(&dataCfg, true, MIDI_ADC_BUF_LOG2);
  channel_config_set_dreq(&dataCfg, DREQ_ADC);
  channel_config_set_chain_to(&dataCfg, ctrlCh);  // when done → trigger ctrl channel

  dma_channel_configure(dataCh, &dataCfg,
      midi_adc_buf,       // dst (ring wraps within 256 bytes)
      &adc_hw->fifo,      // src
      0xFFFFFFFF,          // max count; ctrl channel reloads before it stops
      false);              // don't start yet

  // --- Control channel: reloads data channel's transfer count ---
  // Writes dma_reload_count → data channel's al1_transfer_count_trig (restarts it)
  dma_channel_config ctrlCfg = dma_channel_get_default_config(ctrlCh);
  channel_config_set_transfer_data_size(&ctrlCfg, DMA_SIZE_32);
  channel_config_set_read_increment(&ctrlCfg, false);
  channel_config_set_write_increment(&ctrlCfg, false);
  channel_config_set_chain_to(&ctrlCfg, dataCh);  // chain back (though TRIG already restarts)

  dma_channel_configure(ctrlCh, &ctrlCfg,
      &dma_hw->ch[dataCh].al1_transfer_count_trig,  // dst: restarts data channel
      &dma_reload_count,                              // src: reload value
      1,                                              // single 32-bit write
      false);                                         // don't start yet

  g_module.midiUart.phase = MIDI_UART_IDLE;
  g_module.midiUart.bitVoteMarkCount = 0;
  g_module.midiUart.errorGuardCount = 0;
  g_module.midiUart.readIdx = 0;
  g_module.midiUart.dbgAdcMin = 0xFF;
  g_module.midiUart.dbgAdcMax = 0x00;
  g_module.midiUart.dbgAdcAtLow = 0xFF;
  g_module.midiUart.dbgStartBits = 0;
  g_module.midiUart.dbgFalseStarts = 0;
  g_module.midiUart.dbgStopFails = 0;
  g_module.midiUart.dbgByteFails = 0;
  g_module.midiUart.dbgNonRtByte = 0;
  g_module.midiUart.dbgBytesDecoded = 0;
  g_module.midiUart.dbgRtClock = 0;
  g_module.midiUart.dbgRtStart = 0;
  g_module.midiUart.dbgRtContinue = 0;
  g_module.midiUart.dbgRtStop = 0;
  g_module.midiUart.dbgRtActiveSense = 0;
  g_module.midiUart.adcThreshold = MIDI_ADC_THRESHOLD;
  g_module.midiUart.adcStartThreshold = MIDI_ADC_THRESHOLD;
  g_module.midiUart.adcWinMin = 0xFF;
  g_module.midiUart.adcWinMax = 0x00;
  g_module.midiUart.adcSampleCount = 0;
  g_module.midiUart.startDebounceCount = 0;
  g_module.midiUart.prevStartLow = false;
  g_module.midiUart.vote1Raw = 0;
  g_module.midiUart.dbgSlopeAssists = 0;
  g_module.midiUart.dbgLastSlope = 0;
  g_module.midiUart.dbgLastSlopeAssistBit = 0;
  g_module.midiUart.dbgLastSlopeExtrap = 0;
  g_module.midiUart.msgHead = 0;
  g_module.midiUart.msgTail = 0;
  g_module.midiUart.dbgMsgDrops = 0;
  g_module.midiUart.dbgStartSample = 0;
  g_module.midiUart.dbgStopSample = 0;
  for (uint8_t i = 0; i < 8; ++i) g_module.midiUart.dbgBitSamples[i] = 0;

  // Start data channel, then ADC
  dma_channel_start(dataCh);
  adc_run(true);
}

// Forward declaration — implemented in midi_clock.cpp; used by drainMidiMessages
void onMidiMessage(uint8_t msg, uint64_t nowUs);

void processMidiAdcSamples() {
  MidiUartState &uart = g_module.midiUart;

  // Find how far DMA has written
  const uint32_t writeAddr = dma_channel_hw_addr(uart.dataChannel)->write_addr;
  const uint32_t writeIdx =
      (uint32_t)(writeAddr - (uint32_t)(uintptr_t)midi_adc_buf)
      & (MIDI_ADC_BUF_SIZE - 1);

  // Process all new samples
  while (uart.readIdx != writeIdx) {
    const uint8_t sample = midi_adc_buf[uart.readIdx];
    uart.readIdx = (uart.readIdx + 1) & (MIDI_ADC_BUF_SIZE - 1);
    if (sample < uart.dbgAdcMin) uart.dbgAdcMin = sample;
    if (sample > uart.dbgAdcMax) uart.dbgAdcMax = sample;

    // Threshold adaptation — runs entirely on Core 1, no cross-core writes.
    // Update every 250,000 samples (~500ms at 500kHz ADC rate).
    if (sample < uart.adcWinMin) uart.adcWinMin = sample;
    if (sample > uart.adcWinMax) uart.adcWinMax = sample;
    if (++uart.adcSampleCount >= 250000) {
      const uint16_t range = uart.adcWinMax >= uart.adcWinMin
          ? uint16_t(uart.adcWinMax) - uint16_t(uart.adcWinMin) : 0;
      if (range >= 50) {
        uart.adcThreshold = uint8_t(uint16_t(uart.adcWinMin) +
            (uint16_t(uart.adcWinMax) - uint16_t(uart.adcWinMin)) * MIDI_ADC_THRESHOLD_PCT / 100);
        uart.adcStartThreshold = uint8_t(uint16_t(uart.adcWinMax) - range / 4);
      } else {
        uart.adcThreshold = MIDI_ADC_THRESHOLD;
        uart.adcStartThreshold = MIDI_ADC_THRESHOLD;
      }
      uart.adcWinMin = 0xFF;
      uart.adcWinMax = 0x00;
      uart.adcSampleCount = 0;
    }

    const bool mark = (sample >= uart.adcThreshold);
    const bool startLow = (sample < uart.adcStartThreshold);
    if (!mark && uart.phase != MIDI_UART_IDLE) {
      if (sample < uart.dbgAdcAtLow) uart.dbgAdcAtLow = sample;
    }

    switch (uart.phase) {
      case MIDI_UART_IDLE:
        if (uart.errorGuardCount > 0) {
          if (mark) uart.errorGuardCount--;  // count down only while line is idle (mark)
        } else if (startLow && !uart.prevStartLow) {  // falling edge detected
          uart.startDebounceCount = 1;
        } else if (startLow && uart.startDebounceCount > 0) {
          uart.startDebounceCount++;
          if (uart.startDebounceCount >= MIDI_START_DEBOUNCE) {
            uart.phase = MIDI_UART_START_VERIFY;
            uart.sampleCount = MIDI_START_DEBOUNCE - 1;  // compensate: debounce already consumed N samples of start bit
            uart.dbgStartBits++;
            uart.startDebounceCount = 0;
          }
        } else {
          uart.startDebounceCount = 0;  // line went high, reset
        }
        break;

      case MIDI_UART_START_VERIFY:
        uart.sampleCount++;
        if (uart.sampleCount == (MIDI_SAMPLES_PER_BIT / 2)) {  // mid start bit — verify still LOW
          uart.dbgStartSample = sample;
          if (mark) {  // went HIGH — false start
            uart.phase = MIDI_UART_IDLE;
            uart.dbgFalseStarts++;
          }
          // else: confirmed still LOW, continue waiting for end of start bit
        } else if (uart.sampleCount == MIDI_SAMPLES_PER_BIT) {  // end of full start bit
          uart.phase = MIDI_UART_DATA;
          uart.sampleCount = 0;
          uart.bitIndex = 0;
          uart.byte = 0;
          uart.bitVoteMarkCount = 0;
        }
        break;

      case MIDI_UART_DATA: {
        uart.sampleCount++;
        // Majority vote across 3 evenly-spaced samples per bit (positions SPB/4, SPB/2, 3*SPB/4).
        // Two of three must agree; single-sample noise spikes cannot flip a bit.
        const uint8_t posInBit = uart.sampleCount % MIDI_SAMPLES_PER_BIT;
        if (posInBit == MIDI_SAMPLES_PER_BIT / 4 ||
            posInBit == MIDI_SAMPLES_PER_BIT / 2 ||
            posInBit == 3 * MIDI_SAMPLES_PER_BIT / 4) {
          if (mark) uart.bitVoteMarkCount++;
          if (posInBit == MIDI_SAMPLES_PER_BIT / 4) {
            uart.vote1Raw = sample;  // save v1 for slope assist at deciding vote
          }
          if (posInBit == 3 * MIDI_SAMPLES_PER_BIT / 4) {  // last vote: decide bit
            uart.dbgBitSamples[uart.bitIndex] = sample;
            bool bitIsMark = (uart.bitVoteMarkCount >= 2);
            if (!bitIsMark && sample >= uart.vote1Raw) {
              // Slope assist: project v3 forward by SPB/4 samples.
              // slope = v3 - v1 over SPB/2 (8 samples); extrap = v3 + slope/2
              const uint8_t slope = sample - uart.vote1Raw;
              const uint16_t vExtrap = uint16_t(sample) + slope / 2;
              if (slope >= MIDI_SLOPE_MIN && vExtrap >= uart.adcThreshold) {
                bitIsMark = true;
                uart.dbgSlopeAssists++;
                uart.dbgLastSlope = slope;
                uart.dbgLastSlopeAssistBit = uart.bitIndex;
                uart.dbgLastSlopeExtrap = vExtrap > 255 ? 255 : (uint8_t)vExtrap;
              }
            }
            if (bitIsMark) uart.byte |= (1u << uart.bitIndex);
            uart.bitVoteMarkCount = 0;
            uart.bitIndex++;
            if (uart.bitIndex == 8) {
              uart.phase = MIDI_UART_STOP;
              uart.sampleCount = 0;
            }
          }
        }
        break;
      }

      case MIDI_UART_STOP:
        uart.sampleCount++;
        if (uart.sampleCount == (MIDI_SAMPLES_PER_BIT / 2)) {  // mid stop bit
          uart.dbgStopSample = sample;
          if (!mark) {
            uart.dbgStopFails++;
            uart.errorGuardCount = MIDI_SAMPLES_PER_BIT;  // require idle line before next start
          } else if (uart.byte >= 0xF0 && uart.byte < 0xF8) {
            uart.dbgNonRtByte++;  // valid non-RT system message (SysEx, MTC, etc.) — ignored by design
          } else if (uart.byte < 0xF0) {
            uart.dbgByteFails++;  // byte < 0xF0: likely bit error in an RT message
            uart.errorGuardCount = MIDI_SAMPLES_PER_BIT;  // prevent cascading spurious starts
          } else {
            uart.dbgBytesDecoded++;
            uart.dbgLastByte = uart.byte;
            switch (uart.byte) {
              case 0xF8: uart.dbgRtClock++; break;
              case 0xFA: uart.dbgRtStart++; break;
              case 0xFB: uart.dbgRtContinue++; break;
              case 0xFC: uart.dbgRtStop++; break;
              case 0xFE: uart.dbgRtActiveSense++; break;
              default: break;
            }
            // Push decoded byte + decode timestamp to SPSC ring buffer for Core 0 to drain.
            {
              const uint8_t next = (uart.msgHead + 1u) & 0x0Fu;
              if (next != uart.msgTail) {
                uart.msgBuf[uart.msgHead]     = uart.byte;
                uart.msgTimeBuf[uart.msgHead] = time_us_64();
                __dmb();  // ensure byte and timestamp are visible before head advances
                uart.msgHead = next;
              }
              else { uart.dbgMsgDrops++; }  // buffer full — should not occur at RT-only MIDI rates
            }
          }
          uart.phase = MIDI_UART_IDLE;
        }
        break;
    }
    uart.prevStartLow = startLow;
  }
}

void runMidiUartCore1() {
  while (true) {
    processMidiAdcSamples();
  }
}

void drainMidiMessages(uint64_t nowUs) {
  MidiUartState &uart = g_module.midiUart;
  while (uart.msgHead != uart.msgTail) {
    const uint8_t  byte  = uart.msgBuf[uart.msgTail];
    const uint64_t msgUs = uart.msgTimeBuf[uart.msgTail];
    __dmb();  // ensure reads complete before tail advances
    uart.msgTail = (uart.msgTail + 1u) & 0x0Fu;
    onMidiMessage(byte, msgUs);
  }
}

}  // namespace certainty

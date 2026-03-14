# Future Improvements

## MIDI UART: Lower adaptive threshold for slow-rising signals

Currently `adcThreshold` is set to the midpoint (50th percentile) of the observed ADC range
every 500ms in `appLoop`:

```cpp
uart.adcThreshold = (uint8_t)mid;  // mid = (dbgAdcMin + dbgAdcMax) / 2
```

Some MIDI sources have very slow optocoupler rise times — the signal can take 7–8 full bit
periods (~240µs) to reach its settled HIGH level. For 0xF8 (`11111000` LSB-first), bit 3 is
the first HIGH bit after three LOW bits. With a midpoint threshold, this bit's three majority-
vote samples (at positions 4, 8, 12 of the 16-sample bit period) are all still below threshold,
so it decodes as SPACE and the byte comes out as 0xF0 → byteFail. Nearly every 0xF8 is lost.

**Proposed fix:** use the 25th–30th percentile of the range instead of the 50th:

```cpp
const uint16_t range = uint16_t(uart.dbgAdcMax) - uint16_t(uart.dbgAdcMin);
uart.adcThreshold = uint8_t(uint16_t(uart.dbgAdcMin) + range / 4);  // 25th percentile
```

**Diagnostic signature of this problem in the debug log:**
- Narrow ADC range (~60 counts vs ~120 for a healthy source)
- `bits` progression gradually climbing across a single byte, e.g. `130,130,130,155,172,180,184,186`
- `byteFail` count roughly equal to `starts` count (almost every decode fails)
- `rtF8` near zero despite a running MIDI clock source
- `clocks` counter barely incrementing

**Why it is safe for clean sources:** A source with wide signal swing (e.g. adc=129..251,
range=122) would get threshold ≈ 159 instead of 190. SPACE bits at ~130 still have 29 counts
of margin; MARK bits at ~249 gain margin. No decoding impact.

**Risk:** Sources with a high settled SPACE level (optocoupler that doesn't pull fully low)
could see false MARKs if the threshold drops too close to the SPACE floor. Monitor `dbgAdcMin`
and `low` in the debug print; if the SPACE floor is significantly above 0, a 25th-percentile
threshold may be too low. A configurable fraction or a floor clamp could mitigate this.

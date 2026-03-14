#include "src/app.h"
#include "src/midi_uart.h"

void setup() {
  certainty::appSetup();
}

void loop() {
  certainty::appLoop();
}

void setup1() {}
void loop1()  { certainty::runMidiUartCore1(); }

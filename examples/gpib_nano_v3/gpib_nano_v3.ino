/*
  HP GPIB (General Purpose Interface Bus) Low-Level Protocol Driver

  This sketch provides a high-level command interface for GPIB control. It
  implements non-blocking state machines for protocol-level commands like
  initializing the bus, writing data, and listening for data.

 --- Serial Command Reference ---
  All commands must begin with a '*' and be terminated with a newline (Enter) or comma (,).

  ** Protocol Commands **
  - *INIT <addr>: Sends an Interface Clear (IFC), enables remote control (REN),
    and sends UNLISTEN and UNTALK commands.
  - *WRITE <string>: Sends a string to the bus, asserting EOI on the last character.
    including setting the talker and listener.
  - *LISTEN: Configures the bus by commanding the previously initialized device
    to TALK, then puts the controller into a listening state to receive a data
    string. The operation completes when EOI is detected or after a timeout.
*/

// Include the library with helper functions and pin definitions

#include <GPIBnano.h>

void setup() {
  Serial.begin(115200);
  gpibNano.begin();
}

void loop() {
  gpibNano.processGPIB();
  if(gpibNano.isResult()){
    Serial.println(gpibNano.result());
  }
}

#include "powerboard.h"


void setup() {
  Serial.begin(9600);
  Serial.println("PowerBoard Setup");

  for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
    bus[i].init();
  }

  Serial.println("Initializing RoveComm...");
  RoveComm.begin(RC_POWERBOARD_FIRSTOCTET, RC_POWERBOARD_SECONDOCTET, RC_POWERBOARD_THIRDOCTET, RC_POWERBOARD_FOURTHOCTET, &TCPServer);
  Serial.println("Complete");

  Telemetry.begin(telemetry, TELEMETRY_PERIOD);
}


void loop() {

  packet = RoveComm.read();

  switch(packet.data_id) {

    // Enable
    case RC_POWERBOARD_ENABLEBUS_DATA_ID:
    {
      uint16_t data = *((uint16_t*) packet.data);
      for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
        if(data & (1<<i)) {
          bus[i].enable();
        }
      }
    }

    // Disable
    case RC_POWERBOARD_DISABLEBUS_DATA_ID:
    {
      uint16_t data = *((uint16_t*) packet.data);
      for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
        if(data & (1<<i)) {
          bus[i].disable();
        }
      }
    }

    // Set
    case RC_POWERBOARD_SETBUS_DATA_ID:
    {
      uint16_t data = *((uint16_t*) packet.data);
      for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
        if(data & (1<<i)) {
          bus[i].enable();
        } else {
          bus[i].disable();
        }
      }
    }

  }

}


void telemetry() {
  busStatus = 0;
  for (unit8_t i = 0; i < NUM_TOGGLEABLE; i++) {
    if(bus[i].enabled()) {
      busStatus &= (1<<i);
    }
  }
  RoveComm.write(RC_POWERBOARD_BUSSTATUS_DATA_ID, RC_POWERBOARD_BUSSTATUS_DATA_COUNT, busStatus);
  
  for (unit8_t i = 0; i < NUM_BUSSES; i++) {
    if(bus[i].overcurrent()) {
      currents[i] = bus[i].readCurrent();
    }
  }
  RoveComm.write(RC_POWERBOARD_BUSCURRENT_DATA_ID, RC_POWERBOARD_BUSCURRENT_DATA_COUNT, currents);

  overcurrent = 0;
  for (unit8_t i = 0; i < NUM_BUSSES; i++) {
    if(bus[i].overcurrent()) {
      overcurrent &= (1<<i);
    }
  }
  RoveComm.write(RC_POWERBOARD_BUSOVERCURRENT_DATA_ID, RC_POWERBOARD_BUSOVERCURRENT_DATA_COUNT, overcurrent);
}
#include "powerboard.h"
#include "pinassignments.h"
#include "bus.h"

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  Serial.println("PowerBoard Setup");


  for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
    bus[i].init();
  }

  RoveComm.begin(RC_POWERBOARD_FIRSTOCTET, RC_POWERBOARD_SECONDOCTET, RC_POWERBOARD_THIRDOCTET, RC_POWERBOARD_FOURTHOCTET, &TCPServer);
  Telemetry.begin(telemetry, TELEMETRY_UPDATE);
}

void loop() {
  // put your main code here, to run repeatedly:

switch(packet.data_id) {
  case RC_POWERBOARD_BUSENABLE_DATA_ID:
  {
    // Enable
    packet = RoveComm.read();
    uint16_t data = *((uint16_t*) packet.data);
    for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
      if(data & (1<<i)) {
        bus[i].enable();
      };
  }
  }
  case RC_POWERBOARD_BUSDISABLE_DATA_ID:
  {
    // Disable
    packet = RoveComm.read();
    uint16_t data = *((uint16_t*) packet.data);
    for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
      if(data & (1<<i)) {
        bus[i].disable();
      };
  }
  }
  case RC_POWERBOARD_BUSSET_DATA_ID:
  {
    // Set
    packet = RoveComm.read();
    uint16_t data = *((uint16_t*) packet.data);
    for (uint8_t i = 0; i < NUM_TOGGLEABLE; i++) {
      if(data & (1<<i)) {
        bus[i].enable();
      } else {
        bus[i].disable();
      };
  }
  }
}
  

  

  
}

void telemetry() {
  busStatus = 0;
  for (unit8_t i = 0; i < NUM_TOGGLEABLE; i++) {
    if(bus[i].enabled()) {
      busStatus &= (1<<i);
    };
  }
  RoveComm.write(RC_POWERBOARD_BUSSTATUS_DATA_ID, RC_POWERBOARD_BUSSTATUS_DATA_COUNT, busStatus);
  
  for (unit8_t i = 0; i < NUM_BUSSES; i++) {
    if(bus[i].overcurrent()) {
      currents[i] = bus[i].readCurrent();
    };
  }
  RoveComm.write(RC_POWERBOARD_BUSCURRENT_DATA_ID, RC_POWERBOARD_BUSCURRENT_DATA_COUNT, currents);
  overcurrent = 0;
  for (unit8_t i = 0; i < NUM_BUSSES; i++) {
    if(bus[i].overcurrent()) {
      overcurrent &= (1<<i);
    };
  }
  RoveComm.write(RC_POWERBOARD_BUSOVERCURRENT_DATA_ID, RC_POWERBOARD_BUSOVERCURRENT_DATA_COUNT, overcurrent);
  };
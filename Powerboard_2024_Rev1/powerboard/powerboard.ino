#include "powerboard.h"
#include "pinassignments.h"
#include "bus.h"

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  Serial.println("PowerBoard Setup");

  busSetup();

  RoveComm.begin(RC_POWERBOARD_FIRSTOCTET, RC_POWERBOARD_SECONDOCTET, RC_POWERBOARD_THIRDOCTET, RC_POWERBOARD_FOURTHOCTET, &TCPServer);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void busSetup(){
  bus[0] = bus(M1_CTL, M1_CS, OVERCURRENT_HIGH);
  bus[1] = bus(M2_CTL, M2_CS, OVERCURRENT_HIGH);
  bus[2] = bus(M3_CTL, M3_CS, OVERCURRENT_HIGH);
  bus[3] = bus(M4_CTL, M4_CS, OVERCURRENT_HIGH);
  bus[4] = bus(M5_CTL, M5_CS, OVERCURRENT_HIGH);
  bus[5] = bus(M6_CTL, M6_CS, OVERCURRENT_HIGH);
  bus[6] = bus(MS_CTL, MS_CS, OVERCURRENT_HIGH);
  bus[7] = bus(AUX_CTL, AUX_CS, OVERCURRENT_HIGH);
  bus[8] = bus(HC_SPARE_CTL, HC_SPARE_CS, OVERCURRENT_HIGH);
  bus[9] = bus(ROUTERPI_CTL, ROUTERPI_CS, OVERCURRENT_LOW);
  bus[10] = bus(DIFFGPS_CTL, DIFFGPS_CS, OVERCURRENT_LOW);
  bus[11] = bus(CORE_CTL, CORE_CS, OVERCURRENT_LOW);
  bus[12] = bus(LC_SPARE_CTL, LC_SPARE_CS, OVERCURRENT_LOW);
  bus[13] = bus(CAM_CTL, CAM_CS, OVERCURRENT_LOW);

  for(int i = 0; i < NUM_BUS; i++){
    pinMode(bus[i].ctl_pin, OUTPUT);
  }
}

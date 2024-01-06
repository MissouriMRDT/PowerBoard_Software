#ifndef powerboard_h
#define powerboard_h

#include "pinassignments.h"
#include "bus.h"

#include <RoveComm.h>

#define TELEMETRY_UPDATE        150000
IntervalTimer telemetry;

#define WATCHDOG_TIMEOUT_TELEOP         300000
#define WATCHDOG_TIMEOUT_AUTONOMY       1500000
IntervalTimer watchdog;
bool watchdogOverride = false;
uint8_t watchdogMode = 0; // 0: Teleop, 1: Autonomy

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
rovecomm_packet packet;

// Number of Ports
#define NUM_MOTORS              7   // Motors (1-6) plus spare
#define NUM_HIGH_CURRENT        2   // Aux, Spare (20A)
#define NUM_LOW_CURRENT         4   // RouterPi, DiffGPS, Core, Spare (5A)
#define NUM_12V                 1   // Cam
#define NUM_NETWORKING          4   // POE, Net Switch 1, Net Switch 2, Net Switch 3
#define NUM_BUS                 18  // Total number of ports
#define NUM_BOARDS              1   // Total number of boards on rover to receive telemetry from, not including powerboard

// Current Sensing Limits
#define OVERCURRENT_HIGH        19500 //mA
#define OVERCURRENT_LOW         4875  //mA

#define HIGH_CURRENT_mA_MAX     22000 //mA
#define LOW_CURRENT_mA_MAX      5500  //mA

// Arrays
float motorCurrents[NUM_MOTORS] = {};
float highCurrents[NUM_HIGH_CURRENT] = {};
float lowCurrents[NUM_LOW_CURRENT] = {};
float lowVoltCurrents[NUM_12V] = {};
float networkingCurrents[NUM_NETWORKING] = {};

uint8_t motorOverCurrent = 0;
uint8_t highOverCurrent = 0;
uint8_t lowOverCurrent = 0;
uint8_t lowVoltOverCurrent = 0;
uint8_t networkingOverCurrent = 0;

Bus bus[NUM_BUS];

void busSetup();

#endif
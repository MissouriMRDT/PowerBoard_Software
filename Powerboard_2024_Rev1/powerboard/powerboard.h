#ifndef powerboard_h
#define powerboard_h

#include "pinassignments.h"
#include "bus.h"

#include <RoveComm.h>

#define TELEMETRY_UPDATE        100000
IntervalTimer telemetry;

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
rovecomm_packet packet;

// Number of Ports
#define NUM_TOGGLEABLE          14
#define NUM_NONTOGGLEABLE       4
#define NUM_BUSSES                 (NUM_TOGGLEABLE + NUM_NONTOGGLEABLE)

// Current Sensing Limits
#define HIGH_CURRENT_MAX     22000 //mA
#define LOW_CURRENT_MAX      5500  //mA

// Arrays
float currents[NUM_BUSSES];

uint16_t busStatus = 0;
uint32_t overcurrent = 0;

Bus bus[NUM_BUSSES] = {
    Bus(M1_CTL, M1_CS, HIGH_CURRENT_MAX),
    Bus(M2_CTL, M2_CS, HIGH_CURRENT_MAX),
    Bus(M3_CTL, M3_CS, HIGH_CURRENT_MAX),
    Bus(M4_CTL, M4_CS, HIGH_CURRENT_MAX),
    Bus(M5_CTL, M5_CS, HIGH_CURRENT_MAX),
    Bus(M6_CTL, M6_CS, HIGH_CURRENT_MAX),
    Bus(MS_CTL, MS_CS, HIGH_CURRENT_MAX),
    Bus(AUX_CTL, AUX_CS, HIGH_CURRENT_MAX),
    Bus(HC_SPARE_CTL, HC_SPARE_CS, HIGH_CURRENT_MAX),
    Bus(ROUTERPI_CTL, ROUTERPI_CS, LOW_CURRENT_MAX),
    Bus(DIFFGPS_CTL, DIFFGPS_CS, LOW_CURRENT_MAX),
    Bus(CORE_CTL, CORE_CS, LOW_CURRENT_MAX),
    Bus(LC_SPARE_CTL, LC_SPARE_CS, LOW_CURRENT_MAX),
    Bus(CAM_CTL, CAM_CS, LOW_CURRENT_MAX),
    Bus(NS1_CS, LOW_CURRENT_MAX),
    Bus(NS2_CS, LOW_CURRENT_MAX),
    Bus(NS3_CS, LOW_CURRENT_MAX),
    Bus(POE_CS, LOW_CURRENT_MAX),
};

#endif
#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"

// Numbers of Toggleable Ports
#define NUM_12V_PORTS           8   // Everything except Aux cuz Manifest
#define NUM_MOTORS              7       

//12V Current Sensing
/*
#define MULTIMEDIA_SENSE        PD4
#define NAV_SENSE               PD0
#define GIMBAL_ACT_SENSE        PD1
#define DRIVE_SENSE             PK3
#define SCISENSOR_ACT_SENSE     PD2
#define NETSWITCH_SENSE         PB5
#define CAM1_SENSE              PB4
#define CAM2_SENSE              PK0
#define BBB_SENSE               PD7
#define AUX_LOG_SENSE           PK2
*/

//12V CTL
#define SPARE_CTL               0
#define NAV_CTL                 1
#define MULTIMEDIA_CTL          2
#define GIMBAL_CTL              3
#define DRIVE_CTL               4
#define CAM2_CTL                5
#define CAM1_CTL                6
#define BBB_CTL                 7
#define AUX_CTL                 8

/*
//PACK CURRENT
#define P_MOTOR1_SENSE          PE4
#define P_MOTOR2_SENSE          PE5
#define P_MOTOR3_SENSE          PD3  
#define P_MOTOR4_SENSE          PE0
#define P_MOTOR5_SENSE          PE1
#define P_MOTOR6_SENSE          PE2
#define P_MOTOR7_SENSE          PE3
#define P_POE_SENSE             PD5
#define P_AUX_SENSE             PK1
*/

//PACK BUSSES
#define POE_CTL                 29
#define PACK_SPARE_CTL          37

#define MOTOR1_CTL              33       
#define MOTOR2_CTL              32
#define MOTOR3_CTL              34
#define MOTOR4_CTL              31
#define MOTOR5_CTL              35
#define MOTOR6_CTL              30
#define MOTORS_CTL              36

#define MOTOR_DELAY             2000

//uint8_t currentSense12V[] = {MULTIMEDIA_SENSE, NAV_SENSE, GIMBAL_ACT_SENSE, DRIVE_SENSE, SCISENSOR_ACT_SENSE, NETSWITCH_SENSE, CAM1_SENSE, CAM2_SENSE, BBB_SENSE, AUX_LOG_SENSE};
uint8_t twelveVoltBusses[NUM_12V_PORTS] = {GIMBAL_CTL, DRIVE_CTL, MULTIMEDIA_CTL, NAV_CTL, CAM1_CTL, CAM2_CTL, BBB_CTL, SPARE_CTL};
//uint8_t currentSensePack[9] = {P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_MOTOR5_SENSE, P_MOTOR6_SENSE, P_MOTOR7_SENSE, P_POE_SENSE, P_AUX_SENSE};
uint8_t motorBusses[NUM_MOTORS] = {MOTOR1_CTL, MOTOR2_CTL, MOTOR3_CTL, MOTOR4_CTL, MOTOR5_CTL, MOTOR6_CTL, MOTORS_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_POWERBOARD_PORT);

#endif
#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"

// Numbers of Toggleable Ports
#define NUM_12V_PORTS           8   // Everything except Aux cuz Manifest
#define NUM_MOTORS              7       

IntervalTimer Telemetry;

//12V Current Sensing

#define MULTIMEDIA_SENSE        A11
#define NAV_SENSE               A12
#define GIMBAL_ACT_SENSE        A10
#define DRIVE_SENSE             A0
#define CAM1_SENSE              A2
#define CAM2_SENSE              A1
#define BBB_SENSE               A3
#define AUX_SENSE               A4
#define SPARE_12V_SENSE         A13

#define OVERCURRENT_PACK        19500 //mA
#define OVERCURRENT_12V         4800 //mA
#define CURRENT_ADC_MIN         0
#define CURRENT_ADC_MAX         4096
#define CURRENT_mA_MIN          0
#define CURRENT_mA_MAX          20000

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


//PACK CURRENT
#define P_MOTOR1_SENSE          A14
#define P_MOTOR2_SENSE          A15
#define P_MOTOR3_SENSE          A16  
#define P_MOTOR4_SENSE          A17
#define P_MOTOR5_SENSE          A5
#define P_MOTOR6_SENSE          A6
#define P_MOTOR7_SENSE          A7
#define P_POE_SENSE             A8

// Current sensing variables

float MOTORBUSSENSEPINS[NUM_MOTORS] = {P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_MOTOR5_SENSE, P_MOTOR6_SENSE, P_MOTOR7_SENSE};
float TWELVELOGICBUSPINS[NUM_12V_PORTS] = {GIMBAL_ACT_SENSE, DRIVE_SENSE, MULTIMEDIA_SENSE, NAV_SENSE, CAM1_SENSE, CAM2_SENSE, BBB_SENSE, SPARE_12V_SENSE};

float P_MOTOR1_CURRENT = 0;
float P_MOTOR2_CURRENT = 0;
float P_MOTOR3_CURRENT = 0;
float P_MOTOR4_CURRENT = 0;
float P_MOTOR5_CURRENT = 0;
float P_MOTOR6_CURRENT = 0;
float P_MOTOR7_CURRENT = 0;

float AUX_CURRENT = 0;

float GIMBAL_ACT_CURRENT = 0;
float DRIVE_CURRENT = 0;
float MULTIMEDIA_CURRENT = 0;
float NAV_CURRENT = 0;
float CAM1_CURRENT = 0;
float CAM2_CURRENT = 0;
float BBB_CURRENT = 0;
float SPARE_12V_CURRENT = 0;

float MOTORBUSCURRENTS[NUM_MOTORS] = {P_MOTOR1_CURRENT, P_MOTOR2_CURRENT, P_MOTOR3_CURRENT, P_MOTOR4_CURRENT, P_MOTOR5_CURRENT, P_MOTOR6_CURRENT, P_MOTOR7_CURRENT};
float TWELVELOGICBUSCURRENTS[NUM_12V_PORTS] = {GIMBAL_ACT_CURRENT, DRIVE_CURRENT, MULTIMEDIA_CURRENT, NAV_CURRENT, CAM1_CURRENT, CAM2_CURRENT, BBB_CURRENT, SPARE_12V_CURRENT};

uint8_t motorOverCurrent = 0;
uint8_t twelveActOverCurrent = 0;
uint8_t twelveLogicOverCurrent = 0;

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

#define MOTOR_DELAY             500

//uint8_t currentSense12V[] = {MULTIMEDIA_SENSE, NAV_SENSE, GIMBAL_ACT_SENSE, DRIVE_SENSE, SCISENSOR_ACT_SENSE, NETSWITCH_SENSE, CAM1_SENSE, CAM2_SENSE, BBB_SENSE, AUX_LOG_SENSE};
uint8_t twelveVoltBusses[NUM_12V_PORTS] = {GIMBAL_CTL, DRIVE_CTL, MULTIMEDIA_CTL, NAV_CTL, CAM1_CTL, CAM2_CTL, BBB_CTL, SPARE_CTL};
//uint8_t currentSensePack[9] = {P_MOTOR1_SENSE, P_MOTOR2_SENSE, P_MOTOR3_SENSE, P_MOTOR4_SENSE, P_MOTOR5_SENSE, P_MOTOR6_SENSE, P_MOTOR7_SENSE, P_POE_SENSE, P_AUX_SENSE};
uint8_t motorBusses[NUM_MOTORS] = {MOTOR1_CTL, MOTOR2_CTL, MOTOR3_CTL, MOTOR4_CTL, MOTOR5_CTL, MOTOR6_CTL, MOTORS_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_POWERBOARD_PORT);

#endif
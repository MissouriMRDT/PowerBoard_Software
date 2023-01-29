#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"

// Numbers of Toggleable Ports
#define NUM_LOW_CURRENT         7   // Gimbal, Drive, Multi, Nav, Cam, BBB, Spare (1A)
#define NUM_MOTORS              7   // Motors plus spare   

// IntervalTimer Telemetry;

/*

// Current Sensing

#define OVERCURRENT_PACK        19500 //mA
#define OVERCURRENT_12V         4800 //mA
#define CURRENT_ADC_MIN         0
#define CURRENT_ADC_MAX         4096
#define CURRENT_mA_MIN          0
#define CURRENT_mA_MAX          20000

*/

// High Current CTL

#define MOTOR_1_CTL             0       
#define MOTOR_2_CTL             1
#define MOTOR_3_CTL             2
#define MOTOR_4_CTL             3
#define MOTOR_5_CTL             4
#define MOTOR_6_CTL             5
#define MOTOR_SPARE_CTL         6

#define AUX_CTL                 7
#define HIGH_CURRENT_SPARE_CTL  8

#define MOTOR_DELAY             500 // delay between initial motor port toggles, used to kill the board power when they all toggled on at once
// actual value of the delay is arbitrary

// Low Current CTL

#define CAM_CTL                 9
#define MULTIMEDIA_CTL          10
#define GIMBAL_CTL              11
#define DRIVE_CTL               12
#define LOW_CURRENT_SPARE_CTL   24
#define BBB_CTL                 25 
#define NAV_CTL                 26

/*

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

*/

// Toggle Pin Arrays

uint8_t lowCurrentPins[NUM_LOW_CURRENT] = {GIMBAL_CTL, DRIVE_CTL, MULTIMEDIA_CTL, NAV_CTL, CAM_CTL, BBB_CTL, LOW_CURRENT_SPARE_CTL};
uint8_t motorPins[NUM_MOTORS] = {MOTOR_1_CTL, MOTOR_2_CTL, MOTOR_3_CTL, MOTOR_4_CTL, MOTOR_5_CTL, MOTOR_6_CTL, MOTOR_SPARE_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_POWERBOARD_PORT);

// function declarations

void setPins();
void setPinStates();

/*structs

struct Bus      
{
    uint8_t i_max;          //max allowable current  
    uint8_t imeas_pin;      //current pin on teensy
    uint8_t imeas_val;      //current measure value

    void set_Values(const uint8_t & I_max, uint8_t &Iimeas_pin, uint8_t & Imeas_val);
}

struct toggle_Bus : Bus    //for all busses except NET and POE
{
    bool toggle_status;    //toggle bus on/off
    uint8_t ctl_pin;       //control pin on teensy

    void set_Values(const uint8_t & Ctl_pin, const uint8_t & I_max, uint8_t & Imeas_pin, uint8_t & Imeas_val, bool & Toggle_status);
}

void bus_Setup(Bus Bus[]);

*/

#endif
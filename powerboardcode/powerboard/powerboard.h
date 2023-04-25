#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"

// Numbers of Toggleable Ports
#define NUM_MOTORS              7   // Motors plus spare   
#define NUM_HIGH_CURRENT        2   // Aux, Spare (20A)
#define NUM_LOW_CURRENT         7   // Gimbal, Drive, Multi, Nav, Cam, BBB, Spare (5A)

// IntervalTimer Telemetry;

// Current Sensing

#define OVERCURRENT_HIGH        19500 //mA
#define OVERCURRENT_LOW         4875 //mA

#define HIGH_CURRENT_mA_MAX     22000 //mA
#define LOW_CURRENT_mA_MAX      5500 //mA

// High Current CTL

#define MOTOR_1_CTL             32
#define MOTOR_2_CTL             34
#define MOTOR_3_CTL             33
#define MOTOR_4_CTL             37
#define MOTOR_5_CTL             36
#define MOTOR_6_CTL             35
#define MOTOR_SPARE_CTL         29

#define AUX_CTL                 31
#define HIGH_CURRENT_SPARE_CTL  30

#define MOTOR_DELAY             500 // delay between initial motor port toggles, used to kill the board power when they all toggled on at once
// actual value of the delay is arbitrary

// Low Current CTL

#define GIMBAL_CTL              10
#define DRIVE_CTL               7
#define MULTIMEDIA_CTL          11
#define NAV_CTL                 28
#define CAM_CTL                 12
#define BBB_CTL                 9
#define LOW_CURRENT_SPARE_CTL   8

// High Current CS

#define MOTOR_1_CS              16
#define MOTOR_2_CS              17
#define MOTOR_3_CS              18
#define MOTOR_4_CS              23
#define MOTOR_5_CS              22
#define MOTOR_6_CS              21
#define MOTOR_SPARE_CS          20

#define AUX_CS                  15
#define HIGH_CURRENT_SPARE_CS   19

// Low Current CS

#define GIMBAL_CS               27
#define DRIVE_CS                24
#define MULTIMEDIA_CS           38
#define NAV_CS                  40
#define CAM_CS                  39
#define BBB_CS                  26
#define LOW_CURRENT_SPARE_CS    25

#define POE_CS                  14
#define NET_SWITCH_CS           41

// Current sensing variables

float motorSensePins[NUM_MOTORS] = {MOTOR_1_CS, MOTOR_2_CS, MOTOR_3_CS, MOTOR_4_CS, MOTOR_5_CS, MOTOR_6_CS, MOTOR_SPARE_CS};
float highCurrentSensePins[NUM_HIGH_CURRENT] = {AUX_CS, HIGH_CURRENT_SPARE_CS};
float lowCurrentSensePins[NUM_LOW_CURRENT] = {GIMBAL_CS, DRIVE_CS, MULTIMEDIA_CS, NAV_CS, CAM_CS, BBB_CS, LOW_CURRENT_SPARE_CS};
float networkCurrentSensePins[2] = {POE_CS, NET_SWITCH_CS};

float MOTOR_1_CURRENT = 0;
float MOTOR_2_CURRENT = 0;
float MOTOR_3_CURRENT = 0;
float MOTOR_4_CURRENT = 0;
float MOTOR_5_CURRENT = 0;
float MOTOR_6_CURRENT = 0;
float MOTOR_SPARE_CURRENT = 0;

float AUX_CURRENT = 0;
float HIGH_CURRENT_SPARE_CURRENT = 0;

float GIMBAL_CURRENT = 0;
float DRIVE_CURRENT = 0;
float MULTIMEDIA_CURRENT = 0;
float NAV_CURRENT = 0;
float CAM_CURRENT = 0;
float BBB_CURRENT = 0;
float LOW_CURRENT_SPARE_CURRENT = 0;

float POE_CURRENT = 0;
float NET_SWITCH_CURRENT = 0;

float motorSenseCurrents[NUM_MOTORS] = {MOTOR_1_CURRENT, MOTOR_2_CURRENT, MOTOR_3_CURRENT, MOTOR_4_CURRENT, MOTOR_5_CURRENT, MOTOR_6_CURRENT, MOTOR_SPARE_CURRENT};
float highCurrentSenseCurrents[NUM_HIGH_CURRENT] = {AUX_CURRENT, HIGH_CURRENT_SPARE_CURRENT};
float lowCurrentSenseCurrents[NUM_LOW_CURRENT] = {GIMBAL_CURRENT, DRIVE_CURRENT, MULTIMEDIA_CURRENT, NAV_CURRENT, CAM_CURRENT, BBB_CURRENT, LOW_CURRENT_SPARE_CURRENT};
float networkCurrentSenseCurrents[2] = {POE_CURRENT, NET_SWITCH_CURRENT};

uint8_t motorOverCurrent = 0;
uint8_t highOverCurrent = 0;
uint8_t lowOverCurrent = 0;
uint8_t networkOverCurrent = 0;

// Toggle Pin Arrays

uint8_t motorPins[NUM_MOTORS] = {MOTOR_1_CTL, MOTOR_2_CTL, MOTOR_3_CTL, MOTOR_4_CTL, MOTOR_5_CTL, MOTOR_6_CTL, MOTOR_SPARE_CTL};
uint8_t highCurrentPins[NUM_HIGH_CURRENT] = {AUX_CTL, HIGH_CURRENT_SPARE_CTL};
uint8_t lowCurrentPins[NUM_LOW_CURRENT] = {GIMBAL_CTL, DRIVE_CTL, MULTIMEDIA_CTL, NAV_CTL, CAM_CTL, BBB_CTL, LOW_CURRENT_SPARE_CTL};

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

// function declarations

void setPins();
void setPinStates();
void measureCurrent();
void overCurrent();
void telemetry();

//structs
/*
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
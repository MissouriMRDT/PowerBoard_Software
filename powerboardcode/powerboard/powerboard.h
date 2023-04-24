#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"

// Numbers of Toggleable Ports
#define NUM_MOTORS              7   // Motors plus spare   
#define NUM_HIGH_CURRENT        2   // Aux, Spare (20A)
#define NUM_LOW_CURRENT         5   // Gimbal, Drive, Multi, Nav, Spare (5A)
#define NUM_12V                 4   // Cam, Net Switch 1, Net Switch 2, Spare (12V)
#define NUM_BUS                 18  // Total number of ports

// IntervalTimer Telemetry;

// Current Sensing

#define OVERCURRENT_HIGH        19500 //mA
#define OVERCURRENT_LOW         4875 //mA

#define HIGH_CURRENT_mA_MAX     22000 //mA
#define LOW_CURRENT_mA_MAX      5500 //mA

// High Current CTL

#define MOTOR_1_CTL             35
#define MOTOR_2_CTL             36
#define MOTOR_3_CTL             37
#define MOTOR_4_CTL             7
#define MOTOR_5_CTL             6
#define MOTOR_6_CTL             5
#define MOTOR_SPARE_CTL         28

#define AUX_CTL                 34
#define HIGH_CURRENT_SPARE_CTL  29

#define MOTOR_DELAY             500 // delay between initial motor port toggles, used to kill the board power when they all toggled on at once
// actual value of the delay is arbitrary

// Low Current CTL

#define GIMBAL_CTL              9
#define DRIVE_CTL               8
#define MULTIMEDIA_CTL          10
#define NAV_CTL                 11
#define LOW_CURRENT_SPARE_CTL   12

// 12V CTL
#define CAM_CTL                 30
#define NET_SWITCH_1_CTL        33
#define NET_SWITCH_2_CTL        32
#define LOW_VOLTAGE_SPARE_CTL   31

// High Current CS

#define MOTOR_1_CS              A4
#define MOTOR_2_CS              A5
#define MOTOR_3_CS              A6
#define MOTOR_4_CS              A7
#define MOTOR_5_CS              A8
#define MOTOR_6_CS              A9
#define MOTOR_SPARE_CS          A17

#define AUX_CS                  A3
#define HIGH_CURRENT_SPARE_CS   A0

// Low Current CS

#define GIMBAL_CS               A11
#define DRIVE_CS                A10
#define MULTIMEDIA_CS           A12
#define NAV_CS                  A13
#define LOW_CURRENT_SPARE_CS    A16

// 12V CS
#define CAM_CS                  A1
#define LOW_VOLTAGE_SPARE_CS    A2
#define DEFAULT_CS              A15

float initialCurrent = 0;
bool initialToggle = false;
bool initialOvercurrent = false;

// Current arrays
float motorCurrents[NUM_MOTORS] = {};
float highCurrents[NUM_HIGH_CURRENT] = {};
float lowCurrents[NUM_LOW_CURRENT] = {};
float lowVoltCurrents[NUM_12V] = {};

uint8_t motorOverCurrent = 0;
uint8_t highOverCurrent = 0;
uint8_t lowOverCurrent = 0;
uint8_t lowVoltOverCurrent = 0;

RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

// function declarations

void busSetup();
void setPins();
void setPinStates();
void measureCurrent();
void dataPack();
void overCurrent();
void telemetry();

//struct

struct Bus
{
    uint8_t ctl_pin;        //control pin on teensy
    uint16_t i_overcurrent; //max allowable current
    uint16_t i_max;         //max readable current  
    uint8_t imeas_pin;      //current pin on teensy
    float imeas_val;        //current measure value
    bool toggle_status;     //toggle bus on/off
    bool overcurrent;       //overcurrent status

    void set_Values(const uint8_t & Ctl_pin, const uint16_t & I_overcurrent, const uint16_t & I_max, const uint8_t & Imeas_pin, float & Imeas_val, bool & Toggle_status, bool & Overcurrent);
};

struct Bus Port[NUM_BUS];

#endif
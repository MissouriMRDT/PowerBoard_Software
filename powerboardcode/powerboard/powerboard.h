#ifndef POWERBOARD_H
#define POWERBOARD_H
#include "RoveComm.h"
#include <SD.h>
#include <TimeLib.h>

IntervalTimer Telemetry;
RoveCommEthernet RoveComm;
rovecomm_packet packet; 
uint8_t* data;
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
File blackBox;

// Number of Toggleable Ports
#define NUM_MOTORS              7   // Motors plus spare   
#define NUM_HIGH_CURRENT        2   // Aux, Spare (20A)
#define NUM_LOW_CURRENT         5   // Gimbal, Drive, Multi, Nav, Spare (5A)
#define NUM_12V                 4   // Cam, Net Switch 1, Net Switch 2, Spare (12V)
#define NUM_BUS                 18  // Total number of ports
#define NUM_BOARDS              9  // Total number of boards on rover, not including powerboard

// Current Sensing Limits

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

#define MOTOR_1_CS              18
#define MOTOR_2_CS              19
#define MOTOR_3_CS              20
#define MOTOR_4_CS              21
#define MOTOR_5_CS              22
#define MOTOR_6_CS              23
#define MOTOR_SPARE_CS          41

#define AUX_CS                  17
#define HIGH_CURRENT_SPARE_CS   14

// Low Current CS

#define GIMBAL_CS               25
#define DRIVE_CS                24
#define MULTIMEDIA_CS           26
#define NAV_CS                  27
#define LOW_CURRENT_SPARE_CS    40

// 12V CS
#define CAM_CS                  15
#define LOW_VOLTAGE_SPARE_CS    16
#define DEFAULT_CS              39

// Current arrays
float motorCurrents[NUM_MOTORS] = {};
float highCurrents[NUM_HIGH_CURRENT] = {};
float lowCurrents[NUM_LOW_CURRENT] = {};
float lowVoltCurrents[NUM_12V] = {};

uint8_t motorOverCurrent = 0;
uint8_t highOverCurrent = 0;
uint8_t lowOverCurrent = 0;
uint8_t lowVoltOverCurrent = 0;

// function declarations

void busSetup();
void setPins();
void setPinStates();
void subscribeAll();
void measureCurrent();
void dataPack();
void printTime();
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

    void set_Values(const uint8_t & Ctl_pin, const uint16_t & I_overcurrent, const uint16_t & I_max, const uint8_t Imeas_pin, float Imeas_val, bool Toggle_status, bool Overcurrent);
};

struct Bus Port[NUM_BUS];

//board ips
uint8_t ips[NUM_BOARDS][4] = 
{
    {RC_DRIVEBOARD_FIRSTOCTET, RC_DRIVEBOARD_SECONDOCTET, RC_DRIVEBOARD_THIRDOCTET, RC_DRIVEBOARD_FOURTHOCTET},
    {RC_BMSBOARD_FIRSTOCTET, RC_BMSBOARD_SECONDOCTET, RC_BMSBOARD_THIRDOCTET, RC_BMSBOARD_FOURTHOCTET},
    {RC_NAVBOARD_FIRSTOCTET, RC_NAVBOARD_SECONDOCTET, RC_NAVBOARD_THIRDOCTET, RC_NAVBOARD_FOURTHOCTET},
    {RC_ARMBOARD_FIRSTOCTET, RC_ARMBOARD_SECONDOCTET, RC_ARMBOARD_THIRDOCTET, RC_ARMBOARD_FOURTHOCTET},
    {RC_SCIENCEACTUATIONBOARD_FIRSTOCTET, RC_SCIENCEACTUATIONBOARD_SECONDOCTET, RC_SCIENCEACTUATIONBOARD_THIRDOCTET, RC_SCIENCEACTUATIONBOARD_FOURTHOCTET},
    {RC_SCIENCESENSORSBOARD_FIRSTOCTET, RC_SCIENCESENSORSBOARD_SECONDOCTET, RC_SCIENCESENSORSBOARD_THIRDOCTET, RC_SCIENCESENSORSBOARD_FOURTHOCTET},
    {RC_AUTONOMYBOARD_FIRSTOCTET, RC_AUTONOMYBOARD_SECONDOCTET, RC_AUTONOMYBOARD_THIRDOCTET, RC_AUTONOMYBOARD_FOURTHOCTET},
    {RC_CAMERA1BOARD_FIRSTOCTET, RC_CAMERA1BOARD_SECONDOCTET, RC_CAMERA1BOARD_THIRDOCTET, RC_CAMERA1BOARD_FOURTHOCTET},
    {RC_HEATERBOARD_FIRSTOCTET, RC_HEATERBOARD_SECONDOCTET, RC_HEATERBOARD_THIRDOCTET, RC_HEATERBOARD_FOURTHOCTET},
};

#endif
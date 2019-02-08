//Program: PowerBoard_Software_Functions.h
//Programmer: Evan Hite
//Purpose: To hold all constants and function prototypes for 
//         PowerBoard_Software.ino and PowerBoard_Software_Functions.cpp

#ifndef POWERBOARD_SOFTWARE_FUNCTIONS_HEADER_H
#define POWERBOARD_SOFTWARE_FUNCTIONS_HEADER_H
#include "RoveComm.h"
#include <Energia.h>
using namespace std ;

RoveCommEthernetUdp RoveComm;

//Global Constants

//Delays
const int ROVER_POWER_RESET_DELAY = 3000; //Delay to reset rover power
const int ROVECOMM_DELAY = 10; //Delay to send Rovecomm a package
const int DEBOUNCE_DELAY = 10; //Delay after current or voltage trip

//////////////////////////////////////////////Pinmap
// Control Pins
const int ACT_CTL             = PN_3;
const int LOGIC_CTL           = PD_1;
const int COM_CTL             = PH_2;
const int COM_LOGIC_CTL       = PP_2;
const int AUX_CTL             = PK_5;
const int M1_CTL              = PK_7;
const int M2_CTL              = PK_6;
const int M3_CTL              = PH_1;
const int M4_CTL              = PH_0;
const int M5_CTL              = PM_2;
const int M6_CTL              = PM_1;
const int M7_CTL              = PM_0;
const int FAN_CTL             = PM_3;

// Sensor Volts/Amps Readings Pins
const int ACT_I_MEAS          = PE_2;
const int LOGIC_I_MEAS        = PE_1;
const int COM_I_MEAS          = PE_0;
const int AUX_I_MEAS          = PD_0;
const int M1_I_MEAS           = PK_3;
const int M2_I_MEAS           = PK_2;
const int M3_I_MEAS           = PK_1; 
const int M4_I_MEAS           = PK_0;
const int M5_I_MEAS           = PB_5;
const int M6_I_MEAS           = PB_4;
const int M7_I_MEAS           = PE_3;
const int PACK_VOLTAGE        = PE_5;

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
const float VCC                 = 3.3;       //volts
const float ADC_MAX             = 4096;      //bits
const float ADC_MIN             = 0;         //bits

//////////////////////////////////////////////Sensor
// ACS722LLCTR-40AU-T IC Sensor Specs 
const float SENSOR_SENSITIVITY   = 0.066;    //volts/amp
const float SENSOR_SCALE         = 0.1;      //volts/amp
const float SENSOR_BIAS          = VCC * SENSOR_SCALE;

const float CURRENT_MAX          = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;
const float CURRENT_MIN          = -SENSOR_BIAS / SENSOR_SENSITIVITY;

const float VOLTS_MIN            = 0;
const float VOLTS_MAX            = 33.6;

//Safest Test pin
const int ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD = 5; //5
const int ESTOP_12V_ACT_MAX_AMPS_THRESHOLD       = 15; //15  
const int ESTOP_AUX_MAX_AMPS_THRESHOLD           = 20; //20  (480W/30V)
const int ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD     = 22; //22

//Functions/////////////////////////////////////////////////////////////////////////

//Description: Checks the pin for bouncing voltages to avoid false positive
//Pre: Bouncing_pin is a pin on the tiva that reads amperage, max_amps_threshold
// is the maximum aperage the device can tolerate
//Post: Returns a bool of true if the pin has too much current
bool singleDebounce(int bouncing_pin, int max_amps_threshold);

//Description: Scales the input value x from analog input range (0 to 3.3) to actual values
//This is for current only, x is input, in_min and in_max is ADC value. out_max and out_min
//are the values that we are mapping to, has an offset of 1.
//Pre:
//Post:
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max) ;

//Description:Scales the input value x from analog input range (0 to 3.3) to actual values
//This is for voltage. It is the exact same as mapFloats, except there is no offset.
//Pre:
//Post:
float scale(float x, float in_min, float in_max, float out_min, float out_max) ;

//Description:
//Pre:
//Post:
void Configure_Pins ();

//Description:
//Pre:
//Post:
void Pin_Initialization ();

//Description:
//Pre:
//Post:
void Communication_Begin (uint8_t Bus []) ;

//Description:
//Pre:
//Post:
void Shut_Off( const int BUS_I_MEAS, uint8_t Bus[], const int BUS_CTL, const int ESTOP_AMP_THRESHOLD) ;

//Description:
//Pre:
//Post:
void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Bus[]) ;

//Description:
//Pre:
//Post:
void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS) ;

#endif

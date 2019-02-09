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
#define ROVER_POWER_RESET_DELAY  3000 //Delay to reset rover power
#define ROVECOMM_DELAY  10 //Delay to send Rovecomm a package
#define DEBOUNCE_DELAY  10 //Delay after current or voltage trip

//////////////////////////////////////////////Pinmap
// Control Pins
#define ACT_CTL_PIN              PN_3
#define LOGIC_CTL_PIN            PD_1
#define COM_CTL_PIN              PH_2
#define COM_LOGIC_CTL_PIN        PP_2
#define AUX_CTL_PIN              PK_5
#define M1_CTL_PIN               PK_7
#define M2_CTL_PIN               PK_6
#define M3_CTL_PIN               PH_1
#define M4_CTL_PIN               PH_0
#define M5_CTL_PIN               PM_2
#define M6_CTL_PIN               PM_1
#define M7_CTL_PIN               PM_0
#define FAN_CTL_PIN              PM_3

// Sensor Volts/Amps Readings Pins
#define ACT_I_MEAS_PIN           PE_2
#define LOGIC_I_MEAS_PIN         PE_1
#define COM_I_MEAS_PIN           PE_0
#define AUX_I_MEAS_PIN           PD_0
#define M1_I_MEAS_PIN            PK_3
#define M2_I_MEAS_PIN            PK_2
#define M3_I_MEAS_PIN            PK_1 
#define M4_I_MEAS_PIN            PK_0
#define M5_I_MEAS_PIN            PB_5
#define M6_I_MEAS_PIN            PB_4
#define M7_I_MEAS_PIN            PE_3
#define PACK_VOLTAGE_PIN         PE_5

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
#define VCC                  3.3       //volts
#define ADC_MAX              4096      //bits
#define ADC_MIN              0         //bits

//////////////////////////////////////////////Sensor
// ACS722LLCTR-40AU-T IC Sensor Specs 
#define SENSOR_SENSITIVITY    0.066    //volts/amp
#define SENSOR_SCALE          0.1      //volts/amp
#define SENSOR_BIAS           VCC * SENSOR_SCALE

#define CURRENT_MAX           ((VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY)
#define CURRENT_MIN           -SENSOR_BIAS / SENSOR_SENSITIVITY

#define VOLTS_MIN             0
#define VOLTS_MAX             33.6

//Safest Test pin
#define ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD  5 //5
#define ESTOP_12V_ACT_MAX_AMPS_THRESHOLD        15 //15  
#define ESTOP_AUX_MAX_AMPS_THRESHOLD            20 //20  (480W/30V)
#define ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD      22 //22

//Functions/////////////////////////////////////////////////////////////////////////

//Description: Checks the pin for bouncing voltages to avoid false positive
//Pre: Bouncing_pin is a pin on the tiva that reads amperage, max_amps_threshold
// is the maximum aperage the device can tolerate
//Post: Returns a bool of true if the pin has too much current
bool singleDebounce(const int & bouncing_pin,const int & max_amps_threshold);

//Description: Scales the input value x from analog input range (0 to 3.3) to actual values
//This is for current only, x is input, in_min and in_max is ADC value. out_max and out_min
//are the values that we are mapping to, has an offset of 1.
//Pre:
//Post:
//float mapFloats(float x, float in_min, float in_max, float out_min, float out_max) ;

//Description:Scales the input value x from analog input range (0 to 3.3) to actual values
//This is for voltage. It is the exact same as mapFloats, except there is no offset.
//Pre:
//Post:
//float scale(float x, float in_min, float in_max, float out_min, float out_max) ;

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
void Shut_Off( const int & BUS_I_MEAS_PIN, uint8_t Bus[], const int & BUS_CTL_PIN, const int & ESTOP_AMP_THRESHOLD) ;

//Description:
//Pre:
//Post:
void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Bus[]) ;

//Description:
//Pre:
//Post:
void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS_PIN) ;

#endif

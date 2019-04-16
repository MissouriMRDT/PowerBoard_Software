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

//Struct For Motor Busses, Below are Global Constants
struct PB_Bus
{
  uint8_t rovecomm_cell ; //Bit position in RoveComm packet, either 0 for ACL, or 1 for Motor Busses
  uint8_t bit_code_off ; //Binary code with 0 being the position in which the bus exists in the rovecomm packet cell.
  uint8_t bit_code_on ; //Binary code with 1 being the position in which the bus exists in the rovecomm packet cell.
  int imeas_pin ; //current measurment pin on the board
  int ctl_pin ; //control pin on the tiva
  int amp_threshold ; //Amp Threshold on the bus
  int bus_tuning ; //Tuning for each bus current reading
  String identity ; //Identity of the bus  
//Basically a constructor, but I do not need it for that purpose
//I am simply wanting to be able to adjust all the values at once
void Adjust_Values (const uint8_t & Rovecomm_cell, const uint8_t & Bit_code_off, const uint8_t & Bit_code_on, const int & Imeas_pin, const int & Ctl_pin, const int & Amp_threshold, const int & Bus_tuning, const String Identity) ;
} ;

//Global Constants

//Delays
#define ROVER_POWER_RESET_DELAY  3000 //Delay to reset rover power
#define ROVECOMM_DELAY  10 //Delay to send Rovecomm a package
#define DEBOUNCE_DELAY  10 //Delay after current or voltage trip
#define ROVECOMM_UPDATE_DELAY 1000 //Delay so that Rovecomm is not overloaded

//////////////////////////////////////////////Pinmap
// Control Pins for Busses
#define ACT_CTL_PIN              PN_3 //In rev 2
#define LOGIC_CTL_PIN            PD_1 //In rev 2
#define COMM_CTL_PIN             PH_2 //In rev 2
#define COMM_LOGIC_CTL_PIN        PP_2 //In rev 2
#define EM_CTL_PIN               PK_5 //Was Auxillary
#define FM_CTL_PIN               PK_7 //Was M1
#define MM_CTL_PIN               PK_6 //Was M2
#define BM_CTL_PIN               PH_1 //Was M3
#define AUX_CTL_PIN              PH_0 //Was M4
#define ROCKET_CTL_PIN           PM_2 //Was M7
#define GE_CTL_PIN               PM_1 //Was M6
//#define M7_CTL_PIN               PM_0 //No longer in use
#define FAN_CTL_PIN              PM_3 //In rev 2

// Sensor Volts/Amps Readings Pins
#define ACT_I_MEAS_PIN           PE_2 //In rev 2
#define LOGIC_I_MEAS_PIN         PE_1 //In rev 2
#define COMM_I_MEAS_PIN          PE_0 //In rev 2
#define EM_I_MEAS_PIN            PD_0 //Was Auxillary in rev 1
#define FM_I_MEAS_PIN            PK_3 //was M1
#define MM_I_MEAS_PIN            PK_2 //was M2
#define BM_I_MEAS_PIN            PK_1 //was M3
#define AUX_I_MEAS_PIN           PK_0 //was M4
#define ROCKET_I_MEAS_PIN        PB_5 //Was M5
#define GE_I_MEAS_PIN            PB_4 //Was M6
//#define M7_I_MEAS_PIN            PE_3 //No longer in use
//#define PACK_VOLTAGE_PIN         PE_5 //No longer in use

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
#define VCC                  3300      //volts
#define ADC_MAX              4096      //bits
#define ADC_MIN              420       //meme bits

//////////////////////////////////////////////Sensor
// ACS722LLCTR-40AU-T IC Sensor Specs 
#define SENSOR_SENSITIVITY    0.066    //volts/amp
#define SENSOR_SCALE          0.1      //volts/amp
#define SENSOR_BIAS           VCC * SENSOR_SCALE

#define CURRENT_MAX           40000//((VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY)
#define CURRENT_MIN           0//-SENSOR_BIAS / SENSOR_SENSITIVITY

#define VOLTS_MIN             0
#define VOLTS_MAX             33600

//Safest Test pin
#define ESTOP_12V_COMM_LOGIC_MAX_AMPS_THRESHOLD  5000 //5 amps, 5 amp fuse
#define ESTOP_12V_ACT_MAX_AMPS_THRESHOLD        2000 //2 amps, 20 amp fuse  
#define ESTOP_AUX_MAX_AMPS_THRESHOLD            19000 //17 amps, 20 amp fuse(360W/21.6) = 16.6 Amps, due to inaccurate readings raised to 19
#define ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD      30000 //30 amps, 40 amp fuse
#define ESTOP_ROCKET_BUS_MAX_AMPS_THRESHOLD     4000 //4 amps, 5 amp fuse 

//Tuning Variables
#define LOGIC_COMM_TUNER  1085 //Tuner for ADC values for Logic and Comm busses
#define ACT_TUNER         1132 //Tuner for ADC values for Actuation bus
#define AUX_TUNER         1050 //Tuner for ADC values for Auxilliary bus
#define MOTOR_TUNER       1005 //Tuner for ADC values for motor busses
#define ROCKET_TUNER      1000 //Tuner for ADC values for Rocket antennas
#define CURRENT_AVERAGE      5 //The amount of current readings that we average over to get accurate current readings

//Number of Busses
#define ALC_BUSSES        3 //Number of Actuation, logic and Communication busses
#define MOTOR_BUSSES      7 //Number of busses besides actuation, logic, and communication

//Functions/////////////////////////////////////////////////////////////////////////

//Description: Checks the pin for bouncing voltages to avoid false positive
//Pre: Bouncing_pin is a pin on the tiva that reads amperage, max_amps_threshold
// is the maximum aperage the device can tolerate
//Post: Returns a bool of true if the pin has too much current
bool singleDebounce(const int & bouncing_pin,const int & max_amps_threshold, const int & Tuner);

//Description: Sets the pins on the tiva to certain buses on the powerboard
//Pre: None
//Post: All pins on the tiva are set to correct values.
void Configure_Pins ();

//Description: Turns all pins to on after setting all to low
//Pre: Pins must be in correct position from Configure_Pins
//Post: All pins are set high
void Pin_Initialization ();

//Description: Begins communication with RoveComm and sends a packet to rovecomm
//Pre: Bus[] must have a size of RC_POWERBOARD_BUSENABLED_DATACOUNT to work.
//Post: Sends to RoveComm a packet that the powerboard is live.
void Communication_Begin (uint8_t Bus []) ;

//Description: Checks overcurrent on the Bus and shuts off in need be
//Pre: BUS_I_MEAS_PIN must be a pin for current measurement, Bus[] must be the same array from Commuincation_Begin.
// BUS_CTL_PIN must be the relevent pin that controls the bus. and ESTOP_AMP_THRESHOLD must also be the relevent estop threshold.
//Post: Shuts off the bus if there is an overcurrent situation and writes this to Bus[]
bool Shut_Off(const PB_Bus & Bus, uint8_t Send_Recieve[], const uint16_t & Current_Reading) ;

//Description: Takes a packet from RoveComm and turn on or off a bus
//Pre: Enable_Disable should be a packet recieved from RoveComm, and Bus should be from Shut_Off
//Post: Turns on or off all buses that the packet says should be on or off and writes this to Bus[].
void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Send_Recieve[], const PB_Bus Bus[]) ;

//Description: Reads a current sensing pin and saves this value to current_reading
//Pre: Current_Reading should be an array part, for RoveComm reasons. BUS_I_MEAS_PIN should be a current measuring pin
//that corresponds to the spot on the Current_Reading array.
//Post: Returns a value to current_reading of the current reading value from the BUS_I_MEAS_PIN in mA.
void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS_PIN, const int & Tuner) ;

//Description:
//Pre:
//Post:
void Bus_Setup(PB_Bus Bus[]) ;

#endif

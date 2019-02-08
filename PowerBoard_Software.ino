//RoveWare Powerboard ACS_722 Interface
//
// Created for Zenith by: Judah Schad, jrs6w7
// Altered for Gryphon by: Jacob Lipina, jrlwd5
// Altered for Valkyrie by: Evan Hite erhtpc
// Using http://www.digikey.com/product-detail/en/allegro-microsystems-llc/ACS722LLCTR-40AU-T/620-1640-1-ND/4948876
//
// Standard C
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//Local Files
#include "PowerBoard_Software_Functions_Header.h"

//Packet Variables
uint8_t Bus[] {0,0} ; //Bus to Enable or Disable
rovecomm_packet Enable_Disable ; //packet reception variable
uint16_t Current_Reading[RC_POWERBOARD_IMEASmA_DATACOUNT] ; //Current Reading for all busses

//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{
  Configure_Pins () ;
  Pin_Initialization () ;
  Communication_Begin (Bus) ;
}//end setup

/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{ 
  //Current Readings to Report back to Base Station
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_COMMENTRY], COM_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_LOGENTRY], LOGIC_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_ACTENTRY], ACT_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_AUXENTRY], AUX_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M1ENTRY], M1_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M2ENTRY], M2_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M3ENTRY], M3_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M4ENTRY], M4_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M5ENTRY], M5_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M6ENTRY], M6_I_MEAS) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M7ENTRY], M7_I_MEAS) ;
  RoveComm.write(RC_POWERBOARD_IMEASmA_DATAID, RC_POWERBOARD_IMEASmA_DATACOUNT, Current_Reading) ;
  
  //Checking for Over Currents on all busses
  Shut_Off(COM_I_MEAS, Bus, COM_CTL, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) ;
  Shut_Off(ACT_I_MEAS, Bus, ACT_CTL, ESTOP_12V_ACT_MAX_AMPS_THRESHOLD) ;
  Shut_Off(LOGIC_I_MEAS, Bus, LOGIC_CTL, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) ;
  Shut_Off(AUX_I_MEAS, Bus, AUX_CTL, ESTOP_AUX_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M1_I_MEAS, Bus, M1_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M2_I_MEAS, Bus, M2_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M3_I_MEAS, Bus, M3_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M4_I_MEAS, Bus, M4_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M5_I_MEAS, Bus, M5_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M6_I_MEAS, Bus, M6_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  Shut_Off(M7_I_MEAS, Bus, M7_CTL, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Send out a summary of what is off after current check
  delay(ROVECOMM_DELAY) ;
  
  /////////////////////////////////////////////RED Control and Telem RoveComm
  //Recieves Pack form rovecomm and shuts off or turns on busses at need
  Enable_Disable = RoveComm.read();
  Bus_Enable(Enable_Disable, Bus) ;
  RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Send whats on and off after the message, to match up
}  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool singleDebounce(int bouncing_pin, int max_amps_threshold)
{
  int adc_threshhold = map(max_amps_threshold, CURRENT_MIN, CURRENT_MAX, ADC_MIN, ADC_MAX);
  
  if( analogRead(bouncing_pin) > adc_threshhold)
  {  
    delay(DEBOUNCE_DELAY);
    
    if( analogRead(bouncing_pin) > adc_threshhold)
    {
       return true;
    }//end if
  }// end if 
  return false;
}

//float mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
//{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min + 1; //+1 added for offset
//}

//float scale(float x, float in_min, float in_max, float out_min, float out_max)
//{
// return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

void Configure_Pins ()
{
  // Control Pins are outputs
  pinMode(ACT_CTL, OUTPUT);
  pinMode(COM_LOGIC_CTL, OUTPUT);
  pinMode(COM_CTL, OUTPUT);
  pinMode(LOGIC_CTL, OUTPUT);
  pinMode(AUX_CTL, OUTPUT);
  pinMode(M1_CTL, OUTPUT);
  pinMode(M2_CTL, OUTPUT);
  pinMode(M3_CTL, OUTPUT); 
  pinMode(M4_CTL, OUTPUT);
  pinMode(M5_CTL, OUTPUT);
  pinMode(M6_CTL, OUTPUT);
  pinMode(M7_CTL, OUTPUT);
  pinMode(FAN_CTL, OUTPUT);

  //Current Measurement pins are inputs
  pinMode(ACT_I_MEAS, INPUT);
  pinMode(COM_I_MEAS, INPUT);
  pinMode(LOGIC_I_MEAS, INPUT);
  pinMode(AUX_I_MEAS, INPUT);
  pinMode(M1_I_MEAS, INPUT);
  pinMode(M2_I_MEAS, INPUT);
  pinMode(M3_I_MEAS, INPUT);
  pinMode(M4_I_MEAS, INPUT);
  pinMode(M5_I_MEAS, INPUT);
  pinMode(M6_I_MEAS, INPUT);
  pinMode(M7_I_MEAS, INPUT);
  pinMode(PACK_VOLTAGE, INPUT);
  return ;
}

void Pin_Initialization ()
{
  digitalWrite(ACT_CTL, LOW);
  digitalWrite(COM_LOGIC_CTL, LOW);
  digitalWrite(COM_CTL, LOW);
  digitalWrite(LOGIC_CTL, LOW);
  digitalWrite(AUX_CTL, LOW);
  digitalWrite(M1_CTL, LOW);
  digitalWrite(M2_CTL, LOW);
  digitalWrite(M3_CTL, LOW);
  digitalWrite(M4_CTL, LOW);
  digitalWrite(M5_CTL, LOW);
  digitalWrite(M6_CTL, LOW);
  digitalWrite(M7_CTL, LOW);
  digitalWrite(FAN_CTL, LOW);
   // Turn on everything when we begin
  delay(ROVER_POWER_RESET_DELAY);

  //After Delay, make sure everything turns on
  digitalWrite(ACT_CTL, HIGH);
  digitalWrite(COM_LOGIC_CTL, HIGH);
  digitalWrite(COM_CTL, HIGH);
  digitalWrite(LOGIC_CTL, HIGH);
  digitalWrite(AUX_CTL, HIGH);
  digitalWrite(M1_CTL, HIGH);
  digitalWrite(M2_CTL, HIGH);
  digitalWrite(M3_CTL, HIGH);
  digitalWrite(M4_CTL, HIGH);
  digitalWrite(M5_CTL, HIGH);
  digitalWrite(M6_CTL, HIGH);
  digitalWrite(M7_CTL, HIGH);
  digitalWrite(FAN_CTL, HIGH);
  
  return ;
}

void Communication_Begin (uint8_t Bus []) 
{
  bool order = false ; //Indicates first set of Bus's bits are filled
  RoveComm.begin(RC_POWERBOARD_FOURTHOCTET);
  Serial.begin(9600); //15000 is the baud rate that the integrated tiva chip needs to be set at so the serial monitor reads accurately at 9600.
  Serial.println("Setting Up...");
  delay(ROVECOMM_DELAY); 
  for(int i = 0 ; i < RC_POWERBOARD_BUSENABLE_DATACOUNT; i++)
  {
    if(order == false)
    {
      for(int j = 0 ; j < 3 ; j++) //3 represents the number of logic/comm/actuation busses
      {
        bitSet(Bus[i], j) ;
      }
    }
    if( order == true )
    {
      for(int j = 0; j < 8 ; j++)
      {
        bitSet(Bus[i] , j) ;
      }
    }
    order = true ;
  }
  RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Initial states of Busses
}

void Shut_Off( const int BUS_I_MEAS, uint8_t Bus[], const int BUS_CTL, const int ESTOP_AMP_THRESHOLD)
{
  uint8_t time1 = 0 ;
  if(singleDebounce(BUS_I_MEAS, ESTOP_AMP_THRESHOLD) ) //If pin is tripped
  {
     if(BUS_CTL == COM_CTL)//Special rules for communication bus
     {
       bitWrite(Bus[0],RC_POWERBOARD_BUSENABLED_COMMBIT, 0) ;
       RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ;
       digitalWrite(COM_CTL, LOW);
       time1 = millis();
       delay(ROVECOMM_DELAY);
       if(millis()>=(time1+10000))            //it is turned off in case the overcurrent was just a random spike. 
       {                                      //If there actually is a short in the bus, the bus will turn itself 
                                              //back on
         digitalWrite(COM_CTL,HIGH);
         bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_COMMBIT) ;
       } 
     }
     else //All other busses
     {
       switch(BUS_CTL)
       {
         case ACT_CTL:
           bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT, 0) ;
           digitalWrite(BUS_CTL, LOW);
           delay(ROVECOMM_DELAY);
           break ;
         case LOGIC_CTL:
           bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M1_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M1BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M2_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M2BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M3_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M3BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M4_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M4BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M5_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M5BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M6_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M6BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case M7_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M7BIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
         case AUX_CTL:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_AUXBIT, 0) ;
           digitalWrite(BUS_CTL, LOW) ;
           delay(ROVECOMM_DELAY) ;
           break ;
       }
     }
  }
  return ;
}

void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Bus[])
{
  //Communication Bus
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_COMMBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_COMMBIT, 0) ;
    RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Sends a shut off packet before shutting off
    digitalWrite(COM_CTL, LOW) ;
    delay(ROVECOMM_DELAY) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_COMMBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_COMMBIT) ;
    digitalWrite(COM_CTL, HIGH) ;
    delay(ROVECOMM_DELAY) ;
    RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Gives the system a chance to boot up   
  }
  //Logic Bus
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_LOGBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT, 0) ;
    digitalWrite(LOGIC_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_LOGBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT) ;
    digitalWrite(LOGIC_CTL, HIGH) ;
  }
  //Actuation Bus
    if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_ACTBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT, 0) ;
    digitalWrite(ACT_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_ACTBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT) ;
    digitalWrite(ACT_CTL, HIGH) ;
  }
  //Motor Bus 1
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M1BIT) == 0)
  {
    bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M1BIT, 0) ;
    digitalWrite(M1_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M1BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M1BIT) ;
    digitalWrite(M1_CTL, HIGH) ;
  }
  //Motor Bus 2
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M2BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M2BIT, 0) ;
    digitalWrite(M2_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M2BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M2BIT) ;
    digitalWrite(M2_CTL, HIGH) ;
  }
  //Motor Bus 3
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M3BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M3BIT, 0) ;
    digitalWrite(M3_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M3BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M3BIT) ;
    digitalWrite(M3_CTL, HIGH) ;
  }
  //Motor Bus 4
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M4BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M4BIT, 0) ;
    digitalWrite(M4_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M4BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M4BIT) ;
    digitalWrite(M4_CTL, HIGH) ;
  }
  //Motor Bus 5
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M5BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M5BIT, 0) ;
    digitalWrite(M3_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M5BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M5BIT) ;
    digitalWrite(M5_CTL, HIGH) ;
  }  
  //Motor Bus 6
    if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M6BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M6BIT, 0) ;
    digitalWrite(M6_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M6BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M6BIT) ;
    digitalWrite(M6_CTL, HIGH) ;
  }
  //Motor Bus 7
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M7BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M7BIT, 0) ;
    digitalWrite(M7_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M7BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M7BIT) ;
    digitalWrite(M7_CTL, HIGH) ;
  }
  //Auxilliary Bus
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_AUXBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_AUXBIT, 0) ;
    digitalWrite(AUX_CTL, LOW) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_AUXBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_AUXBIT) ;
    digitalWrite(AUX_CTL, HIGH) ;
  }
  return ;
}

void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS)
{
  float adc_reading = analogRead(BUS_I_MEAS) ;
  current_reading = static_cast<uint16_t>(map(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX)*1000) ;
  return ;
}



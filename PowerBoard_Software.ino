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
//rovecomm_packet Same ; //Packet to be used in case of no packet reception
uint16_t Current_Reading[RC_POWERBOARD_IMEASmA_DATACOUNT] ; //Current Reading for all busses

//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{
  Serial.begin(9600) ;
  delay(500) ;
//Serial.println("Got here...") ;
  Configure_Pins () ; //Configures pins to correct busses
  Pin_Initialization () ; //Sets pins to low then to high
  Communication_Begin (Bus) ; //Sends to base station that everything is now on and communication begins
}//end setup

/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{
//Serial.println("In the loop") ; 
  //Current Readings to Report back to Base Station
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_COMMENTRY], COM_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_LOGENTRY], LOGIC_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_ACTENTRY], ACT_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_AUXENTRY], AUX_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M1ENTRY], M1_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M2ENTRY], M2_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M3ENTRY], M3_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M4ENTRY], M4_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M5ENTRY], M5_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M6ENTRY], M6_I_MEAS_PIN) ;
  Pin_Read(Current_Reading[RC_POWERBOARD_IMEASmA_M7ENTRY], M7_I_MEAS_PIN) ;
  RoveComm.write(RC_POWERBOARD_IMEASmA_DATAID, RC_POWERBOARD_IMEASmA_DATACOUNT, Current_Reading) ;
  
  //Checking for Over Currents on all busses
Serial.println("Checking comms") ;
delay(10) ;
  Shut_Off(COM_I_MEAS_PIN, Bus, COM_CTL_PIN, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking ACT") ;
delay(10) ;
  Shut_Off(ACT_I_MEAS_PIN, Bus, ACT_CTL_PIN, ESTOP_12V_ACT_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking log") ;
delay(10) ;
  Shut_Off(LOGIC_I_MEAS_PIN, Bus, LOGIC_CTL_PIN, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking AUX") ;
delay(10) ;
  Shut_Off(AUX_I_MEAS_PIN, Bus, AUX_CTL_PIN, ESTOP_AUX_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking M1") ;
delay(10) ;
  Shut_Off(M1_I_MEAS_PIN, Bus, M1_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m2") ;
delay(10) ;
  Shut_Off(M2_I_MEAS_PIN, Bus, M2_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m3") ;
delay(10) ;
  Shut_Off(M3_I_MEAS_PIN, Bus, M3_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m4") ;
delay(10) ;
  Shut_Off(M4_I_MEAS_PIN, Bus, M4_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m5") ;
delay(10) ;
  Shut_Off(M5_I_MEAS_PIN, Bus, M5_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m6") ;
delay(10) ;
  Shut_Off(M6_I_MEAS_PIN, Bus, M6_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
Serial.println("Checking m7") ;
delay(10) ;
  Shut_Off(M7_I_MEAS_PIN, Bus, M7_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ;
  RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Send out a summary of what is off after current check
  delay(ROVECOMM_DELAY) ;
  
  /////////////////////////////////////////////RED Control and Telem RoveComm
  //Recieves Pack form rovecomm and shuts off or turns on busses at need
    Enable_Disable = RoveComm.read();
    if(Enable_Disable.data_id != 0)
    {  
Serial.println("Packet Recieved") ;
delay(10) ;
      Bus_Enable(Enable_Disable, Bus) ;
      RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Send whats on and off after the message, to match up
      //Same = Enable_Disable ;
    }
  //Bus_Enable(Same, Bus) ;
  //RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Send whats on and off after the message, to match up
}  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool singleDebounce(const int & bouncing_pin, const int & max_amps_threshold)
{
  int adc_threshhold = map(max_amps_threshold, CURRENT_MIN, CURRENT_MAX, ADC_MIN, ADC_MAX); //Get reading off pin
  bool trip = false ;
  if( analogRead(bouncing_pin) > adc_threshhold) //If pin reading is high
  {
//Serial.println(analogRead(bouncing_pin)) ; //Debug Code
//delay(100) ;
//Serial.println(adc_threshhold) ;
//delay(100) ;  
//Serial.println("Pin was tripped") ;
//delay(1000) ;
    delay(DEBOUNCE_DELAY);
    
    if( analogRead(bouncing_pin) > adc_threshhold) //If pin reading is still high
    {
       trip = true; //Sends back true to indicate shut off
    }//end if
    else
    {
//Serial.println("False Alarm") ; //Debug Output Code
//delay(1000) ;
      trip = false ;
    }
  }// end if 
  return trip;
}

//Functions from Years past. No longer in use.
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
  pinMode(ACT_CTL_PIN, OUTPUT);
  pinMode(COM_LOGIC_CTL_PIN, OUTPUT);
  pinMode(COM_CTL_PIN, OUTPUT);
  pinMode(LOGIC_CTL_PIN, OUTPUT);
  pinMode(AUX_CTL_PIN, OUTPUT);
  pinMode(M1_CTL_PIN, OUTPUT);
  pinMode(M2_CTL_PIN, OUTPUT);
  pinMode(M3_CTL_PIN, OUTPUT); 
  pinMode(M4_CTL_PIN, OUTPUT);
  pinMode(M5_CTL_PIN, OUTPUT);
  pinMode(M6_CTL_PIN, OUTPUT);
  pinMode(M7_CTL_PIN, OUTPUT);
  pinMode(FAN_CTL_PIN, OUTPUT);

  //Current Measurement pins are inputs
  pinMode(ACT_I_MEAS_PIN, INPUT);
  pinMode(COM_I_MEAS_PIN, INPUT);
  pinMode(LOGIC_I_MEAS_PIN, INPUT);
  pinMode(AUX_I_MEAS_PIN, INPUT);
  pinMode(M1_I_MEAS_PIN, INPUT);
  pinMode(M2_I_MEAS_PIN, INPUT);
  pinMode(M3_I_MEAS_PIN, INPUT);
  pinMode(M4_I_MEAS_PIN, INPUT);
  pinMode(M5_I_MEAS_PIN, INPUT);
  pinMode(M6_I_MEAS_PIN, INPUT);
  pinMode(M7_I_MEAS_PIN, INPUT);
  pinMode(PACK_VOLTAGE_PIN, INPUT);
  return ;
}

void Pin_Initialization ()
{
  digitalWrite(ACT_CTL_PIN, LOW);
  digitalWrite(COM_LOGIC_CTL_PIN, LOW);
  digitalWrite(COM_CTL_PIN, LOW);
  digitalWrite(LOGIC_CTL_PIN, LOW);
  digitalWrite(AUX_CTL_PIN, LOW);
  digitalWrite(M1_CTL_PIN, LOW);
  digitalWrite(M2_CTL_PIN, LOW);
  digitalWrite(M3_CTL_PIN, LOW);
  digitalWrite(M4_CTL_PIN, LOW);
  digitalWrite(M5_CTL_PIN, LOW);
  digitalWrite(M6_CTL_PIN, LOW);
  digitalWrite(M7_CTL_PIN, LOW);
  digitalWrite(FAN_CTL_PIN, LOW);
   // Turn on everything when we begin
  delay(ROVER_POWER_RESET_DELAY); //Three Second Delay

  //After Delay, make sure everything turns on
  digitalWrite(ACT_CTL_PIN, HIGH);
  digitalWrite(COM_LOGIC_CTL_PIN, HIGH);
  digitalWrite(COM_CTL_PIN, HIGH);
  digitalWrite(LOGIC_CTL_PIN, HIGH);
  digitalWrite(AUX_CTL_PIN, HIGH);
  digitalWrite(M1_CTL_PIN, HIGH);
  digitalWrite(M2_CTL_PIN, HIGH);
  digitalWrite(M3_CTL_PIN, HIGH);
  digitalWrite(M4_CTL_PIN, HIGH);
  digitalWrite(M5_CTL_PIN, HIGH);
  digitalWrite(M6_CTL_PIN, HIGH);
  digitalWrite(M7_CTL_PIN, HIGH);
  digitalWrite(FAN_CTL_PIN, HIGH); //Fans Always On
  
  return ;
}

void Communication_Begin (uint8_t Bus []) 
{
  bool order = false ; //Indicates first set of Bus's bits are filled
  RoveComm.begin(RC_POWERBOARD_FOURTHOCTET);
  //Serial.begin(9600); //15000 is the baud rate that the integrated tiva chip needs to be set at so the serial monitor reads accurately at 9600.
  delay(100) ;
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

void Shut_Off( const int & BUS_I_MEAS_PIN, uint8_t Bus[], const int & BUS_CTL_PIN, const int & ESTOP_AMP_THRESHOLD)
{
  uint8_t time1 = 0 ;
  if(singleDebounce(BUS_I_MEAS_PIN, ESTOP_AMP_THRESHOLD) ) //If pin is tripped
  {
     if(BUS_CTL_PIN == COM_CTL_PIN)//Special rules for communication bus
     {
//Serial.println("Shutting off Comms") ; //Debug Code
//delay(500) ;
       bitWrite(Bus[0],RC_POWERBOARD_BUSENABLED_COMMBIT, 0) ;
       RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ;
       digitalWrite(COM_CTL_PIN, LOW);
       time1 = millis();
       delay(ROVECOMM_DELAY);
       if(millis()>=(time1+10000))            //it is turned off in case the overcurrent was just a random spike. 
       {                                      //If there actually is a short in the bus, the bus will turn itself 
                                              //back on
//Serial.println("shut on Comms") ; //Debug Code
//delay(500) ;         
         digitalWrite(COM_CTL_PIN,HIGH);
         bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_COMMBIT) ;
       } 
     }
     else //All other busses
     {
       switch(BUS_CTL_PIN)
       {
         case ACT_CTL_PIN:
           bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW);
//Serial.println("shut off ACT_CTL_PIN") ; //Debug Code
           delay(ROVECOMM_DELAY);
           break ;
         case LOGIC_CTL_PIN:
           bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off Logic_CTL_PIN") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M1_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M1BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off M1") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M2_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M2BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m2") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M3_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M3BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m3") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M4_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M4BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m4") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M5_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M5BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m5") ; //Debug Code           
           delay(ROVECOMM_DELAY) ;
           break ;
         case M6_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M6BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m6") ; //Debug Code
           delay(ROVECOMM_DELAY) ;
           break ;
         case M7_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M7BIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off m7") ; //Debug Code           
           delay(ROVECOMM_DELAY) ;
           break ;
         case AUX_CTL_PIN:
           bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_AUXBIT, 0) ;
           digitalWrite(BUS_CTL_PIN, LOW) ;
//Serial.println("shut off Aux") ; //Debug Code
           delay( ROVECOMM_DELAY ) ;
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
    digitalWrite(COM_CTL_PIN, LOW) ;
//Serial.println("Turned off Comm") ; //Debug Code
//delay(10) ;
    delay(ROVECOMM_DELAY) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_COMMBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_COMMBIT) ;
    digitalWrite(COM_CTL_PIN, HIGH) ;
//Serial.println("Turned on Comm") ; //Debug Code
//delay(10) ;
    delay(ROVECOMM_DELAY) ;
    RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Bus) ; //Gives the system a chance to boot up   
  }
  //Logic Bus
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_LOGBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT, 0) ;
    digitalWrite(LOGIC_CTL_PIN, LOW) ;
//Serial.println("Turned off Logic") ; //Debug Code
//delay(10) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_LOGBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_LOGBIT) ;
    digitalWrite(LOGIC_CTL_PIN, HIGH) ;
//Serial.println("Turned on Logic") ; //Debug Code
//delay(10) ;
  }
  //Actuation Bus
    if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_ACTBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT, 0) ;
    digitalWrite(ACT_CTL_PIN, LOW) ;
//Serial.println("Turned off Act") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_ALCENTRY], RC_POWERBOARD_BUSENABLE_ACTBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_ACTBIT) ;
    digitalWrite(ACT_CTL_PIN, HIGH) ;
//Serial.println("Turned on ACT") ; //Debug Code
//delay(10) ;
  }
  //Motor Bus 1
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M1BIT) == 0)
  {
    bitWrite(Bus[1], RC_POWERBOARD_BUSENABLED_M1BIT, 0) ;
    digitalWrite(M1_CTL_PIN, LOW) ;
//Serial.println("Turned off M1") ; //Debug Code
//delay(10) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M1BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M1BIT) ;
    digitalWrite(M1_CTL_PIN, HIGH) ;
//Serial.println("Turned on M1") ; //Debug Code
//delay(10) ;
  }
  //Motor Bus 2
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M2BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M2BIT, 0) ;
    digitalWrite(M2_CTL_PIN, LOW) ;
//Serial.println("Turned off M2") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M2BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M2BIT) ;
    digitalWrite(M2_CTL_PIN, HIGH) ;
//Serial.println("Turned on M2") ; //Debug Code
//delay(10) ;  
  }
  //Motor Bus 3
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M3BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M3BIT, 0) ;
    digitalWrite(M3_CTL_PIN, LOW) ;
//Serial.println("Turned off M3") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M3BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M3BIT) ;
    digitalWrite(M3_CTL_PIN, HIGH) ;
//Serial.println("Turned on M3") ; //Debug Code
//delay(10) ;  
  }
  //Motor Bus 4
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M4BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M4BIT, 0) ;
    digitalWrite(M4_CTL_PIN, LOW) ;
//Serial.println("Turned off M4") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M4BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M4BIT) ;
    digitalWrite(M4_CTL_PIN, HIGH) ;
//Serial.println("Turned on M4") ; //Debug Code
//delay(10) ;  
  }
  //Motor Bus 5
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M5BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M5BIT, 0) ;
    digitalWrite(M5_CTL_PIN, LOW) ;
//Serial.println("Turned off M5") ; //Debug Code
//delay(10) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M5BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M5BIT) ;
    digitalWrite(M5_CTL_PIN, HIGH) ;
//Serial.println("Turned on M5") ; //Debug Code
//delay(10) ;  
  }  
  //Motor Bus 6
    if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M6BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M6BIT, 0) ;
    digitalWrite(M6_CTL_PIN, LOW) ;
//Serial.println("Turned off M6") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M6BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M6BIT) ;
    digitalWrite(M6_CTL_PIN, HIGH) ;
//Serial.println("Turned on M6") ; //Debug Code
//delay(10) ;
  }
  //Motor Bus 7
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M7BIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_M7BIT, 0) ;
    digitalWrite(M7_CTL_PIN, LOW) ;
//Serial.println("Turned off M7") ; //Debug Code
//delay(10) ;  
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_M7BIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_M7BIT) ;
    digitalWrite(M7_CTL_PIN, HIGH) ;
//Serial.println("Turned on M7") ; //Debug Code
//delay(10) ;
  }
  //Auxilliary Bus
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_AUXBIT) == 0)
  {
    bitWrite(Bus[0], RC_POWERBOARD_BUSENABLED_AUXBIT, 0) ;
    digitalWrite(AUX_CTL_PIN, LOW) ;
//Serial.println("Turned off AUX") ; //Debug Code
//delay(10) ;
  }
  if(bitRead(Enable_Disable.data[RC_POWERBOARD_BUSENABLE_MOTORSENTRY], RC_POWERBOARD_BUSENABLE_AUXBIT) == 1)
  {
    bitSet(Bus[0], RC_POWERBOARD_BUSENABLED_AUXBIT) ;
    digitalWrite(AUX_CTL_PIN, HIGH) ;
//Serial.println("Turned on AUX") ; //Debug Code
//delay(10) ;  
  }
  return ;
}

void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS_PIN)
{
  // Using http://www.digikey.com/product-detail/en/allegro-microsystems-llc/ACS722LLCTR-40AU-T/620-1640-1-ND/4948876
  float adc_reading = analogRead(BUS_I_MEAS_PIN) ;
  current_reading = static_cast<uint16_t>(map(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX)*1000) ;
  return ;
}



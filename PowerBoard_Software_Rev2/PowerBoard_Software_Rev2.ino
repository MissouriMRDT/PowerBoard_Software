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

//Bus Control Variables
uint8_t Send_Recieve[] {0,0} ; //Bus to Enable or Disable
PB_Bus Bus[RC_POWERBOARD_IMEASmA_DATACOUNT] ; //An array of all the Busses on the Powerboard

//Packet Reception and Sent Variables
rovecomm_packet Enable_Disable ; //packet reception variable
uint16_t Current_Reading[RC_POWERBOARD_IMEASmA_DATACOUNT] ; //Current Reading for all busses
uint16_t error_reading[CURRENT_AVERAGE][RC_POWERBOARD_IMEASmA_DATACOUNT] ;
bool Bus_Tripped ; //To determine whether or not to send a packet based on overcurrents
bool sent_packet = true ; //To determine whether or not to send a packet of current values
uint32_t last_time_packet = 0 ; //Time since last packet or current values sent
bool Overcurrent = false ; //Shows whether or not a bus has overcurrented
int times_through = 0 ;
int average_holder = 0 ;

//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{
  Serial.begin(9600) ;
  delay(500) ;
//Serial.println("Got here...") ;
  Configure_Pins () ; //Configures pins to correct busses
  Pin_Initialization () ; //Sets pins to low then to high
  Bus_Setup(Bus) ; //Sets up the PowerBoard Busses in software
  Communication_Begin (Send_Recieve) ; //Sends to base station that everything is now on and communication begins
}//end setup

/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{
  //Current Readings to Report back to Base Station//////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Code to setup sending a current reading packet every second
  if(sent_packet == true)
  {
    last_time_packet = millis() ; //Timestamp
    sent_packet = false ; //So last_time_packet will not be overwritten too quickly
  }
  for(int i = 0 ; i < (RC_POWERBOARD_IMEASmA_DATACOUNT) ; i++)
  {
    Pin_Read(Current_Reading[i], Bus[i].imeas_pin, Bus[i].bus_tuning) ;
    //Serial.print(Bus[i].identity) ;
    //Serial.println(" Current Reading") ;
    //delay(10) ;
  }
  for(int k = 0 ; k <(RC_POWERBOARD_IMEASmA_DATACOUNT) ; k++)
  {
    error_reading[times_through][k] = Current_Reading[k] ;
  }
  times_through++ ;
  //Serial.println("") ;
  //End of Current Reads
  //Sends Current Values back to basestation every second, after the board has run through the code once
  if(millis() >= (last_time_packet+ROVECOMM_UPDATE_DELAY))
  {
    RoveComm.write(RC_POWERBOARD_IMEASmA_DATAID, RC_POWERBOARD_IMEASmA_DATACOUNT, Current_Reading) ; //Sends back current readings
    delay(ROVECOMM_DELAY) ;
    RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Send_Recieve) ; //Sends back present state of motor busses
    sent_packet = true ; //So we can update 
    delay(ROVECOMM_DELAY) ;
//Serial.println("Hello") ; //Debug Code
//delay(10) ;
  }
  
  //Checking for Over Currents on all busses//////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Serial.println("Checking comms") ; //Serial Debugging Code
//delay(10) ;
  if(times_through >= CURRENT_AVERAGE)
  {
    times_through = 0 ; //Reset the averaging of the current reads
    for(int L = 0 ; L < (RC_POWERBOARD_IMEASmA_DATACOUNT) ; L++) //Averaging the Current Readings over the amount specified by the Current_Average
    {
      for(int M = 0 ; M < CURRENT_AVERAGE ; M++)
      {
        average_holder = average_holder + error_reading[M][L] ;
      }
      average_holder = average_holder/CURRENT_AVERAGE ;
      Current_Reading[L] = average_holder ;
      average_holder = 0 ;
    }
    for(int j = 0 ; j < (RC_POWERBOARD_IMEASmA_DATACOUNT) ; j++)
    {
      Bus_Tripped = Shut_Off(Bus[j], Send_Recieve, Current_Reading[j]) ;
      Serial.print(Current_Reading[j]) ;
      Serial.print(" ") ;
      Serial.print(Bus[j].identity) ;
      Serial.println(" Overcurrent check") ;
      delay(10) ;
      if(Bus_Tripped == true)
      {
        Overcurrent = true ;
      }
    }
    Serial.println("") ;
    //End of Overcurrents
    //If any bus senses an overcurrent, then a packet will be sent 
    if(Overcurrent == true)
    {
      RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Send_Recieve) ; //Send out a summary of what is off after current check
      delay(ROVECOMM_DELAY) ;
      Bus_Tripped = false ;
      Overcurrent = false ;
    }
  }
  /////////////////////////////////////////////RED Control and Telem RoveComm
  //Recieves Pack form rovecomm and shuts off or turns on busses at need
    Enable_Disable = RoveComm.read();
    if(Enable_Disable.data_id == RC_POWERBOARD_BUSENABLE_DATAID)
    {  
      Bus_Enable(Enable_Disable, Send_Recieve, Bus) ;
      //Serial.println("") ;
      //Serial.println("Packet Recieved") ;
      //Serial.println("") ;
      //delay(3000) ;
      
    }
}  
//End of Main Loop


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Function 1.////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool singleDebounce(const int & bouncing_pin, const int & max_amps_threshold, const int & Tuner)
{
  int adc_threshhold = ((map(max_amps_threshold, CURRENT_MIN, CURRENT_MAX, ADC_MIN, ADC_MAX)*1000)/(Tuner)); //Get reading off pin
  bool trip = false ;
  if(analogRead(bouncing_pin) > adc_threshhold) //If pin reading is high
  {
    delay(DEBOUNCE_DELAY);
    if( analogRead(bouncing_pin) > adc_threshhold) //If pin reading is still high
    {
       trip = true; //Sends back true to indicate shut off
    }//end if
    else
    {
      trip = false ;
    }
  }// end if 
  return trip;
}

//Function 2.////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Configure_Pins ()
{
  // Control Pins are outputs
  pinMode(ACT_CTL_PIN, OUTPUT);
  pinMode(COMM_LOGIC_CTL_PIN, OUTPUT);
  pinMode(COMM_CTL_PIN, OUTPUT);
  pinMode(LOGIC_CTL_PIN, OUTPUT);
  pinMode(EM_CTL_PIN, OUTPUT);
  pinMode(FM_CTL_PIN, OUTPUT);
  pinMode(MM_CTL_PIN, OUTPUT);
  pinMode(BM_CTL_PIN, OUTPUT); 
  pinMode(AUX_CTL_PIN, OUTPUT);
  pinMode(ROCKET_CTL_PIN, OUTPUT);
  pinMode(GE_CTL_PIN, OUTPUT);
  pinMode(FAN_CTL_PIN, OUTPUT);

  //Current Measurement pins are inputs
  pinMode(ACT_I_MEAS_PIN, INPUT);
  pinMode(COMM_I_MEAS_PIN, INPUT);
  pinMode(LOGIC_I_MEAS_PIN, INPUT);
  pinMode(EM_I_MEAS_PIN, INPUT);
  pinMode(FM_I_MEAS_PIN, INPUT);
  pinMode(MM_I_MEAS_PIN, INPUT);
  pinMode(BM_I_MEAS_PIN, INPUT);
  pinMode(AUX_I_MEAS_PIN, INPUT);
  pinMode(ROCKET_I_MEAS_PIN, INPUT);
  pinMode(GE_I_MEAS_PIN, INPUT);
  return ;
}

//Function 4.////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pin_Initialization ()
{
  digitalWrite(ACT_CTL_PIN, LOW);
  digitalWrite(COMM_LOGIC_CTL_PIN, LOW);
  digitalWrite(COMM_CTL_PIN, LOW);
  digitalWrite(LOGIC_CTL_PIN, LOW);
  digitalWrite(EM_CTL_PIN, LOW);
  digitalWrite(FM_CTL_PIN, LOW);
  digitalWrite(MM_CTL_PIN, LOW);
  digitalWrite(BM_CTL_PIN, LOW);
  digitalWrite(AUX_CTL_PIN, LOW);
  digitalWrite(ROCKET_CTL_PIN, LOW);
  digitalWrite(GE_CTL_PIN, LOW);
  digitalWrite(FAN_CTL_PIN, LOW);
   // Turn on everything when we begin
  delay(ROVER_POWER_RESET_DELAY); //Three Second Delay

  //After Delay, make sure everything turns on
  digitalWrite(ACT_CTL_PIN, HIGH);
  digitalWrite(COMM_LOGIC_CTL_PIN, HIGH);
  digitalWrite(COMM_CTL_PIN, HIGH);
  digitalWrite(LOGIC_CTL_PIN, HIGH);
  digitalWrite(EM_CTL_PIN, HIGH);
  digitalWrite(FM_CTL_PIN, HIGH);
  digitalWrite(MM_CTL_PIN, HIGH);
  digitalWrite(BM_CTL_PIN, HIGH);
  digitalWrite(AUX_CTL_PIN, HIGH);
  digitalWrite(ROCKET_CTL_PIN, HIGH);
  digitalWrite(GE_CTL_PIN, HIGH);
  digitalWrite(FAN_CTL_PIN, HIGH); //Fans Always On
  return ;
}

//Function 5./////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Communication_Begin (uint8_t Send_Recieve []) //This function may have to change
{
  RoveComm.begin(RC_POWERBOARD_FOURTHOCTET);
  delay(100) ;
  Serial.println("Setting Up...");
  delay(ROVECOMM_DELAY); 
  uint8_t ALC_Bits = 1 ;
  uint8_t Motor_Bits = 1 ;
  uint8_t binary_hold = 2 ;
  for(int i = 1 ; i < ALC_BUSSES ; i++)
  {
    ALC_Bits = ALC_Bits + binary_hold ;
    binary_hold = 2*binary_hold ;
  }
  binary_hold = 2 ;
  for(int i = 1 ; i < MOTOR_BUSSES ; i++)
  {
    Motor_Bits = Motor_Bits + binary_hold ;
    binary_hold = 2*binary_hold ;
  }
  Send_Recieve[0] = ALC_Bits ;
  Send_Recieve[1] = Motor_Bits ;
  RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Send_Recieve) ; //Initial states of Busses
}

//Function 6.////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Shut_Off( const PB_Bus & Bus, uint8_t Send_Recieve[], const uint16_t & Current_Reading)
{ 
  bool Bus_Tripped = false ;
  //Special Variables in case of the communication bus being tripped
  static uint32_t time1 = 0 ;
  static bool comms_off = false ;
  const uint8_t comm_on_bit_code = 4 ;
  if(Current_Reading > Bus.amp_threshold ) //If pin is tripped
  {
     Bus_Tripped = true ;
     //Special Case for the Communication Bus
     if(Bus.ctl_pin == COMM_CTL_PIN)//Special rules for communication bus
     {
       Send_Recieve[0] = Send_Recieve[0] & Bus.bit_code_off ;
       RoveComm.write(RC_POWERBOARD_BUSENABLED_DATAID, RC_POWERBOARD_BUSENABLED_DATACOUNT, Send_Recieve) ; //Send a packet to rovecomm prior to shutting off comms
       delay(ROVECOMM_DELAY) ;
       digitalWrite(COMM_CTL_PIN, LOW); //Turn off the comms
       if(time1 == 0) //Timing Code, to turn on comms in 10 seconds, first half
       {
         comms_off = true ;
         time1 = millis();
       }
     }
     else //All other busses
     {
       Send_Recieve[Bus.rovecomm_cell] = Send_Recieve[Bus.rovecomm_cell] & Bus.bit_code_off ; 
       //Bitwise and with ones in bit code will result in only changing the specific bit I want changed
       digitalWrite(Bus.ctl_pin, LOW) ; //Turning off the bus
       delay(ROVECOMM_DELAY) ; //Just adding a delay in case the switch takes longer than expected                                                                                   
     }
  }
  if(comms_off == true) //Second half of timing code to turn on comms after 10 seconds
  {
    if(millis()>=(time1+10000))            //it is turned off in case the overcurrent was just a random spike. 
    {   
      comms_off = false ;                  //If there actually is a short in the bus, the bus will turn itself 
      time1 = 0 ;                          //back on        
      digitalWrite(COMM_CTL_PIN,HIGH);
      Send_Recieve[0] = Send_Recieve[0] | comm_on_bit_code ;  
    }
  } 
    return Bus_Tripped ;
}

//Function 7.//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Send_Recieve[], const PB_Bus Bus[])
{
  int ALC_to_Motor_Count = ALC_BUSSES ;
  int Start_Point = 0 ;
  int bit_helper = 1 ;
for(int i = 0 ; i < (RC_POWERBOARD_BUSENABLE_DATACOUNT) ; i++)
{
  for(int j = Start_Point ; j < ALC_to_Motor_Count ; j++)
  {
    //Serial.println(j) ;
    //delay(10) ;
    //Serial.print(Enable_Disable.data[i]) ;
    //Serial.println( " <- data sent") ;
    //delay(10) ;
    if(bitRead(Enable_Disable.data[i] , (bit_helper)) == 1) //Changing State of the Bus
    {
      //Serial.print("Changing State of ") ;
      //delay(10) ;
      if(Enable_Disable.data[2] == 1) //Turn on
      {
        //Serial.print(Bus[j].identity) ;
        //Serial.println(" Turn On") ;
        //delay(10) ;
        Send_Recieve[i] = Send_Recieve[i] | Bus[j].bit_code_on ;
        //digitalWrite(Bus[j].ctl_pin, HIGH) ;
        //delay(ROVECOMM_DELAY) ;
      }
      else //Turn OFF
      {
        //Serial.print(Bus[j].identity) ;
        //Serial.println( " Turn OFF") ;
        //delay(10) ;
        Send_Recieve[i] = Send_Recieve[i] & Bus[j].bit_code_off ;
        digitalWrite(Bus[j].ctl_pin, LOW) ;
        delay(ROVECOMM_DELAY) ;
      }
    }
    bit_helper++ ;
  }
  Start_Point = ALC_to_Motor_Count;
  ALC_to_Motor_Count = ALC_to_Motor_Count + MOTOR_BUSSES ;
  bit_helper = 1 ;
}
  return ;
}

//Function 8.////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS_PIN, const int & Tuner)
{
  // Using http://www.digikey.com/product-detail/en/allegro-microsystems-llc/ACS722LLCTR-40AU-T/620-1640-1-ND/4948876
  int adc_reading = analogRead(BUS_I_MEAS_PIN) ;
  if(adc_reading < ADC_MIN)
  {
    adc_reading = ADC_MIN ; 
  }
  if(adc_reading > ADC_MAX)
  {
    adc_reading = ADC_MAX ;
  }
  current_reading = ((((map(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX)*1180)/1000)*Tuner)/1000);
  //Serial.println(current_reading) ;
  return ;
}

void PB_Bus::Adjust_Values (const uint8_t & Rovecomm_cell, const uint8_t & Bit_code_off, const uint8_t & Bit_code_on, const int & Imeas_pin, const int & Ctl_pin, const int & Amp_threshold, const int & Bus_tuning, const String Identity) 
{
  rovecomm_cell = Rovecomm_cell ;
  bit_code_off = Bit_code_off ; //Binary converted to decimal (255-bit_code_on)
  bit_code_on = Bit_code_on ; //Binary converted to decimal, 2^(place in cell)
  imeas_pin = Imeas_pin ;
  ctl_pin = Ctl_pin ;
  amp_threshold = Amp_threshold ;
  bus_tuning = Bus_tuning ;
  identity = Identity ;
}
void Bus_Setup(PB_Bus Bus[])
{
  Bus[0].Adjust_Values(RC_POWERBOARD_BUSENABLE_ALCENTRY, 254, 1, ACT_I_MEAS_PIN, ACT_CTL_PIN, ESTOP_12V_ACT_MAX_AMPS_THRESHOLD, ACT_TUNER, "Actuation") ;
  Bus[1].Adjust_Values(RC_POWERBOARD_BUSENABLE_ALCENTRY, 253, 2, LOGIC_I_MEAS_PIN, LOGIC_CTL_PIN, ESTOP_12V_COMM_LOGIC_MAX_AMPS_THRESHOLD, LOGIC_COMM_TUNER, "Logic") ;
  Bus[2].Adjust_Values(RC_POWERBOARD_BUSENABLE_ALCENTRY, 251, 4, COMM_I_MEAS_PIN, COMM_CTL_PIN, ESTOP_12V_COMM_LOGIC_MAX_AMPS_THRESHOLD, LOGIC_COMM_TUNER, "Communications") ;
  Bus[3].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 254, 1, FM_I_MEAS_PIN, FM_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD, MOTOR_TUNER, "Front Motors") ;
  Bus[4].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 253, 2, MM_I_MEAS_PIN, MM_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD, MOTOR_TUNER, "Middle Motors") ;
  Bus[5].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 251, 4, BM_I_MEAS_PIN, BM_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD, MOTOR_TUNER, "Back Motors") ;
  Bus[6].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 247, 8, EM_I_MEAS_PIN, EM_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD, MOTOR_TUNER, "Extra Motors") ;
  Bus[7].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 239, 16, ROCKET_I_MEAS_PIN, ROCKET_CTL_PIN, ESTOP_ROCKET_BUS_MAX_AMPS_THRESHOLD, ROCKET_TUNER, "Rockets") ;
  Bus[8].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 223, 32, GE_I_MEAS_PIN, GE_CTL_PIN, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD, MOTOR_TUNER, "General Extra") ;
  Bus[9].Adjust_Values(RC_POWERBOARD_BUSENABLE_MOTORSENTRY, 191, 64, AUX_I_MEAS_PIN, AUX_CTL_PIN, ESTOP_AUX_MAX_AMPS_THRESHOLD, AUX_TUNER, "Auxillary") ;
}


  

/*
*   POWERBOARD SOFTWARE
*   Patrick Sanchez
*   Missouri University of Science and Technology
*   Mars Rover Design Team
*   2020, Revision 1
*/

//C types
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//RoveComm
#include <RoveComm.h>

RoveCommEthernet RoveComm;
rovecomm_packet packet;

const uint16_t NO_ROVECOMM_MESSAGE          = 0;

//Current readings
const uint16_t MOTOR_CURRENT_READING      = 3105; //M1,M2,M3,M4,M5,M6,M7,Spare (last 2 unused atm)
const uint16_t ACT_LOG_CURRENT_READING    = 3106; //Actuation, Logic
const uint16_t MISC_30V_CURRENT_READING   = 3107; //12V Board, Rockets, Auxiliary
const uint16_t VAC_CURRENT_READING        = 3108; //Vacuum

//Bus control
//Disable = 0, Enable = 1
const uint16_t MOTOR_ENABLE      = 3100; //M1,M2,M3,M4,M5,M6,M7,Spare (last 2 unused atm)
const uint16_t ACT_LOG_ENABLE    = 3101; //Actuation,Logic
const uint16_t MISC_30V_ENABLE   = 3102; //12V Board, Rockets, Auxiliary
const uint16_t VAC_ENABLE        = 3103; //Vacuum
const uint16_t PATCH_ENABLE      = 3104; //PB1,PB2,PB3,PB4,PB5,PB6,PB7,PB8


//TIVA PIN MAPPINGS
//ANALOG PINS
//30V
#define RKT_ISENSE PE_4
#define TWV_ISENSE PE_5
#define AUX_ISENSE PD_3
#define VAC_ISENSE PE_2
#define M1_ISENSE PD_4
#define M2_ISENSE PK_1
#define M3_ISENSE PK_3
#define M4_ISENSE PD_2
#define M5_ISENSE PB_4
#define M6_ISENSE PE_1
//12V
#define LOG_ISENSE PD_7
#define ACT_ISENSE PE_3

//TEMPORARY MAPPINGS BECAUSE I DID BAD
#define LOG_EN PA_6
#define ACT_EN PD_7
//end temp stuff

//DIGITAL PINS
//Error LED
#define ERROR PB_3
//30V
#define RKT_EN PB_2
#define TWV_EN PC_6
#define AUX_EN PC_7 
#define VAC_EN PC_5
#define M1_EN PP_1
#define M2_EN PD_5
#define M3_EN PQ_0
#define M4_EN PN_5
#define M5_EN PP_0
#define M6_EN PE_0
//12V
//#define LOG_EN PM_4
//#define ACT_EN PA_6
#define VOUT_EN PM_5
//Patch Panel
#define PB1_EN PL_3
#define PB2_EN PF_3
#define PB3_EN PL_2
#define PB4_EN PG_0
#define PB5_EN PL_1
#define PB6_EN PL_4
#define PB7_EN PL_0
#define PB8_EN PL_5

//Current sense mapping values
const uint16_t ADCMin = 330;
const uint16_t ADCMax = 4095;
const uint16_t ISENSEMin = 0;
const uint16_t ISENSEMax = 40000;
const float CURR_SCALER = 1.15;

//Overcurrent constants (amps)
const uint8_t OCP_MOTOR = 18;
const uint8_t OCP_12V = 18;
const uint8_t OCP_30V = 18;
const uint8_t OCP_RKT = 4;
const uint8_t OCP_VAC = 13;
const uint16_t RESET_TIME = 3000;  //ms between tripping OCP and turn on


struct Bus
{
  int EN_PIN;
  int ISENSE_PIN = -1;//default for busses without ISENSE pins

  Bus(const int en, const int isense)
  {
    EN_PIN = en;
    ISENSE_PIN = isense;
    pinMode(ISENSE_PIN,INPUT);
    pinMode(EN_PIN,OUTPUT);
  }

  Bus(const int en)
  {
    EN_PIN = en;
    pinMode(EN_PIN,OUTPUT);
  }

};

const Bus busses_Motor[6] = {Bus(M1_EN,M1_ISENSE),Bus(M2_EN,M2_ISENSE),Bus(M3_EN,M3_ISENSE),Bus(M4_EN,M4_ISENSE),Bus(M5_EN,M5_ISENSE),Bus(M6_EN,M6_ISENSE)};
const Bus busses_12V[3] = {Bus(ACT_EN,ACT_ISENSE),Bus(LOG_EN,LOG_ISENSE),Bus(VOUT_EN)};
const Bus busses_30V[3] = {Bus(TWV_EN,TWV_ISENSE),Bus(RKT_EN,RKT_ISENSE),Bus(AUX_EN,AUX_ISENSE)};
const Bus vac = Bus(VAC_EN,VAC_ISENSE);

void setup() {
  Serial.begin(9600);
  RoveComm.begin(RC_POWERBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_POWERBOARD_PORT);
  delay(100);
  Serial.println("Started");
}

void loop()
{
  delay(200);
  //read
  packet = RoveComm.read();
  //Serial.println(packet.data_id);

  


  //Motor bus switches
  switch(packet.data_id)
  {
    case RC_POWERBOARD_MOTOR_BUSENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is M1, bit 1 is M2, etc.
      for(int i = 0; i < 6; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable motor
          digitalWrite(busses_Motor[i].EN_PIN,HIGH);
        }
        else
        {
          //disable motor
          digitalWrite(busses_Motor[i].EN_PIN,LOW);
        }
      }
      break;
    }
    case RC_POWERBOARD_12V_BUSENABLE_DATAID:
    {      
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is Actuation, bit 1 is Logic, bit 2 is VOUT.
      for(int i = 0; i < 3; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable bus
          digitalWrite(busses_12V[i].EN_PIN,HIGH);
        }
        else
        {
          //disable bus
          digitalWrite(busses_12V[i].EN_PIN,LOW);
        }
      }
      break;
    }
    case RC_POWERBOARD_30V_BUSENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is 12V, bit 1 is Rocket, bit 2 is Aux.
      for(int i = 0; i < 3; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable bus
          digitalWrite(busses_30V[i].EN_PIN,HIGH);
        }
        else
        {
          //disable bus
          digitalWrite(busses_30V[i].EN_PIN,LOW);
        }
      }
      break;
    }
    case RC_POWERBOARD_VACUUM_ENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks first bit.
      if(packet.data[0] & 1) 
      {
        //enable bus
        digitalWrite(vac.EN_PIN,HIGH);
      }
      else
      {
        //disable bus
        digitalWrite(vac.EN_PIN,LOW);
      }
      break;
    }
    default:
    {
      Serial.println("Unfamiliar code");
    }
  }



  //Motor current output
  float curr_Motor[6] = {0,0,0,0,0,0};
  float temp;
  float real;
  for(int i = 0; i < 6; i++)
  {
    temp = analogRead(busses_Motor[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_Motor[i] = real;
  }
  RoveComm.write(RC_POWERBOARD_MOTOR_BUS_CURRENT_DATAID, RC_POWERBOARD_MOTOR_BUS_CURRENT_DATACOUNT, curr_Motor);

  /*
  //12V current output
  float curr_12V[2] = {0,0};
  for(int i = 0; i < 2; i++)
  {
    temp = analogRead(busses_12V[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_12V[i] = real;
  }
  RoveComm.write(RC_POWERBOARD_12V_BUS_CURRENT_DATAID, RC_POWERBOARD_12V_BUS_CURRENT_DATACOUNT, curr_12V);
  */

  //30V current output
  float curr_30V[3] = {0,0,0};
  for(int i = 0; i < 3; i++)
  {
    temp = analogRead(busses_30V[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_30V[i] = real;
  }
  RoveComm.write(RC_POWERBOARD_30V_BUS_CURRENT_DATAID, RC_POWERBOARD_30V_BUS_CURRENT_DATACOUNT, curr_30V);

  //Vac current output
  float curr_vac = analogRead(vac.ISENSE_PIN);
  curr_vac = map(curr_vac,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
  curr_vac = (curr_vac*CURR_SCALER)/1000;
  RoveComm.write(RC_POWERBOARD_VACUUM_CURRENT_DATAID, RC_POWERBOARD_VACUUM_CURRENT_DATACOUNT, curr_vac);

  //Overcurrent Protection - motors
  uint8_t error_motor = 0;
  for(int i = 0; i < 6; i++)
  {
    if(curr_Motor[i] >= OCP_MOTOR)
    {
      //disable bus
      digitalWrite(busses_Motor[i].EN_PIN,LOW);
      //save error
      error_motor |= 1<<i;
    }
  }
  /*ACTUATION AND LOGIC CURRENT SENSE DISABLED UNTIL REV 2
  uint8_t error_12V = 0;
  for(int i = 0; i < 6; i++)
  {
    if(curr_Motor[i] >= OCP_MOTOR)
    {
      //disable bus
      digitalWrite(busses_Motor[i].EN_PIN,LOW);
      //save error
      error_motor |= 1<<i;
    }
  }
  */
  uint8_t error_30V = 0;
  for(int i = 0; i < 6; i++)
  {
    if(curr_30V[i] >= OCP_MOTOR || (i==1 && curr_30V[i] >= OCP_RKT))
    {
      //disable bus
      digitalWrite(busses_30V[i].EN_PIN,LOW);
      //save error
      error_30V |= 1<<i;
    }
  }
  bool error_vac = false;
  if(curr_vac >= OCP_VAC)
  {
    //disable bus
    digitalWrite(vac.EN_PIN,LOW);
    //save error
    error_vac = true;
  }

  /*
  if(error_motor /*|| error_12V || error_30V || error_vac)
  {
    Serial.println("Shit's fucked");
    //NEED ERROR LED ACTIVATION
    RoveComm.write(RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATAID, RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATACOUNT, error_motor);
    //wait, then reenable
    Serial.println("Made it past the write");
    delay(RESET_TIME);
    //reset errored motors
    for(int i = 0; i < 6; i++)
    {
      //checks for motor busses with errors and turns them on again
      if(error_motor & 1<<i)
      {
        //enable motor
        digitalWrite(busses_Motor[i].EN_PIN,HIGH);
      }
    }
    /*12V current sense is big dead for now
    //Actuation and Logic reset
    for(int i = 0; i < 2; i++)
    {
      //checks for 12V busses with errors and turns them on again
      if(error_12V & 1<<i)
      {
        //enable motor
        digitalWrite(busses_12V[i].EN_PIN,HIGH);
      }
    }
    
    //
    for(int i = 0; i < 3; i++)
    {
      //checks for motor busses with errors and turns them on again
      if(error_motor & 1<<i)
      {
        //enable motor
        digitalWrite(busses_Motor[i].EN_PIN,HIGH);
      }
    }
  }
    */
}
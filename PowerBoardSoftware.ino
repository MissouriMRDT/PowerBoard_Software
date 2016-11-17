//RoveWare Powerboard ACS_722 Interface
//
// Judah jrs6w7
//
// Using http://www.allegromicro.com/en/Products/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS722.aspx
//
// Standard C
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//Energia
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//RoveWare
#include <RoveBoard.h>
#include <RoveEthernet.h>
#include <RoveComm.h>

// RED can toggle the bus by bool
const uint16_t NO_ROVECOMM_MESSAGE          = 0;

const uint16_t M1_CURRENT_READING           = 1104;
const uint16_t M2_CURRENT_READING           = 1105;
const uint16_t M3_CURRENT_READING           = 1106;
const uint16_t M4_CURRENT_READING           = 1107;
const uint16_t M5_CURRENT_READING           = 1108;
const uint16_t M6_CURRENT_READING           = 1109;
const uint16_t M7_CURRENT_READING           = 1110;
const uint16_t 12V_EXTRA_CURRENT_READING    = 1111; //changed name
const uint16_t 12V_ACT_CURRENT_READING      = 1112; //changed name
const uint16_t 12V_LOGIC_CURRENT_READING    = 1113; //changed name
const uint16_t 12V_COM_CURRENT_READING      = 1114; //created new
const uint16_t PACK_VOLTAGE_READING         = 1115; //created new

const uint16_t ROVER_POWER_RESET           = 1041;

const uint16_t POWER_BUS_ENABLE             = 1088;
const uint16_t POWER_BUS_DISABLE            = 1089;
const uint16_t POWER_BUS_OVER_CURRENT       = 1090;

const uint8_t BUS_M1_ON_OFF                 = 0;
const uint8_t BUS_M2_ON_OFF                 = 1;
const uint8_t BUS_M3_ON_OFF                 = 2;
const uint8_t BUS_M4_ON_OFF                 = 3;
const uint8_t BUS_M5_ON_OFF                 = 4;
const uint8_t BUS_M6_ON_OFF                 = 5;
const uint8_t BUS_M7_ON_OFF                 = 6;
const uint8_t BUS_12V_EXTRA_ON_OFF          = 7;  //changed name
const uint8_t BUS_12V_ACT_ON_OFF            = 8;  //changed name
const uint8_t BUS_12V_LOGIC_ON_OFF          = 9;  //changed name
const uint8_t BUS_12V_COM_ON_OFF            = 10; //created new
const uint8_t BUS_12V_COM/LOGIC_ON_OFF      = 11; //created new

const int ROVER_POWER_RESET_DELAY          = 3000;

const int ` = 2000;

//Rovecomm :: RED packet :: data_id and data_value with number of data bytes size
uint16_t data_id       = 0;
size_t   data_size     = 0; 
uint8_t  data_value    = 0;

const int ROVECOMM_DELAY = 10;

//////////////////////////////////////////////Pinmap
// Control Pins

//0 MIN VOLT    3.036 MAX_VOLT  RESISTOR DIVIDER = 11;      
//const int BATTERYPACK_CNTRL  = 11;
const int 12V_EXTRA_CNTRL     = 17;   
const int 12V_ACT_CNTRL       = 13;
const int 12V_LOGIC_CNTRL     = 18; //created new
const int 12V_COM_CNTRL       = 34; //created new
const int 12V_COM/LOGIC_CNTRL = 11; //created new
const int M1_CNTRL            = 58;
const int M2_CNTRL            = 57;
const int M3_CNTRL            = 74;
const int M4_CNTRL            = 53;
const int M5_CNTRL            = 73;
const int M6_CNTRL            = 72;
const int M7_CNTRL            = 71;


// Sensor Volts/Amps Readings Pins
const int 12V_EXTRA_AMPS      = 26;
const int 12V_ACT_AMPS        = 25;
const int 12V_LOGIC_AMPS      = 24;
const int 12V_COM_AMPS        = 23;
const int M1_AMPS             = 68;
const int M2_AMPS             = 67;
const int M3_AMPS             = 66; 
const int M4_AMPS             = 45;
const int M5_AMPS             = 65;
const int M6_AMPS             = 64;
const int M7_AMPS             = 63;
const int PACK_VOLTAGE        = 6;

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
const float VCC                 = 3.3;       //volts
const float ADC_MAX             = 4096;      //bits
const float ADC_MIN             = 0;         //bits
float adc_reading = 0;

//////////////////////////////////////////////Sensor
// ACS_722 IC Sensor Specs 
const float SENSOR_SENSITIVITY   = 0.033;    //volts/amp
const float SENSOR_SCALE         = 0.5;      //volts/amp
//const float SENSOR_SENSITIVITY   = 0.125;    //volts/amp
//const float SENSOR_SCALE         = 0.1;      //volts/amp
const float SENSOR_BIAS          = VCC * SENSOR_SCALE;

const float CURRENT_MAX          = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;
const float CURRENT_MIN          = -SENSOR_BIAS / SENSOR_SENSITIVITY;
float current_reading            = 0;

const int DEBOUNCE_DELAY = 10;

//Safest Test pin
const int ESTOP_12V_COM/LOGIC_MAX_AMPS_THRESHOLD = 5;
const int ESTOP_12V_EXTRA/ACT_MAX_AMPS_THRESHOLD = 15;    
const int ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD = 22;

// Checks the pin for bouncing voltages to avoid false positives
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
}//end fntcn


///////////////////////////////////////////////Implementation
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end fnctn


//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{  
  // Control Pins are outputs
  pinMode(12V_EXTRA_CNTRL, OUTPUT);
  pinMode(12V_ACT_CNTRL, OUTPUT);
  pinMode(12V_COM/LOGIC_CNTRL, OUTPUT);
  pinMode(12V_COM_CNTRL, OUTPUT);
  pinMode(12V_LOGIC_CNTRL, OUTPUT);
  
  pinMode(M1_CNTRL, OUTPUT);
  pinMode(M2_CNTRL, OUTPUT);
  pinMode(M3_CNTRL, OUTPUT); 
  pinMode(M4_CNTRL, OUTPUT);
  pinMode(M5_CNTRL, OUTPUT);
  pinMode(M6_CNTRL, OUTPUT);
  pinMode(M7_CNTRL, OUTPUT);
  
  // Turn on everything when we begin
  delay(ROVER_POWER_RESET_DELAY);
  
  digitalWrite(12V_EXTRA_CNTRL, HIGH);
  digitalWrite(12V_ACT_CNTRL, HIGH);
  digitalWrite(12V_COM/LOGIC_CNTRL, HIGH);
  digitalWrite(12V_COM_CNTRL, HIGH);
  digitalWrite(12V_LOGIC_CNTRL, HIGH);
    
  digitalWrite(M1_CNTRL, HIGH);
  digitalWrite(M2_CNTRL, HIGH);
  digitalWrite(M3_CNTRL, HIGH);
  digitalWrite(M4_CNTRL, HIGH);
  digitalWrite(M5_CNTRL, HIGH);
  digitalWrite(M6_CNTRL, HIGH);
  digitalWrite(M7_CNTRL, HIGH);
 
  roveComm_Begin(192, 168, 1, 132);
  
}//end setup

//Loop
//
/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{ 
  if( singleDebounce(12V_EXTRA_AMPS, ESTOP_12V_EXTRA/ACT_MAX_AMPS_THRESHOLD) ) //checks current reading and sends error msg to base
  {                                                                            //station then turns off the bus if too high       
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_EXTRA_ON_OFF), &BUS_12V_EXTRA_ON_OFF);
    delay(500);
    digitalWrite(12V_EXTRA_CNTRL, LOW);   
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(12V_ACT_AMPS, ESTOP_12V_EXTRA/ACT_MAX_AMPS_THRESHOLD) )
  {
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_ACT_ON_OFF), &BUS_12V_ACT_ON_OFF);
    delay(500);
    digitalWrite(12V_ACT_AMPS, LOW);
    delay(ROVECOMM_DELAY);
  }//end if

  if( singleDebounce(12V_LOGIC_AMPS, ESTOP_12V_COM/LOGIC_MAX_AMPS_THRESHOLD) )
  {
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_LOGIC_ON_OFF), &BUS_12V_LOGIC_ON_OFF);
    delay(500);
    digitalWrite(12V_LOGIC_AMPS, LOW);
    delay(ROVECOMM_DELAY);
  }//end if

  if( singleDebounce(12V_COM_AMPS, ESTOP_12V_COM/LOGIC_MAX_AMPS_THRESHOLD) )
  {
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_COM_ON_OFF), &BUS_12V_COM_ON_OFF);
    delay(500);
    digitalWrite(12V_COM_AMPS, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M1_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ) 
  {
    digitalWrite(M1_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M1_ON_OFF), &BUS_M1_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M2_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M2_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M2_ON_OFF), &BUS_M2_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
   if(singleDebounce(M3_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M3_AMPS, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M3_ON_OFF), &BUS_M3_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M4_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M4_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M4_ON_OFF), &BUS_M4_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M5_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M5_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M5_ON_OFF), &BUS_M5_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M6_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M6_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M6_ON_OFF), &BUS_M6_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M7_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    digitalWrite(M7_CNTRL, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M7_ON_OFF), &BUS_M7_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  /////////////////////////////////////////////RED Control and Telem RoveComm

  //If there is no message data_id gets set to zero
  roveComm_GetMsg(&data_id, &data_size, &data_value);
  
  switch (data_id) //either 0 or 1 
  {   
    //Don't do anything for data_id zero 
    case NO_ROVECOMM_MESSAGE: //data_id is 0; do nothing
      break; 
      
    case POWER_BUS_ENABLE: //data_id is 1088
      
      switch (data_value)
      { 
        case BUS_12V_EXTRA_ON_OFF:
          digitalWrite(12V_EXTRA_CNTRL, HIGH);
          break;

        case BUS_12V_ACT_ON_OFF:
          digitalWrite(12V_ACT_CNTRL, HIGH);
          break;

        case BUS_12V_COM/LOGIC_ON_OFF:
          digitalWrite(12V_COM/LOGIC_CNTRL, HIGH);
          break;
          
        case BUS_12V_LOGIC_ON_OFF:
          digitalWrite(12V_LOGIC_CNTRL, HIGH);
          break;

        case BUS_12V_COM_ON_OFF:
          digitalWrite(12V_COM_CNTRL, HIGH);
          break;
          
        case BUS_M1_ON_OFF:
          digitalWrite(M1_CNTRL, HIGH);
          break;
          
        case BUS_M2_ON_OFF:
          digitalWrite(M2_CNTRL, HIGH);
          break;
          
        case BUS_M3_ON_OFF:
          digitalWrite(M3_CNTRL, HIGH);
          break;
          
        case BUS_M4_ON_OFF:
          digitalWrite(M4_CNTRL, HIGH);
          break;
          
        case BUS_M5_ON_OFF:
          digitalWrite(M5_CNTRL, HIGH);
          break;
          
        case BUS_M6_ON_OFF:
          digitalWrite(M6_CNTRL, HIGH);
          break;
          
        case BUS_M7_ON_OFF:
          digitalWrite(M7_CNTRL, HIGH);
          break;
          
        default:
          //Serial.print("Unrecognized data :");
          //Serial.println(data);
          break; 
       }//endswitch 
       break;  
     
    case POWER_BUS_DISABLE: //data_id id 1089
    
      switch ( data_value )
      { 
        case BUS_12V_EXTRA_ON_OFF:
          digitalWrite(12V_EXTRA_CNTRL, LOW);
          break;

        case BUS_12V_ACT_ON_OFF:
          digitalWrite(12V_ACT_CNTRL, LOW);
          break;

        case BUS_12V_COM/LOGIC_ON_OFF:
          digitalWrite(12V_COM/LOGIC_CNTRL, LOW);
          break;
          
        case BUS_12V_LOGIC_ON_OFF:
          digitalWrite(12V_LOGIC_CNTRL, LOW);
          break;

        case BUS_12V_COM_ON_OFF:
          digitalWrite(12V_COM_CNTRL, LOW);
          break;
          
        case BUS_M1_ON_OFF:
          digitalWrite(M1_CNTRL, LOW);
          break;
          
        case BUS_M2_ON_OFF:
          digitalWrite(M2_CNTRL, LOW);
          break;
          
        case BUS_M3_ON_OFF:
          digitalWrite(M3_CNTRL, LOW);
          break;
          
        case BUS_M4_ON_OFF:
          digitalWrite(M4_CNTRL, LOW);
          break;
          
        case BUS_M5_ON_OFF:
          digitalWrite(M5_CNTRL, LOW);
          break;
          
        case BUS_M6_ON_OFF:
          digitalWrite(M6_CNTRL, LOW);
          break;
          
        case BUS_M7_ON_OFF:
          digitalWrite(M7_CNTRL, LOW);
          break;
          
        default:
          //Serial.print("Unrecognized data :");
          //Serial.println(data);
          break; 
       }//endswitch 
       break;
       
    case ROVER_POWER_RESET: //data_id is 1041
      
      digitalWrite(M1_CNTRL, LOW);
      digitalWrite(M2_CNTRL, LOW);
      digitalWrite(M3_CNTRL, LOW);
      digitalWrite(M4_CNTRL, LOW);
      digitalWrite(M5_CNTRL, LOW);
      digitalWrite(M6_CNTRL, LOW);
      digitalWrite(M7_CNTRL, LOW);  
                  
      digitalWrite(12V_ACT_CNTRL, LOW);
      digitalWrite(12V_EXTRA_CNTRL, LOW);
      digitalWrite(12V_LOGIC_CNTRL, LOW);
      digitalWrite(12V_COM_CNTRL, LOW);
      digitalWrite(12V_COM/LOGIC_CNTRL, LOW);
     
      delay(ROVER_POWER_RESET_DELAY);

      digitalWrite(12V_EXTRA_CNTRL, HIGH);
      digitalWrite(12V_ACT_CNTRL, HIGH);
      digitalWrite(12V_COM/LOGIC_CNTRL, HIGH);
      digitalWrite(12V_COM_CNTRL, HIGH);
      digitalWrite(12V_LOGIC_CNTRL, HIGH);
    
      digitalWrite(M1_CNTRL, HIGH);
      digitalWrite(M2_CNTRL, HIGH);
      digitalWrite(M3_CNTRL, HIGH);
      digitalWrite(M4_CNTRL, HIGH);
      digitalWrite(M5_CNTRL, HIGH);
      digitalWrite(M6_CNTRL, HIGH);
      digitalWrite(M7_CNTRL, HIGH);  
      
     break;
         
    default:
      //Serial.print("Unrecognized data_id :");
      //Serial.println(data_id);
      break; 
  }//endswitch 
 
  adc_reading = analogRead(12V_EXTRA_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(12V_EXTRA_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(12V_ACT_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(12V_ACT_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(12V_LOGIC_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(12V_LOGIC_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(12V_COM_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(12V_COM_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M1_AMPS); 
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);  
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  
  adc_reading = analogRead(M2_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M2_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M3_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M3_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M4_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M4_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M5_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M5_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M6_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M6_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M7_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M7_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
}//end loop


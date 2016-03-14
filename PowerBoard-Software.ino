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



//Hardware
//
// Todo Mike and Cameron sign off
//
//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
const float VCC                 = 3.3;       //volts
const float ADC_MAX             = 4096;      //bits
const float ADC_MIN             = 0;         //bits

float adc_reading = 0;

//////////////////////////////////////////////Sensor
// ACS_722 IC Sensor Specs 
const float SENSOR_SENSITIVITY   = 0.132;    //volts/amp
const float SENSOR_SCALE         = 0.1;      //volts/amp
const float SENSOR_BIAS          = VCC * SENSOR_SCALE;

const float CURRENT_MAX = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;
const float CURRENT_MIN = -SENSOR_BIAS / SENSOR_SENSITIVITY;

float current_reading             = 0;



//Testing
//
//////////////////////////////////////////////Debug Flags
const int SOFTWARE_FUSES_DEBUG =          0;
const int RED_COMMS_DEBUG =               0;

const int ECHO_SERIAL_MONITOR_DEBUG =     1;
const int DELAY_SERIAL_MILLIS_DEBUG =     100;

//Todo Cameron and Mike 
//////////////////////////////////////////////Hardware Calibration
const int ANALOG_DEBOUNCE_TIME_MICROS = 250;
const int ANALOG_TRY_COUNT = 250;

const int ANALOG_ACCEPTABLE_DRIFT = 250;

const int DIGITAL_DEBOUNCE_TIME_MICROS = 250;
const int DIGITAL_TRY_COUNT = 250; 

const int PIN_TOO_NOISY = -1;

//Safest Test pin
const int ESTOP_5V_BUS_MAX_CURRENT_THRESHOLD = 1;



//Platform
//////////////////////////////////////////////Energia
// Energia libraries used by RoveWare itself
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//////////////////////////////////////////////Roveware
#include "RoveEthernet.h"
#include "RoveComm.h"

// RED udp device id by fourth octet
const int POWERBOARD_IP_DEVICE_ID   = 51;

// RED can toggle the bus by bool
const uint16_t NO_ROVECOMM_MESSAGE   = 0;
const uint16_t BUS_5V_ON_OFF         = 207;

//Rovecomm :: RED packet :: data_id and data_value with number of data bytes size
uint16_t data_id       = 0;
size_t   data_size     = 0; 
uint16_t data_value    = 0;



//////////////////////////////////////////////Pinmap
// Control Pins
//Todo const int BATTERYPACK_CNTRL?
const int BUS_5V_CNTRL_PP_2  = 11;
const int BUS_12V_CNTRL_PN_3 = 12;
const int M1_CNTRL_PK_7      = 71;
const int M2_CNTRL_PQ_1      = 52;
const int M3_CNTRL_PK_6      = 72;
const int M4_CNTRL_PP_3      = 53;
const int M5_CNTRL_PH_1      = 73;
const int M6_CNTRL_PH_0      = 74;
const int M7_CNTRL_PA_7      = 57;
const int M8_CNTRL_PP_5      = 58;

// Sensor Volts/Amps Readings Pins
//Todo const int BATTERYPACK_VOLTS_PE3?
const int BUS_5V_AMPS_PE_2   = 25;
const int BUS_12V_AMPS_PD_7  = 27;
const int M1_AMPS_PK_3       = 68;
const int M2_AMPS_PK_2       = 67 ;
const int M3_AMPS_PK_1       = 66; 
const int M4_AMPS_PD_4       = 45;
const int M5_AMPS_PK_0       = 65;
const int M6_AMPS_PB_5       = 64;
const int M7_AMPS_PB_4       = 63;
const int M8_AMPS_PD_2       = 42;

//Developing 
//
// Todo: Connor/Reed Edit
//
// Checks the pin for bouncing voltages to avoid false positives
int digitalDebounce(int bouncing_pin);
int analogDebounce(int bouncing_pin);

//////////////////////////////////////////////User Display
// Map analog read voltage value from the ACS_722 to a human readable RED display current value
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max);



//Begin
//
// Todo Mike and Cameron sign off
//
//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{  
  // Control Pins are outputs
  pinMode(BUS_5V_CNTRL_PP_2, OUTPUT);
  pinMode(BUS_12V_CNTRL_PN_3, OUTPUT);
  
  pinMode(M1_CNTRL_PK_7, OUTPUT);
  pinMode(M2_CNTRL_PQ_1, OUTPUT);
  pinMode(M3_CNTRL_PK_6, OUTPUT); 
  pinMode(M4_CNTRL_PP_3, OUTPUT);
  pinMode(M5_CNTRL_PH_1, OUTPUT);
  pinMode(M6_CNTRL_PH_0, OUTPUT);
  pinMode(M7_CNTRL_PA_7, OUTPUT);
  pinMode(M8_CNTRL_PP_5, OUTPUT);
  
  // Turn on everything when we begin
  digitalWrite(BUS_5V_CNTRL_PP_2, HIGH);
  digitalWrite(BUS_12V_CNTRL_PN_3, HIGH);
  
  digitalWrite(M1_CNTRL_PK_7, HIGH);
  digitalWrite(M2_CNTRL_PQ_1, HIGH);
  digitalWrite(M3_CNTRL_PK_6, HIGH);
  digitalWrite(M4_CNTRL_PP_3, HIGH);
  digitalWrite(M5_CNTRL_PH_1, HIGH);
  digitalWrite(M6_CNTRL_PH_0, HIGH);
  digitalWrite(M7_CNTRL_PA_7, HIGH);
  digitalWrite(M8_CNTRL_PP_5, HIGH);
  
  // initialize serial communication at 9600 bits per second:
  if(ECHO_SERIAL_MONITOR_DEBUG)
  {
    Serial.begin(9600);
  }//end if
  
  if(RED_COMMS_DEBUG)
  { 
    roveComm_Begin(192, 168, 1, POWERBOARD_IP_DEVICE_ID);
  }// end if
}//end setup



//Loop
//
// Todo Reed and Connor sign off
//
/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{
  
  /////////////////////////////////////////////Software Fuse
  if(SOFTWARE_FUSES_DEBUG)
  {
    int bus_5V_software_fuse_volts = analogDebounce(BUS_5V_AMPS_PE_2);
    
    if(bus_5V_software_fuse_volts > ESTOP_5V_BUS_MAX_CURRENT_THRESHOLD) 
    {
      digitalWrite(BUS_5V_CNTRL_PP_2, HIGH);
    }//end if
    
  }//end if
   
  /////////////////////////////////////////////RED Control and Telem RoveComm
  if(RED_COMMS_DEBUG)
  {
    //If there is no message data_id gets set to zero
    roveComm_GetMsg(&data_id, &data_size, &data_value);
    
    switch (data_id) 
    {   
      //Don't do anything for data_id zero 
      case NO_ROVECOMM_MESSAGE:
        break;
      
      case BUS_5V_ON_OFF:
        digitalWrite(BUS_5V_CNTRL_PP_2, (bool)data_value);
        break;
        
      default:
        //Serial.print("Unrecognized data_id :");
        //Serial.println(data_id);
        break; 
    }//endswitch 
    
  }//end if
  
  /////////////////////////////////////////////Serial Monitor
  if(ECHO_SERIAL_MONITOR_DEBUG)
  {
    adc_reading = analogRead(BUS_5V_AMPS_PE_2);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    
    Serial.print("5V_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    
    delay(DELAY_SERIAL_MILLIS_DEBUG);
  }//end if
  
}//end loop



//Developing
///////////////////////////////////////////////Implementation
int digitalDebounce(int bouncing_pin)
{    
  // Count the bounces
  int digital_trend_count = 0;
  bool digital_reading = digitalRead(bouncing_pin);  
  
  // Read a bouncing pin and save the state
  bool last_digital_reading = digital_reading;   
  
  // Get timestamp from the system clock counter
  unsigned long system_time_micros = micros(); 
 
 // Spin for a max of millisec
  while(system_time_micros != ( micros()  + DIGITAL_DEBOUNCE_TIME_MICROS) )
  {
    digital_reading = digitalRead(bouncing_pin);
    
    if(digital_reading == last_digital_reading)
    {
      digital_trend_count++;
    }//end if
    
    if( (digital_reading != last_digital_reading) && (digital_trend_count > 0) )
    {
       digital_trend_count--; 
       last_digital_reading = digital_reading;
    }//end if
  
    if(digital_trend_count > DIGITAL_TRY_COUNT)
    {   
      
      return digital_reading;   
    }else{         
      
      last_digital_reading = digital_reading;
    }//end else
  }//end while
  
  return PIN_TOO_NOISY;
}//end functn



//Developing
///////////////////////////////////////////////Implementation
int analogDebounce(int bouncing_pin)
{    
  // Count the bounces
  int analog_trend_count = 0;
  bool analog_reading = analogRead(bouncing_pin);  
  
  // Read a bouncing pin and save the state
  bool last_analog_reading = analog_reading;   
  
  // Get timestamp from the system clock counter
  unsigned long system_time_micros = micros(); 
 
 // Spin for a max of millisec
  while(system_time_micros != ( micros()  + ANALOG_DEBOUNCE_TIME_MICROS) )
  {
    analog_reading = analogRead(bouncing_pin);
    
    if( analog_trend_count && (abs(analog_reading - last_analog_reading) < ANALOG_ACCEPTABLE_DRIFT)  )    
    {
      analog_trend_count++;
    }//end if
    
    if( analog_trend_count && (abs(analog_reading - last_analog_reading) > ANALOG_ACCEPTABLE_DRIFT)  )    
    {
       analog_trend_count--; 
       last_analog_reading = last_analog_reading;
    }//end if
  
    if(analog_trend_count > ANALOG_TRY_COUNT)
    {   
      
      return analog_reading;   
    }else{         
      
      last_analog_reading = analog_reading;
    }//end else
  }//end while
  
  return PIN_TOO_NOISY;
}//end functn




///////////////////////////////////////////////Implementation
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end fnctn

/*  TODO : debug array loop: Serial Super Debug 
    adc_reading = analogRead(BUS_12V_CNTRL_PN_3);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("1V_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M1_AMPS_PK_3);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M1_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M2_AMPS_PK_2);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M2_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M3_AMPS_PK_1);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M3_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M4_AMPS_PD_4);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M4_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M5_AMPS_PK_0);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M5_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M6_AMPS_PB_5);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M6_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M7_AMPS_PB_4);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M7_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M8_AMPS_PD_2);
    current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M8_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);*/

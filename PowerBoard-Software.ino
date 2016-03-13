/* The IC
// http://www.allegromicro.com/en/Products/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS722.aspx
*/

// Standard C integers of specific size (such as int16_t)
#include <stdint.h>

// Energia libraries used by RoveWare itself
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// RoveWare 
#include "RoveEthernet.h"
#include "RoveComm.h"

// Debugging test Flags
const uint8_t DELAY_MILLISECONDS_DEBUG = 100;
const uint8_t ECHO_SERIAL_MONITOR_DEBUG = 1;
const uint8_t SOFTWARE_FUSES_DEBUG = 0;
const uint8_t RED_COMMS_DEBUG = 0;



// Tiva1294C RoveBoard Specs
const float VCC = 3.3;
const float ADC_MIN = 0;
const float ADC_MAX = 4096;
float adc_reading = 0;

// ACS_722 IC Sensor Specs 
const float SENSOR_SCALE = 0.1; // Volts per Amp
const float SENSOR_SENSITIVITY = 0.132;
const float SENSOR_BIAS = VCC * SENSOR_SCALE;
float current_reading = 0;

// RED Display Value Bounds
const float CURRENT_MIN =  - SENSOR_BIAS / SENSOR_SENSITIVITY;
const float CURRENT_MAX = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;

// Map analog read voltage value from the ACS_722 to a human readable RED display current value
float map_floats(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end fnctn



// Sensor Volts/Amps Readings Pins
const uint8_t BUS_5V_AMPS_PE_2     = 25;
const uint8_t BUS_12V_AMPS_PD_7    = 27;
//Todo const uint8_t BATTERYPACK_VOLTS_PE3?

const uint8_t M1_AMPS_PK_3     = 68;
const uint8_t M2_AMPS_PK_2     = 67 ;
const uint8_t M3_AMPS_PK_1     = 66; 
const uint8_t M4_AMPS_PD_4     = 45;
const uint8_t M5_AMPS_PK_0     = 65;
const uint8_t M6_AMPS_PB_5     = 64;
const uint8_t M7_AMPS_PB_4     = 63;
const uint8_t M8_AMPS_PD_2     = 42;



// Control Pins
const uint8_t BUS_5V_CNTRL_PP_2  = 11;
const uint8_t BUS_12V_CNTRL_PN_3 = 12;
//Todo const uint8_t BATTERYPACK_CNTRL?

const uint8_t M1_CNTRL_PK_7    = 71;
const uint8_t M2_CNTRL_PQ_1    = 52;
const uint8_t M3_CNTRL_PK_6    = 72;
const uint8_t M4_CNTRL_PP_3    = 53;
const uint8_t M5_CNTRL_PH_1    = 73;
const uint8_t M6_CNTRL_PH_0    = 74;
const uint8_t M7_CNTRL_PA_7    = 57;
const uint8_t M8_CNTRL_PP_5    = 58;

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
  
}//end setup



// the loop routine runs over and over again forever:
void loop() 
{
  
  if(ECHO_SERIAL_MONITOR_DEBUG)
  {
    adc_reading = analogRead(BUS_5V_AMPS_PE_2);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("5V_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(BUS_12V_CNTRL_PN_3);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("1V_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M1_AMPS_PK_3);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M1_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M2_AMPS_PK_2);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M2_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M3_AMPS_PK_1);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M3_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M4_AMPS_PD_4);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M4_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M5_AMPS_PK_0);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M5_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M6_AMPS_PB_5);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M6_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M7_AMPS_PB_4);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M7_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
    adc_reading = analogRead(M8_AMPS_PD_2);
    current_reading = map_floats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
    Serial.print("M8_BUS_AMPS: "); 
    Serial.println(current_reading, DEC);
    delay(DELAY_MILLISECONDS_DEBUG);
    
  }//end if
  
  
  
  /*
  if(SOFTWARE_FUSES_DEBUG)
  {
    //debounce
  }//end if
  
  if(RED_COMMS_DEBUG)
  {
    //rovecomm
  }//end if*/

}//end loop

#include "powerboard.h"
void setup() 
{
    Serial.begin(115200);
    setPins();
    setPinStates();
    Serial.begin(115200);
    RoveComm.begin(RC_POWERBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_POWERBOARD_MAC);
}

void loop() 
{
    packet = RoveComm.read();
    data = (uint8_t*)packet.data;
    if(packet.data_id != 0)
    {
        switch(packet.data_id)
        {
            case RC_POWERBOARD_MOTORBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable Motor Busses: ");
                for(int i = 0; i < NUM_MOTORS; i++)
                {
                    if (data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(motorBusses[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(motorBusses[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_TWELVEVACTBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable Aux Bus");
                if (data[0] & 1)
                {
                    Serial.println("Enabling Aux");
                    digitalWrite(AUX_CTL, HIGH);
                }
                else
                {
                    Serial.println("Disabling Aux");
                    digitalWrite(AUX_CTL, LOW);
                }
                break;
            case RC_POWERBOARD_TWELVEVLOGICBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Busses");
                for(int i = 0; i < NUM_12V_PORTS; i++)
                {
                    if (data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(twelveVoltBusses[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(twelveVoltBusses[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_THIRTYVBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 30V Spare");
                if (data[0] & 1)
                {
                    Serial.println("Enabling Spare");
                    digitalWrite(PACK_SPARE_CTL, HIGH);
                }
                else
                {
                    Serial.println("Disabling Spare");
                    digitalWrite(PACK_SPARE_CTL, LOW);
                }
                break;
        }
    }
}

void setPins()
{
    // sets Spare to OUTPUT
    pinMode(PACK_SPARE_CTL, OUTPUT);

    // sets motor busses to OUTPUT
    for (int i = 0 ; i < NUM_MOTORS ; i++)
    {
        pinMode(motorBusses[i], OUTPUT);
    }

    // sets Aux to OUTPUT
    pinMode(AUX_CTL, OUTPUT);

    // sets 12V busses to OUTPUT
    for (int i = 0 ; i < NUM_12V_PORTS ; i++)
    {
        pinMode(twelveVoltBusses[i], OUTPUT);
    }
}

void setPinStates()
{
    // turns on 12 volt busses
    for (int i = 0 ; i < NUM_12V_PORTS ; i++)
    {
        digitalWrite(twelveVoltBusses[i], HIGH);
    }

    // turns on Aux bus
    digitalWrite(AUX_CTL, HIGH);

    // turns on motor busses after a delay
    for (int i = 0 ; i < NUM_MOTORS ; i++)
    {
        delay(MOTOR_DELAY);
        digitalWrite(motorBusses[i], HIGH);
    }
}
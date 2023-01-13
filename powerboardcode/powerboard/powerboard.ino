#include "powerboard.h"
void setup() 
{    
    Serial.begin(115200);
    setPins();
    setPinStates();
    RoveComm.begin(RC_POWERBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_POWERBOARD_MAC);
    // Telemetry.begin(telemetry, 1500000);
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
                        digitalWrite(motorPins[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(motorPins[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_HIGHBUSENABLE_DATA_ID:
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
                if (data[0] & 1<<1)
                {
                    Serial.println("Enabling High Current Spare");
                    digitalWrite(HIGH_CURRENT_SPARE_CTL, HIGH);
                }
                else
                {
                    Serial.println("Disabling High Current Spare");
                    digitalWrite(HIGH_CURRENT_SPARE_CTL, LOW);
                }
                break;
            case RC_POWERBOARD_LOWBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Busses");
                for(int i = 0; i < NUM_LOW_CURRENT; i++)
                {
                    if (data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(lowCurrentPins[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(lowCurrentPins[i], LOW);
                    }
                }
                break;
        }
    }
    // measureCurrent();
    // overCurrent();
}

void setPins()
{
    // sets low current busses to OUTPUT
    for (int i = 0 ; i < NUM_LOW_CURRENT ; i++)
    {
        pinMode(lowCurrentPins[i], OUTPUT);
    }

    // sets motor busses to OUTPUT
    for (int i = 0 ; i < NUM_MOTORS ; i++)
    {
        pinMode(motorPins[i], OUTPUT);
    }

    // sets other high current busses to OUTPUT
    pinMode(AUX_CTL, OUTPUT);
    pinMode(HIGH_CURRENT_SPARE_CTL, OUTPUT);
}

void setPinStates()
{
    // turns on low current busses
    for (int i = 0 ; i < NUM_LOW_CURRENT ; i++)
    {
        digitalWrite(lowCurrentPins[i], HIGH);
    }

    // turns on high current busses (other than motors)
    digitalWrite(AUX_CTL, HIGH);
    digitalWrite(HIGH_CURRENT_SPARE_CTL, HIGH);

    // turns on motor busses after a delay
    for (int i = 0 ; i < NUM_MOTORS ; i++)
    {
        digitalWrite(motorPins[i], HIGH);
        if (i != (NUM_MOTORS - 1))
        {
            delay(MOTOR_DELAY);
        }
    }

}

/*

float senseCurrent(const uint8_t sensePin)
{
    float meas_current = analogRead(sensePin);
    float current = map(meas_current, CURRENT_ADC_MIN, CURRENT_ADC_MAX, CURRENT_mA_MIN, CURRENT_mA_MAX);
    return current;
}



void measureCurrent()
{
    for (uint8_t i = 0; i < 7; i++)
    {
        MOTORBUSCURRENTS[i] = senseCurrent(MOTORBUSSENSEPINS[i]);

        if (MOTORBUSCURRENTS[i] > OVERCURRENT_PACK)
        {
            motorOverCurrent |= (1 << i);
            digitalWrite(motorBusses[i], LOW);
            delay(1000);
            digitalWrite(motorBusses[i], HIGH);
        }
        else
        {
            motorOverCurrent &= !(1 << i);
        }
    }
    
    AUX_CURRENT = senseCurrent(AUX_SENSE);
    if (AUX_CURRENT > OVERCURRENT_12V)
    {
        twelveActOverCurrent = 1;
        digitalWrite(AUX_CTL, LOW);
        delay(1000);
        digitalWrite(AUX_CTL, HIGH);
    }
    else
    {
        twelveActOverCurrent = 0;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        TWELVELOGICBUSCURRENTS[i] = senseCurrent(TWELVELOGICBUSPINS[i]);

        if (TWELVELOGICBUSCURRENTS[i] > OVERCURRENT_12V)
        {
            twelveLogicOverCurrent |= (1 << i);
            digitalWrite(twelveVoltBusses[i], LOW);
            delay(1000);
            digitalWrite(twelveVoltBusses[i], HIGH);
        }
        else
        {
            twelveLogicOverCurrent &= !(1 << i);
        }
    }
    return;
}

void overCurrent()
{
    if (motorOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_MOTORBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_MOTORBUSOVERCURRENT_DATA_COUNT, motorOverCurrent);
    }
    if (twelveActOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_TWELVEVACTBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_TWELVEVACTBUSOVERCURRENT_DATA_COUNT, twelveActOverCurrent);
    }
    if (twelveLogicOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_TWELVEVLOGICBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_TWELVEVLOGICBUSOVERCURRENT_DATA_COUNT, twelveLogicOverCurrent);
    }
    return;
}

void telemetry()
{
    RoveComm.write(RC_POWERBOARD_MOTORBUSCURRENT_DATA_ID, RC_POWERBOARD_MOTORBUSCURRENT_DATA_COUNT, MOTORBUSCURRENTS);
    delay(100);
    RoveComm.write(RC_POWERBOARD_TWELVEVACTBUSCURRENT_DATA_ID, RC_POWERBOARD_TWELVEVACTBUSCURRENT_DATA_COUNT, AUX_CURRENT);
    delay(100);
    RoveComm.write(RC_POWERBOARD_TWELVEVLOGICBUSCURRENT_DATA_ID, RC_POWERBOARD_TWELVEVLOGICBUSCURRENT_DATA_COUNT, TWELVELOGICBUSCURRENTS);
    delay(100);
}

*/
#include "powerboard.h"
void setup() 
{
    Serial.begin(9600);
    setPins();
    setPinStates();
    Serial.begin(9600);
    RoveComm.begin(RC_POWERBOARD_FOURTHOCTET, &TCPServer);
    delay(100);
}

void loop() 
{
    packet = RoveComm.read();
    if(packet.data_id != 0)
    {
        switch(packet.data_id)
        {
            case RC_POWERBOARD_MOTORBUSENABLE_DATA_ID:
                break;
            case RC_POWERBOARD_12VACTBUSENABLE_DATA_ID:
                break;
            case RC_POWERBOARD_12VLOGICBUSENABLE_DATA_ID:
                // Bit 0 is Drive control, Bit 1 is Auxilary logic control, Bit 2 is Multimedia logic control
                // Bit 3 is Gimbal logic control, Bit 4 is Navigation control, Bit 5 is Camera control
                // Bit 6 is Extra control
                Serial.printIn("Enable/Disable 12V Logic Busses");
                for(int i = 0; i < 7; i++)
                {
                    if (packet.data[0] & 1<<i)
                    {
                        Serial.printIn("Enabling Bus:");
                        Serial.printIn(i);
                        digitalWrite(logic12V[i], HIGH);
                    }
                    else
                    {
                        Serial.printIn("Disabling Bus:");
                        Serial.printIn(i);
                        digitalWrite(logic12V[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_30VBUSENABLE_DATA_ID:
                break;
            case RC_POWERBOARD_VACUUMENABLE_DATA_ID:
                break;
        }
    }
}

void setPins()
{
    // sets input/output of pack busses and 12V current sense
    for (int i = 0 ; i < 4 ; i++)
    {
        pinMode(currentSense12V[i], INPUT);
        pinMode(bussesPack[i], OUTPUT);
    }
    // sets input/output of pack current sense
    for (int i = 0 ; i < 10 ; i++)
    {
        pinMode(currentSensePack[i], INPUT);
    }
    // sets output of motor busses
    for (int i = 0 ; i < 5 ; i++)
    {
        pinMode(bussesMotor[i], OUTPUT);
    }
    // sets output of 12V Actuation
    for (int i = 0 ; i < 3 ; i++)
    {
        pinMode(actuation12V[i], OUTPUT);
    }
    // sets output of 12V Logic
    for (int i = 0 ; i < 7 ; i++)
    {
        pinMode(logic12V[i], OUTPUT);
    }
    pinMode(drivePack, OUTPUT);
}

void setPinStates()
{
    // turn on 12 volt busses
    for (int i = 0 ; i < 11 ; i++)
    {
        digitalWrite(busses12V[i], HIGH);
    }
    // turns off motor busses
    for (int i = 0 ; i < 5 ; i++)
    {
        digitalWrite(bussesMotor[i], LOW);
    }
    // turns on Pack busses
    for (int i = 0 ; i < 4 ; i++)
    {
        digitalWrite(bussesPack[i], HIGH);
    }
    //turns off pack drive board bus
    digitalWrite(drivePack, LOW);
    delay(MOTOR_DELAY);
    // turns on motor busses after a delay
    for (int i = 0 ; i < 5 ; i++)
    {
        digitalWrite(bussesMotor[i], HIGH);
    }
    delay(DRIVE_DELAY);
    // turns on pack drive bus
    digitalWrite(drivePack, HIGH);
}

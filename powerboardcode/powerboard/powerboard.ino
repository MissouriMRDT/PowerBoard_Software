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
                Serial.println("Enable/Disable Motor Busses");
                for(int i = 0; i < 5; i++)
                {
                    if (packet.data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(bussesMotor[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(bussesMotor[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_12VACTBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Actuation Busses");
                for(int i = 0; i < 3; i++)
                {
                    if (packet.data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(actuation12V[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(actuation12V[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_12VLOGICBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Logic Busses");
                for(int i = 0; i < 7; i++)
                {
                    if (packet.data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(logic12V[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(logic12V[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_30VBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 30V Busses");
                for(int i = 0; i < 4; i++)
                {
                    if (packet.data[0] & 1<<i)
                    {
                        Serial.println("Enabling Bus:");
                        Serial.println(i);
                        digitalWrite(bussesPack[i], HIGH);
                    }
                    else
                    {
                        Serial.println("Disabling Bus:");
                        Serial.println(i);
                        digitalWrite(bussesPack[i], LOW);
                    }
                }
                break;
            case RC_POWERBOARD_VACUUMENABLE_DATA_ID:
                Serial.println("Enable/Disable Vacuum Busses");
                if (packet.data[0] == 1)
                {
                    Serial.println("Enabling Bus:");
                    Serial.println(0);
                    digitalWrite(vacuumCtrl[0], HIGH);
                }
                else
                {
                    Serial.println("Disabling Bus:");
                    Serial.println(0);
                    digitalWrite(vacuumCtrl[0], LOW);
                }
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
    for (int i = 0 ; i < 7 ; i++)
    {
        digitalWrite(logic12V[i], HIGH);
    }
    // turn on 12 volt busses
    for (int i = 0 ; i < 3 ; i++)
    {
        digitalWrite(actuation12V[i], HIGH);
    }
    // turns on Pack busses
    for (int i = 0 ; i < 3 ; i++)
    {
        digitalWrite(bussesPackStart[i], HIGH);
    }
    //turns off pack drive board bus
    digitalWrite(drivePack, LOW);
    digitalWrite(vacuumCtrl[0], LOW);
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

void read_CurrentSense()
{
    int motor1_sense = analogRead(P_MOTOR1_SENSE);
    int motor2_sense = analogRead(P_MOTOR2_SENSE);
    int motor3_sense = analogRead(P_MOTOR3_SENSE);
    int motor4_sense = analogRead(P_MOTOR4_SENSE);
    int spare_sense = analogRead(P_SPARE_SENSE);
    int rocket_sense = analogRead(P_ROCKET_SENSE);
    int twelve_sense = analogRead(P_TWELVE_SENSE);
    int vacuum_sense = analogRead(P_VACUUM_SENSE);
    int drive_sense = analogRead(P_DRIVE_SENSE);
    int aux_sense = analogRead(P_AUX_SENSE);
     
    Serial.println("Motor 1 sense: ");
    Serial.println(motor1_sense);
    Serial.println("Motor 2 sense: ");
    Serial.println(motor2_sense);
    Serial.println("Motor 3 sense: ");
    Serial.println(motor3_sense);
    Serial.println("Motor 4 sense: ");
    Serial.println(motor4_sense);
    Serial.println("Spare sense: ");
    Serial.println(spare_sense);
    Serial.println("Rocket sense: ");
    Serial.println(rocket_sense);
    Serial.println("Twelve sense: ");
    Serial.println(twelve_sense);
    Serial.println("Vacuum sense: ");
    Serial.println(vacuum_sense);
    Serial.println("Drive sense: ");
    Serial.println(drive_sense);
    Serial.println("Aux sense: ");
    Serial.println(aux_sense);
}
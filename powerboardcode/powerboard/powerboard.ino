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
                for(int i = 0; i < 7; i++) //way i<7
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
            case RC_POWERBOARD_TWELVEVACTBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Actuation Busses");
                for(int i = 0; i < 5; i++)
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
            case RC_POWERBOARD_TWELVEVLOGICBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 12V Logic Busses");
                for(int i = 0; i < 8; i++)
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
            case RC_POWERBOARD_THIRTYVBUSENABLE_DATA_ID:
                Serial.println("Enable/Disable 30V Busses");
                for(int i = 0; i < 2; i++)
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
        }
    }
}

void setPins()
{
    // sets POE and Aux(Spare) to OUTPUT
    for (int i = 0 ; i < 2 ; i++)
    {
        //pinMode(currentSense12V[i], INPUT);
        pinMode(bussesPack[i], OUTPUT);
    }
    // sets input/output of pack current sense
    /*for (int i = 0 ; i < 10 ; i++)
    {
        pinMode(currentSensePack[i], INPUT);
    }
    */
    // sets motor busses to OUTPUT
    for (int i = 0 ; i < 7 ; i++)
    {
        pinMode(bussesMotor[i], OUTPUT);
    }
    // sets output of 12V Actuation
    for (int i = 0 ; i < 5 ; i++)
    {
        pinMode(actuation12V[i], OUTPUT);
    }
    // sets output of 12V Logic
    for (int i = 0 ; i < 8 ; i++)
    {
        pinMode(logic12V[i], OUTPUT);
    }
}

void setPinStates()
{
    // turn on 12 volt busses
    for (int i = 0 ; i < 8 ; i++)
    {
        digitalWrite(logic12V[i], HIGH);
    }
    // turn on 12 volt actuation busses
    for (int i = 0 ; i < 5 ; i++)
    {
        digitalWrite(actuation12V[i], HIGH);
    }
    // turns on Pack busses
    for (int i = 0 ; i < 2 ; i++)
    {
        digitalWrite(bussesPack[i], HIGH);
    }
    delay(MOTOR_DELAY);
    // turns on motor busses after a delay
    for (int i = 0 ; i < 7 ; i++)
    {
        digitalWrite(bussesMotor[i], HIGH);
    }
}
/*
void read_CurrentSense()
{
    int motor1_sense = analogRead(P_MOTOR1_SENSE);
    int motor2_sense = analogRead(P_MOTOR2_SENSE);
    int motor3_sense = analogRead(P_MOTOR3_SENSE);
    int motor4_sense = analogRead(P_MOTOR4_SENSE);
    int motor5_sense = analogRead(P_MOTOR5_SENSE);
    int motor6_sense = analogRead(P_MOTOR6_SENSE);
    int motor7_sense = analogRead(P_MOTOR7_SENSE);
    int poe_sense = analogRead(P_POE_SENSE);
    int aux_sense = analogRead(P_AUX_SENSE);
    int multimedia_sense = analogRead(MULTIMEDIA_SENSE);
    int nav_sense = analogRead(NAV_SENSE);
    int gimbal_sense = analogRead(GIMBAL_ACT_SENSE);
    int drive_sense = analogRead(DRIVE_SENSE);
    int scisensor_sense = analogRead(SCISENSOR_ACT_SENSE);
    int netswitch_sense = analogRead(NETSWITCH_SENSE);
    int cam1_sense = analogRead(CAM1_SENSE);
    int cam2_sense = analogRead(CAM2_SENSE);
    int bbb_sense = analogRead(BBB_SENSE);
    int aux_log_sense = analogRead(AUX_LOG_SENSE);

    Serial.println("Motor 1 sense: ");
    Serial.println(motor1_sense);
    Serial.println("Motor 2 sense: ");
    Serial.println(motor2_sense);
    Serial.println("Motor 3 sense: ");
    Serial.println(motor3_sense);
    Serial.println("Motor 4 sense: ");
    Serial.println(motor4_sense);
    Serial.println("Motor 5 sense: ");
    Serial.println(motor5_sense);
    Serial.println("Motor 6 sense: ");
    Serial.println(motor6_sense);
    Serial.println("Motor 7 sense: ");
    Serial.println(motor7_sense);
    Serial.println("POE sense: ");
    Serial.println(poe_sense);
    Serial.println("Aux sense: ");
    Serial.println(aux_sense);
    Serial.println("Multimedia sense: ");
    Serial.println(multimedia_sense);
    Serial.println("Nav sense: ");
    Serial.println(nav_sense);
    Serial.println("Gimbal sense: ");
    Serial.println(gimbal_sense);
    Serial.println("Drive sense: ");
    Serial.println(drive_sense);    
    Serial.println("SciSensor sense: ");
    Serial.println(scisensor_sense);    
    Serial.println("Cam 1 sense: ");
    Serial.println(cam1_sense);    
    Serial.println("Cam 2 sense: ");
    Serial.println(cam2_sense);    
    Serial.println("BBB sense: ");
    Serial.println(bbb_sense);    
    Serial.println("Aux Log sense: ");
    Serial.println(Aux_Log_sense);
}
*/
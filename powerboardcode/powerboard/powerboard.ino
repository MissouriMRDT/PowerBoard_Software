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
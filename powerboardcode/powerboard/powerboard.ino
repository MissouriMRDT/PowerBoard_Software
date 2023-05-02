#include "powerboard.h"
void setup() 
{    
    Serial.begin(115200);
    setSyncProvider(Teensy3Clock.get());
    busSetup();
    setPins();
    setPinStates();
    RoveComm.begin(RC_POWERBOARD_FIRSTOCTET, RC_POWERBOARD_SECONDOCTET, RC_POWERBOARD_THIRDOCTET, RC_POWERBOARD_FOURTHOCTET, &TCPServer);
    subscribeAll();
    Telemetry.begin(telemetry, 1500000);
    SD.begin();
    File blackBox = SD.open("Black Box", FILE_WRITE);
    blackBox.println();
}

void loop() 
{
    packet = RoveComm.read();
    data = (uint8_t*)packet.data;
    switch(packet.data_id)
    {
        case RC_POWERBOARD_MOTORBUSENABLE_DATA_ID:
            Serial.println("Enable/Disable Motor Bus: ");
            for(int i = 0; i < NUM_MOTORS; i++)
            {
                if (data[0] & 1<<i)
                {
                    Serial.println("Enabling Bus:");
                    Serial.println(i);
                    digitalWrite(Port[i].ctl_pin, HIGH);
                }
                else
                {
                    Serial.println("Disabling Bus:");
                    Serial.println(i);
                    digitalWrite(Port[i].ctl_pin, LOW);
                }
            }
            break;
        case RC_POWERBOARD_HIGHBUSENABLE_DATA_ID:
            Serial.println("Enable/Disable High Current Bus");
			for(int i = 0; i < NUM_HIGH_CURRENT; i++)
			{
				if (data[0] & 1<<i)
				{
					Serial.println("Enabling Bus:");
					Serial.println(i);
					digitalWrite(Port[i + NUM_MOTORS].ctl_pin, HIGH);
				}
				else
				{
					Serial.println("Disabling Bus:");
					Serial.println(i);
					digitalWrite(Port[i + NUM_MOTORS].ctl_pin, LOW);
				}
			}
            break;
        case RC_POWERBOARD_LOWBUSENABLE_DATA_ID:
            Serial.println("Enable/Disable Low Current Bus");
            for(int i = 0; i < NUM_LOW_CURRENT; i++)
            {
                if (data[0] & 1<<i)
                {
                    Serial.println("Enabling Bus:");
                    Serial.println(i);
                    digitalWrite(Port[i + NUM_MOTORS + NUM_HIGH_CURRENT].ctl_pin, HIGH);
                }
                else
                {
                    Serial.println("Disabling Bus:");
                    Serial.println(i);
                    digitalWrite(Port[i + NUM_MOTORS + NUM_HIGH_CURRENT].ctl_pin, LOW);
                }
            }
            break;
    }
    measureCurrent();
    dataPack();
    overCurrent();
}

void Bus::set_Values(const uint8_t & Ctl_pin, const uint16_t & I_overcurrent, const uint16_t & I_max, const uint8_t Imeas_pin, float Imeas_val = 0.0, bool Toggle_status = false, bool Overcurrent = false)
{
    ctl_pin = Ctl_pin;
    i_overcurrent = I_overcurrent;
    i_max = I_max;
    imeas_pin = Imeas_pin;
    imeas_val = Imeas_val;
    toggle_status = Toggle_status;
    overcurrent = Overcurrent;
}

void busSetup()
{
    Port[0].set_Values(MOTOR_1_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_1_CS);                          //motor 1
    Port[1].set_Values(MOTOR_2_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_2_CS);                          //motor 2
    Port[2].set_Values(MOTOR_3_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_3_CS);                          //motor 3
    Port[3].set_Values(MOTOR_4_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_4_CS);                          //motor 4
    Port[4].set_Values(MOTOR_5_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_5_CS);                          //motor 5
    Port[5].set_Values(MOTOR_6_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_6_CS);                          //motor 6
    Port[6].set_Values(MOTOR_SPARE_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, MOTOR_SPARE_CS);                  //motor spare
    Port[7].set_Values(AUX_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, AUX_CS);                                  //aux
    Port[8].set_Values(HIGH_CURRENT_SPARE_CTL, OVERCURRENT_HIGH, HIGH_CURRENT_mA_MAX, HIGH_CURRENT_SPARE_CS);    //spare 20A
    Port[9].set_Values(GIMBAL_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, GIMBAL_CS);                              //gimbal
    Port[10].set_Values(DRIVE_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, DRIVE_CS);                               //drive
    Port[11].set_Values(MULTIMEDIA_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, MULTIMEDIA_CS);                     //multimedia
    Port[12].set_Values(NAV_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, NAV_CS);                                   //nav
    Port[13].set_Values(LOW_CURRENT_SPARE_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, LOW_CURRENT_SPARE_CS);       //spare 5A
    Port[14].set_Values(CAM_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, CAM_CS);                                   //cam
    Port[15].set_Values(LOW_VOLTAGE_SPARE_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, LOW_VOLTAGE_SPARE_CS);       //spare 12V
    Port[16].set_Values(NET_SWITCH_1_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, DEFAULT_CS);                      //net switch 1
    Port[17].set_Values(NET_SWITCH_2_CTL, OVERCURRENT_LOW, LOW_CURRENT_mA_MAX, DEFAULT_CS);                      //net switch 2
}

void setPins()
{
    for (int i = 0; i < NUM_BUS; i++)
    {
        pinMode(Port[i].ctl_pin, OUTPUT);
    }
}

void setPinStates()
{
    // Turns on ports except motors
    for (int i = NUM_MOTORS; i < NUM_BUS; i++)
    {
        digitalWrite(Port[i].ctl_pin, HIGH);
    }

    // Turns on motor ports with a delay
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        digitalWrite(Port[i].ctl_pin, HIGH);
        delay(MOTOR_DELAY);
    }
}

void subscribeAll()
{
    for (int i = 0; i < NUM_BOARDS; i++)
    {
        RoveComm.writeTo(RC_ROVECOMM_SUBSCRIBE_DATA_ID, 0, NULL, ips[i][0], ips[i][1], ips[i][2], ips[i][3], RC_ROVECOMM_ETHERNET_UDP_PORT);
    }
}

void measureCurrent()
{
    for (uint8_t i = 0; i < NUM_BUS - 2; i++)
    {
        uint16_t meas_current = analogRead(Port[i].imeas_pin);
        float current = map(meas_current, 0, 1023, 0, Port[i].i_max);
        Port[i].imeas_val = current;
        if (Port[i].imeas_val > Port[i].i_overcurrent)
        {
            Port[i].overcurrent = true;
            digitalWrite(Port[i].ctl_pin, LOW);
            delay(1000);
            digitalWrite(Port[i].ctl_pin, HIGH);
        }
        else
        {
            Port[i].overcurrent = false;
        }
    }
}

void dataPack()
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motorCurrents[i] = Port[i].imeas_val;
        if (Port[i].overcurrent)
        {
            motorOverCurrent |= (1 << i);
        }
        else
        {
            motorOverCurrent &= ~(1 << i);
        }
    }
    for (uint8_t i = 0; i < NUM_HIGH_CURRENT; i++)
    {
        highCurrents[i] = Port[i + NUM_MOTORS].imeas_val;
        if (Port[i].overcurrent)
        {
            highOverCurrent |= (1 << i);
        }
        else
        {
            highOverCurrent &= ~(1 << i);
        }
    }
    for (uint8_t i = 0; i < NUM_LOW_CURRENT; i++)
    {
        lowCurrents[i] = Port[i + NUM_MOTORS + NUM_HIGH_CURRENT].imeas_val;
        if (Port[i].overcurrent)
        {
            lowOverCurrent |= (1 << i);
        }
        else
        {
            lowOverCurrent &= ~(1 << i);
        }
    }
}

void printTime()
{
    time_t t = now();
    tmElements_t tm;
    breakTime(t, tm);
    char buffer[14];
    sprintf(buffer, "%02d:%02d:%02d %02d/%02d", tm.Hour, tm.Minute, tm.Second, tm.Month, tm.Day);
    blackBox.println(buffer);
}

void overCurrent()
{
    if (motorOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_MOTORBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_MOTORBUSOVERCURRENT_DATA_COUNT, motorOverCurrent);
    }
    if (highOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_HIGHBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_HIGHBUSOVERCURRENT_DATA_COUNT, highOverCurrent);
    }
    if (lowOverCurrent)
    {
        RoveComm.write(RC_POWERBOARD_LOWBUSOVERCURRENT_DATA_ID, RC_POWERBOARD_LOWBUSOVERCURRENT_DATA_COUNT, lowOverCurrent);
    }
}

void telemetry()
{
    RoveComm.write(RC_POWERBOARD_MOTORBUSCURRENT_DATA_ID, RC_POWERBOARD_MOTORBUSCURRENT_DATA_COUNT, motorCurrents);
    delay(100);
    RoveComm.write(RC_POWERBOARD_HIGHBUSCURRENT_DATA_ID, RC_POWERBOARD_HIGHBUSCURRENT_DATA_COUNT, highCurrents);
    delay(100);
    RoveComm.write(RC_POWERBOARD_LOWBUSCURRENT_DATA_ID, RC_POWERBOARD_LOWBUSCURRENT_DATA_COUNT, lowCurrents);
    delay(100);

    printTime();
    blackBox.print("Motor Currents: ");
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        blackBox.print(motorCurrents[i]);
        blackBox.print(" ");
    }
    blackBox.println();
    blackBox.print("High Currents: ");
    for (uint8_t i = 0; i < NUM_HIGH_CURRENT; i++)
    {
        blackBox.print(highCurrents[i]);
        blackBox.print(" ");
    }
    blackBox.println();
    blackBox.print("Low Currents: ");
    for (uint8_t i = 0; i < NUM_LOW_CURRENT; i++)
    {
        blackBox.print(lowCurrents[i]);
        blackBox.print(" ");
    }
    blackBox.println();
    blackBox.println();
    blackBox.flush();
}
#include "powerboard.h"
void setup() 
{    
    Serial.begin(115200);
    setPins();
    setPinStates();
    RoveComm.begin(RC_POWERBOARD_FIRSTOCTET, RC_POWERBOARD_SECONDOCTET, RC_POWERBOARD_THIRDOCTET, RC_POWERBOARD_FOURTHOCTET, &TCPServer);
    //Telemetry.begin(telemetry, 1500000);
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
                Serial.println("Enable/Disable Motor Bus: ");
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
                Serial.println("Enable/Disable High Current Bus");
				for(int i = 0; i < NUM_HIGH_CURRENT; i++)
				{
					if (data[0] & 1<<i)
					{
						Serial.println("Enabling Bus:");
						Serial.println(i);
						digitalWrite(highCurrentPins[i], HIGH);
					}
					else
					{
						Serial.println("Disabling Bus:");
						Serial.println(i);
						digitalWrite(highCurrentPins[i], LOW);
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
    measureCurrent();
    overCurrent();
    telemetry();
}

void setPins()
{
    // sets motor busses to OUTPUT
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pinMode(motorPins[i], OUTPUT);
    }

    // sets other high current busses to OUTPUT
	for (int i = 0; i < NUM_HIGH_CURRENT; i++)
	{
		pinMode(highCurrentPins[i], OUTPUT);
	}

    // sets low current busses to OUTPUT
    for (int i = 0; i < NUM_LOW_CURRENT; i++)
    {
        pinMode(lowCurrentPins[i], OUTPUT);
    }
}

void setPinStates()
{
    // turns on low current busses
    for (int i = 0; i < NUM_LOW_CURRENT; i++)
    {
        digitalWrite(lowCurrentPins[i], HIGH);
    }

    // turns on high current busses (other than motors)
	for (int i = 0; i < NUM_HIGH_CURRENT; i++)
	{
		digitalWrite(highCurrentPins[i], HIGH);
	}

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
void Bus::set_Values(const uint8_t & I_max, uint8_t &Imeas_pin, uint8_t & Imeas_val)
{
    i_max = I_max;
    imeas_pin = Imeas_pin;
    imeas_val = Imeas_val;
}

void toggle_Bus::set_Values(uint8_t & Ctl_pin, const uint8_t & I_max, uint8_t & Imeas_pin, uint8_t & Imeas_val, bool & Toggle_status)
{
    ctl_pin = Ctl_pin;
    i_max = I_max;
    imeas_pin = Imeas_pin;
    imeas_val = Imeas_val;
    toggle_status = Toggle_status;
}

void bus_Setup(Bus Bus[])
{
    Bus[0].set_Values(MOTOR_1_CTL);                 //motor 1
    Bus[1].set_Values(MOTOR_2_CTL);                 //motor 2
    Bus[2].set_Values(MOTOR_3_CTL);                 //motor 3
    Bus[3].set_Values(MOTOR_4_CTL);                 //motor 4
    Bus[4].set_Values(MOTOR_5_CTL);                 //motor 5
    Bus[5].set_Values(MOTOR_6_CTL);                 //motor 6
    Bus[6].set_Values(MOTOR_SPARE_CTL);             //motor spare
    Bus[7].set_Values(AUX_CTL);                     //aux
    Bus[8].set_Values(HIGH_CURRENT_SPARE_CTL);      //spare 20A
    Bus[9].set_Values(GIMBAL_CTL);                  //gimbal
    Bus[10].set_Values(DRIVE_CTL);                  //drive
    Bus[11].set_Values(MULTIMEDIA_CTL);             //multimedia
    Bus[12].set_Values(NAV_CTL);                    //nav
    Bus[13].set_Values(CAM_CTL);                    //cam
    Bus[14].set_Values(BBB_CTL);                    //black box
    Bus[15].set_Values(LOW_CURRENT_SPARE_CTL);      //spare 1A
    Bus[16].set_Values();                           //POE
    Bus[17].set_Values();                           //network switch
}
*/

float senseCurrent(const uint8_t sensePin, const bool highPort)
{
    uint16_t meas_current = analogRead(sensePin);
    float current;
	if(highPort)
	{
    	float current = map(meas_current, 0, 1023, 0, HIGH_CURRENT_mA_MAX);
	}
	else
	{
    	float current = map(meas_current, 0, 1023, 0, LOW_CURRENT_mA_MAX);
	}
    if (sensePin == MOTOR_6_CS)
    {
        Serial.println();
        Serial.println("Motor 6:");
        Serial.println(meas_current);
        Serial.println(current);
        Serial.println();
    }
    return current;
}

void measureCurrent()
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motorSenseCurrents[i] = senseCurrent(motorSensePins[i], true);
        Serial.println(i);
        Serial.println(motorSenseCurrents[i]);
        if (motorSenseCurrents[i] > OVERCURRENT_HIGH)
        {
            motorOverCurrent |= (1 << i);
            digitalWrite(motorPins[i], LOW);
            delay(1000);
            digitalWrite(motorPins[i], HIGH);
        }
        else
        {
            motorOverCurrent &= ~(1 << i);
        }
    }
    
	for (uint8_t i = 0; i < NUM_HIGH_CURRENT; i++)
	{
		highCurrentSenseCurrents[i] = senseCurrent(highCurrentSensePins[i], true);

		if (highCurrentSenseCurrents[i] > OVERCURRENT_HIGH)
		{
			highOverCurrent |= (1 << i);
			digitalWrite(highCurrentSensePins[i], LOW);
			delay(1000);
			digitalWrite(highCurrentSensePins[i], HIGH);
		}
		else
		{
			highOverCurrent &= ~(1 << i);
		}
	}

    for (uint8_t i = 0; i < NUM_LOW_CURRENT; i++)
    {
        lowCurrentSenseCurrents[i] = senseCurrent(lowCurrentSensePins[i], false);

        if (lowCurrentSenseCurrents[i] > OVERCURRENT_LOW)
        {
            lowOverCurrent |= (1 << i);
            digitalWrite(lowCurrentSensePins[i], LOW);
            delay(1000);
            digitalWrite(lowCurrentSensePins[i], HIGH);
        }
        else
        {
            lowOverCurrent &= ~(1 << i);
        }
    }

	for (uint8_t i = 0; i < 2; i++)
	{
		networkCurrentSenseCurrents[i] = senseCurrent(networkCurrentSensePins[i], false);

		if (networkCurrentSenseCurrents[i] > OVERCURRENT_LOW)
		{
			networkOverCurrent |= (1 << i);
		}
		else
		{
			networkOverCurrent &= ~(1 << i);
		}
	}
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
    RoveComm.write(RC_POWERBOARD_MOTORBUSCURRENT_DATA_ID, RC_POWERBOARD_MOTORBUSCURRENT_DATA_COUNT, motorSenseCurrents);
    delay(100);
    RoveComm.write(RC_POWERBOARD_HIGHBUSCURRENT_DATA_ID, RC_POWERBOARD_HIGHBUSCURRENT_DATA_COUNT, highCurrentSenseCurrents);
    delay(100);
    RoveComm.write(RC_POWERBOARD_LOWBUSCURRENT_DATA_ID, RC_POWERBOARD_LOWBUSCURRENT_DATA_COUNT, lowCurrentSenseCurrents);
    delay(100);
}
#include "powerboard.h"
void setup() {

  Serial.begin(9600);
  setPins();
  setPinStates();
}

void loop() 
{
  // put your main code here, to run repeatedly: 
  
}

void setPins()
{
  
  for (int i = 0 ; i < 4 ; i++)
  {
    pinMode(currentSense[i], OUTPUT);
  }

  for (int i = 0 ; i < 10 ; i++)
  {
    pinMode(busses[i], INPUT);
    pinMode(P_CurrentSense[i], OUTPUT);
  }

  for (int i = 0 ; i < 9 ; i++)
  {
    pinMode(P_busses[i], OUTPUT);
  }
}

void setPinStates()
{
  // ONLY THESE FOR NOW BECAUSE THEY ARE THE ONLY FUNCTIONING ONES :)
  for (int i = 3 ; i < 9 ; i++)
  {
    digitalWrite(busses[i], LOW);
  }
}

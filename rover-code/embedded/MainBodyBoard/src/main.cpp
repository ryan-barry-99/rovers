#include <Arduino.h>
#include "../include/MainBodyBoard.h"

MainBodyBoard mbb;

void setup() 
{
  mbb = MainBodyBoard();
  Serial.begin(9600);
}

void loop() 
{
    mbb.updateSubsystems();
    delay(1);
}


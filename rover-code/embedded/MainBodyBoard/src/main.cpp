#include <Arduino.h>
#include "../include/MainBodyBoard.h"

MainBodyBoard mbb;

void setup() 
{
  mbb = MainBodyBoard();
}

void loop() 
{
    mbb.updateSubsystems();
    delay(1);
}


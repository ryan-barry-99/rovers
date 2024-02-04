#include "../../SoilTransferer.h"

SoilTransferer::SoilTransferer( CAN *can)
{
    // Constructor
    this->can = can;         
    currPos = 0;
    targetPos = 0;
    for (int i = 0; i < 6; i++)
    {
        vibrators[i] = new Vibrator((Vibrator::VibratorID)i);
    }
}
int SoilTransferer::getPos() {return caroMotor.getPosition();}

int SoilTransferer::getTargetPos() {return targetPos;}

bool[] SoilTransferer::activeVibrators() 
{
    // Returns an array of vibrators that are currently active
    bool activeVibrators[6] = {0,0,0,0,0,0};
    for(int i = 0; i < 6; i++)
    {
        if(vibrators[i].isActive())
        {
            activeVibrators[i] = 1;
        }
    }
    return activeVibrators;
}
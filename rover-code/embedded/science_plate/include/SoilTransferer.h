#ifndef SoilTransferer_h
#define SoilTransferer_h

#include "SoilTransferer.h"
#include "StepperMotor.h"
#include "Vibrator.h"
#include "CAN.h"
#include "EnumsList.h"

/*
Attributes
- positions: PosSoilTransfer
- currPos: float
- targetPos: float
- caroMotor: StepperMotor
- vibrators: Vibrator[6]
- can: *CAN
*/

class SoilTransferer {
    private:
        enum PosSoilTransfer 
        {
            POS1 = 0,
            POS2 = 1,
            POS3 = 2,
            POS4 = 3,
            POS5 = 4,
            POS6 = 5
        };
        StepperMotor caroMotor = new Motor(StepperPins::SOILTRANSFER, null);
        CAN::CAN *can;

        float currPos;
        float targetPos;
        Vibrator vibrators[6];
        
    public:
        SoilTransferer(CAN::CAN *can);

        void transferSoil();

        int getPos();
        int getTargetPos();
        bool[] activeVibrators(); 
};

#endif
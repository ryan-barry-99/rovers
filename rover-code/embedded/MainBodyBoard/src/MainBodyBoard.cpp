#include "../include/MainBodyBoard.h"

MainBodyBoard::MainBodyBoard()
{
    #ifndef DEBUG_STATUS_LIGHT
    pinMode(STATUS_LIGHT_PIN, OUTPUT);
    #endif
}

MainBodyBoard::~MainBodyBoard()
{
    
}

void MainBodyBoard::updateSubsystems(void)
{
    #ifndef DEBUG_STATUS_LIGHT
    if(statusLightWait == 0)
    {
        if(statusLightOn)
        {
            digitalWrite(STATUS_LIGHT_PIN, LOW);
            statusLightOn = false;
        }
        else
        {
            digitalWrite(STATUS_LIGHT_PIN, HIGH);
            statusLightOn = true;
        }
        statusLightWait = 1000;
    }
    else
    {
        statusLightWait--;
    }
    #endif

    #ifndef DEBUG_CAN || DEBUG_STATUS_LIGHT
    if(can.isEStop())
    {
        statusLightWait = -1;
        digitalWrite(STATUS_LIGHT_PIN, HIGH);
    }
    else if(statusLightWait < 0)
    {
        statusLightWait = 0;
    }
    #endif

    #ifndef DEBUG_CAN || DEBUG_DRIVEBASE
    drive_base.updateVelocity();
    #endif

    #ifndef DEBUG_CAN || DEBUG_TEMP
    temp_subsystem.updateFans();
    #endif
}


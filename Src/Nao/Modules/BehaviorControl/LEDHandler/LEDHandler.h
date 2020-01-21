#pragma once

#include "Tools/Module/Blackboard.h"

class LEDHandlerBase
{
public:
    /** REQUIRES */
    const SystemSensorData &theSystemSensorData = Blackboard::getInstance().systemSensorData;

    int chargingLightSlowness = 5;
};

class LEDHandler : public LEDHandlerBase
{
public:
    enum EyeColor
    {
        red,
        green,
        blue,
        white,
        magenta,
        yellow,
        cyan,
        NumOfEyeColor
    };

    void update();
    void update(LEDRequest &ledRequest);
    void setHead(LEDRequest &ledRequest);

    size_t chargingLED = 0;
    const LEDRequest::LED headLEDCircle[LEDRequest::numOfHeadLEDs] =
    {
        LEDRequest::headLedRearLeft2,
        LEDRequest::headLedRearLeft1,
        LEDRequest::headLedRearLeft0,
        LEDRequest::headLedMiddleLeft0,
        LEDRequest::headLedFrontLeft0,
        LEDRequest::headLedFrontLeft1,
        LEDRequest::headLedFrontRight1,
        LEDRequest::headLedFrontRight0,
        LEDRequest::headLedMiddleRight0,
        LEDRequest::headLedRearRight0,
        LEDRequest::headLedRearRight1,
        LEDRequest::headLedRearRight2
    };
};
#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Module/Blackboard.h"

class FootSupportProviderBase
{
public:
    /* REQUIRES */
    const FsrSensorData &theFsrSensorData = Blackboard::getInstance().fsrSensorData;
    
    float minPressure = 0.1f; /**< Minimum pressure assumed. */
    float maxPressure = 5.0f; /**< Maximum pressure assumed. */
    float outerWeight = 0.8f; /**< Weights for outer FSRs. */
    float innerWeight = 0.3f; /**< Weights for inner FSRs. */
};

class FootSupportProvider : public FootSupportProviderBase
{
public:
    float weights[Legs::NumOfLeg][FsrSensors::NumOfFsrSensor];          /**< Weights for the individual FSRs. */
    float highestPressure[Legs::NumOfLeg][FsrSensors::NumOfFsrSensor]; /**< Highest pressure measured so far per FSR. */

    void update();
    void update(FootSupport &theFootSupport);
    FootSupportProvider();
};
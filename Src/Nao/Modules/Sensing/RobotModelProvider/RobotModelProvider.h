#pragma once

#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Blackboard.h"

class RobotModelProviderBase
{
public:
    /* REQUIRES */
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const MassCalibration &theMassCalibration = Blackboard::getInstance().massCalibration;
    const RobotDimensions &theRobotDimensions = Blackboard::getInstance().robotDimensions;

    /* PROVIDERS */
    // RobotModel &_theRobotModel = Blackboard::getInstance().robotModel;
};

class RobotModelProvider : public RobotModelProviderBase
{
public:
    void update();
    void update(RobotModel &robotModel);
};
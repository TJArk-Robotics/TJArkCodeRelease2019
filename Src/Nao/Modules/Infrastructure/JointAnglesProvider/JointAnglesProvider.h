#pragma once 

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Module/Blackboard.h"

class JointAnglesProviderBase
{
public:
    // JoinSensorData
    const JointSensorData &theJointSensorData = Blackboard::getInstance().jointSensorData;
};

class JointAnglesProvider : public JointAnglesProviderBase
{
public:
    void update();
    void update(JointAngles &jointAngles);
};
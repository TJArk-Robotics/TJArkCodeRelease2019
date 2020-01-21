#include "JointAnglesProvider.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"

void JointAnglesProvider::update()
{
    // JointSensorData
    if (!Blackboard::getInstance().updatedMap["JointSensorData"])
    {
        JointSensorData &_theJointSensorData = Blackboard::getInstance().jointSensorData;
        NaoProvider::theInstance->update(_theJointSensorData);
        Blackboard::getInstance().updatedMap["JointSensorData"] = true;
    }
}

void JointAnglesProvider::update(JointAngles &jointAngles)
{
    update();
    jointAngles = theJointSensorData;
}
#include "RobotModelProvider.h"

// Provider
#include "Tools/Module/ModuleManager.h"

void RobotModelProvider::update()
{
    // const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        // JointAnglesProvider jointAnglesProvider;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // const MassCalibration &theMassCalibration = Blackboard::getInstance().massCalibration;
    // const RobotDimensions &theRobotDimensions = Blackboard::getInstance().robotDimensions;
}

void RobotModelProvider::update(RobotModel &robotModel)
{
    update();
    robotModel.setJointData(theJointAngles, theRobotDimensions, theMassCalibration);
}
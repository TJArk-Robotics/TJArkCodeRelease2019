#include "SelfLocator.h"
#include "Tools/Module/ModuleManager.h"

void SelfLocator::update()
{
    // ArucoMarker
    if (!Blackboard::getInstance().updatedMap["ArucoMarker"])
    {
        ArucoMarker &_theArucoMarker = Blackboard::getInstance().arucoMarker;
        ModuleManager::theInstance->markerDetector.update(_theArucoMarker);
        Blackboard::getInstance().updatedMap["ArucoMarker"] = true;
    }
}

void SelfLocator::update(RobotPose &robotPose)
{
    update();
}
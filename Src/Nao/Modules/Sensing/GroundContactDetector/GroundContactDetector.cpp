#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"
// Provider
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"

void GroundContactDetector::update()
{
    // FrameInfo
    {
        FrameInfo &_theFrameinfo = Blackboard::getInstance().frameInfo;
        NaoProvider::theInstance->update(_theFrameinfo);
    }

    // FsrSensorData
    if (!Blackboard::getInstance().updatedMap["FsrSensorData"])
    {
        FsrSensorData &_theFsrSensorData = Blackboard::getInstance().fsrSensorData;
        NaoProvider::theInstance->update(_theFsrSensorData);
        Blackboard::getInstance().updatedMap["FsrSensorData"] = true;
    }

    // InertialSensorData
    if (!Blackboard::getInstance().updatedMap["InertialSensorData"])
    {
        InertialSensorData &_theInertialSensorData = Blackboard::getInstance().inertialSensorData;
        NaoProvider::theInstance->update(_theInertialSensorData);
        Blackboard::getInstance().updatedMap["InertialSensorData"] = true;
    }
}

void GroundContactDetector::update(GroundContactState &groundContactState)
{
    update();

    if (groundContactState.contact)
    {
        if (theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] >= minPressureToKeepContact)
            lastTimeWithPressure = theFrameInfo.time;
        groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
        if (!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
            SystemCall::playSound("high.wav");
    }
    else
    {
        if (std::abs(theInertialSensorData.gyro.y()) > maxGyroYToRegainContact || theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] < minPressureToRegainContact || theFsrSensorData.totals[Legs::left] < minPressurePerFootToRegainContact || theFsrSensorData.totals[Legs::right] < minPressurePerFootToRegainContact)
            lastTimeWithoutPressure = theFrameInfo.time;
        groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
        if (groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
            SystemCall::playSound("ground.wav");
    }
}
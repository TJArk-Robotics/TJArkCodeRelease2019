/*
 *  @file:NaoProvider.cpp
 *  TODO: sensor data in V6 is different to V5, now still use V5 data format.
 *        see JointSensorData in BhumanCodeRelease2018 and ours.
 */

#include "NaoProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Tools/Settings.h"
#include "Tools/Global.h"

#include "libagent/bhuman.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

thread_local NaoProvider *NaoProvider::theInstance = nullptr;

NaoProvider::NaoProvider()
{
    NaoProvider::theInstance = this;
    memset(&gameControlData, 0, sizeof(gameControlData));

    for (int i = 0; i < Joints::NumOfJoint; i++)
    {
        clippedLastFrame[i] = SensorData::off;
    }
}

NaoProvider::~NaoProvider()
{
    NaoProvider::theInstance = nullptr;
}

void NaoProvider::finishFrame()
{
    if (theInstance)
    {
        theInstance->send();
    }
}

void NaoProvider::waitForFrameData()
{
    if (theInstance)
    {
        theInstance->naoBody.wait();
    }
}

void NaoProvider::send()
{
    float *actuators;
    naoBody.openActuators(actuators);
    int j = 0;
    ASSERT(headYawPositionActuator == 0);
    ASSERT(static_cast<int>(Joints::NumOfJoint) - 1 == lbhNumOfPositionActuatorIds); //rHipYawPitch missin lbh
    for (int i = 0; i < Joints::NumOfJoint; ++i, ++j)
    {
        if (i == Joints::rHipYawPitch) // missing on Nao
        {
            ++i;
        }
        if (theJointRequest.angles[i] == SensorData::off)
        {
            actuators[j] = 0.f;
            actuators[j + lbhNumOfPositionActuatorIds] = 0.f; // stiffness
        }
        else
        {
            actuators[j] = theJointRequest.angles[i] + theJointCalibration.offsets[i];
            actuators[j + lbhNumOfPositionActuatorIds] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[i]) / 100.f;
        }
    }
    j += lbhNumOfPositionActuatorIds;
    ASSERT(j == faceLedRedLeft0DegActuator);

    // actuators[lbhNumOfPositionActuatorIds] = 0.7f;

    const LEDRequest &ledRequest(theLEDRequest);
    bool on = (theFrameInfo.time / 50 & 8) != 0;
    bool fastOn = (theFrameInfo.time / 10 & 8) != 0;
    for (int i = 0; i < LEDRequest::NumOfLED; ++i)
    {
        actuators[j++] = (ledRequest.ledStates[i] == LEDRequest::on || (ledRequest.ledStates[i] == LEDRequest::blinking && on) || (ledRequest.ledStates[i] == LEDRequest::fastBlinking && fastOn)) ? 1.0f : (ledRequest.ledStates[i] == LEDRequest::half ? 0.5f : 0.0f);
    }

    naoBody.closeActuators();
    // naoBody.setTeamInfo(theInstance->global.theSettings.teamNumber, theInstance->global.theSettings.teamColor, theInstance->global.theSettings.playerNumber);
}

void NaoProvider::update()
{
    // JointCalibration
    // JointLimits
    // LEDRequest update in motion
    // FrameInfo
    {
        FrameInfo &_theFrameInfo = Blackboard::getInstance().frameInfo;
        update(_theFrameInfo);
    }
}

void NaoProvider::update(FrameInfo &frameInfo)
{
    frameInfo.time = std::max(frameInfo.time + 1, Time::getCurrentSystemTime());
    gameControlData = naoBody.getGameControlData();
}

void NaoProvider::update(FsrSensorData &fsrSensorData)
{
    float *sensors = naoBody.getSensors();
    for (int leg = 0; leg < Legs::NumOfLeg; leg++)
    {
        for (int sensor = 0; sensor < FsrSensors::NumOfFsrSensor; sensor++)
        {
            fsrSensorData.pressures[leg][sensor] = sensors[lFSRFrontLeftSensor + leg * FsrSensors::NumOfFsrSensor + sensor];
        }
        fsrSensorData.totals[leg] = sensors[lFSRTotalSensor + leg];
    }
}

void NaoProvider::update(InertialSensorData &inertialSensorData)
{
    float *sensors = naoBody.getSensors();

    // TODO:
    inertialSensorData.gyro.x() = sensors[gyroXSensor];
    inertialSensorData.gyro.y() = sensors[gyroYSensor];
    inertialSensorData.gyro.z() = sensors[gyroZSensor]; // Aldebarans z-gyron is negated in V5, but in V6 it is positive.

    inertialSensorData.acc.x() = -sensors[accXSensor];
    inertialSensorData.acc.y() = sensors[accYSensor];
    inertialSensorData.acc.z() = -sensors[accZSensor];

    inertialSensorData.angle.x() = sensors[angleXSensor];
    inertialSensorData.angle.y() = sensors[angleYSensor];
    // inertialSensorData.angle.z() = -sensors[angleZSensor];  // TODO:
}

void NaoProvider::update(JointSensorData &jointSensorData)
{
    update();
    float *sensors = naoBody.getSensors();

    int j = 0;
    for (int i = 0; i < Joints::NumOfJoint; i++)
    {
        if (i == Joints::rHipYawPitch)
        {
            jointSensorData.angles[i] = jointSensorData.angles[Joints::lHipYawPitch];
            jointSensorData.currents[i] = jointSensorData.currents[Joints::lHipYawPitch];
            jointSensorData.temperatures[i] = jointSensorData.temperatures[Joints::lHipYawPitch];
            jointSensorData.status[i] = jointSensorData.status[Joints::lHipYawPitch];
        }
        else
        {
            jointSensorData.angles[i] = sensors[j++] - theJointCalibration.offsets[i];
            jointSensorData.currents[i] = static_cast<short>(1000.f * sensors[j++]);
            jointSensorData.temperatures[i] = static_cast<unsigned char>(sensors[j++]);
            // jointSensorData.status[i] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int *>(&sensors[j++]));
        }
    }
    jointSensorData.timestamp = theFrameInfo.time;
}

void NaoProvider::update(KeyStates &keyStates)
{
    float *sensors = naoBody.getSensors();

    for (int i = 0, j = headTouchFrontSensor; i < KeyStates::NumOfKey; i++, j++)
    {
        keyStates.pressed[i] = sensors[j] != 0;
    }
}

void NaoProvider::update(RobotInfo &robotInfo)
{
    // robotInfo.number = theInstance->global.getSettings().playerNumber;
    // (RoboCup::RobotInfo&) robotInfo = gameControlData.teams[0].players[0];
    // std:: cout << "robotinfo: " << (int)robotInfo.penalty << std::endl;

    // team[0]

    // std::cout << "[INFO] NaoProvider" << std::endl;
    // std::cout << std::endl;
}

void NaoProvider::update(SystemSensorData &systemSensorData)
{
    update();
    float *sensors = naoBody.getSensors();
    if (theFrameInfo.getTimeSince(lastBodyTemperatureReadTime) * 1000 > 10)
    {
        lastBodyTemperatureReadTime = theFrameInfo.time;
        systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
    }
    systemSensorData.batteryCurrent = sensors[batteryCurrentSensor];
    systemSensorData.batteryLevel = sensors[batteryChargeSensor];
    systemSensorData.batteryTemperature = sensors[batteryTemperatureSensor];
    const short statusValue = static_cast<short>(sensors[batteryStatusSensor]);
    systemSensorData.batteryCharging = statusValue & 0b10000000;
}
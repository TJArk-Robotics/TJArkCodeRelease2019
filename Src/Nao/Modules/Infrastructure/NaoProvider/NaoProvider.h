#pragma once

#include "Platform/Nao/NaoBody.h"
#include "Tools/Global.h"
#include "Tools/Module/Blackboard.h"

class NaoProviderBase
{
public:
    /** REQUIRES */
    const JointCalibration &theJointCalibration = Blackboard::getInstance().jointCalibration;
    const JointLimits &theJointLimits = Blackboard::getInstance().jointLimits;
    const LEDRequest &theLEDRequest = Blackboard::getInstance().ledRequest;
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;

    /* USES */
    const RobotInfo &theRobotInfo = Blackboard::getInstance().robotInfo;
    const JointRequest &theJointRequest = Blackboard::getInstance().jointRequest;

    /** PROVIDES */
    FrameInfo &_theFrameInfo = Blackboard::getInstance().frameInfo;
    FsrSensorData &_theFsrSensorData = Blackboard::getInstance().fsrSensorData;
    InertialSensorData &_theInertialSensorData = Blackboard::getInstance().inertialSensorData;
    JointSensorData &_theJointSensorData = Blackboard::getInstance().jointSensorData;
    KeyStates &_theKeyStates = Blackboard::getInstance().keyStates;
    RobotInfo &_theRobotInfo = Blackboard::getInstance().robotInfo;
    SystemSensorData &_theSystemSensorData = Blackboard::getInstance().systemSensorData;
};

/**
 * @class NaoProvider
 * A module that provides information from the Nao.
 */
class NaoProvider : public NaoProviderBase
{
public:
    static thread_local NaoProvider *theInstance; /**< The only instance of this module. */
    NaoBody naoBody;
    RoboCup::RoboCupGameControlData gameControlData; /**< The last game control data received. */
    // Global global;
    float clippedLastFrame[Joints::NumOfJoint]; /**< Array that indicates whether a certain joint value was clipped in the last frame (and what was the value)*/
    unsigned lastBodyTemperatureReadTime = 0;
    NaoProvider();
    ~NaoProvider();

    static void finishFrame();
    static void waitForFrameData();

    void update();
    void update(FrameInfo &frameInfo);
    void update(FsrSensorData &fsrSensorData);
    void update(InertialSensorData &inertialSensorData);
    void update(JointSensorData &jointSensorData);
    void update(KeyStates &keyStates);
    void update(RobotInfo &robotInfo);
    void update(SystemSensorData &systemSensorData);

    /** The function sends a command to the Nao. */
    void send();
};
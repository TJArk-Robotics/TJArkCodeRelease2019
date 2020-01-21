#include "Behavior.h"
#include <iostream>

void Behavior::tick()
{
    update();
    receive();
    soccer.execute();
    headControl2018.execute();
    send();
}

void Behavior::update()
{
    FrameInfo &_theFrameInfo = Blackboard::getInstance().frameInfo;
    NaoProvider::theInstance->update(_theFrameInfo);

    KeyStates &_theKeyStates = Blackboard::getInstance().keyStates;
    _theKeyStates = blackboard->keyStates;

    LEDRequest &_theLEDRequest = Blackboard::getInstance().ledRequest;
    ModuleManager::theInstance->ledHandler.update(_theLEDRequest);

    GroundContactState &_theGroundContactState = Blackboard::getInstance().groundContactState;
    ModuleManager::theInstance->groundContactDetector.update(_theGroundContactState);

    JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
    ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
}

void Behavior::receive()
{
    MotionInfo &_theMotionInfo = Blackboard::getInstance().motionInfo;
    _theMotionInfo = blackboard->motionInfo;

    HeadMotionEngineOutput &_theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;
    _theHeadMotionEngineOutput = blackboard->headMotionEngineOutput;
}

void Behavior::send()
{
    blackboard->ledRequest = Blackboard::getInstance().ledRequest;

    blackboard->motionRequest = Blackboard::getInstance().motionRequest;

    blackboard->armMotionRequest = Blackboard::getInstance().armMotionRequest;

    blackboard->headMotionRequest = Blackboard::getInstance().headMotionRequest;
}
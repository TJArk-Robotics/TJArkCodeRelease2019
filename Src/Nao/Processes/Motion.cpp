#include "Motion.h"
#include "Platform/Timer.h"
#include <iostream>

#include <map>
#include <string>

void Motion::tick()
{
    resetUpdate();
    update();
    receive();
    testMotion();
    NaoProvider::finishFrame();
    NaoProvider::waitForFrameData();
    send();
}

void Motion::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedMap;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}

void Motion::update()
{
    /** Update LEDRequest */
    LEDRequest &_theLEDRequest = Blackboard::getInstance().ledRequest;
    _theLEDRequest = blackboard->ledRequest;

    /** Update JointRequest */
    JointRequest &_theJointRequest = Blackboard::getInstance().jointRequest;
    ModuleManager::theInstance->motionCombinator.update(_theJointRequest);

    /** Update MotionInfo */
    MotionInfo &_theMotionInfo = Blackboard::getInstance().motionInfo;
    ModuleManager::theInstance->motionCombinator.update(_theMotionInfo);

    /** Update KeyStates */
    KeyStates &_theKeyStates = Blackboard::getInstance().keyStates;
    NaoProvider::theInstance->update(_theKeyStates);

    /** Update HeadEngine */
}

void Motion::testMotion()
{
    // std::cout << "ArmLeftAngleReuqest: " << Blackboard::getInstance().jointRequest.angles[Joints::lShoulderPitch] << std::endl;
    // std::cout << "ArmMotionSelection Left:  " << Blackboard::getInstance().armMotionSelection.targetArmMotion[Arms::left] << std::endl;
    // std::cout << "ArmMotionSelection Right: " << Blackboard::getInstance().armMotionSelection.targetArmMotion[Arms::right] << std::endl;
    // std::cout << "OdometryData: " << Blackboard::getInstance().odometryData.translation << std::endl;
    // std::cout << std::endl;

    // std::cout << "MotionInfo.motion: " << Blackboard::getInstance().motionInfo.motion << std::endl << std::endl;
}

void Motion::send()
{
    /** send to Behavior */
    blackboard->keyStates = Blackboard::getInstance().keyStates;
    blackboard->motionInfo = Blackboard::getInstance().motionInfo;
    blackboard->headMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;
}

void Motion::receive()
{
    MotionRequest &_theMotionRequest = Blackboard::getInstance().motionRequest;
    _theMotionRequest = blackboard->motionRequest;

    ArmMotionRequest &_theArmMotionRequest = Blackboard::getInstance().armMotionRequest;
    _theArmMotionRequest = blackboard->armMotionRequest;

    LEDRequest &_theLEDRequest = Blackboard::getInstance().ledRequest;
    _theLEDRequest = blackboard->ledRequest;

    HeadMotionRequest &_theHeadMotionRequest = Blackboard::getInstance().headMotionRequest;
    _theHeadMotionRequest = blackboard->headMotionRequest;
}
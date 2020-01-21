#pragma once

#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Tools/Module/Blackboard.h"
#include <array>

class MotionSelectorBase
{
public:
    /* REQUIRES */
    const MotionRequest &theMotionRequest = Blackboard::getInstance().motionRequest;
    const GroundContactState &theGroundContactState = Blackboard::getInstance().groundContactState;
    const ArmMotionRequest &theArmMotionRequest = Blackboard::getInstance().armMotionRequest;
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;
    const MotionInfo &theMotionInfo = Blackboard::getInstance().motionInfo;

    /* USES */
    const WalkingEngineOutput &theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
    const SpecialActionsOutput &theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
    const ArmKeyFrameEngineOutput &theArmKeyFrameEngineOutput = Blackboard::getInstance().armKeyFrameEngineOutput;
    

    /* PROVIDES */
    // LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;

    int playDeadDelay;
    std::array<int, MotionRequest::NumOfMotion> interpolationTimes;
    std::array<int, ArmMotionSelection::NumOfArmMotion> armInterPolationTimes;
    void loadsParameters()
    {
        playDeadDelay = 2000;
        interpolationTimes[MotionRequest::walk] = 790;
        interpolationTimes[MotionRequest::kick] = 200;
        interpolationTimes[MotionRequest::specialAction] = 200;
        interpolationTimes[MotionRequest::stand] = 600;
        interpolationTimes[MotionRequest::getUp] = 1;
        interpolationTimes[MotionRequest::fall] = 1;

        armInterPolationTimes[ArmMotionSelection::walkArms] = 300;
        armInterPolationTimes[ArmMotionSelection::kickArms] = 300;
        armInterPolationTimes[ArmMotionSelection::specialActionArms] = 200;
        armInterPolationTimes[ArmMotionSelection::standArms] = 300;
        armInterPolationTimes[ArmMotionSelection::getUpArms] = 1;
        armInterPolationTimes[ArmMotionSelection::fallArms] = 1;
        armInterPolationTimes[ArmMotionSelection::keyFrameS] = 300;
    }
};

class MotionSelector : public MotionSelectorBase
{
public:
    MotionSelector();
    ~MotionSelector() { theInstance = nullptr; }

    static void sitDown();

    static thread_local MotionSelector* theInstance;
    bool forceSitDown = false;
    MotionRequest::Motion lastLegMotion = MotionRequest::specialAction;
    MotionRequest::Motion prevLegMotion = MotionRequest::specialAction;
    std::array<ArmMotionSelection::ArmMotion, Arms::NumOfArm> lastArmMotion;
    std::array<ArmMotionSelection::ArmMotion, Arms::NumOfArm> prevArmMotion;
    unsigned lastExecution = 0;
    unsigned lastExecutionArm = 0;
    unsigned lastExecutionLeg = 0;
    SpecialActionRequest::SpecialActionID lastActiveSpecialAction = SpecialActionRequest::playDead;
    std::array<ArmKeyFrameRequest::ArmKeyFrameID, Arms::NumOfArm> lastActiveArmKeyFrame;

    void update();
    void update(LegMotionSelection &legMotionSelection);
    void update(ArmMotionSelection &armMotionSelection);

    void interpolate(float *ratios, const int amount, const int interpolationTime, const int targetMotion);
};
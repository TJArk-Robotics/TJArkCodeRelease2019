#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class ArmMotionCombinatorBase
{
public:
    /* REQUIRES */
    const ArmKeyFrameEngineOutput &theArmKeyFrameEngineOutput = Blackboard::getInstance().armKeyFrameEngineOutput;
    const ArmMotionSelection &theArmMotionSelection = Blackboard::getInstance().armMotionSelection;
    const GetUpEngineOutput &theGetUpEngineOutput = Blackboard::getInstance().getUpEngineOutput;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const KickEngineOutput &theKickEngineOutput = Blackboard::getInstance().kickEngineOutput;
    // FallEngine
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    const SpecialActionsOutput &theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
    const StandArmRequest &theStandArmRequest = Blackboard::getInstance().standArmRequest;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;
    const WalkingEngineOutput &theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;

    /* PROVIDES */
    // ArmJointRequest &_theArmJointRequest = Blackboard::getInstance().armJointRequest;
    // ArmMotionInfo &_theArmMotionInfo = Blackboard::getInstance().armMotionInfo;

};

class ArmMotionCombinator : public ArmMotionCombinatorBase
{
public:
    JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */
    JointRequest lastJointRequest;

    void update();
    void update(ArmJointRequest &armJointRequest);
    void update(ArmMotionInfo &armMotionInfo);
};
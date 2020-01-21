#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Module/Blackboard.h"

class LegMotionCombinatorBase
{
public:
    /* REQUIRES */
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const WalkingEngineOutput &theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    const StandLegRequest &theStandLegRequest = Blackboard::getInstance().standLegRequest;
    const SpecialActionsOutput &theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;

    /* PROVIDES */
    // LegJointRequest &_theLegMotionRequest = Blackboard::getInstance().legJointRequest;
};

class LegMotionCombinator : public LegMotionCombinatorBase
{
public:
    JointAngles lastJointAngles;/**< The measured joint angles the last time when not interpolating. */
    void update();
    void update(LegJointRequest &legJointRequest);
};
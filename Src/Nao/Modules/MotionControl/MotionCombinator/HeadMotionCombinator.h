#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Blackboard.h"

class HeadMotionCombinatorBase
{
public:
    /* REQUIRES */
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    // GeUpEngineOutput
    const HeadMotionEngineOutput &theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;
    // KickEngineOutput
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    // FallEngineOutput
    const SpecialActionsOutput &theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;
    const WalkingEngineOutput &theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
    const StandLegRequest &theStandLegRequest = Blackboard::getInstance().standLegRequest;

    /* PROVIDES */
    // HeadJointRequest &_theHeadJointRequest = Blackboard::getInstance().headJointRequest;
};

class HeadMotionCombinator : public HeadMotionCombinatorBase
{
public:
    void update();
    void update(HeadJointRequest &headJointRequest);

    JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */
};
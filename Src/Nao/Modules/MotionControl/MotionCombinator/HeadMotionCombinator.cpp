#include "HeadMotionCombinator.h"
#include "MotionCombinator.h"
#include "Tools/Module/ModuleManager.h"

void HeadMotionCombinator::update()
{
    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // GeUpEngineOutput
    // HeadMotionEngineOutput
    if (!Blackboard::getInstance().updatedMap["HeadMotionEngineOutput"])
    {
        HeadMotionEngineOutput &_theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;
        ModuleManager::theInstance->headMotionEngine.update(_theHeadMotionEngineOutput);
        Blackboard::getInstance().updatedMap["HeadMotionEngineOutput"] = true;
    }
    // KickEngineOutput
    // LegMotionSelection
    if (!Blackboard::getInstance().updatedMap["LegMotionSelection"])
    {
        LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
        ModuleManager::theInstance->motionSelector.update(_theLegMotionSelection);
        Blackboard::getInstance().updatedMap["LegMotionSelection"] = true;
    }
    // FallEngineOutput
    // SpecialActionsOutput
    if (!Blackboard::getInstance().updatedMap["SpecialActionsOutput"])
    {
        SpecialActionsOutput &_theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
        ModuleManager::theInstance->specialActions.update(_theSpecialActionsOutput);
        Blackboard::getInstance().updatedMap["SpecialActionsOutput"] = true;
    }
    // StiffnessSettings
    // WalkingEngineOutput
    if (!Blackboard::getInstance().updatedMap["WalkingEngineOutput"])
    {
        WalkingEngineOutput &_theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
        ModuleManager::theInstance->walkingEngine.update(_theWalkingEngineOutput);
        Blackboard::getInstance().updatedMap["WalkingEngineOutput"] = true;
    }
    // StandLegRequest
    if (!Blackboard::getInstance().updatedMap["StandLegRequest"])
    {
        StandLegRequest &_theStandLegRequest = Blackboard::getInstance().standLegRequest;
        ModuleManager::theInstance->walkingEngine.update(_theStandLegRequest);
        Blackboard::getInstance().updatedMap["StandLegRequest"] = true;
    }

    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
}

void HeadMotionCombinator::update(HeadJointRequest &jointRequest)
{
    update();

    const JointRequest *jointRequests[MotionRequest::NumOfMotion];
    // jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
    // jointRequests[MotionRequest::kick] = &theKickEngineOutput;
    // jointRequests[MotionRequest::fall] = &theFallEngineOutput;
    // jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
    // jointRequests[MotionRequest::stand] = &theStandLegRequest;
    // jointRequests[MotionRequest::getUp] = &theGetUpEngineOutput;

    jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::kick] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::fall] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
    jointRequests[MotionRequest::stand] = &theStandLegRequest;
    jointRequests[MotionRequest::getUp] = &theWalkingEngineOutput;

    jointRequest.angles[Joints::headYaw] = theHeadMotionEngineOutput.pan;
    jointRequest.angles[Joints::headPitch] = theHeadMotionEngineOutput.tilt;
    MotionUtilities::copy(*jointRequests[theLegMotionSelection.targetMotion], jointRequest, theStiffnessSettings, Joints::headYaw, Joints::headPitch);

    ASSERT(jointRequest.isValid());

    if (theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] == 1.f)
    {
        lastJointAngles = theJointAngles;
    }
    else // interpolate motions
    {
        const bool interpolateStiffness = !(theLegMotionSelection.targetMotion != MotionRequest::specialAction &&
                                            theLegMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
                                            theLegMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
        for (int i = 0; i < MotionRequest::NumOfMotion; ++i)
            if (i != theLegMotionSelection.targetMotion && theLegMotionSelection.ratios[i] > 0.f)
            {
                MotionUtilities::interpolate(*jointRequests[i], *jointRequests[theLegMotionSelection.targetMotion], theLegMotionSelection.ratios[i],
                                             jointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, Joints::headYaw, Joints::headPitch);
            }
    }

    ASSERT(jointRequest.isValid());
}
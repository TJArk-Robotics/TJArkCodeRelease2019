#include "LegMotionCombinator.h"
#include "MotionCombinator.h"
#include "Tools/RobotParts/Joints.h"

#include "Tools/Module/Blackboard.h"

// Provider
// #include "Modules/Infrastructure/JointAnglesProvider/JointAnglesProvider.h"
// #include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
// #include "Modules/MotionControl/MotionSelector/MotionSelector.h"
#include "Tools/Module/ModuleManager.h"

void LegMotionCombinator::update()
{
    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        // JointAnglesProvider jointAnglesProvider;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // WalkingEngineOutput
    if (!Blackboard::getInstance().updatedMap["WalkingEngineOutput"])
    {
        WalkingEngineOutput &_theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
        // WalkingEngine walkingEngine;
        ModuleManager::theInstance->walkingEngine.update(_theWalkingEngineOutput);
        Blackboard::getInstance().updatedMap["WalkingEngineOutput"] = true;
    }
    // StiffnessSettings
    // if (!Blackboard::getInstance().updatedMap["StiffnessSettings"])
    // {
    //     StiffnessSettings &_theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;
    // }
    // LegMotionSelection
    if (!Blackboard::getInstance().updatedMap["LegMotionSelection"])
    {
        LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
        // MotionSelector motionSelector;
        ModuleManager::theInstance->motionSelector.update(_theLegMotionSelection);
        Blackboard::getInstance().updatedMap["LegMotionSelection"] = true;
    }
    // StandLegRequest
    if (!Blackboard::getInstance().updatedMap["StandLegRequest"])
    {
        StandLegRequest &_theStandLegRequest = Blackboard::getInstance().standLegRequest;
        ModuleManager::theInstance->walkingEngine.update(_theStandLegRequest);
        Blackboard::getInstance().updatedMap["StandLegRequest"];
    }

    // SpecialActionsOutput
    if (!Blackboard::getInstance().updatedMap["SpecialActionsOutput"])
    {
        SpecialActionsOutput &_theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
        ModuleManager::theInstance->specialActions.update(_theSpecialActionsOutput);
        Blackboard::getInstance().updatedMap["SpecialActionsOutput"] = true;
    }
}

void LegMotionCombinator::update(LegJointRequest &jointRequest)
{
    update();
    const JointRequest *jointRequests[MotionRequest::NumOfMotion];
    // jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
    // jointRequests[MotionRequest::kick] = &theKickEngineOutput;
    // jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
    // jointRequests[MotionRequest::stand] = &theStandLegRequest;
    // jointRequests[MotionRequest::getUp] = &theGetUpEngineOutput;
    // jointRequests[MotionRequest::fall] = &theFallEngineOutput;
    jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::kick] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
    jointRequests[MotionRequest::stand] = &theStandLegRequest;
    jointRequests[MotionRequest::getUp] = &theWalkingEngineOutput;
    jointRequests[MotionRequest::fall] = &theWalkingEngineOutput;

    MotionUtilities::copy(*jointRequests[theLegMotionSelection.targetMotion], jointRequest, theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);

    switch (theLegMotionSelection.targetMotion)
    {
    case MotionRequest::walk:
        ASSERT(jointRequests[MotionRequest::walk]->isValid());
        break;
    case MotionRequest::kick:
        ASSERT(jointRequests[MotionRequest::kick]->isValid());
        break;
    case MotionRequest::specialAction:
        ASSERT(jointRequests[MotionRequest::specialAction]->isValid());
        break;
    case MotionRequest::stand:
        ASSERT(jointRequests[MotionRequest::stand]->isValid());
        break;
    case MotionRequest::getUp:
        ASSERT(jointRequests[MotionRequest::getUp]->isValid());
        break;
    case MotionRequest::fall:
        ASSERT(jointRequests[MotionRequest::fall]->isValid());
        break;
    }

    ASSERT(jointRequest.isValid());

    if (theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] == 1.f)
    {
        lastJointAngles = theJointAngles;
    }
    else
    {
        const bool interpolateStiffness = !(theLegMotionSelection.targetMotion != MotionRequest::specialAction &&
                                            theLegMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
                                            theLegMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
        for (int i = 0; i < MotionRequest::NumOfMotion; ++i)
        {
            if (i != theLegMotionSelection.targetMotion && theLegMotionSelection.ratios[i] > 0.f)
            {
                MotionUtilities::interpolate(*jointRequests[i], *jointRequests[theLegMotionSelection.targetMotion], theLegMotionSelection.ratios[i],
                                             jointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, Joints::firstLegJoint, Joints::rAnkleRoll);
                switch (i)
                {
                case MotionRequest::walk:
                    ASSERT(jointRequests[MotionRequest::walk]->isValid());
                    break;
                case MotionRequest::kick:
                    ASSERT(jointRequests[MotionRequest::kick]->isValid());
                    break;
                case MotionRequest::specialAction:
                    ASSERT(jointRequests[MotionRequest::specialAction]->isValid());
                    break;
                case MotionRequest::stand:
                    ASSERT(jointRequests[MotionRequest::stand]->isValid());
                    break;
                case MotionRequest::getUp:
                    ASSERT(jointRequests[MotionRequest::getUp]->isValid());
                    break;
                case MotionRequest::fall:
                    ASSERT(jointRequests[MotionRequest::fall]->isValid());
                    break;
                }
            }
        }
    }

    ASSERT(jointRequest.isValid());
}

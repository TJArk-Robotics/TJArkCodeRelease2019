#include "ArmMotionCombinator.h"
#include "Tools/Module/ModuleManager.h"

void ArmMotionCombinator::update()
{
    // ArmKeyFrameEngineOuput
    if (!Blackboard::getInstance().updatedMap["ArmKeyFrameEngineOuput"])
    {
        ArmKeyFrameEngineOutput &_theArmKeyFrameEngineOutput = Blackboard::getInstance().armKeyFrameEngineOutput;
        ModuleManager::theInstance->armKeyFrameEngine.update(_theArmKeyFrameEngineOutput);
        Blackboard::getInstance().updatedMap["ArmKeyFrameEngineOuput"] = true;
    }
    // ArmMotionSelection
    if (!Blackboard::getInstance().updatedMap["ArmMotionSelection"])
    {
        ArmMotionSelection &_theArmMotionSelection = Blackboard::getInstance().armMotionSelection;
        ModuleManager::theInstance->motionSelector.update(_theArmMotionSelection);
        Blackboard::getInstance().updatedMap["ArmMotionSelection"] = true;
    }
    // GetUpEngineOutput

    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // KickEngineOutput
    // // FallEngine
    // LegMotionSelection
    if (!Blackboard::getInstance().updatedMap["LegMotionSelection"])
    {
        LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
        ModuleManager::theInstance->motionSelector.update(_theLegMotionSelection);
        Blackboard::getInstance().updatedMap["LegMotionSelection"] = true;
    }
    // SpecialActionsOutput
    if (!Blackboard::getInstance().updatedMap["SpecialActionsOutput"])
    {
        SpecialActionsOutput &_theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
        ModuleManager::theInstance->specialActions.update(_theSpecialActionsOutput);
        Blackboard::getInstance().updatedMap["SpecialActionsOutput"] = true;
    }
    // StandArmRequest
    if (!Blackboard::getInstance().updatedMap["StandArmRequest"])
    {
        StandArmRequest &_theStandArmRequest = Blackboard::getInstance().standArmRequest;
        ModuleManager::theInstance->walkingEngine.update(_theStandArmRequest);
        Blackboard::getInstance().updatedMap["StandArmRequest"] = true;
    }
    // StiffnessSettings
    // WalkingEngineOutput
    if (!Blackboard::getInstance().updatedMap["WalkingEngineOutput"])
    {
        WalkingEngineOutput &_theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
        ModuleManager::theInstance->walkingEngine.update(_theWalkingEngineOutput);
        Blackboard::getInstance().updatedMap["WalkingEngineOutput"] = true;
    }
}

void ArmMotionCombinator::update(ArmJointRequest &armJointRequest)
{
    update();

    const JointRequest *armJointRequests[ArmMotionSelection::NumOfArmMotion];
    // armJointRequests[ArmMotionSelection::walkArms] = &theWalkingEngineOutput;
    // armJointRequests[ArmMotionSelection::fallArms] = &theFallEngineOutput;
    // armJointRequests[ArmMotionSelection::kickArms] = &theKickEngineOutput;
    // armJointRequests[ArmMotionSelection::specialActionArms] = &theSpecialActionsOutput;
    // armJointRequests[ArmMotionSelection::standArms] = &theStandArmRequest;
    // armJointRequests[ArmMotionSelection::getUpArms] = &theGetUpEngineOutput;

    armJointRequests[ArmMotionSelection::walkArms] = &theWalkingEngineOutput;
    armJointRequests[ArmMotionSelection::fallArms] = &theWalkingEngineOutput;
    armJointRequests[ArmMotionSelection::kickArms] = &theKickEngineOutput;
    armJointRequests[ArmMotionSelection::specialActionArms] = &theSpecialActionsOutput;
    armJointRequests[ArmMotionSelection::standArms] = &theStandArmRequest;
    armJointRequests[ArmMotionSelection::getUpArms] = &theGetUpEngineOutput;

    armJointRequests[ArmMotionSelection::keyFrameS] = &theArmKeyFrameEngineOutput;

    auto combinateArmMotions = [&](Arms::Arm const arm) {
        const Joints::Joint startJoint = arm == Arms::left ? Joints::lShoulderPitch : Joints::rShoulderPitch;
        const Joints::Joint endJoint = arm == Arms::left ? Joints::lHand : Joints::rHand;
        MotionUtilities::copy(*armJointRequests[theArmMotionSelection.targetArmMotion[arm]], armJointRequest, theStiffnessSettings, startJoint, endJoint);

        ASSERT(armJointRequest.isValid());

        if (theArmMotionSelection.armRatios[arm][theArmMotionSelection.targetArmMotion[arm]] == 1.f)
        {
            lastJointAngles = theJointAngles;
        }
        else
        {
            const bool interpolateStiffness = !(theLegMotionSelection.targetMotion != MotionRequest::specialAction &&
                                                theLegMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
                                                theLegMotionSelection.ratios[MotionRequest::specialAction] > 0.f &&
                                                theArmMotionSelection.armRatios[arm][ArmMotionRequest::none] > 0);

            for (int i = 0; i < ArmMotionSelection::NumOfArmMotion; ++i)
            {
                if (i != theArmMotionSelection.targetArmMotion[arm] && theArmMotionSelection.armRatios[arm][i] > 0)
                {
                    MotionUtilities::interpolate(*armJointRequests[i], *armJointRequests[theArmMotionSelection.targetArmMotion[arm]], theArmMotionSelection.armRatios[arm][i], armJointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, startJoint, endJoint);
                }
            }
        }

        ASSERT(armJointRequest.isValid());
    };

    combinateArmMotions(Arms::left);
    combinateArmMotions(Arms::right);
}

void ArmMotionCombinator::update(ArmMotionInfo &armMotionInfo)
{
    update();

    auto setArmMotionInfo = [&](Arms::Arm const arm) {
        if (theArmMotionSelection.armRatios[arm][theArmMotionSelection.targetArmMotion[arm]] == 1.f)
        {
            armMotionInfo.armMotion[arm] = theArmMotionSelection.targetArmMotion[arm] < ArmMotionSelection::firstNonBodyMotion ? ArmMotionRequest::none : ArmMotionRequest::ArmRequest(theArmMotionSelection.targetArmMotion[arm] - ArmMotionSelection::firstNonBodyMotion + 1);

            switch (theArmMotionSelection.targetArmMotion[arm])
            {
            case ArmMotionSelection::keyFrameS:
                armMotionInfo.armKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;
                break;
            default:
                break;
            }
        }
    };

    setArmMotionInfo(Arms::left);
    setArmMotionInfo(Arms::right);
}
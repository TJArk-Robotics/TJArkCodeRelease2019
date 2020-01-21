#include "MotionSelector.h"
#include "Platform/SystemCall.h"

#include <algorithm>

// Provider
// #include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
// #include "Modules/Sensing/GroundContactDetector/GroundContactDetector.h"
// #include "Modules/MotionControl/MotionCombinator/MotionCombinator.h"
// #include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
#include "Tools/Module/ModuleManager.h"

thread_local MotionSelector *MotionSelector::theInstance = nullptr;

MotionSelector::MotionSelector()
{
    lastArmMotion.fill(ArmMotionSelection::specialActionArms);
    prevArmMotion.fill(ArmMotionSelection::specialActionArms);
    lastActiveArmKeyFrame.fill(ArmKeyFrameRequest::useDefault);
    theInstance = this;
    loadsParameters();
}

void MotionSelector::sitDown()
{
    if (theInstance && SystemCall::getMode() == SystemCall::physicalRobot)
        theInstance->forceSitDown = true;
}

void MotionSelector::update()
{
    // MotionRequest
    // GroundContactState
    if (!Blackboard::getInstance().updatedMap["GroundContactState"])
    {
        GroundContactState &_theGroundContactState = Blackboard::getInstance().groundContactState;
        // GroundContactDetector groundContactDetector;
        ModuleManager::theInstance->groundContactDetector.update(_theGroundContactState);
        Blackboard::getInstance().updatedMap["GroundContactState"] = true;
    }
    // ArmMotionRequest updated from BehaviorControl
    // MotionInfo
    if (!Blackboard::getInstance().updatedMap["MotionInfo"])
    {
        MotionInfo &_theMotionInfo = Blackboard::getInstance().motionInfo;
        ModuleManager::theInstance->motionCombinator.update(_theMotionInfo);
        Blackboard::getInstance().updatedMap["MotionInfo"] = true;
    }
    // USE WalkingEngineOutput
    // ArmMotionRequest
    // FrameInfo
    {
        FrameInfo &_theFrameInfo = Blackboard::getInstance().frameInfo;
        NaoProvider::theInstance->update(_theFrameInfo);
    }
}

void MotionSelector::update(LegMotionSelection &legMotionSelection)
{
    update();
    if (lastExecutionLeg)
    {
        MotionRequest::Motion requestedLegMotion = theMotionRequest.motion;

        if (theMotionRequest.motion == MotionRequest::walk && !theGroundContactState.contact)
        {
            requestedLegMotion = MotionRequest::stand;
        }

        // if (forceSitDown && (lastLegMotion == MotionRequest::walk || lastLegMotion == MotionRequest::stand))
        // {
        //     requestedLegMotion = MotionRequest::specialAction;
        // }

        // check if the target motion can be the requested motion (mainly if leaving is possible)
        // if ((lastLegMotion == MotionRequest::walk && (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
        //     (lastLegMotion == MotionRequest::stand && legMotionSelection.ratios[MotionRequest::stand] == 1.f) || // stand can always be left if fully active
        //     (lastLegMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) ||
        //     (lastLegMotion == MotionRequest::kick && theKickEngineOutput.isLeavingPossible) ||
        //     (lastLegMotion == MotionRequest::getUp && theGetUpEngineOutput.isLeavingPossible) ||
        //     (lastLegMotion == MotionRequest::fall && !theFallEngineOutput.active)) //never immediatly leave kick or get up
        if ( (lastLegMotion == MotionRequest::walk && (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
            (lastLegMotion == MotionRequest::stand && legMotionSelection.ratios[MotionRequest::stand] == 1.f) || 
            (lastLegMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) )
        {
            legMotionSelection.targetMotion = requestedLegMotion;
        }

        // std::cout << "requestedLegMotion:                           " << requestedLegMotion << std::endl;
        // std::cout << "theMotionRequest.motion:                      " << theMotionRequest.motion << std::endl;
        // std::cout << "theMotionInfo.motion:                         " << theMotionInfo.motion << std::endl;
        // std::cout << "lastLegMotion:                                " << lastLegMotion << std::endl;
        // std::cout << "legMotionSelection.ratio[Walk]:               " << legMotionSelection.ratios[MotionRequest::walk] << std::endl;
        // std::cout << "legMotionSelection.targetMotion:              " << legMotionSelection.targetMotion << std::endl;
        // std::cout << "theMotionRequest.specialActionRequest:        " << theMotionRequest.specialActionRequest.specialAction << std::endl;
        // std::cout << "theSpecialActionsOuptut.isLeavingPossible:    " << theSpecialActionsOutput.isLeavingPossible << std::endl;
        // std::cout << "theLegMotionSelection.specialActionMode:      " << theLegMotionSelection.specialActionMode << std::endl;
        // std::cout << "theGroundContactState:                        " << theGroundContactState.contact << std::endl;
        // std::cout << std::endl; 

        // if (theFallEngineOutput.active) // fallengine activates itself
        //     requestedLegMotion = legMotionSelection.targetMotion = MotionRequest::fall;
        // else if (theMotionInfo.motion == MotionRequest::fall && theFallDownState.state == FallDownState::squatting)
        //     requestedLegMotion = legMotionSelection.targetMotion = MotionRequest::getUp;

        if (requestedLegMotion == MotionRequest::specialAction)
        {
            legMotionSelection.specialActionRequest = theMotionRequest.specialActionRequest;
            if (forceSitDown && (lastLegMotion == MotionRequest::walk || lastLegMotion == MotionRequest::stand))
            {
                legMotionSelection.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
                forceSitDown = false;
            }
        }
        else
        {
            legMotionSelection.specialActionRequest = SpecialActionRequest();
            if (legMotionSelection.targetMotion == MotionRequest::specialAction)
                legMotionSelection.specialActionRequest.specialAction = SpecialActionRequest::NumOfSpecialActionID;
        }

        int interpolationTime = interpolationTimes[legMotionSelection.targetMotion];
        // When standing, walking may start instantly
        if (legMotionSelection.targetMotion == MotionRequest::walk && lastLegMotion == MotionRequest::stand && legMotionSelection.ratios[MotionRequest::stand] == 1.f)
            interpolationTime = 1;

        const bool afterPlayDead = prevLegMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead;
        const int bodyInterpolationTime = afterPlayDead ? playDeadDelay : interpolationTime;

        interpolate(legMotionSelection.ratios.data(), MotionRequest::NumOfMotion, bodyInterpolationTime, legMotionSelection.targetMotion);

        if (legMotionSelection.ratios[MotionRequest::specialAction] < 1.f)
        {
            if (legMotionSelection.targetMotion == MotionRequest::specialAction)
                legMotionSelection.specialActionMode = LegMotionSelection::first;
            else
                legMotionSelection.specialActionMode = LegMotionSelection::deactive;
        }
        else
            legMotionSelection.specialActionMode = LegMotionSelection::active;

        if (legMotionSelection.specialActionMode == LegMotionSelection::active && legMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::NumOfSpecialActionID)
            lastActiveSpecialAction = legMotionSelection.specialActionRequest.specialAction;
    }
    if (lastLegMotion != legMotionSelection.targetMotion)
        prevLegMotion = lastLegMotion;

    lastLegMotion = legMotionSelection.targetMotion;

#ifndef NDEBUG
    const Rangef ratioLimits = Rangef::ZeroOneRange();
    for (int i = 0; i < MotionRequest::NumOfMotion; ++i)
        ASSERT(ratioLimits.isInside(legMotionSelection.ratios[i]));
#endif
    lastExecutionLeg = theFrameInfo.time;
}

void MotionSelector::update(ArmMotionSelection &armMotionSelection)
{
    update();
    if (lastExecutionArm)
    {
        for (unsigned i = 0; i < Arms::NumOfArm; ++i)
        {
            Arms::Arm arm = static_cast<Arms::Arm>(i);

            const bool isNone = theArmMotionRequest.armMotion[arm] == ArmMotionRequest::none;

            ArmMotionSelection::ArmMotion requestedArmMotion;
            switch (theLegMotionSelection.targetMotion)
            {
            case MotionRequest::kick:
                requestedArmMotion = ArmMotionSelection::kickArms;
                break;
            case MotionRequest::getUp:
                requestedArmMotion = ArmMotionSelection::getUpArms;
                break;
            case MotionRequest::specialAction:
            {
                //forceNone = !theSpecialActionsOutput.isArmLeavingAllowed;
                if (isNone ||
                    (theLegMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::standHigh &&
                     theLegMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::standHighLookUp)) //FIXME QUICKHACK
                    requestedArmMotion = ArmMotionSelection::specialActionArms;
                else
                    requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
            }
            break;
            case MotionRequest::walk:
            {
                if (isNone)
                    requestedArmMotion = ArmMotionSelection::walkArms;
                else
                    requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
                break;
            }
            case MotionRequest::stand:
            {
                if (isNone)
                    requestedArmMotion = ArmMotionSelection::standArms;
                else
                    requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
                break;
            }
            case MotionRequest::fall:
                requestedArmMotion = ArmMotionSelection::fallArms;
                break;
            default:
                FAIL("Unknown target motion.");
                return;
            }

            // check if the target armmotion can be the requested armmotion (mainly if leaving is possible)
            if (requestedArmMotion != ArmMotionSelection::keyFrameS && !theArmKeyFrameEngineOutput.arms[arm].isFree && theLegMotionSelection.targetMotion != MotionRequest::getUp && theLegMotionSelection.targetMotion != MotionRequest::kick && theLegMotionSelection.targetMotion != MotionRequest::fall)
            {
                armMotionSelection.targetArmMotion[arm] = ArmMotionSelection::keyFrameS;
                armMotionSelection.armKeyFrameRequest.arms[arm].fast = false;
                armMotionSelection.armKeyFrameRequest.arms[arm].motion = ArmKeyFrameRequest::reverse;
            }
            else
            {
                armMotionSelection.targetArmMotion[arm] = requestedArmMotion;
                if (armMotionSelection.targetArmMotion[arm] == ArmMotionSelection::keyFrameS)
                    armMotionSelection.armKeyFrameRequest.arms[arm] = theArmMotionRequest.armKeyFrameRequest.arms[arm];
            }

            const bool afterPlayDead = prevLegMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead &&
                                       prevArmMotion[arm] == ArmMotionSelection::specialActionArms;
            const int armInterpolationTime = afterPlayDead ? playDeadDelay : armInterPolationTimes[armMotionSelection.targetArmMotion[arm]];
            interpolate(armMotionSelection.armRatios[arm].data(), ArmMotionSelection::NumOfArmMotion, armInterpolationTime, armMotionSelection.targetArmMotion[arm]);
        }
    }

    if (lastArmMotion[Arms::left] != armMotionSelection.targetArmMotion[Arms::left])
        prevArmMotion[Arms::left] = lastArmMotion[Arms::left];

    if (lastArmMotion[Arms::right] != armMotionSelection.targetArmMotion[Arms::right])
        prevArmMotion[Arms::right] = lastArmMotion[Arms::right];

    lastArmMotion[Arms::left] = armMotionSelection.targetArmMotion[Arms::left];
    lastArmMotion[Arms::right] = armMotionSelection.targetArmMotion[Arms::right];

    // lastExecution = theFrameInfo.time;
    lastExecutionArm = theFrameInfo.time;
}

void MotionSelector::interpolate(float *ratios, const int amount, const int interpolationTime, const int targetMotion)
{
    // increase / decrease all ratios according to target motion
    const unsigned deltaTime = theFrameInfo.getTimeSince(std::min(lastExecutionArm, lastExecutionLeg));

    // std::cout << "lastExecutionArm: " << lastExecutionArm << std::endl;
    // std::cout << "lastExecutionLeg: " << lastExecutionLeg << std::endl;
    // std::cout << "deltaTime:        " << deltaTime << std::endl << std::endl;

    // const unsigned deltaTime = theFrameInfo.getTimeSince(lastExecution);
    float delta = static_cast<float>(deltaTime) / interpolationTime;
    ASSERT(SystemCall::getMode() == SystemCall::logfileReplay || delta > 0.00001f);
    float sum = 0;
    for (int i = 0; i < amount; i++)
    {
        if (i == targetMotion)
            ratios[i] += delta;
        else
            ratios[i] -= delta;
        ratios[i] = std::max(ratios[i], 0.0f); // clip ratios
        sum += ratios[i];
    }
    ASSERT(sum != 0);
    // normalize ratios
    for (int i = 0; i < amount; i++)
    {
        ratios[i] /= sum;
        if (std::abs(ratios[i] - 1.f) < 0.00001f)
            ratios[i] = 1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
    }
}

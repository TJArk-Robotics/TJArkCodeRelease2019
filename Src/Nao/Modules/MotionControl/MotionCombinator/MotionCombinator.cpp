#include "MotionCombinator.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Module/Blackboard.h"

#include "Representations/Configuration/DamageConfiguration.h"
// Provider
#include "Tools/Module/ModuleManager.h"

void MotionCombinator::update()
{
    // ArmJointRequest
    if (!Blackboard::getInstance().updatedMap["ArmJointRequest"])
    {
        // ArmMotionCombinator armMotionCombinator;
        ArmJointRequest &_theArmJointRequest = Blackboard::getInstance().armJointRequest;
        ModuleManager::theInstance->armMotionCombinator.update(_theArmJointRequest);
        Blackboard::getInstance().updatedMap["ArmJointRequest"] = true;
    }

    // ArmMotionSelection
    if (!Blackboard::getInstance().updatedMap["ArmMotionSelection"])
    {
        ArmMotionSelection &_theArmMotionSelection = Blackboard::getInstance().armMotionSelection;
        // MotionSelector motionSelector;
        ModuleManager::theInstance->motionSelector.update(_theArmMotionSelection);
        Blackboard::getInstance().updatedMap["ArmMotionSelection"] = true;
    }

    // DamageConfigurationBody

    // FallDownState
    if (!Blackboard::getInstance().updatedMap["FallDownState"])
    {
        FallDownState &_theFallDownState = Blackboard::getInstance().fallDownState;
        // FallDownStateProvider fallDownStateProvider;
        ModuleManager::theInstance->fallDownStateProvider.update(_theFallDownState);
        Blackboard::getInstance().updatedMap["FallDownState"] = true;
    }

    // InertialData
    if (!Blackboard::getInstance().updatedMap["InertialData"])
    {
        InertialData &_theInertialData = Blackboard::getInstance().inertialData;
        NaoProvider::theInstance->update(_theInertialData);
        Blackboard::getInstance().updatedMap["InertialData"] = true;
    }

    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        // JointAnglesProvider jointAnglesProvider;
        ModuleManager::theInstance->jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }

    // KeyStates
    if (!Blackboard::getInstance().updatedMap["KeyStates"])
    {
        KeyStates &_theKeyStates = Blackboard::getInstance().keyStates;
        NaoProvider::theInstance->update(_theKeyStates);
        Blackboard::getInstance().updatedMap["KeyStates"] = true;
    }

    // LegJointRequest
    if (!Blackboard::getInstance().updatedMap["LegJointRequest"])
    {
        LegJointRequest &_theLegJointRequest = Blackboard::getInstance().legJointRequest;
        // LegMotionCombinator legMotionCombinator;
        ModuleManager::theInstance->legMotionCombinator.update(_theLegJointRequest);
        Blackboard::getInstance().updatedMap["LegJointRequest"] = true;
    }

    // HeadJointRequest
    if (!Blackboard::getInstance().updatedMap["HeadJointRequest"])
    {
        HeadJointRequest &_theHeadJointRequest = Blackboard::getInstance().headJointRequest;
        ModuleManager::theInstance->headMotionCombinator.update(_theHeadJointRequest);
        Blackboard::getInstance().updatedMap["HeadJointRequest"] = true;
    }

    // LegMotionSelection
    if (!Blackboard::getInstance().updatedMap["LegMotionSelection"])
    {
        LegMotionSelection &_theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
        // MotionSelector motionSelector;
        ModuleManager::theInstance->motionSelector.update(_theLegMotionSelection);
        Blackboard::getInstance().updatedMap["LegMotionSelection"] = true;
    }

    // RobotInfo
    if (!Blackboard::getInstance().updatedMap["RobotInfo"])
    {
        RobotInfo &_theRobotInfo = Blackboard::getInstance().robotInfo;
        NaoProvider::theInstance->update(_theRobotInfo);
        Blackboard::getInstance().updatedMap["RobotInfo"] = true;
    }

    // StiffnessSettings
    // if (!Blackboard::getInstance().updatedMap["StiffnessSettings"])
    // {
    //     Blackboard::getInstance().updatedMap["StiffnessSettings"] = true;
    // }

    // WalkingEngineOutput
    if (!Blackboard::getInstance().updatedMap["WalkingEngineOutput"])
    {
        WalkingEngineOutput &_theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
        // WalkingEngine walkingEngine;
        ModuleManager::theInstance->walkingEngine.update(_theWalkingEngineOutput);
        Blackboard::getInstance().updatedMap["WalkingEngineOutput"] = true;
    }

    // SpecialActionsOutput
    if (!Blackboard::getInstance().updatedMap["SpecialActionsOutput"])
    {
        SpecialActionsOutput &_theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;
        ModuleManager::theInstance->specialActions.update(_theSpecialActionsOutput);
        Blackboard::getInstance().updatedMap["SpecialActionsOutput"] = true;
    }
}

void MotionCombinator::update(JointRequest &jointRequest)
{
    update();
    
    specialActionOdometry += theSpecialActionsOutput.odometryOffset;

    MotionUtilities::copy(theHeadJointRequest, jointRequest, theStiffnessSettings, Joints::headYaw, Joints::headPitch);
    MotionUtilities::copy(theArmJointRequest, jointRequest, theStiffnessSettings, Joints::firstArmJoint, Joints::rHand);
    MotionUtilities::copy(theLegJointRequest, jointRequest, theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);

    ASSERT(jointRequest.isValid());

    Pose2f odometryOffset;

    // std::cout << "targetMotion:      " << theLegMotionSelection.targetMotion << std::endl;
    // std::cout << "ratio:             " << theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] << std::endl;
    // Find fully active motion and set MotionInfo
    if (theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] == 1.f)
    {
        // default values
        motionInfo.motion = theLegMotionSelection.targetMotion;
        motionInfo.isMotionStable = true;
        motionInfo.upcomingOdometryOffset = Pose2f();

        switch (theLegMotionSelection.targetMotion)
        {
        case MotionRequest::walk:
            odometryOffset = theWalkingEngineOutput.odometryOffset;
            motionInfo.walkRequest = theWalkingEngineOutput.executedWalk;
            motionInfo.upcomingOdometryOffset = theWalkingEngineOutput.upcomingOdometryOffset;
            break;
        // case MotionRequest::kick:
        //     odometryOffset = theKickEngineOutput.odometryOffset;
        //     motionInfo.kickRequest = theKickEngineOutput.executedKickRequest;
        //     motionInfo.isMotionStable = theKickEngineOutput.isStable;
        //     break;
        case MotionRequest::specialAction:
            odometryOffset = specialActionOdometry;
            specialActionOdometry = Pose2f();
            motionInfo.specialActionRequest = theSpecialActionsOutput.executedSpecialAction;
            motionInfo.isMotionStable = theSpecialActionsOutput.isMotionStable;
            break;
        // case MotionRequest::getUp:
        //     motionInfo.isMotionStable = false;
        //     odometryOffset = theGetUpEngineOutput.odometryOffset;
        //     break;
        case MotionRequest::fall:
            motionInfo.isMotionStable = false;
            odometryOffset = Pose2f();
            break;
        case MotionRequest::stand:
        default:
            break;
        }
    }

    if (theRobotInfo.hasFeature(RobotInfo::zGyro))
    {
        if (theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen)
            odometryOffset.rotation = 0_deg; // postpone rotation change until being upright again
        else
            odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - odometryData.rotation);
    }
    else
        odometryOffset.rotation = Angle::normalize(odometryOffset.rotation + theFallDownState.odometryRotationOffset);

    if (useAccFusion && theLegMotionSelection.targetMotion == MotionRequest::walk)
        estimateOdometryOffset(odometryOffset.translation);

    odometryData += odometryOffset;

    if (!useStiffnessDirectly)
        applyDynamicStiffness(jointRequest);

    if (debugArms)
        debugReleaseArms(jointRequest);
    applyDamageConfig(jointRequest);
}

void MotionCombinator::update(OdometryData &odometryData)
{
    // not update() here
    odometryData = this->odometryData;

    Pose2f odometryOffset = odometryData;
    odometryOffset -= lastOdometryData;
    lastOdometryData = odometryData;
}

void MotionCombinator::applyDynamicStiffness(JointRequest &jointRequest) const
{
    auto dynamicStiffnessFunction = [&](Joints::Joint joint) {
        ASSERT(jointRequest.stiffnessData.stiffnesses[joint] <= 100);
        if (jointRequest.stiffnessData.stiffnesses[joint] == 0)
            return 0.f;

        const Angle dist = std::abs(jointRequest.angles[joint] - theJointAngles.angles[joint]);

        const Angle maxPos = static_cast<float>(101 - jointRequest.stiffnessData.stiffnesses[joint]) * 1_deg;
        const Angle maxPos_2 = maxPos / 2.f;

        static const float minStiffness = 0.1f;
        static const float stiffnessWorkspace = 1.f - 0.1f;
        static const float graphCompression = stiffnessWorkspace / 2.f;

        if (dist < maxPos_2)
            return minStiffness + sqr(dist / maxPos_2) * graphCompression;
        else if (dist < maxPos)
            return 1.f - sqr((dist - maxPos) / maxPos_2) * graphCompression;
        else
            return 1.f;
    };

    for (Joints::Joint joint = Joints::Joint(0); joint < Joints::NumOfJoint; joint = Joints::Joint(unsigned(joint) + 1))
    {
        jointRequest.stiffnessData.stiffnesses[joint] = static_cast<int>(100.f * dynamicStiffnessFunction(joint));
    }
}

void MotionCombinator::debugReleaseArms(JointRequest &jointRequest) const
{
    if (theKeyStates.pressed[theKeyStates.headFront] || theKeyStates.pressed[theKeyStates.headMiddle] || theKeyStates.pressed[theKeyStates.headRear])
        for (unsigned i = Joints::firstLeftArmJoint; i <= Joints::rHand; i++)
            jointRequest.stiffnessData.stiffnesses[i] = 0;
}

void MotionCombinator::applyDamageConfig(JointRequest &jointRequest) const
{
    for (Joints::Joint j : theDamageConfigurationBody.jointsToEraseStiffness)
        jointRequest.stiffnessData.stiffnesses[j] = 0;
}

void MotionCombinator::estimateOdometryOffset(Vector2f &offset)
{
    auto dynamicModel = [&](Vector3f &state) {
        Vector3f accCorrected = (theInertialData.orientation2D * (useFilteredAcc ? theInertialData.filteredAcc : theInertialData.acc));
        state += (accCorrected - Vector3f(0, 0, Constants::g_1000)) * Constants::motionCycleTime * 1000;
    };
    auto pseudoMeasurement = [&](const Vector3f &state) {
        return state.z();
    };
    auto realMeasurement = [&](const Vector3f &state) {
        return (state * Constants::motionCycleTime).head<2>();
    };
    odometryUKF.predict(dynamicModel, dynamicVariance.cwiseAbs2().asDiagonal());
    odometryUKF.update(0.f, pseudoMeasurement, sqr(pseudoVariance));
    odometryUKF.update<2>(offset, realMeasurement, measurementVariance.cwiseAbs2().asDiagonal());

    offset = odometryUKF.mean.head<2>() * Constants::motionCycleTime;
}
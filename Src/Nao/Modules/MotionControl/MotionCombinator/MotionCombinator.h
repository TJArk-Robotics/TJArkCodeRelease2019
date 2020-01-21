#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
// #include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Module/Blackboard.h"
 
class MotionCombinatorBase
{
public:
    /* REQUIRES */
    const ArmJointRequest &theArmJointRequest = Blackboard::getInstance().armJointRequest;
    const ArmMotionSelection &theArmMotionSelection = Blackboard::getInstance().armMotionSelection;
    const DamageConfigurationBody &theDamageConfigurationBody = Blackboard::getInstance().damageConfigurationBody;
    const FallDownState &theFallDownState = Blackboard::getInstance().fallDownState;
    // const GetUpEngineOutput &theGetUpEngineOutput = Blackboard::getInstance().getUpEngineOutput;
    const InertialData &theInertialData = Blackboard::getInstance().inertialData;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const KeyStates &theKeyStates = Blackboard::getInstance().keyStates;
    // const KickEngineOutput &theKickEngineOutput = Blackboard::getInstance().kickEngineOutput;
    const LegJointRequest &theLegJointRequest = Blackboard::getInstance().legJointRequest;
    const HeadJointRequest &theHeadJointRequest = Blackboard::getInstance().headJointRequest;
    const LegMotionSelection &theLegMotionSelection = Blackboard::getInstance().legMotionSelection;
    const RobotInfo &theRobotInfo = Blackboard::getInstance().robotInfo;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;
    const WalkingEngineOutput &theWalkingEngineOutput = Blackboard::getInstance().walkingEngineOutput;
    const JointRequest &theJointRequest = Blackboard::getInstance().jointRequest;
    const SpecialActionsOutput &theSpecialActionsOutput = Blackboard::getInstance().specialActionsOutput;

    /* PROVIDES */
    // JointRequest &_theJointRequest = Blackboard::getInstance().jointRequest;
    // MotionInfo &_theMotionInfo = Blackboard::getInstance().motionInfo;
    // OdometryData &_theOdometryData = Blackboard::getInstance().odometryData;

    unsigned recoveryTime = 70; /**< The number of frames to interpolate after emergency-stop. */
    bool useStiffnessDirectly = true;
    bool debugArms = false;
    bool useAccFusion = false;
    Vector3f dynamicVariance = Vector3f(1.f, 1.f, 1.f);
    Vector2f measurementVariance = Vector2f(1.f, 1.f);
    float pseudoVariance = 0.1f;
    bool useFilteredAcc = true;
};

class MotionCombinator : public MotionCombinatorBase
{
public:
    OdometryData odometryData;    /**< The odometry data. */
    MotionInfo motionInfo;        /**< Information about the motion currently executed. */
    Pose2f specialActionOdometry; /**< Workaround for accumulating special action odometry. */

    OdometryData lastOdometryData;
    JointRequest lastJointRequest;

    UKF<3> odometryUKF = UKF<3>(Vector3f::Zero()); /**< Estimates the speed of the robot based on walking engine output and accelerometer */

    void update();

    void update(JointRequest &jointRequest);
    void update(OdometryData &odometryData);
    void update(MotionInfo &motionInfo) { motionInfo = this->motionInfo; }

    void applyDynamicStiffness(JointRequest &jointRequest) const;

    void applyDamageConfig(JointRequest &jointRequest) const;
    void debugReleaseArms(JointRequest &jointRequest) const;

    void estimateOdometryOffset(Vector2f &offset);
};

#pragma once

#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "ArmKeyFrameMotion.h"

#include "Tools/Module/Blackboard.h"

class ArmKeyFrameEngineBase
{
public:
    /* REQUIRES */
    const ArmMotionSelection &theArmMotionSelection = Blackboard::getInstance().armMotionSelection;
    const RobotInfo &theRobotInfo = Blackboard::getInstance().robotInfo;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const StiffnessSettings &theStiffnessSettings = Blackboard::getInstance().stiffnessSettings;

    /* USES */
    const MotionInfo &theMotionInfo = Blackboard::getInstance().motionInfo;

    /* PROVIDES */
    // ArmKeyFrameEngineOutput &theArmKeyFrameEngineOutput = Blackboard::getInstance().armKeyFrameEngineOutput;

    std::vector<ArmKeyFrameMotion> allMotions; /**< contains the existing arm motions */

    ArmKeyFrameEngineBase()
    {
        // ArmKeyFrameMotion 0
        ArmKeyFrameMotion armKeyFrameMotion0;
        armKeyFrameMotion0.id = ArmKeyFrameRequest::useDefault;

        ArmKeyFrameMotion::ArmAngles state0;
        state0.angles.push_back(1.5707f);
        state0.angles.push_back(0.2007f);
        state0.angles.push_back(-1.5707f);
        state0.angles.push_back(-0.2007f);
        state0.angles.push_back(-90_deg);
        state0.angles.push_back(0.f);
        state0.stiffness.push_back(80);
        state0.stiffness.push_back(80);
        state0.stiffness.push_back(80);
        state0.stiffness.push_back(80);
        state0.stiffness.push_back(80);
        state0.stiffness.push_back(80);
        state0.steps = 40;

        armKeyFrameMotion0.states.push_back(state0);
        allMotions.push_back(armKeyFrameMotion0);

        // ArmKeyFrameMotion 2
        ArmKeyFrameMotion armKeyFrameMotion1;
        armKeyFrameMotion1.id = ArmKeyFrameRequest::back;
        // state 1
        ArmKeyFrameMotion::ArmAngles state1;
        state1.angles.push_back(119.5_deg);
        state1.angles.push_back(10.0_deg);
        state1.angles.push_back(75.0_deg);
        state1.angles.push_back(-10.0_deg);
        state1.angles.push_back(-90_deg);
        state1.angles.push_back(0);
        state1.stiffness.push_back(90);
        state1.stiffness.push_back(60);
        state1.stiffness.push_back(80);
        state1.stiffness.push_back(90);
        state1.stiffness.push_back(90);
        state1.stiffness.push_back(90);
        state1.steps = 80;

        // state 2
        ArmKeyFrameMotion::ArmAngles state2;
        state2.angles.push_back(119.5_deg);
        state2.angles.push_back(-18.9_deg);
        state2.angles.push_back(75.0_deg);
        state2.angles.push_back(-18.33_deg);
        state2.angles.push_back(-90_deg);
        state2.angles.push_back(0);
        state2.stiffness.push_back(90);
        state2.stiffness.push_back(60);
        state2.stiffness.push_back(80);
        state2.stiffness.push_back(90);
        state2.stiffness.push_back(90);
        state2.stiffness.push_back(90);
        state2.steps = 20;

        // state 3
        ArmKeyFrameMotion::ArmAngles state3;
        state3.angles.push_back(119.5_deg);
        state3.angles.push_back(-18.9_deg);
        state3.angles.push_back(75.0_deg);
        state3.angles.push_back(-18.33_deg);
        state3.angles.push_back(-90_deg);
        state3.angles.push_back(0);
        state3.stiffness.push_back(90);
        state3.stiffness.push_back(60);
        state3.stiffness.push_back(80);
        state3.stiffness.push_back(90);
        state3.stiffness.push_back(90);
        state3.stiffness.push_back(90);
        state3.steps = 10;

        // state 4
        ArmKeyFrameMotion::ArmAngles state4;
        state4.angles.push_back(119.5_deg);
        state4.angles.push_back(-18.9_deg);
        state4.angles.push_back(75.0_deg);
        state4.angles.push_back(-18.33_deg);
        state4.angles.push_back(-90_deg);
        state4.angles.push_back(0);
        state4.stiffness.push_back(20);
        state4.stiffness.push_back(60);
        state4.stiffness.push_back(20);
        state4.stiffness.push_back(90);
        state4.stiffness.push_back(90);
        state4.stiffness.push_back(90);
        state4.steps = 3;

        armKeyFrameMotion1.states.push_back(state1);
        armKeyFrameMotion1.states.push_back(state2);
        armKeyFrameMotion1.states.push_back(state3);
        armKeyFrameMotion1.states.push_back(state4);

        allMotions.push_back(armKeyFrameMotion1);
    }
};

class ArmKeyFrameEngine : public ArmKeyFrameEngineBase
{
public:
    void update();
    void update(ArmKeyFrameEngineOutput &armkeyFrameEngineOutput);

    ArmKeyFrameEngine();

    const ArmKeyFrameRequest &theArmKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;

    /**
     * Class to encapsulate state information about one arm
     */
    class Arm
    {
    public:
        Arms::Arm id;                                    /**< ID of the motion this arm currently executes */
        int firstJoint;                                  /**< Index of the first joint belonging to this arm within the JointAngles' array */
        unsigned stateIndex;                             /**< Index to identify the current set of angles the arm should head to within its currentMotion */
        int interpolationTime;                           /**< Time in motion frames how long current motion is active */
        bool isLastMotionFinished;                       /**< Whether this arm currently performs a motion */
        bool fast;                                       /**< If set to true, current motion will be performed without interpolation */
        bool wasActive;                                  /**< If the arm key frame engine was active last frame */
        ArmKeyFrameMotion::ArmAngles interpolationStart; /**< Set of angles at which interpolation towards the next state began */
        ArmKeyFrameMotion currentMotion;                 /**< The motion which is currently performed by this arm. Contains the actual target states in order they should be reached */
        Arm(Arms::Arm id = Arms::left, int firstJoint = 2) : id(id), firstJoint(firstJoint), stateIndex(0), interpolationTime(0), isLastMotionFinished(true), wasActive(false)
        {
        }

        void startMotion(const ArmKeyFrameMotion &motion, bool fast, const JointAngles &currentJoints)
        {
            stateIndex = 0;
            interpolationTime = 1;
            isLastMotionFinished = false;
            currentMotion = motion;
            this->fast = fast;

            for (unsigned i = 0; i < interpolationStart.angles.size(); ++i)
            {
                interpolationStart.angles[i] = currentJoints.angles[firstJoint + i];
                if (id == Arms::right &&
                    (i + Joints::rShoulderPitch == Joints::rShoulderRoll ||
                     i + Joints::rShoulderPitch == Joints::rElbowYaw ||
                     i + Joints::rShoulderPitch == Joints::rElbowRoll ||
                     i + Joints::rShoulderPitch == Joints::rWristYaw))
                    interpolationStart.angles[i] *= -1.f;
            }
        }
    };

    Arm arms[2];                             /**< There are two arms :) */
    ArmKeyFrameMotion::ArmAngles defaultPos; /**< Default position of an arm. Will be read from configuration */

    /**
     * Performs the update step for a single arm. Checks whether a new motion for the provided arm is
     * possible or continues the currently active motion. If a new motion is possible, the desired
     * ArmKeyFrameMotion to perform is identified and then executed.
     *
     * @ param arm For which arm current update step is performed
     * @ param armKeyFrameEngineOutput Output representation to be filled
    */
    void updateArm(Arm &arm, ArmKeyFrameEngineOutput &armKeyFrameEngineOutput);

    /**
     * Performs the next interpolation step for the provided arm.
     * @ param arm The arm
     * @ param target Target angles for the interpolation. Starting point for interpolation is retrieved
     *      from the Arm instance itself.
     * @ param time Current interpolation time int motion steps
     * @ param result Will contain the four interpolated result angles+stiffness to be set.
     */
    void createOutput(Arm &arm, ArmKeyFrameMotion::ArmAngles target, int &time, ArmKeyFrameMotion::ArmAngles &result);

    /**
     * For a given arm, updates the engine's current output with the calculated target angles.
     * @ param arm The arm
     * @ param armKeyFrameEngineOutput Output representation to be filled
     * @ param values Angles+stiffness to be set for this arm during this motion frame
     */
    void updateOutput(Arm &arm, ArmKeyFrameEngineOutput &armKeyFrameEngineOutput, ArmKeyFrameMotion::ArmAngles &values);
};
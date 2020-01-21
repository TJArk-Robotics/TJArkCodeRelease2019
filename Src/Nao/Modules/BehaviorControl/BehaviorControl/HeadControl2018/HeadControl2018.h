#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Blackboard.h"
#include "CABSL/Cabsl.h"

class HeadControl2018Base
{
public:
    /* REQUIRES */
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;
    const GroundContactState &theGroundContactState = Blackboard::getInstance().groundContactState;
    const HeadMotionEngineOutput &theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;

    /* MODIFIES */
    HeadMotionRequest &theHeadMotionRequest = Blackboard::getInstance().headMotionRequest;
    HeadControlMode &theHeadControlMode = Blackboard::getInstance().headControlMode;
};

class HeadControl2018 : public HeadControl2018Base, public Cabsl<HeadControl2018>
{
#include "Modules/BehaviorControl/BehaviorControl/Options/HeadControl/LookForward.h"
#include "Modules/BehaviorControl/BehaviorControl/Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Modules/BehaviorControl/BehaviorControl/Options/HeadControl/LookLeftAndRight.h"

public:
    HeadControl2018();
    void execute();
    ActivationGraph activationGraph;
};
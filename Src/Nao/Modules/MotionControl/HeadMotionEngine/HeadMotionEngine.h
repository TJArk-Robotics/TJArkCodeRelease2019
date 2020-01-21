#pragma once

#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Blackboard.h"

class HeadMotionEngineBase
{
public:
    /* REQUIRES */
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;
    const GroundContactState &theGroundContactState = Blackboard::getInstance().groundContactState;
    const HeadAngleRequest &theHeadAngleRequest = Blackboard::getInstance().headAngleRequest;
    const HeadLimits &theHeadLimits = Blackboard::getInstance().headLimits;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;

    /* PROVIDES */
    // HeadMotionEngineOutput &_theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;

    int stopAndGoModeFrequenzy = 800;
};

class HeadMotionEngine : public HeadMotionEngineBase
{
public:
    Angle requestedPan;
    Angle requestedTilt;
    Vector2f lastSpeed;

    HeadMotionEngine();
    void update();
    void update(HeadMotionEngineOutput &headMotionEngineOutput);
};
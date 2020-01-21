#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include <vector>
#include <array>

class ArmKeyFrameEngineOutput : public ArmJointRequest
{
public:
    class Arm
    {
    public:
        bool isFree = true;                                                        /**< The arm is ready to release by key fram engine */
        ArmKeyFrameRequest::ArmKeyFrameID motion = ArmKeyFrameRequest::useDefault; /**< The arm motion being executed. */
    };
    std::array<Arm, Arms::NumOfArm> arms;
};
#pragma once

#include "ArmKeyFrameRequest.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Math/Eigen.h"

#include <array>

/**
 * @struct ArmMotionRequest
 * A struct that represents the arm motions that can be requested from the robot.
 */
class ArmMotionRequest
{
public:
    enum ArmRequest
    {
        none,
        keyFrame,

        NumOfArmRequest
    };

    ArmMotionRequest() { armMotion[Arms::left] = armMotion[Arms::right] = none; }

    std::array<ArmMotionRequest::ArmRequest, Arms::NumOfArm> armMotion;
    ArmKeyFrameRequest armKeyFrameRequest;
};

class ArmMotionInfo : public ArmMotionRequest
{
};
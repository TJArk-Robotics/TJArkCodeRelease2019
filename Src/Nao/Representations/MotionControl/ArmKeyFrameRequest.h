/**
 * Request for the key frame engine.
 * @author <a href="mailto:simont@tzi.de>Simon Taddiken</a>
 */

#pragma once 

#include "Tools/RobotParts/Arms.h"

#include <array>

class ArmKeyFrameRequest
{
public:
    enum ArmKeyFrameID
    {
        useDefault,
        back,
        reverse,
        NumOfArmKeyFrameID
    };

    class Arm
    {
    public:
        ArmKeyFrameRequest::ArmKeyFrameID motion = ArmKeyFrameRequest::useDefault;
        bool fast = false;
    };

    std::array<Arm, Arms::NumOfArm> arms;
};
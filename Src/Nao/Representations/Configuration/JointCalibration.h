#pragma once

#include "Tools/Math/BHMath.h"
#include "Tools/RobotParts/Joints.h"
#include <array>

class JointCalibration
{
public:
    JointCalibration()
    {
        offsets.fill(0_deg);
    }

    std::array<Angle, Joints::NumOfJoint> offsets; /**< Information on the calibration of all joints. */
};
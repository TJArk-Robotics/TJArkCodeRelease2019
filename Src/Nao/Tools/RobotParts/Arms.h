#pragma once

#include "Tools/Math/BHMath.h"

namespace Arms
{
enum Arm
{
    left,
    right,
    NumOfArm
};

static const unsigned bothArmsEnumSet = bit(left) | bit(right);
} // namespace Arms

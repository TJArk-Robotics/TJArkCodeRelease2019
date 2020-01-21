#pragma once

#include "Tools/Math/Pose3f.h"

class TorsoMatrix : public Pose3f
{
public:
    Pose3f offset;        /**< The estimated offset (including odometry) from last torso matrix to this one. (relative to the torso) */
    bool isValid = false; /**< Matrix is only valid if robot is on ground. */
    bool leftSupportFoot; /**< Guess whether the left foot is the support foot. */
};
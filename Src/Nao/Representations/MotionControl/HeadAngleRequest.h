#pragma once

#include "Tools/Math/Angle.h"

class HeadAngleRequest
{
public:
    Angle pan = 0_deg;          /**< Head pan target angle. */
    Angle tilt = 0_deg;         /**< Head tilt target angle. */
    Angle speed = 1_deg;        /**< Maximum joint speed to reach target angles in rad/s. */
    bool stopAndGoMode = false; /**< The Head will slow down and stop every HeadMotionEngine.stopAndGoModeFrequenzy milliseconds */
};
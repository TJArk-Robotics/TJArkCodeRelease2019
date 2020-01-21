/**
 * @file Representations/MotionControl/HeadMotionRequest.h
 * This file declares a struct that represents the requested head motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct HeadMotionRequest
 * A struct that represents the requested head motion.
 */
class HeadMotionRequest
{
public:
    enum Mode
    {
        panTiltMode,        // Use pan, tilt and speed
        targetMode,         // A target relative to the center of the hip. Use target and speed
        targetOnGroundMode, // Use target and speed.

        NumOfMode
    };

    enum CameraControlMode
    {
        autoCamera,
        lowerCamera,
        upperCamera,

        NumOfCameraControlMode
    };

    Mode mode = panTiltMode;                           /**< The active head motion mode. */
    CameraControlMode cameraControlMode = lowerCamera; /**< The active camera control mode. */
    Angle pan = 0_deg;                                 /**< Head pan target angle in radians. */
    Angle tilt = 0_deg;                                /**< Head tilt target angle in radians. */
    Angle speed = 1_deg;                               /**< Maximum joint speed to reach target angles in radians/s. */
    Vector3f target;                                   /**< Look at target relative to the robot. */
    bool stopAndGoMode = false;                        /**< The Head will slow down and stop every HeadMotionEngine.stopAndGoModeFrequenzy milliseconds */
};

class TeamHeadControlState
{
public:
    bool checkBall = false;
    bool usesActiveVision = false;
};
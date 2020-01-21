#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Tools/Math/Pose2f.h"

class SpecialActionsOutput : public JointRequest
{
public:
    Pose2f odometryOffset;                      /**< The body motion performed in this step. */
    bool isLeavingPossible;                     /**< Is leaving the motion module possible now? */
    bool isArmLeavingAllowed;                   /**< Is leaving the motion module only for the arms allowed*/
    bool isMotionStable;                        /**< Is the position of the camera directly related to the kinematic chain of joint angles? */
    SpecialActionRequest executedSpecialAction; /**< The special action currently executed. */
};
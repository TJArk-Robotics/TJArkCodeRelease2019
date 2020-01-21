#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"

class KickEngineOutput : public JointRequest
{
public:
    Pose2f odometryOffset;           /**< The body motion performed in this step. */
    bool isLeavingPossible = true;   /**< Is leaving the motion module possible now? */
    bool isStable = true;            /**< Is motion currently stable? */
    bool inAction = false;           /**< Is a motion chain currently stable? */
    bool wasValid = true;            /**< Is a motion chain currently stable? */
    KickRequest executedKickRequest; /**< The kick request that is currently in execution. */
    Pose3f desiredFootPosition;      /**< Which Position shoudl be reached */
};
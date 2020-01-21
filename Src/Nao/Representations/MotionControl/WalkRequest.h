/**
 * @file Representations/MotionControl/WalkRequest.h
 * This file declares a struct that represents a walk request.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author Colin Graf
 */

#pragma once

#include "Representations/Configuration/WalkKicks.h"
#include "Tools/Math/Pose2f.h"

class WalkRequest
{
public:
    using WalkKickRequest = WalkKickVariant;

    enum Mode
    {
        absoluteSpeedMode,
        relativeSpeedMode,
        targetMode,
        runUpMode,
        NumOfMode
    };

    bool isValid() const
    {
        return !std::isnan(static_cast<float>(speed.rotation)) && !std::isnan(speed.translation.x()) && !std::isnan(speed.translation.y()) &&
               (mode != targetMode || (!std::isnan(static_cast<float>(target.rotation)) && !std::isnan(target.translation.x()) && !std::isnan(target.translation.y())));
    }

    Mode mode = absoluteSpeedMode;
    Pose2f speed;
    Pose2f target;
    WalkKickRequest walkKickRequest;
};
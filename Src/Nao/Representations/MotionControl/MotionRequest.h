/**
 * @file Representations/MotionControl/MotionRequest.h
 * This file declares a struct that represents the motions that can be requested from the robot.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "SpecialActionRequest.h"
#include "WalkRequest.h"
#include "KickRequest.h"

/**
 * @struct MotionRequest
 * A struct that represents the motions that can be requested from the robot.
 */
class MotionRequest
{
public:
    enum Motion
    {
        walk,
        kick,
        specialAction,
        stand,
        getUp,
        fall,

        NumOfMotion
    };

    Motion motion = specialAction;
    SpecialActionRequest specialActionRequest;
    WalkRequest walkRequest;
    KickRequest kickRequest;
};
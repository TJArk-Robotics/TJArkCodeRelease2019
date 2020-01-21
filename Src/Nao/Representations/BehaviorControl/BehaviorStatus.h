/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Role.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
class BehaviorStatus
{
public:
    enum Activity
    {
        unknown,
        searchForBall,
        searchForBallAtRecentPosition,
        goToBall,
        takingPosition,
        kick,
        guardGoal,
        catchBall,
        standAndWait,
        gettingUp,
        turn,
        kickoff,

        demo,

        NumOfActivity
    };

    Role::RoleType role = Role::undefined;
    Activity activity = unknown;
    int passTarget = -1;
    Vector2f walkingTo = Vector2f::Zero();
    Vector2f shootingTo = Vector2f::Zero();
};
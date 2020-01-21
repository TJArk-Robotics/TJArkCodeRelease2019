#pragma once

#include <vector>

namespace GetUpMotions
{
enum GetUpMotion
{
    //all motions defined in getUpEngine.cfg
    frontFast,
    backFast,
    backGenu,
    recoverAndWait,
    recoverFast,
    recoverAfterBadBreakUp,
    recoverFromSide,
    recoverFromSideAfterJump,
    recoverFastKeeperJump,
    recoverBadArms,
    stand,
    fromGenuflect,
    fromSitDown,
    fromSumo,
    backFreeJoints,
    frontFreeJoints,
    sitDownFreeJoints,
    backVeryFast,
    NumOfGetUpMotion
};
using GetupMotionVector = std::vector<GetUpMotion>;
}
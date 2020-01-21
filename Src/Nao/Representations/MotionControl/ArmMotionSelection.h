#pragma once

#include "ArmKeyFrameRequest.h"
#include <array>

class ArmMotionSelection
{
public:
    enum ArmMotion
    {
        walkArms,
        kickArms,
        specialActionArms,
        standArms,
        getUpArms,
        fallArms,

        firstNonBodyMotion,
        keyFrameS = firstNonBodyMotion, //assert same order as ArmMotionRequest
        NumOfArmMotion
    };

    ArmMotionSelection()
    {
        targetArmMotion.fill(specialActionArms);
        armRatios[Arms::left].fill(0.f);
        armRatios[Arms::right].fill(0.f);
        armRatios[Arms::left][specialActionArms] = 1;
        armRatios[Arms::right][specialActionArms] = 1;
    }
    std::array<ArmMotion, Arms::NumOfArm> targetArmMotion;          /**< The armmotion that is the destination of the current arminterpolation per arm. */
    std::array<std::array<float, NumOfArmMotion>, Arms::NumOfArm> armRatios; /**< The current ratio of each armmotion in the final joint request, for each arm. */
    ArmKeyFrameRequest armKeyFrameRequest;                                   /**< The key frame request per arm, if it is an active armmotion. */
};
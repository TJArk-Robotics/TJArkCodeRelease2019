#pragma once

#include "MotionRequest.h"
#include <array>

class LegMotionSelection
{
public:
    enum ActivationMode
    {
        deactive,
        active,
        first,
        NumOfActivationMode
    };

    LegMotionSelection()
    {
        ratios.fill(0.f);
        ratios[MotionRequest::specialAction] = 1;
    }

    MotionRequest::Motion targetMotion = MotionRequest::specialAction; /**< The motion that is the destination of the current interpolation. */
    ActivationMode specialActionMode = active;                         /**< Whether and how the special action module is active. */
    std::array<float, MotionRequest::NumOfMotion> ratios;              /**< The current ratio of each motion in the final joint request. */
    SpecialActionRequest specialActionRequest;                         /**< The special action request, if it is an active motion. */
};
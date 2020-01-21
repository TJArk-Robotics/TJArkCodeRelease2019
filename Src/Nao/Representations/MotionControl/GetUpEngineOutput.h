// TODO
#pragma once

#include "Representations/Configuration/GetupMotion.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"

class GetUpEngineOutput : public JointRequest
{
public:
    bool isLeavingPossible = true; /**< Is leaving the motion module possible now? */
    Pose2f odometryOffset;
    int tryCounter = 0;  /**< the number of unsuccessful tries */
    int lineCounter = 0; /**< line of the current motion */
    bool failBack = false;
    bool failFront = false;
    GetUpMotions::GetUpMotion name; /**< current motion */
    bool criticalTriggered = false; /**< motion needs to stop */
    bool optionalLine = false;      /**< are we in a optional line? */
    float theBalanceFloatY = 0.f;   /**< the balanceValue, that will get added on top of the joints for balancing forward and backward */
    float theBalanceFloatX = 0.f;   /**< the balance value, that will get added on top of the joints for balancing sideways */
};
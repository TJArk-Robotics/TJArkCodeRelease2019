/**
 * @file MotionInfo.h
 *
 * Description of currently executed motion
 */

#pragma once

#include "MotionRequest.h"

class MotionInfo : public MotionRequest
{
public:
    /** Helper method to avoid long and faulty expressions in many modules
     * @return true, if the MotionInfo is about a motion that equals standing
     */
    bool isStanding() const
    {
        return motion == MotionRequest::stand || 
            (motion == MotionRequest::specialAction &&
                (specialActionRequest.specialAction == SpecialActionRequest::stand || specialActionRequest.specialAction == SpecialActionRequest::standHigh));
    }

    /** Helper method to avoid long and faulty expressions in some modules
     * @return true, if the MotionInfo is about a motion that performs a kick
     */
    bool isKicking() const
    {
        return motion == MotionRequest::kick ||
            (motion == MotionRequest::walk && walkRequest.walkKickRequest.kickType != WalkKicks::none);
    }
    
    bool isMotionStable = false;
    Pose2f upcomingOdometryOffset;
};
/** Sets all members of the MotionRequest representation for executing a targetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, as a ratio of the maximum speeds [0..1].
 *  @param target Walking target, in mm and radians, relative to the robot.
 */
option(WalkToTarget, (void *)speedArg, (void *)targetArg)
{
    Pose2f* speed = (Pose2f *) speedArg;
    Pose2f* target = (Pose2f *) targetArg;

    /** Set the motion request. */
    initial_state(setRequest)
    {
        transition
        {
            if (theMotionInfo.motion == MotionRequest::walk)
                goto requestIsExecuted;
        }
        action
        {
            _theMotionRequest.motion = MotionRequest::walk;
            _theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
            _theMotionRequest.walkRequest.target = *target;
            _theMotionRequest.walkRequest.speed = *speed;
            _theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        }
    }

    /** The motion process has started executing the request. */
    target_state(requestIsExecuted)
    {
        transition
        {
            if (theMotionInfo.motion != MotionRequest::walk)
                goto setRequest;
        }
        action
        {
            _theMotionRequest.motion = MotionRequest::walk;
            _theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
            _theMotionRequest.walkRequest.target = *target;
            _theMotionRequest.walkRequest.speed = *speed;
            _theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        }
    }
}

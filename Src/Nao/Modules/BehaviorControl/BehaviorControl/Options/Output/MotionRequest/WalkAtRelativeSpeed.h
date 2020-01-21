/** Sets all members of the MotionRequest representation for executing a relativeSpeedMode-WalkRequest
 *  (i.e. Walk at a \c speed)
 *  @param speed Walking speeds, in the range [-1.f .. 1.f].
 *                e.g.  Pose2f(0.f, 1.f, 0.f) lets move the robot forward at full speed
 *                      Pose2f(0.f, 0.5f, 0.5f) lets move the robot diagonal at half of the possible speed
 *                      Pose2f(0.5f, 1.f, 0.f) lets move the robot in a circle
 */

option(WalkAtRelativeSpeed, (void*) speedArg)
{
    /* Get Parameters */
    Pose2f *speed = (Pose2f*) speedArg;

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
            _theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
            _theMotionRequest.walkRequest.speed = *speed;
            _theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        }
    }

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
            _theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
            _theMotionRequest.walkRequest.speed = *speed;
            _theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
        }
    }
}
option(Stand)
{
    /** Set the motion request */
    initial_state(setRequest)
    {
        transition
        {
            if (theMotionInfo.motion == MotionInfo::stand)
                goto requestIsExecuted;
        }
        action
        {
            // std::cout << "[INFO]: theMotionInfo.motion in Stand: " << theMotionInfo.motion << std::endl;
            _theMotionRequest.motion = MotionRequest::stand;
        }
    }

    target_state(requestIsExecuted)
    {
        transition
        {
            if (theMotionInfo.motion != MotionRequest::stand)
                goto setRequest;
        }
        action
        {
            _theMotionRequest.motion = MotionRequest::stand;
        }
    }
}
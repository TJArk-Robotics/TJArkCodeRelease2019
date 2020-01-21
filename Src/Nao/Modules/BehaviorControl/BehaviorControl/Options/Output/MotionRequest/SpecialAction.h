// /** Sets all members of the MotionRequest representation for executing a SpecialAction */
// option(SpecialAction, (void*) idArg, (bool)(false)mirror)
// {
//     SpecialActionRequest::SpecialActionID* id = (SpecialActionRequest::SpecialActionID*) idArg;
//     /** Set the motion request. */
//     initial_state(setRequest)
//     {
//         transition
//         {
//             if (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == *id && theMotionInfo.specialActionRequest.mirror == mirror)
//                 goto requestIsExecuted;
//         }
//         action
//         {
//             _theMotionRequest.motion = MotionRequest::specialAction;
//             _theMotionRequest.specialActionRequest.specialAction = *id;
//             _theMotionRequest.specialActionRequest.mirror = mirror;
//         }
//     }

//     /** The motion process has started executing the request. */
//     target_state(requestIsExecuted)
//     {
//         transition
//         {
//             std::cout << "[INFO] SpecialAction.h " << std::endl;
//             std::cout << "MotionRequest.motion:                             " << _theMotionRequest.motion << std::endl;
//             std::cout << "MotionRequest.specialActionRequest.specialAction: " << _theMotionRequest.specialActionRequest.specialAction << std::endl;
//             std::cout << "MotionRequest.specialActionRequest.mirror:        " << _theMotionRequest.specialActionRequest.mirror << std::endl;
//             std::cout << "SpecialActionID:                                  " << *id << std::endl << std::endl;
//             if (theMotionInfo.motion != MotionRequest::specialAction || theMotionInfo.specialActionRequest.specialAction != *id || theMotionInfo.specialActionRequest.mirror != mirror)
//                 goto setRequest;
//         }
//         action
//         {
//             _theMotionRequest.motion = MotionRequest::specialAction;
//             _theMotionRequest.specialActionRequest.specialAction = *id;
//             _theMotionRequest.specialActionRequest.mirror = mirror;
//         }
//     }
// }

option(SpecialAction, (SpecialActionRequest::SpecialActionID) id, (bool)(false)mirror)
{
    /** Set the motion request. */
    initial_state(setRequest)
    {
        transition
        {
            if (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == id && theMotionInfo.specialActionRequest.mirror == mirror)
                goto requestIsExecuted;
        }
        action
        {
            _theMotionRequest.motion = MotionRequest::specialAction;
            _theMotionRequest.specialActionRequest.specialAction = id;
            _theMotionRequest.specialActionRequest.mirror = mirror;
        }
    }

    /** The motion process has started executing the request. */
    target_state(requestIsExecuted)
    {
        transition
        {
            if (theMotionInfo.motion != MotionRequest::specialAction || theMotionInfo.specialActionRequest.specialAction != id || theMotionInfo.specialActionRequest.mirror != mirror)
                goto setRequest;
        }
        action
        {
            _theMotionRequest.motion = MotionRequest::specialAction;
            _theMotionRequest.specialActionRequest.specialAction = id;
            _theMotionRequest.specialActionRequest.mirror = mirror;
        }
    }
}

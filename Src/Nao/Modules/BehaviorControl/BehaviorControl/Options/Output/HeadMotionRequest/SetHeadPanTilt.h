option(SetHeadPanTilt, (float) pan, (float) tilt, (float)(pi) speed, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera, (bool)(false) stopAndGoMode)
{
    initial_state(setRequest)
    {
        transition
        {
            if (state_time > 200 && !theHeadMotionEngineOutput.moving)
                goto targetReached;
        }
        action
        {
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.cameraControlMode = camera;
            theHeadMotionRequest.pan = pan;
            theHeadMotionRequest.tilt = tilt;
            theHeadMotionRequest.speed = speed;
            theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
        }
    }

    target_state(targetReached)
    {
        transition
        {
            goto setRequest;
        }
        action
        {
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.cameraControlMode = camera;
            theHeadMotionRequest.pan = pan;
            theHeadMotionRequest.tilt = tilt;
            theHeadMotionRequest.speed = speed;
        }
    }
}
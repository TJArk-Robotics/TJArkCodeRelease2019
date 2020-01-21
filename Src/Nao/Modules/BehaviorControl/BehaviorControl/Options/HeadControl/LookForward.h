option(LookForward, (float)(0.f) tilt, (float)(0.f) pan)
{
    initial_state(lookForward)
    {
        action
        {
            SetHeadPanTilt(pan, tilt, 150_deg);
        }
    }
}
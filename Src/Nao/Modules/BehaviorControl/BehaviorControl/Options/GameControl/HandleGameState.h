option(HandleGameState)
{
    initial_state(ready)
    {
        transition
        {
            if (state_time > 1000.f)
                goto playing;
        }
        action
        {
            HeadControlMode(HeadControl::none);
            SetHeadPanTilt(0.f, 0.f, 150_deg);
            Stand();
        }
    }

    state(playing)
    {
        action
        {
            PlayingState();
        }
    }
}
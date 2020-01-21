option(LookLeftAndRight)
{
    initial_state(lookLeft)
    {
        transition
        {
            if (action_done)
                goto lookRight;
        }
        action
        {
            SetHeadPanTilt(0_deg + 30_deg, 0_deg, 100_deg);
        }
    }

    state(lookRight)
    {
        transition
        {
            if (action_done)
                goto lookLeft;
        }
        action
        {
            SetHeadPanTilt(0_deg - 30_deg, 0_deg, 100_deg);
        }
    }
}
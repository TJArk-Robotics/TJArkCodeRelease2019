option(HandlePenaltyState)
{
    initial_state(penalized)
    {
        transition
        {
            if (action_done)
                goto notPenalized;
        }
        action
        {
            SetHeadControlMode(HeadControl::lookForward);
            SpecialAction(SpecialActionRequest::standHigh);
            ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
        }
    }

    state(notPenalized)
    {
        transition
        {
            if (action_done)
                goto penalized;
        }
        action
        {
            HandleGameState();
        }
    }
}
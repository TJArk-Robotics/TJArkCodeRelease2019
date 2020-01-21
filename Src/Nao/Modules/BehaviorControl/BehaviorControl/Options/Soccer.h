option(Root)
{
    initial_state(playDead)
    {
        transition
        {
            if (action_done)
            {
                goto standUp;
            }
        }
        action
        {
            SpecialAction(SpecialActionRequest::playDead);
            ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
        }
    }

    state(standUp)
    {
        transition
        {
            if (action_done)
                goto playSoccer;
        }
        action
        {
            LookForward();
            Stand();
        }
    }

    state(playSoccer)
    {
        transition
        {
            if (action_done)
                goto waitForSecondButtonPress;
        }
        action
        {
            HandlePenaltyState();
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    /** The following two states check whether the chest button is quickly pressed another two times. */
    state(waitForSecondButtonPress)
    {
        transition
        {
            if (action_done) // chest button pressed and released for the second time
                goto waitForThirdButtonPress;
            else if (action_aborted) // too slow -> abort
                goto playSoccer;
        }
        action
        {
            HandlePenaltyState();
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    state(waitForThirdButtonPress)
    {
        transition
        {
            if (action_done) // chest button pressed and released for the third time
                goto sitDown;
            else if (action_aborted) // too slow -> abort
                goto playSoccer;
        }
        action
        {
            // HandlePenaltyState();
            ButtonPressedAndReleased(KeyStates::chest, 1000, 200);
        }
    }

    state(sitDown)
    {
        transition
        {
            if (action_done)
            {
                goto playDeadDoNotRecover;
            }
        }
        action
        {
            // SpecialActionRequest::SpecialActionID id = SpecialActionRequest::sitDown;
            // SpecialAction(&id);
            SpecialAction(SpecialActionRequest::sitDown);
        }
    }

    state(playDeadDoNotRecover)
    {
        transition
        {
            if (action_done)
                goto standUp;
        }
        action
        {
            // SpecialActionRequest::SpecialActionID id = SpecialActionRequest::playDead;
            // SpecialAction(&id);
            SpecialAction(SpecialActionRequest::playDead);
            ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
        }
    }
}

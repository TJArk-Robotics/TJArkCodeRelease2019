option(SetHeadControlMode, (HeadControl::Mode) mode)
{
    initial_state(set)
    {
        action
        {
            theHeadControlMode = mode;
        }
    }
}
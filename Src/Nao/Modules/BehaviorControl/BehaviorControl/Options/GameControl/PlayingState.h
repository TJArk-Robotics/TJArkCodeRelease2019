option(PlayingState)
{
    initial_state(demo)
    {
        transition
        {
            if (state_time > 15000.f)
            {
                std::cout << "[INFO] Go to Stand\n" << std::endl;
                goto stand;
            }
        }
        action
        {
            Pose2f speed(0.3f, 0.6f, 0.6f);
            Pose2f target(0.f, 1000.f, 0.f);
            // WalkAtRelativeSpeed(&speed);
            WalkToTarget(&speed, &target);
            LookForward();
        }
    }

    state(stand)
    {
        action
        {
            Stand();
        }
    }
}
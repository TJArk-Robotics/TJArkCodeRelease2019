#pragma once

class FallDownState
{
public:
    enum State
    {
        pickedUp,
        upright,
        staggering,
        falling,
        fallen,
        squatting,
        NumOfState
    };

    enum Direction
    {
        none,
        front,
        left,
        back,
        right,
        NumOfDirection
    };

    State state = pickedUp;     /**< Current state of the robot's body. */
    Direction direction = none; /**< The robot is falling / fell into this direction. */
    float odometryRotationOffset = 0;
};
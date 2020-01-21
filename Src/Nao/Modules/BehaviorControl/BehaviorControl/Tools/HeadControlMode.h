#pragma once

class HeadControl
{
public:
    enum Mode
    {
        none,
        off,
        lookActive,
        lookActiveWithBall,
        lookActiveWithOwnBall,
        lookActiveWithoutBall,
        lookAtBall,
        lookAtBallMirrored,
        lookAtOwnBall,
        lookAtOwnBallMirrored,
        lookAtGlobalBall,
        lookAtGlobalBallMirrored,
        lookForward,
        lookLeftAndRight,
        lookRightAndLeft,

        NumOfMode
    };
};

using HeadControlMode = HeadControl::Mode;
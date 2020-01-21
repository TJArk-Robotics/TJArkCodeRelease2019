#pragma once

#include "CABSL/Cabsl.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Module/Blackboard.h"
#include <iostream>

class Soccer : public Cabsl<Soccer>
{
public:
    Soccer();

    void execute();

    /** Requirment */
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;
    const MotionInfo &theMotionInfo = Blackboard::getInstance().motionInfo;
    const KeyStates &theKeyStates = Blackboard::getInstance().keyStates;
    const HeadMotionEngineOutput &theHeadMotionEngineOutput = Blackboard::getInstance().headMotionEngineOutput;

    /* Provides */
    MotionRequest &_theMotionRequest = Blackboard::getInstance().motionRequest;
    HeadMotionRequest &theHeadMotionRequest = Blackboard::getInstance().headMotionRequest;
    HeadControlMode &theHeadControlMode = Blackboard::getInstance().headControlMode;

#include "Options.h"

    ActivationGraph activationGraph;
};
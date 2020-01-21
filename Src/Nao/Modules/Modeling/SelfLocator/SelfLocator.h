#pragma once

#include "Tools/Module/Blackboard.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

class SelfLocatorBase
{
public:
    /* REQUIRES */
    // const cv::Mat theImageTop = Blackboard::getInstance().usingImage;
    const ArucoMarker theArucoMarker = Blackboard::getInstance().arucoMarker;
    

    /* PROVIDES */
    // RobotPose theRobotPose = Blackboard::getInstance().robotPose;
};

class SelfLocator : public SelfLocatorBase
{
public:
    void update();
    void update(RobotPose &robotPose);
};
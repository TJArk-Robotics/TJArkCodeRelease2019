#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include <opencv2/opencv.hpp>
#include "Platform/Timer.h"

class LowerCognition : Adapter
{
public:
    LowerCognition(Blackboard *bb);
    ~LowerCognition() { cap.release(); }
    void tick();

    int getCameraID(CameraInfo::Camera cameraInfo);
    void setCameraSettings(int camID);

    cv::VideoCapture cap;
};
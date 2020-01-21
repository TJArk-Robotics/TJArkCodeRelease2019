#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include "Representations/Infrastructure/CameraInfo.h"

#include "Tools/ImageTransmission/ImageTransmissionServer.h"

#include <opencv2/opencv.hpp>

class Cognition : Adapter
{
public:
    Cognition(Blackboard *bb);
    ~Cognition() 
    { 
        camTop.release(); 
        camLower.release();
        server.socketDisconnect();
    }
    void tick();

    cv::VideoCapture camTop;
    cv::VideoCapture camLower;
    ImageTransmissionServer server;

    int getCameraID(CameraInfo::Camera cameraInfo);
    void encodeImage(cv::Mat &image, std::vector<uchar> &buff);
};

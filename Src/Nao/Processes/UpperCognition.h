#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>

class UpperCognition : Adapter
{
public:
    UpperCognition(Blackboard *bb);
    ~UpperCognition() { cap.release(); }
    void tick();

    int getCameraID(CameraInfo::Camera cameraInfo);
    void setCameraSettings(int camID);
    void updateImageToBlackboard(const cv::Mat &frame);
    void updateImageToDebug(const cv::Mat &frame);

    bool detect(cv::Mat &im);

    /* reset blackboard updatedMap to false to make sure all representations are not updated 
     * before update();
     */
    void resetUpdate();
    
public:
    cv::VideoCapture cap;
};
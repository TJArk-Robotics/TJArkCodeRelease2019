#include "LowerCognition.h"
#include "Tools/Module/Blackboard.h"

#include <unistd.h>
#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <thread>

int LowerCognition::getCameraID(CameraInfo::Camera cameraInfo)
{
    if (cameraInfo == CameraInfo::upper)
    {
        FILE *fp = NULL;
        char cmd[512];
        sprintf(cmd, "ls -l /dev/video-top 2>/dev/null");
        if ((fp = popen(cmd, "r")) != NULL)
        {
            fgets(cmd, sizeof(cmd), fp);
            pclose(fp);
        }
        char *p = cmd;
        while (*p != 0)
            p++;
        p = p - 2;
        int videoID = *(p) - '0';
        if ((videoID != 0) && (videoID != 1))
            videoID = -1;
        return videoID;
    }
    else
    {
        FILE *fp = NULL;
        char cmd[512];
        sprintf(cmd, "ls -l /dev/video-bottom 2>/dev/null");
        if ((fp = popen(cmd, "r")) != NULL)
        {
            fgets(cmd, sizeof(cmd), fp);
            pclose(fp);
        }
        char *p = cmd;
        while (*p != 0)
            p++;
        p = p - 2;
        int videoID = *(p) - '0';
        if ((videoID != 0) && (videoID != 1))
            videoID = -1;
        return videoID;
    }
}

void LowerCognition::setCameraSettings(int camID)
{
    FILE *fp = NULL;
    char cmd[512];
    sprintf(cmd, "v4l2-ctl -d %d -c focus_auto=0", camID);
    if ((fp = popen(cmd, "r")) != NULL)
    {
        fgets(cmd, sizeof(cmd), fp);
        pclose(fp);
    }
    usleep(100000);
    sprintf(cmd, "v4l2-ctl -d %d -c focus_absolute=0", camID);
    if ((fp = popen(cmd, "r")) != NULL)
    {
        fgets(cmd, sizeof(cmd), fp);
        pclose(fp);
    }
    usleep(100000);
}

LowerCognition::LowerCognition(Blackboard *bb) : Adapter(bb)
{
    int camLowID = getCameraID(CameraInfo::lower);
    std::cout << "[INFO] camera low id -----> " << camLowID << std::endl;

    if (camLowID != 0 && camLowID != 1)
    {
        std::cout << "[WARNING] camLowerID is not 0 or 1!" << std::endl;
        camLowID = 1;
    }

    cap.open(camLowID);

    while (!cap.open(camLowID))
    {
        usleep(100000);
        static int i = 0;
        if (i++ > 100)
        {
            std::cout << "[ERROR] Cannot open camTop!" << std::endl;
            break;
        }
    }

    if (!cap.isOpened())
    {
        blackboard->camLowOpen = false;
        std::cerr << "[ERROR] Cannot open video low!" << std::endl;
    }
    else
    {
        setCameraSettings(camLowID);
        blackboard->camLowOpen = true;
    }
}

void LowerCognition::tick()
{
    cv::Mat frame;
    if (cap.isOpened())
    {
        // acquireLock(visionMutex);
        cap >> frame;
        int writingImage = 0;
        if (writingImage == blackboard->newestImageLow)
            writingImage++;
        if (writingImage == blackboard->readingImageLow)
            if (++writingImage == blackboard->newestImageLow)
                ++writingImage;
        blackboard->imageLow[writingImage] = frame;
        usleep(1000);
        blackboard->newestImageLow = writingImage;
    }
}
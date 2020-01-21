#include "UpperCognition.h"
#include "Aruco/CmdLineParser.h"
#include "Aruco/dirreader.h"
#include "Aruco/tools.h"
#include "aruco/aruco.h"

#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Angle.h"
#include "Platform/Timer.h"
#include "unistd.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include <thread>

int UpperCognition::getCameraID(CameraInfo::Camera cameraInfo)
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

void UpperCognition::setCameraSettings(int camID)
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

UpperCognition::UpperCognition(Blackboard *bb) : Adapter(bb)
{
    int camTopID = getCameraID(CameraInfo::upper);
    std::cout << "[INFO] camera top id -----> " << camTopID << std::endl;

    if (camTopID != 0 && camTopID != 1)
    {
        std::cout << "[WARNING] camTopID is not 0 or 1!" << std::endl;
        camTopID = 0;
    }

    cap.open(camTopID);

    while (!cap.open(camTopID))
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
        blackboard->camTopOpen = false;
        std::cout << "[ERROR] Cannot open video top!" << std::endl;
    }
    else
    {
        setCameraSettings(camTopID);
        blackboard->camTopOpen = true;
    }
}

void UpperCognition::resetUpdate()
{
    std::map<std::string, bool> &_map = Blackboard::getInstance().updatedMap;
    std::map<std::string, bool>::iterator it;
    for (it = _map.begin(); it != _map.end(); it++)
    {
        it->second = false;
    }
}

void UpperCognition::updateImageToBlackboard(const cv::Mat &frame)
{
    int writingImage = 0;
    if (writingImage == Blackboard::getInstance().newestImageTop)
        writingImage++;
    if (writingImage == Blackboard::getInstance().readingImageTop)
        if (++writingImage == Blackboard::getInstance().newestImageTop)
            ++writingImage;
    Blackboard::getInstance().imageTop[writingImage] = frame;
    Blackboard::getInstance().newestImageTop = writingImage;
}

void UpperCognition::updateImageToDebug(const cv::Mat &frame)
{
    int writingImage = 0;
    if (writingImage == blackboard->newestImageTop)
        writingImage++;
    if (writingImage == blackboard->readingImageTop)
        if (++writingImage == blackboard->newestImageTop)
            ++writingImage;
    blackboard->imageTop[writingImage] = frame;
    blackboard->newestImageTop = writingImage;
}

void UpperCognition::tick()
{
    resetUpdate();

    cv::Mat frame;
    if (cap.isOpened())
    {
        // acquireLock(visionMutex);
        cap >> frame;
        updateImageToBlackboard(frame);

        RobotPose &_theRobotPose = Blackboard::getInstance().robotPose;
        ModuleManager::theInstance->selfLocator.update(_theRobotPose);

        cv::Mat &theUsingImage = Blackboard::getInstance().usingImage;

        updateImageToDebug(theUsingImage);
    }

    // std::cout << "RobotPose: " << std::endl;
    // Pose3f robotPose;
    // robotPose.rotation << 0.6124, -0.3536, 0.7071,
    //     0.6124, -0.3536, -0.7071,
    //     0.5000, 0.8660, 0;
    // std::cout << "rotation: \n"
    //           << robotPose.rotation << std::endl;
    // Vector3f eulerAngle = robotPose.rotation.eulerAngles(0, 1, 2);
    // std::cout << "eulerAngle: " << eulerAngle << std::endl;
    // std::cout << "euler: " << toDegrees(eulerAngle(0,0)) << ' ' << toDegrees(eulerAngle(1,0)) << ' ' << toDegrees(eulerAngle(2,0)) << std::endl;
}
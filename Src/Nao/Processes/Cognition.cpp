#include "Cognition.h"
#include "Aruco/CmdLineParser.h"
#include "Aruco/dirreader.h"
#include "Aruco/tools.h"
#include "aruco/aruco.h"

#include "Platform/Timer.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include <thread>

bool detect(cv::Mat &im)
{
    cv::Mat InImage;
    const std::string imagePath = "1.jpg";
    const std::string camParamPath = "out_m.yml";
    const std::string mdtectorParametersPath = "aruco_calibration_grid_board_a4.yml";
    const std::string dictionaryName = "ARUCO_MIP_36h12";
    const float markerSize = 0.038f;
    /* load image */
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(camParamPath);

    aruco::MarkerDetector MDetector;
    MDetector.loadParamsFromFile(mdtectorParametersPath);
    MDetector.setDictionary(dictionaryName, 0.f);

    // open image
    // InImage = cv::imread(imagePath);
    InImage = im;
    // ok, let's detect
    std::vector<aruco::Marker> Markers = MDetector.detect(InImage, CamParam, markerSize);

    if (Markers.size() == 0)
    {
        std::cout << "No Marker" << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        Markers[i].draw(InImage, cv::Scalar(0, 0, 255));
        std::cout << Markers[i].Tvec << std::endl;
    }
    // draw a 3d cube in each marker if there is 3d info
    if (CamParam.isValid() && markerSize != -1)
    {
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            aruco::CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam, 2);
            aruco::CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam, 2);
        }
    }

    // cv::imwrite("2.jpg", InImage);
    return true;
}

Cognition::Cognition(Blackboard *bb) : Adapter(bb)
{
    int camTopID = getCameraID(CameraInfo::upper);
    int camLowerID = getCameraID(CameraInfo::lower);

    std::cout << "[INFO] camera top id -----> " << camTopID << std::endl;
    std::cout << "[INFO] camera lower id ---> " << camLowerID << std::endl;

    if (camTopID != 0 && camTopID != 1)
    {
        std::cout << "[WARNING] camTopID is not 0 or 1!" << std::endl;
        camTopID = 0;
    }
    if (camLowerID != 0 && camLowerID != 1)
    {
        std::cout << "[WARNING] camLowerID is not 0 or 1!" << std::endl;
        camLowerID = 1;
    }
    while (!camTop.open(camTopID))
    {
        usleep(100000);
        static int i = 0;
        if (i++ > 100)
        {
            std::cout << "[ERROR] Cannot open camTop!" << std::endl;
            break;
        }
    }
    // while(!camLower.open(camLowerID))
    // {
    //     usleep(100000);
    //     static int i = 0;
    //     if (i++ > 100)
    //     {
    //         std::cout << "[ERROR] Cannot open camLowe!" << std::endl;
    //         break;
    //     }
    // }
}

int Cognition::getCameraID(CameraInfo::Camera cameraInfo)
{
    if (cameraInfo == CameraInfo::upper)
    {
        FILE *fp = NULL;
        char cmd[512];
        sprintf(cmd, "ls -l /dev/video-top 2>/dev/null; echo $?");
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
        sprintf(cmd, "ls -l /dev/video-bottom 2>/dev/null; echo $?");
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

void Cognition::tick()
{
    Timer timer1;
    Timer timer2;
    cv::Mat frameTop;
    cv::Mat frameLower;

    /**< Send Image to Debug. */

    if (camTop.isOpened())
    {
        camTop >> frameTop;
        int writingImage = 0;
        // if (writingActuators == naoBodyAccess.lbhData->newestActuators)
        //     ++writingActuators;
        // if (writingActuators == naoBodyAccess.lbhData->readingActuators)
        //     if (++writingActuators == naoBodyAccess.lbhData->newestActuators)
        //         ++writingActuators;
        if (writingImage == blackboard->newestImageTop)
            writingImage++;
        if (writingImage == blackboard->readingImageTop)
            if (++writingImage == blackboard->newestImageTop)
                ++writingImage;
        blackboard->imageTop[writingImage] = frameTop;
        usleep(1000);
        blackboard->newestImageTop = writingImage;
    }



    // std::vector<uchar> buff;
    // if (camTop.isOpened())
    // {
    //     camTop >> frameTop;
    //     encodeImage(frameTop, buff);
    // }
    
    // timer1.reset();

    // if (fork() == 0)
    // {
    //     while (camTop.isOpened())
    //     {
    //         camTop >> frameTop;
    //         if (!frameTop.empty())
    //         {
    //             if (server.transmit(frameTop) < 0)
    //                 break;
    //         }
    //     }
    //     server.socketDisconnect();
    // }

    // usleep(9000);

    // if (camLower.isOpened())
    // {
    //     camLower >> frameLower;

    //     static int i = 0;
    //     static int j = 0;
    //     i = (i+1) % 30;
    //     if (i == 0)
    //     {
    //         std::cout << "Lower: " << j++ << std::endl;
    //     }
    // }
    // timer2.reset();
    // camLower >> frameLower;
    // usleep(9000);
    // unsigned int time1 = timer1.elapsed_ms();
    // unsigned int time2 = timer2.elapsed_ms();
    // std::cout << "time1: " << time1 << std::endl;
    // std::cout << "time2: " << time2 << std::endl;
}

// jpg encode
void Cognition::encodeImage(cv::Mat &image, std::vector<uchar> &buff)
{
    // std::vector<uchar> buff;
    std::vector<int> param = std::vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 15; //default(95) 0-100

    cv::imencode(".jpg", image, buff, param);
}
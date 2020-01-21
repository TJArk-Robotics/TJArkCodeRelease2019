#include "Debug.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Platform/Timer.h"

#define TIME_START \
    Timer timer1;  \
    timer1.reset();
#define TIME_END std::cout << "time: " << timer1.elapsed_ms() << std::endl;

void Debug::tick()
{
    server.socketConnect(8000);

    while (true)
    {

        cv::Mat imageTop;
        blackboard->readingImageTop = blackboard->newestImageTop;
        int readingImageTop = blackboard->readingImageTop;
        imageTop = blackboard->imageTop[readingImageTop];

        cv::Mat imageLow;
        blackboard->readingImageLow = blackboard->newestImageLow;
        int readingImageLow = blackboard->readingImageLow;
        imageLow = blackboard->imageLow[readingImageLow];

        if (!imageTop.empty() && !imageLow.empty())
        {
            std::vector<uchar> buffTop;
            JPEGCompress(imageTop, buffTop, 50);

            std::vector<uchar> buffLow;
            JPEGCompress(imageLow, buffLow, 50);

            char *data;

            union SendLen {
                uint32_t len;
                uint8_t data[4];
            };

            SendLen sendLenTop;
            sendLenTop.len = buffTop.size();
            SendLen sendLenLow;
            sendLenLow.len = buffLow.size();
            SendLen sendLenSum;
            sendLenSum.len = sendLenTop.len + sendLenLow.len + 4;

            // header + sumNum + topNum + top + low
            int sendDataLength = 2 + 4 + 4 + sendLenTop.len + sendLenLow.len;

            data = new char[sendDataLength];

            int iit = 0;

            // 2 header
            data[iit++] = 'F';
            data[iit++] = 'F';

            // 4 sumNum
            data[iit++] = sendLenSum.data[0];
            data[iit++] = sendLenSum.data[1];
            data[iit++] = sendLenSum.data[2];
            data[iit++] = sendLenSum.data[3];

            // 4 topNum
            data[iit++] = sendLenTop.data[0];
            data[iit++] = sendLenTop.data[1];
            data[iit++] = sendLenTop.data[2];
            data[iit++] = sendLenTop.data[3];

            // top
            std::vector<uchar>::iterator itTop;
            
            for (itTop = buffTop.begin(); itTop != buffTop.end(); itTop++)
                data[iit++] = *itTop;

            // low
            std::vector<uchar>::iterator itLow;
            for (itLow = buffLow.begin(); itLow != buffLow.end(); itLow++)
                data[iit++] = *itLow;

            if (server.transmit(data, sendDataLength) < 0)
            {
                delete[] data;
                break;
            }

            delete[] data;
        }
    }

    server.socketDisconnect();
}

void Debug::JPEGCompress(const cv::Mat &src, std::vector<uchar> &buff, int quality)
{
    // std::vector<uchar> buff;
    std::vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(quality);
    cv::imencode(".jpg", src, buff, params);
}
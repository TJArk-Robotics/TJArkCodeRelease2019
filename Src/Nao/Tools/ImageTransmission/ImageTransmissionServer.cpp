#include "ImageTransmissionServer.h"

int ImageTransmissionServer::socketConnect(int PORT)
{
    socketFd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in serverSockAddr;
    serverSockAddr.sin_family = AF_INET;
    serverSockAddr.sin_port = htons(PORT);
    serverSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socketFd, (struct sockaddr*)&serverSockAddr, sizeof(serverSockAddr)) == -1)
    {
        perror("bind");
        return -1;
    }

    if (listen(socketFd, 5) == -1)
    {
        perror("listen");
        return -1;
    }

    struct sockaddr_in clientAddr;
    socklen_t len = sizeof(clientAddr);
    connFd = accept(socketFd, (struct sockaddr*)&clientAddr, &len);

    if (connFd < 0)
    {
        perror("connect\n");
        return -1;
    }
    else
    {
        printf("connect successful!\n");
        return 1;
    }

    close(socketFd);
}

void ImageTransmissionServer::socketDisconnect()
{
    close(socketFd);
    close(connFd);
}

int ImageTransmissionServer::transmit(cv::Mat& image)
{
    if (image.empty())
    {
        printf("empty image\n");
        return -1;
    }

    if (image.cols != IMAGE_WIDTH || image.rows != IMAGE_HEIGHT || image.type() != CV_8UC3)
    {
        printf("not satisfied image!\n");
        return -1;
    }

    for (int k = 0; k < PACKAGE_NUM; k++)
    {
        int num1 = IMAGE_HEIGHT / PACKAGE_NUM * k;
        for (int i = 0; i < IMAGE_HEIGHT / PACKAGE_NUM; i++)
        {
            int num2 = i * IMAGE_WIDTH * 3;
            uchar* ucdata = image.ptr<uchar>(i + num1);
            for (int j = 0; j < IMAGE_WIDTH * 3; j++)
            {
                data.buf[num2 + j] = ucdata[j];
            }
        }
        if (k == PACKAGE_NUM - 1)
            data.flag = 2;
        else
            data.flag = 1;

        if (send(connFd, (char *)(&data), sizeof(data), 0) < 0)
        {
            // printf("send image error: %s(errno: %d)\n", strerror(errno), errno);
            return -1;
        }
    }
}
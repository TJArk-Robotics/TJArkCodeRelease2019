#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>

const int PACKAGE_NUM = 2;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const int BLOCKSIZE = IMAGE_HEIGHT * IMAGE_WIDTH * 3 / PACKAGE_NUM;

struct sendBuf
{
    char buf[BLOCKSIZE];
    int flag;
};

class ImageTransmissionServer
{
public:
    ImageTransmissionServer() {}
    ~ImageTransmissionServer() { socketDisconnect(); }
    int socketFd;
    int connFd;
private:
    struct sendBuf data;
    int needRecv;
    int count;

public:
    // open socket
    int socketConnect(int PORT);
    // transmit image
    int transmit(cv::Mat& image);
    // close socket
    void socketDisconnect();
};
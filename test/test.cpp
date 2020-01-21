#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <csignal>
#include <opencv2/opencv.hpp>

static bool run = true;

static void sighandlerShutdown(int sig)
{
    if (run)
    {
        printf("Caught signal %i\nShutting down...\n", sig);
    }
    run = false;
}

class CameraInfo
{
public:
    enum Camera
    {
        upper,
        lower
    };
};

int main(int argc,char **argv)
{
    cv::VideoCapture cam;
    cam.open(0);
    cv::Mat frame;

    signal(SIGINT, sighandlerShutdown);
    signal(SIGTERM, sighandlerShutdown);

    while (cam.isOpened() && run)
    {
        cam >> frame;
        std::cout << "width: " << frame.size().width << " height: " << frame.size().height << std::endl;
        std::cout << std::endl;
    }

    cam.release();

    return EXIT_SUCCESS;
}
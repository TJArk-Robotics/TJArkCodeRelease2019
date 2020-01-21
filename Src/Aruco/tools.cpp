#include "tools.h"

namespace Aruco
{
cv::Mat __resize(const cv::Mat &in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}

Pose3f getMarkerLocation(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
    cv::Mat m33(3, 3, CV_32FC1);
    cv::Rodrigues(Rvec, m33);

    RotationMatrix rot;
    Vector3f vec;

    // rotation matrix 
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rot(i, j) = m33.at<float>(i, j);
        }
    }
    
    for (int i = 0; i < 3; i++)
    {
        vec(i, 0) = Tvec.ptr<float>(0)[i];
    } 

    Pose3f location(rot, vec);
    // location = location.inverse();
    return location;
}
}
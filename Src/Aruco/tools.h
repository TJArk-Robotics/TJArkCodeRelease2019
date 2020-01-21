#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <stdexcept>
#include <vector>

#include "aruco/aruco.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/Pose3f.h"

#include "Aruco/dirreader.h"
#include "Aruco/CmdLineParser.h"

namespace Aruco
{
cv::Mat __resize(const cv::Mat &in, int width);

Pose3f getMarkerLocation(const cv::Mat &Rvec, const cv::Mat &Tvec);
}
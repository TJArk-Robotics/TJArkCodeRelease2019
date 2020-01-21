#pragma once

#include "Tools/Math/Eigen.h"

class IMUCalibration
{
public:
    AngleAxisf rotation = AngleAxisf::Identity();
};
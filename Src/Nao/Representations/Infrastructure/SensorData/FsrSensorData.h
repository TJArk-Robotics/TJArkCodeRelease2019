#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Legs.h"
#include <array>

class FsrSensorData
{
public:
    FsrSensorData()
    {
        for (int leg = 0; leg < Legs::NumOfLeg; leg++)
        {
            pressures[leg].fill(SensorData::off);
            totals[leg] = 0.f;
        }
    }

    std::array<std::array<float, FsrSensors::NumOfFsrSensor>, Legs::NumOfLeg> pressures; /**< Values of the pressure sensors in each foot (in kg) */
    std::array<float, Legs::NumOfLeg> totals;                                                 /**< Total mass pressing on the left foot (in kg) */
};
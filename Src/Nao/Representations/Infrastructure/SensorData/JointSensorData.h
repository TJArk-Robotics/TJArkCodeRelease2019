#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Motion/SensorData.h"
#include <array>

class JointSensorData : public JointAngles
{
public:
    JointSensorData()
    {
        currents.fill(SensorData::off);
        temperatures.fill(0);
        status.fill(TemperatureStatus::regular);
    }

    enum TemperatureStatus
    {
        regular,
        hot,
        veryHot,
        criticallyHot,
        NumOfTemperatureStatus
    };

    std::array<short, Joints::NumOfJoint> currents;                                 /**< The currents of all motors. */
    std::array<unsigned char, Joints::NumOfJoint> temperatures;                     /**< The currents of all motors. */
    std::array<JointSensorData::TemperatureStatus, Joints::NumOfJoint> status; /**< The status of all motors. */
};
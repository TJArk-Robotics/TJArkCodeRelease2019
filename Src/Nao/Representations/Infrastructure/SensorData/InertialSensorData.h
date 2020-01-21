#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"

class InertialSensorData
{
public:
    Vector3a gyro = Vector3a::Zero();  /**< The change in orientation around the x-, y-, and z-axis (in radian/s). */
    Vector3f acc = Vector3f::Zero();   /**< The acceleration along the x-, y- and z-axis (in m/s^2). */
    Vector3a angle = Vector3a::Zero(); /**< The orientation of the torso (in rad). */
};
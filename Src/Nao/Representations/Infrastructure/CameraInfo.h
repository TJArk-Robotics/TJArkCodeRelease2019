#pragma once

#include "Tools/Math/Eigen.h"

class CameraInfo
{
public:
    enum Camera
    {
        upper,
        lower,
        NumOfCamera
    };

    /**
     * Intrinsic camera parameters: axis skew is modelled as 0 (90° perfectly orthogonal XY)
     * and the same has been modeled for focal axis aspect ratio; distortion is considering
     * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
     */
    float focalLength;
    float focalLengthInv; // (1/focalLength) used to speed up certain calculations
    float focalLenPow2;
    float focalLengthHeight;
    float focalLengthHeightInv;

    CameraInfo() { onRead(); };
    CameraInfo(Camera camera) : camera(camera) { onRead(); }

    void updateFocalLength();
    void onRead() 
    {
        if (camera == upper)
        {
            width = 640;
            height = 480;
            openingAngleWidth = 56.3_deg;
            openingAngleHeight = 43.7_deg;
            opticalCenter = Vector2f(0.5, 0.5);
        }
        if (camera == lower)
        {
            width = 640;
            height = 480;
            openingAngleWidth = 56.3_deg;
            openingAngleHeight = 43.7_deg;
            opticalCenter = Vector2f(0.5, 0.5);
        }
        updateFocalLength(); 
    }

    Camera camera;
    int width;
    int height;
    Angle openingAngleWidth;
    Angle openingAngleHeight;
    Vector2f opticalCenter;
};
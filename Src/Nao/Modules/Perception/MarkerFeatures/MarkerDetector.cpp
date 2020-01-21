#include "MarkerDetector.h"

#include "Aruco/CmdLineParser.h"
#include "Aruco/dirreader.h"
#include "Aruco/tools.h"
#include "aruco/aruco.h"

void MarkerDetector::update()
{
    // theUsingImage
    if (!Blackboard::getInstance().updatedMap["UsingImage"])
    {
        Blackboard::getInstance().readingImageTop = Blackboard::getInstance().newestImageTop;
        int readingImageTop = Blackboard::getInstance().readingImageTop;
        Blackboard::getInstance().usingImage = Blackboard::getInstance().imageTop[readingImageTop];
        Blackboard::getInstance().updatedMap["UsingImage"] = true;
    }
}

void MarkerDetector::update(ArucoMarker &arucoMarker)
{
    update();

    if (!theUsingImage.empty())
    {
        detect(theUsingImage, arucoMarker);
        arucoMarker.isValid = true;
    }
    else
    {
        arucoMarker.isValid = false;
    }
}

bool MarkerDetector::detect(cv::Mat &im, ArucoMarker &arucoMarker)
{
    cv::Mat InImage;
    const std::string imagePath = "1.jpg";
    const std::string camParamPath = "./Config/out_m.yml";
    const std::string mdtectorParametersPath = "./Config/aruco_calibration_grid_board_a4.yml";
    const std::string dictionaryName = "ARUCO_MIP_36h12";
    const float markerSize = 0.157f;
    /* load image */
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(camParamPath);

    aruco::MarkerDetector MDetector;
    MDetector.loadParamsFromFile(mdtectorParametersPath);
    MDetector.setDictionary(dictionaryName, 0.f);

    // open image
    // InImage = cv::imread(imagePath);
    InImage = im;
    // ok, let's detect
    std::vector<aruco::Marker> Markers = MDetector.detect(InImage, CamParam, markerSize);

    if (Markers.size() == 0)
    {
        // std::cout << "No Marker" << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        Markers[i].draw(InImage, cv::Scalar(0, 0, 255));
        std::cout << Markers[i].id << std::endl;

        // get marker location
        // camera coordinate:
        // x: head to lshoulder
        // y: body to head
        // z: lookforward
        Pose3f markerTocameraCoordinate = Aruco::getMarkerLocation(Markers[i].Rvec, Markers[i].Tvec);
        Pose3f cameraCoordinateToRobotCoordinate;
        // cameraCoordinateToRobotCoordinate.rotateZ(-90_deg).rotateX(-90_deg);
        cameraCoordinateToRobotCoordinate.rotateX(90_deg).rotateY(90_deg);
        Pose3f markerToCameraPos = cameraCoordinateToRobotCoordinate * markerTocameraCoordinate;

        // std::cout << "markerToCameraPos:" << std::endl;
        // std::cout << "translation: " << markerToCameraPos.translation << std::endl;

        if (Markers[i].id == 1)
        {
        }
        else
        {
            arucoMarker.markers[ArucoMarker::others].id = Markers[i].id;
            arucoMarker.markers[ArucoMarker::others].positionInCameraCoord = markerToCameraPos;
        }
    }
    // draw a 3d cube in each marker if there is 3d info
    if (CamParam.isValid() && markerSize != -1)
    {
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            // aruco::CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam, 2);
            // aruco::CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam, 2);
        }
    }

    im = InImage;
    return true;
    // cv::imwrite("2.jpg", InImage);
}
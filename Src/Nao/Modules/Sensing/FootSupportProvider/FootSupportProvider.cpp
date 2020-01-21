#include "FootSupportProvider.h"

// Provider
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"

FootSupportProvider::FootSupportProvider()
{
    weights[Legs::left][FsrSensors::fl] = weights[Legs::left][FsrSensors::bl] = outerWeight;
    weights[Legs::right][FsrSensors::fr] = weights[Legs::right][FsrSensors::br] = -outerWeight;
    weights[Legs::left][FsrSensors::fr] = weights[Legs::left][FsrSensors::br] = innerWeight;
    weights[Legs::right][FsrSensors::fl] = weights[Legs::right][FsrSensors::bl] = -innerWeight;
    for (int leg = 0; leg < Legs::NumOfLeg; leg++)
    {
        for (int sensor = 0; sensor < FsrSensors::NumOfFsrSensor; sensor++)
        {
            highestPressure[leg][sensor] = minPressure;
        }
    }
}

void FootSupportProvider::update()
{
    // FsrSensorData
    if (!Blackboard::getInstance().updatedMap["FsrSensorData"])
    {
        FsrSensorData &_theFsrSensorData = Blackboard::getInstance().fsrSensorData;
        NaoProvider::theInstance->update(_theFsrSensorData);
        Blackboard::getInstance().updatedMap["FsrSensorData"] = true;
    }
}

void FootSupportProvider::update(FootSupport &theFootSupport)
{
    update();
    float totalPressure = 0.f;
    float weightedSum = 0.f;
    for (int leg = 0; leg < Legs::NumOfLeg; leg++)
    {
        for (int sensor = 0; sensor < FsrSensors::NumOfFsrSensor; sensor++)
        {
            float pressure = std::min(maxPressure, theFsrSensorData.pressures[leg][sensor]);
            highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
            pressure /= highestPressure[leg][sensor];
            totalPressure += pressure;
            weightedSum += weights[leg][sensor] * pressure;
        }
    }
    if (std::abs(totalPressure) > 0.f)
    {
        float prevSupport = theFootSupport.support;
        theFootSupport.support = weightedSum / totalPressure;
        theFootSupport.switched = prevSupport * theFootSupport.support < 0.f;
    }
    else
    {
        theFootSupport.support = 0.f;
        theFootSupport.switched = false;
    }
}
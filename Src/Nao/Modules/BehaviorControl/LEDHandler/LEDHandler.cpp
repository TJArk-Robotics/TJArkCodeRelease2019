#include "LEDHandler.h"
#include <algorithm>

// Provider
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"

void LEDHandler::update()
{
    // SystemSensorData
    if (!Blackboard::getInstance().updatedMap["SystemSensorData"])
    {
        SystemSensorData &_theSystemSensorData = Blackboard::getInstance().systemSensorData;
        NaoProvider::theInstance->update(_theSystemSensorData);
        Blackboard::getInstance().updatedMap["SystemSensorData"] = true;        
    }
}

void LEDHandler::update(LEDRequest &ledRequest)
{
    update();
    for (int i = 0; i < ledRequest.NumOfLED; i++)
    {
        ledRequest.ledStates[i] = LEDRequest::off;
    }

    setHead(ledRequest);
}

void LEDHandler::setHead(LEDRequest &ledRequest)
{
    if (theSystemSensorData.batteryCharging)
    {
        for (LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
        {
            ledRequest.ledStates[i] = LEDRequest::off;
        }
        ++chargingLED %= (LEDRequest::numOfHeadLEDs * chargingLightSlowness);
        const LEDRequest::LED currentLED = headLEDCircle[chargingLED / chargingLightSlowness];
        const LEDRequest::LED nextLED = headLEDCircle[(chargingLED / chargingLightSlowness + 1u) % LEDRequest::numOfHeadLEDs];
        ledRequest.ledStates[currentLED] = LEDRequest::on;
        ledRequest.ledStates[nextLED] = LEDRequest::on;
    }
}
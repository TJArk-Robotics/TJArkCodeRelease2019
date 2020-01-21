/**
 * @file Platform/Nao/NaoBody.h
 * Declaration of a class for accessing the body of the nao via NaoQi/libbhuman
 * @author Colin Graf
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include <cstdio>

/**
 * @class NaoBody
 * Encapsulates communication with lola_connector
 */

class NaoBody
{
private:
    int writingActuators = -1; /**< The index of the opened exclusive actuator writing buffers. */

    FILE *fdCpuTemp = nullptr;

public:
    ~NaoBody();

    /**
     * Initializes access to lola_connector
     * @return whether the initialization was successful or not
     */
    bool init();

    /** Finalize access to lola_connector */
    void cleanup();

    /** Waits for a new set of sensor data */
    bool wait();

    /**
     * Activates the eye-blinking mode to indicate a crash.
     * @param termSignal The termination signal that was raised by the crash.
     */
    void setCrashed(int termSignal);

    /**
     * Accesses the id of the robot's head.
     * @return The name.
     */
    const char *getHeadId() const;

    /**
     * Accesses the id of the robot's body.
     * @return The name.
     */
    const char *getBodyId() const;

    RobotInfo::NaoVersion getHeadVersion();
    RobotInfo::NaoVersion getBodyVersion();
    RobotInfo::NaoType getHeadType();
    RobotInfo::NaoType getBodyType();

    /**
     * Accesses the sensor value buffer.
     * @return An array of sensor values. Ordered corresponding to lbhSensorNames of bhuman.h
     */
    float *getSensors();

    /**< Access the lastest data from the GameController. */
    const RoboCup::RoboCupGameControlData& getGameControlData() const;

    /**< Access CPU temperature sensor */
    float getCPUTemperature();

    /**< Determine status of wlan hardware. */
    bool getWlanStatus();

    /**
     * Accesses the actuator value buffer for writting.
     * @param actuator A reference to a variable to store a pointer to the actuator value buffer in.
     *                 Ordered corresponding to lbhActuatorNames of bhuman.h
     */
    void openActuators(float *&actuators);

    /**< Commits the actuator value buffer. */
    void closeActuators();

    /**< Sets the current team info that will be used by lola_connector */
    void setTeamInfo(int teamNumber, int teamColor, int playerNumber);

    void data();
};

#include "LoLAConnector/lola_connector.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include "LoLAConnector/joints.h"
#include "LoLAConnector/lola_frame.h"

//bhuman include
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>
#include "LoLAConnector/bhuman.h"
#include <iostream>
#include <chrono>

using namespace boost::asio;
using namespace std;
using namespace std::chrono;

//libgamectrl
#include <arpa/inet.h>
#include "UdpComm.h"
static const int BUTTON_DELAY = 30;             /**< Button state changes are ignored when happening in less than 30 ms. */
static const int GAMECONTROLLER_TIMEOUT = 2000; /**< Connected to GameController when packet was received within the last 2000 ms. */
static const int ALIVE_DELAY = 1000;            /**< Send an alive signal every 1000 ms. */
float ledRequest[lbhNumOfLedActuatorIds];       //Request LEDs
enum Button
{
    chest,
    leftFootLeft,
    leftFootRight,
    rightFootLeft,
    rightFootRight,
    numOfButtons
};

enum LED
{
    chestRed,
    chestGreen,
    chestBlue,
    leftFootRed,
    leftFootGreen,
    leftFootBlue,
    rightFootRed,
    rightFootGreen,
    rightFootBlue,
    numOfLEDs
};

static bool lola_shutdown = false;
std::chrono::high_resolution_clock::time_point lasttime;

static const float sitDownAngles[25] =
    {
        0.f,
        0.f,

        1.57f,
        0.09f,
        0.f,
        0.f,
        0.f,
        0.f,

        1.57f,
        -0.09f,
        0.f,
        0.f,
        -1.57f,
        0.f,

        0.f,
        0.f,
        -0.87f,
        2.16f,
        -1.18f,
        0.f,

        0.f,
        -0.87f,
        2.16f,
        -1.18f,
        0.f};

int memoryHandle;                      /**< The file handle of the shared memory. */
LBHData *data = (LBHData *)MAP_FAILED; /**< The shared memory. */
sem_t *sem = SEM_FAILED;               /**< The semaphore used to notify bhuman about new data. */
int CycleIndex;
RoboCupGameControlData gameCtrlDataForGC;

class GameCtrl
{
    UdpComm *udp;                                                                   /**< The socket used to communicate. */
    in_addr gameControllerAddress;                                                  /**< The address of the GameController PC. */
    const int *playerNumber;                                                        /** Points to where ALMemory stores the player number. */
    const int *teamNumberPtr;                                                       /** Points to where ALMemory stores the team number. The number be set to 0 after it was read. */
    const int *defaultTeamColour;                                                   /** Points to where ALMemory stores the default team color. */
    int teamNumber;                                                                 /**< The team number. */
    RoboCupGameControlData gameCtrlData;                                            /**< The local copy of the GameController packet. */
    uint8_t previousState;                                                          /**< The game state during the previous cycle. Used to detect when LEDs have to be updated. */
    uint8_t previousgamePhase;                                                      /**< The secondary game state during the previous cycle. Used to detect when LEDs have to be updated. */
    uint8_t previouskickingTeam;                                                    /**< The kick-off team during the previous cycle. Used to detect when LEDs have to be updated. */
    uint8_t previousTeamColour;                                                     /**< The team colour during the previous cycle. Used to detect when LEDs have to be updated. */
    uint8_t previousPenalty;                                                        /**< The penalty set during the previous cycle. Used to detect when LEDs have to be updated. */
    bool previousChestButtonPressed;                                                /**< Whether the chest button was pressed during the previous cycle. */
    bool previousLeftFootButtonPressed;                                             /**< Whether the left foot bumper was pressed during the previous cycle. */
    bool previousRightFootButtonPressed;                                            /**< Whether the right foot bumper was pressed during the previous cycle. */
    std::chrono::high_resolution_clock::time_point whenChestButtonStateChanged;     /**< When last state change of the chest button occured (DCM time). */
    std::chrono::high_resolution_clock::time_point whenLeftFootButtonStateChanged;  /**< When last state change of the left foot bumper occured (DCM time). */
    std::chrono::high_resolution_clock::time_point whenRightFootButtonStateChanged; /**< When last state change of the right foot bumper occured (DCM time). */
    std::chrono::high_resolution_clock::time_point whenPacketWasReceived;           /**< When the last GameController packet was received (DCM time). */
    std::chrono::high_resolution_clock::time_point whenPacketWasSent;               /**< When the last return packet was sent to the GameController (DCM time). */
    std::chrono::high_resolution_clock::time_point startTime;                       //start time of the libgamectl
    int lastTeamNum;

  public:
    const float *buttons[numOfButtons]; /**< Pointers to where ALMemory stores the current button states. */
    /**
     * Resets the internal state when an application was just started.
     */
    void init()
    {
        memset(&gameControllerAddress, 0, sizeof(gameControllerAddress));
        previousState = (uint8_t)-1;
        previousgamePhase = (uint8_t)-1;
        previouskickingTeam = (uint8_t)-1;
        previousTeamColour = (uint8_t)-1;
        previousPenalty = (uint8_t)-1;
        previousChestButtonPressed = false;
        previousLeftFootButtonPressed = false;
        previousRightFootButtonPressed = false;
        whenChestButtonStateChanged = startTime;
        whenLeftFootButtonStateChanged = startTime;
        whenRightFootButtonStateChanged = startTime;
        whenPacketWasReceived = startTime;
        whenPacketWasSent = startTime;
        memset(&gameCtrlData, 0, sizeof(gameCtrlData));
    }

    /**
     * Close all resources acquired.
     * Called when initialization failed or during destruction.
     */
    void closeGameCtrl()
    {
        if (udp)
            delete udp;
        fprintf(stderr, "libGameCtrl: Stopping.\n");
        fprintf(stderr, "libGameCtrl: Stopped.\n");
    }

    /**
     * Publishes the current state of the GameController packet in ALMemory.
     */
    void publish()
    {
        memcpy(&gameCtrlDataForGC, &gameCtrlData, sizeof(gameCtrlData));
    }

    /**
     * Sets the LEDs whenever the state they visualize changes.
     * Regularily sends the return packet to the GameController.
     */
    void handleOutput(Leds &leds)
    {
        auto now = high_resolution_clock::now();

        if (teamNumber && *playerNumber &&
            *playerNumber <= gameCtrlData.playersPerTeam &&
            (gameCtrlData.teams[0].teamNumber == teamNumber ||
             gameCtrlData.teams[1].teamNumber == teamNumber))
        {
            const TeamInfo &team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];
            duration<double> dtotalwhenPacketWasReceived = duration_cast<duration<double>>(now - whenPacketWasReceived);
            duration<double> dtotalwhenPacketWasSent = duration_cast<duration<double>>(now - whenPacketWasSent);

            if (gameCtrlData.state != previousState ||
                gameCtrlData.gamePhase != previousgamePhase ||
                gameCtrlData.kickingTeam != previouskickingTeam ||
                team.teamColour != previousTeamColour ||
                team.players[*playerNumber - 1].penalty != previousPenalty)
            {
                switch (team.teamColour)
                {
                case TEAM_BLUE:
                    setLED(leds, leftFootRed, 0.f, 0.f, 1.f);
                    break;
                case TEAM_RED:
                    setLED(leds, leftFootRed, 1.f, 0.f, 0.f);
                    break;
                case TEAM_YELLOW:
                    setLED(leds, leftFootRed, 1.f, 1.f, 0.f);
                    break;
                default:
                    setLED(leds, leftFootRed, 0.f, 0.f, 0.f);
                }
                if (gameCtrlData.state == STATE_INITIAL &&
                    gameCtrlData.gamePhase == GAME_PHASE_PENALTYSHOOT &&
                    gameCtrlData.kickingTeam == team.teamNumber)
                    setLED(leds, rightFootRed, 0.f, 1.f, 0.f);
                else if (gameCtrlData.state == STATE_INITIAL &&
                         gameCtrlData.gamePhase == GAME_PHASE_PENALTYSHOOT &&
                         gameCtrlData.kickingTeam != team.teamNumber)
                    setLED(leds, rightFootRed, 1.f, 1.0f, 0.f);
                else if (dtotalwhenPacketWasReceived.count() * 1000 < GAMECONTROLLER_TIMEOUT &&
                         gameCtrlData.state <= STATE_SET &&
                         gameCtrlData.kickingTeam == team.teamNumber)
                    setLED(leds, rightFootRed, 1.f, 1.f, 1.f);
                else
                    setLED(leds, rightFootRed, 0.f, 0.f, 0.f);
                if (team.players[*playerNumber - 1].penalty != PENALTY_NONE)
                    setLED(leds, chestRed, 1.f, 0.f, 0.f);
                else
                    switch (gameCtrlData.state)
                    {
                    case STATE_READY:
                        setLED(leds, chestRed, 0.f, 0.f, 1.f);
                        break;
                    case STATE_SET:
                        setLED(leds, chestRed, 1.f, 1.0f, 0.f);
                        break;
                    case STATE_PLAYING:
                        setLED(leds, chestRed, 0.f, 1.f, 0.f);
                        break;
                    default:
                        setLED(leds, chestRed, 0.f, 0.f, 0.f);
                    }

                previousState = gameCtrlData.state;
                previousgamePhase = gameCtrlData.gamePhase;
                previouskickingTeam = gameCtrlData.kickingTeam;
                previousTeamColour = team.teamColour;
                previousPenalty = team.players[*playerNumber - 1].penalty;
            }

            if (dtotalwhenPacketWasReceived.count() * 1000 < GAMECONTROLLER_TIMEOUT &&
                dtotalwhenPacketWasSent.count() * 1000 >= ALIVE_DELAY &&
                send(GAMECONTROLLER_RETURN_MSG_ALIVE))
                whenPacketWasSent = now;
        }
    }

    /**
     * Sets states in the LED request.
     * @param led The index of the red channel of an RGB LED.
     * @param red The red intensity [0..1].
     * @param green The green intensity [0..1].
     * @param blue The blue intensity [0..1].
     */
    void setLED(Leds &leds, LED led, float red, float green, float blue)
    {
        if (led == chestRed)
        {
            // leds.chestLed.r = red;
            // leds.chestLed.g = green;
            // leds.chestLed.b = blue;
            ledRequest[chestBoardLedRedActuator - faceLedRedLeft0DegActuator] = red;
            ledRequest[chestBoardLedRedActuator - faceLedRedLeft0DegActuator + 1] = green;
            ledRequest[chestBoardLedRedActuator - faceLedRedLeft0DegActuator + 2] = blue;
        }
        else if (led == leftFootRed)
        {
            // leds.foots.left.r =  red;
            // leds.foots.left.g =  green;
            // leds.foots.left.b =  blue;
            ledRequest[lFootLedRedActuator - faceLedRedLeft0DegActuator] = red;
            ledRequest[lFootLedRedActuator - faceLedRedLeft0DegActuator + 1] = green;
            ledRequest[lFootLedRedActuator - faceLedRedLeft0DegActuator + 2] = blue;
        }
        else if (led == rightFootRed)
        {
            // leds.foots.right.r =  red;
            // leds.foots.right.g =  green;
            // leds.foots.right.b =  blue;
            ledRequest[rFootLedRedActuator - faceLedRedLeft0DegActuator] = red;
            ledRequest[rFootLedRedActuator - faceLedRedLeft0DegActuator + 1] = green;
            ledRequest[rFootLedRedActuator - faceLedRedLeft0DegActuator + 2] = blue;
        }
    }

    /**
     * Handles the button interface.
     * Resets the internal state when a new team number was set.
     * Receives packets from the GameController.
     * Initializes gameCtrlData when teamNumber and playerNumber are available.
     */
    void handleInput()
    {
        auto now = high_resolution_clock::now();
        duration<double> dtotalwhenPacketWasReceived = duration_cast<duration<double>>(now - whenPacketWasReceived);
        duration<double> dtotalwhenChestButtonStateChanged = duration_cast<duration<double>>(now - whenChestButtonStateChanged);
        duration<double> dtotalwhenLeftFootButtonStateChanged = duration_cast<duration<double>>(now - whenLeftFootButtonStateChanged);
        duration<double> dtotalwhenRightFootButtonStateChanged = duration_cast<duration<double>>(now - whenRightFootButtonStateChanged);
        if (*teamNumberPtr != 0 && *teamNumberPtr != lastTeamNum)
        { // new team number was set -> reset internal structure
            teamNumber = *teamNumberPtr;
            data->teamInfo[0] = 0;
            init();
            lastTeamNum = teamNumber;
        }

        if (receive())
        {
            if (whenPacketWasReceived == startTime)
                previousState = (uint8_t)-1; // force LED update on first packet received
            whenPacketWasReceived = now;
            publish();
        }
        // std::cout<<"teamNumber is "<<teamNumber<<"  "<<*playerNumber<<std::endl;
        if (teamNumber && *playerNumber)
        {
            // init gameCtrlData if invalid
            if (gameCtrlData.teams[0].teamNumber != teamNumber &&
                gameCtrlData.teams[1].teamNumber != teamNumber)
            {
                uint8_t teamColour = (uint8_t)*defaultTeamColour;
                if (teamColour != TEAM_BLUE && teamColour != TEAM_RED && teamColour != TEAM_YELLOW)
                    teamColour = TEAM_BLACK;
                gameCtrlData.teams[0].teamNumber = (uint8_t)teamNumber;
                gameCtrlData.teams[0].teamColour = teamColour;
                gameCtrlData.teams[1].teamColour = teamColour ^ 1; // we don't know better
                if (!gameCtrlData.playersPerTeam)
                    gameCtrlData.playersPerTeam = (uint8_t)*playerNumber; // we don't know better
                publish();
            }
            TeamInfo &team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];
            // std::cout<<"teamnum is "<<(int)team.players[2].penalty<<std::endl;
            if (*playerNumber <= gameCtrlData.playersPerTeam)
            {
                bool chestButtonPressed = *buttons[chest] != 0.f;
                if (chestButtonPressed != previousChestButtonPressed && dtotalwhenChestButtonStateChanged.count() * 1000 >= BUTTON_DELAY)
                {
                    if (chestButtonPressed && whenChestButtonStateChanged != startTime) // ignore first press, e.g. for getting up
                    {
                        // std::cout<<"chestButtonPressed is "<<chestButtonPressed<<"  "<<team.players[*playerNumber - 1].penalty<<std::endl;
                        RobotInfo &player = team.players[*playerNumber - 1];
                        if (player.penalty == PENALTY_NONE)
                        {
                            player.penalty = PENALTY_MANUAL;
                            if (dtotalwhenPacketWasReceived.count() * 1000 < GAMECONTROLLER_TIMEOUT &&
                                send(GAMECONTROLLER_RETURN_MSG_ALIVE))
                                whenPacketWasSent = now;
                        }
                        else
                        {
                            player.penalty = PENALTY_NONE;
                            if (dtotalwhenPacketWasReceived.count() * 1000 < GAMECONTROLLER_TIMEOUT &&
                                send(GAMECONTROLLER_RETURN_MSG_ALIVE))
                                whenPacketWasSent = now;
                            else
                                gameCtrlData.state = STATE_PLAYING;
                        }
                        publish();
                    }

                    previousChestButtonPressed = chestButtonPressed;
                    whenChestButtonStateChanged = now;
                }

                if (gameCtrlData.state == STATE_INITIAL)
                {
                    bool leftFootButtonPressed = *buttons[leftFootLeft] != 0.f || *buttons[leftFootRight] != 0.f;
                    if (leftFootButtonPressed != previousLeftFootButtonPressed && dtotalwhenLeftFootButtonStateChanged.count() * 1000 >= BUTTON_DELAY)
                    {
                        if (leftFootButtonPressed)
                        {
                            team.teamColour = (team.teamColour + 1) & 3; // cycle between TEAM_BLUE .. TEAM_BLACK
                            publish();
                        }
                        previousLeftFootButtonPressed = leftFootButtonPressed;
                        whenLeftFootButtonStateChanged = now;
                    }

                    bool rightFootButtonPressed = *buttons[rightFootLeft] != 0.f || *buttons[rightFootRight] != 0.f;
                    if (rightFootButtonPressed != previousRightFootButtonPressed && dtotalwhenRightFootButtonStateChanged.count() * 1000 >= BUTTON_DELAY)
                    {
                        if (rightFootButtonPressed)
                        {
                            if (gameCtrlData.gamePhase == GAME_PHASE_NORMAL)
                            {
                                gameCtrlData.gamePhase = GAME_PHASE_PENALTYSHOOT;
                                gameCtrlData.kickingTeam = team.teamNumber;
                            }
                            else if (gameCtrlData.kickingTeam == team.teamNumber)
                                gameCtrlData.kickingTeam = 0;
                            else
                                gameCtrlData.gamePhase = GAME_PHASE_NORMAL;
                            publish();
                        }
                        previousRightFootButtonPressed = rightFootButtonPressed;
                        whenRightFootButtonStateChanged = now;
                    }
                }
            }
            else
                fprintf(stderr, "Player number %d too big. Maximum number is %d.\n", *playerNumber, gameCtrlData.playersPerTeam);
        }
    }

    /**
     * Sends the return packet to the GameController.
     * @param message The message contained in the packet (GAMECONTROLLER_RETURN_MSG_MAN_PENALISE,
     *                GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE or GAMECONTROLLER_RETURN_MSG_ALIVE).
     */
    bool send(uint8_t message)
    {
        RoboCupGameControlReturnData returnPacket;
        returnPacket.team = (uint8_t)teamNumber;
        returnPacket.player = (uint8_t)*playerNumber;
        returnPacket.message = message;
        return !udp || udp->write((const char *)&returnPacket, sizeof(returnPacket));
    }

    /**
     * Receives a packet from the GameController.
     * Packets are only accepted when the team number is know (nonzero) and
     * they are addressed to this team.
     */
    bool receive()
    {
        bool received = false;
        int size;
        RoboCupGameControlData buffer;
        struct sockaddr_in from;
        while (udp && (size = udp->read((char *)&buffer, sizeof(buffer), from)) > 0)
        {
            if (size == sizeof(buffer) &&
                !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
                buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
                teamNumber &&
                (buffer.teams[0].teamNumber == teamNumber ||
                 buffer.teams[1].teamNumber == teamNumber))
            {
                gameCtrlData = buffer;
                if (memcmp(&gameControllerAddress, &from.sin_addr, sizeof(in_addr)))
                {
                    memcpy(&gameControllerAddress, &from.sin_addr, sizeof(in_addr));
                    udp->setTarget(inet_ntoa(gameControllerAddress), GAMECONTROLLER_RETURN_PORT);
                }

                received = true;
            }
        }
        return received;
    }

    GameCtrl() : udp(0), teamNumber(0)
    {
        startTime = high_resolution_clock::now();
        init();
        lastTeamNum = 0;
        try
        {
            playerNumber = &data->teamInfo[2];
            teamNumberPtr = &data->teamInfo[0];
            defaultTeamColour = &data->teamInfo[1];
            udp = new UdpComm();
            if (!udp->setBlocking(false) ||
                !udp->setBroadcast(true) ||
                !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
                !udp->setLoopback(false))
            {
                fprintf(stderr, "libgamectrl: Could not open UDP port\n");
                delete udp;
                udp = 0;
                // continue, because button interface will still work
            }

            publish();
        }
        catch (string &e)
        {
            fprintf(stderr, "libgamectrl: %s\n", e.c_str());
            closeGameCtrl();
        }
    }

    /**
     * Close all resources acquired.
     */
    ~GameCtrl()
    {
        closeGameCtrl();
    }
};

class BHuman
{
  private:
    static BHuman *theInstance;
    static const int allowedFrameDrops = 6;
    GameCtrl *gameCtrl = nullptr;
    float sensorPtrs[lbhNumOfSensorIds];           /** Pointers to where NaoQi stores the current sensor values. */
    float requestedActuators[lbhNumOfActuatorIds]; /**< The previous actuator values requested. */

    int lastReadingActuators = -1;          /**< The previous actuators read. For detecting frames without seemingly new data from bhuman. */
    int actuatorDrops = 0;                  /**< The number of frames without seemingly new data from bhuman. */
    int frameDrops = allowedFrameDrops + 1; /**< The number frames without a reaction from bhuman. */

    enum State
    {
        sitting,
        standingUp,
        standing,
        sittingDown,
        preShuttingDown,
        preShuttingDownWhileSitting,
        shuttingDown
    } state;
    float phase = 0.f; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */
    int ledIndex = 0;  /**< The index of the last LED set. */

    int rightEarLEDsChangedTime = 0;                                                        // Last time when the right ear LEDs were changed by the B-Human code
    float requestedRightEarLEDs[earsLedRight324DegActuator - earsLedRight0DegActuator + 1]; // The state prevously requested by the B-Human code

    float startAngles[lbhNumOfPositionActuatorIds];      /**< Start angles for standing up or sitting down. */
    float startStiffness[lbhNumOfPositionActuatorIds];   /**< Start stiffness for sitting down. */
    float positionRequest[lbhNumOfPositionActuatorIds];  /**< Request angles */
    float stiffnessRequest[lbhNumOfPositionActuatorIds]; /**< Request stiffness */

    int startPressedTime = 0;         /**< The last time the chest button was not pressed. */
    unsigned lastBHumanStartTime = 0; /**< The last time bhuman was started. */
                                      /** Close all resources acquired. Called when initialization failed or during destruction. */

    void close()
    {
        fprintf(stderr, "libbhuman: Stopping.\n");
        if (sem != SEM_FAILED)
            sem_close(sem);
        if (data != MAP_FAILED)
            munmap(data, sizeof(LBHData));

        fprintf(stderr, "libbhuman: Stopped.\n");
    }

  public:
    bool parseNAOVersion(const std::string &versionString, NAOVersion &version)
    {
        version = NAOVersion::V6;
        return true;
        // if(versionString.size() == 4)
        // {
        //     if(versionString[0] == '6')
        //         version = NAOVersion::V6;
        //     else if(versionString[0] == '5')
        //         version = NAOVersion::V5;
        //     else if(versionString[0] == '4')
        //         version = NAOVersion::V4;
        //     else if(versionString[0] == '3' && versionString[2] == '3')
        //         version = NAOVersion::V33;
        //     else if(versionString[0] == '3' && versionString[2] == '2')
        //         version = NAOVersion::V32;
        //     else
        //         return false;
        //     return true;
        // }
        // else
        //     return false;
    }

    /**
   * Set the eye LEDs based on the current state.
   * Shutting down -> Lower segments are red.
   * bhuman crashed -> Whole eyes quickly flash red.
   * bhuman not running -> Lower segments flash blue.
   * @param actuators The actuator values a part of which will be set by this method.
   */
    void setEyeLeds(float *actuators)
    {
        for (int i = faceLedRedLeft0DegActuator; i <= faceLedBlueRight315DegActuator; ++i)
            actuators[i] = 0.f;
        if (state == shuttingDown)
        {
            actuators[faceLedRedLeft180DegActuator] = 1.f;
            actuators[faceLedRedRight180DegActuator] = 1.f;
        }
        else if (data->state != okState)
        {
            // set the "libbhuman is active and bhuman crashed" leds
            float blink = float(CycleIndex / 20 & 1);
            for (int i = faceLedRedLeft0DegActuator; i <= faceLedRedLeft315DegActuator; ++i)
                actuators[i] = blink;
            for (int i = faceLedRedRight0DegActuator; i <= faceLedRedRight315DegActuator; ++i)
                actuators[i] = 1.f - blink;
        }
        else
        {
            // set the "libbhuman is active and bhuman is not running" LEDs
            float blink = float(CycleIndex / 50 & 1);
            actuators[faceLedBlueLeft180DegActuator] = blink;
            actuators[faceLedBlueRight180DegActuator] = blink;
        }
    }

    /**
   * Shows the battery state in the right ear if the robot is in the standing state
   * and bhuman has not used one of these LEDs in the past 5 seconds.
   * @param actuators The actuator values a part of which will be set by this method.
   */
    void setBatteryLeds(float *actuators)
    {
        for (int i = earsLedRight0DegActuator; i <= earsLedRight324DegActuator; ++i)
            if (actuators[i] != requestedActuators[i])
            {
                rightEarLEDsChangedTime = CycleIndex;
                requestedActuators[i] = actuators[i];
            }
        if (state != standing || CycleIndex - rightEarLEDsChangedTime > 500)
        {
            for (int i = 0; i < 10; ++i)
            {
                actuators[earsLedRight0DegActuator + i] = i < int(sensorPtrs[batteryChargeSensor] * 10.f) ? 1.f : 0.f;
                // std::cout<<"led "<<i<<"  "<<actuators[earsLedRight0DegActuator + i]<<"  "<<sensorPtrs[batteryChargeSensor]<<std::endl;
            }
        }
    }

    /**
   * Copies everything that's not for servos from one set of actuator values to another.
   * @param srcActuators The actuator values from which is copied.
   * @param destActuators The actuator values to which is copied.
   */
    void copyNonServos(const float *srcActuators, float *destActuators)
    {
        for (int i = faceLedRedLeft0DegActuator; i < lbhNumOfActuatorIds; ++i)
            destActuators[i] = srcActuators[i];
    }

    /**
   * Handles the different states libbhuman can be in.
   * @param actuators The actuator values requested. They will not be changed, but might
   *                  be used as result of this method.
   * @return The actuator values that should be set. In the standing state, they are
   *         identical to the actuators passed to this method. In all other states,
   *         they are different.
   */
    float *handleState(float *actuators)
    {
        static float controlledActuators[lbhNumOfActuatorIds];
        switch (state)
        {
        sitting:
            state = sitting;
        case sitting:
            memset(controlledActuators, 0, sizeof(controlledActuators));
            if (frameDrops > allowedFrameDrops ||
                (actuators[lHipPitchStiffnessActuator] == 0.f && actuators[rHipPitchStiffnessActuator] == 0.f))
                return controlledActuators;

            for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                startAngles[i] = sensorPtrs[i * lbhNumOfDifSensors];

        standingUp:
            state = standingUp;
            phase = 0.f;

        case standingUp:
            if (phase < 1.f && frameDrops <= allowedFrameDrops)
            {
                memset(controlledActuators, 0, sizeof(controlledActuators));
                phase = std::min(phase + 0.005f, 1.f);
                for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                {
                    controlledActuators[i] = actuators[i] * phase + startAngles[i] * (1.f - phase);
                    float h = std::min(actuators[i + headYawStiffnessActuator], 0.5f);
                    controlledActuators[i + headYawStiffnessActuator] = actuators[i + headYawStiffnessActuator] * phase + h * (1.f - phase);
                }
                return controlledActuators;
            }
            state = standing;

        case standing:
            if (frameDrops <= allowedFrameDrops)
                return actuators; // use original actuators

        case preShuttingDown:
            for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
            {
                startAngles[i] = positionRequest[i];
                if (actuators[lHipPitchStiffnessActuator] == 0.f && actuators[rHipPitchStiffnessActuator] == 0.f)
                    startStiffness[i] = 0.f;
                else if (i >= lShoulderPitchPositionActuator && i <= rElbowRollPositionActuator)
                    startStiffness[i] = 0.4f;
                else
                    startStiffness[i] = std::min<float>(stiffnessRequest[i], 0.3f);
            }
            state = state == preShuttingDown ? shuttingDown : sittingDown;
            phase = 0.f;
            
        case sittingDown:
        case shuttingDown:
        shuttingDown:
            if ((phase < 1.f && frameDrops > allowedFrameDrops) || state == shuttingDown)
            {
                memset(controlledActuators, 0, sizeof(controlledActuators));
                phase = std::min(phase + 0.005f, 1.f);
                for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                {
                    controlledActuators[i] = sitDownAngles[i] * phase + startAngles[i] * (1.f - phase);
                    controlledActuators[i + headYawStiffnessActuator] = startStiffness[i];
                }
                return controlledActuators;
            }
            else if (frameDrops <= allowedFrameDrops)
            {
                for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                    startAngles[i] = positionRequest[i];
                goto standingUp;
            }
            else
                goto sitting;

        case preShuttingDownWhileSitting:
            for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                startStiffness[i] = 0.f;
            state = shuttingDown;
            phase = 0.995f;
            goto shuttingDown;
        }
    }

    // void writeRobocup(Robocup &robocup)
    // {
    //     robocup.safety = false;        // (bool) 1: filter is active, triggles on the acceleration; 0: filter is inactive, triggles off the accleration
    // }

    void writeJoints(Joints &joints)
    {
        joints.head[HeadYaw].angle = positionRequest[headYawPositionActuator];
        joints.head[HeadYaw].stiffness = stiffnessRequest[headYawPositionActuator];
        joints.head[HeadPitch].angle = positionRequest[headPitchPositionActuator];
        joints.head[HeadPitch].stiffness = stiffnessRequest[headPitchPositionActuator];
        for (int i = 0; i < ARM_JOINT_SIZE; i++)
        {
            joints.arms[i].angle = positionRequest[lShoulderPitchPositionActuator + i];
            joints.arms[i].stiffness = stiffnessRequest[lShoulderPitchPositionActuator + i];
        }
        for (int i = 0; i < LEG_JOINT_SIZE; i++)
        {
            joints.legs[i].angle = positionRequest[lHipYawPitchPositionActuator + i];
            joints.legs[i].stiffness = stiffnessRequest[lHipYawPitchPositionActuator + i];
        }
    }

    void writeLeds(Leds &leds)
    {
        //lEye Red
        int index = 0;
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.left[i].r = ledRequest[index++];
        }
        //lEye Green
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.left[i].g = ledRequest[index++];
        }
        //lEye Blue
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.left[i].b = ledRequest[index++];
        }
        //rEye Red
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.right[i].r = ledRequest[index++];
        }
        //rEye Green
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.right[i].g = ledRequest[index++];
        }
        //rEye Blue
        for (int i = 0; i < 8; i++)
        {
            leds.eyes.right[i].b = ledRequest[index++];
        }
        //lEar
        for (int i = 0; i < 10; i++)
        {
            leds.ears.left[i] = ledRequest[index++];
        }
        //rEar
        for (int i = 0; i < 10; i++)
        {
            leds.ears.right[i] = ledRequest[index++];
        }
        //chest
        leds.chestLed.r = ledRequest[index++];
        leds.chestLed.g = ledRequest[index++];
        leds.chestLed.b = ledRequest[index++];
        //head led
        for (int i = 0; i < 12; i++)
        {
            leds.head[i] = ledRequest[index++];
        }
        //foot led
        leds.foots.left.r = ledRequest[index++];
        leds.foots.left.g = ledRequest[index++];
        leds.foots.left.b = ledRequest[index++];
        leds.foots.right.r = ledRequest[index++];
        leds.foots.right.g = ledRequest[index++];
        leds.foots.right.b = ledRequest[index++];
    }

    void extract_sensors(float *sensors, const LolaSensorFrame &sensor_frame)
    {
        int index = 0;
        for (int i = 0; i < HEAD_JOINT_SIZE; i++)
        {
            sensors[index++] = sensor_frame.joints.head[i].angle;
            sensors[index++] = sensor_frame.joints.head[i].current;
            sensors[index++] = sensor_frame.joints.head[i].temperature;
        }
        for (int i = 0; i < ARM_JOINT_SIZE; i++)
        {
            sensors[index++] = sensor_frame.joints.arms[i].angle;
            sensors[index++] = sensor_frame.joints.arms[i].current;
            sensors[index++] = sensor_frame.joints.arms[i].temperature;
        }
        for (int i = 0; i < LEG_JOINT_SIZE; i++)
        {
            sensors[index++] = sensor_frame.joints.legs[i].angle;
            sensors[index++] = sensor_frame.joints.legs[i].current;
            sensors[index++] = sensor_frame.joints.legs[i].temperature;
        }
        sensors[index++] = sensor_frame.touch.headTouch.front;
        sensors[index++] = sensor_frame.touch.headTouch.middle;
        sensors[index++] = sensor_frame.touch.headTouch.rear;
        sensors[index++] = sensor_frame.touch.lHandTouch.back;
        sensors[index++] = sensor_frame.touch.lHandTouch.left;
        sensors[index++] = sensor_frame.touch.lHandTouch.right;
        sensors[index++] = sensor_frame.touch.rHandTouch.back;
        sensors[index++] = sensor_frame.touch.rHandTouch.left;
        sensors[index++] = sensor_frame.touch.rHandTouch.right;
        sensors[index++] = sensor_frame.touch.lFootBumper.left;
        sensors[index++] = sensor_frame.touch.lFootBumper.right;
        sensors[index++] = sensor_frame.touch.rFootBumper.left;
        sensors[index++] = sensor_frame.touch.rFootBumper.right;
        sensors[index++] = sensor_frame.touch.ChestBtn;
        sensors[index++] = sensor_frame.imu.gyr.roll;
        sensors[index++] = sensor_frame.imu.gyr.pitch;
        sensors[index++] = sensor_frame.imu.gyr.yaw;
        sensors[index++] = sensor_frame.imu.accel.x;
        sensors[index++] = sensor_frame.imu.accel.y;
        sensors[index++] = sensor_frame.imu.accel.z;
        sensors[index++] = sensor_frame.imu.angles.X;
        sensors[index++] = sensor_frame.imu.angles.Y;
        sensors[index++] = sensor_frame.battery.current;
        sensors[index++] = sensor_frame.battery.charge;
        sensors[index++] = sensor_frame.battery.status;
        sensors[index++] = sensor_frame.battery.temp;
        sensors[index++] = sensor_frame.fsr.left.fl;
        sensors[index++] = sensor_frame.fsr.left.fr;
        sensors[index++] = sensor_frame.fsr.left.rl;
        sensors[index++] = sensor_frame.fsr.left.rr;
        sensors[index++] = sensor_frame.fsr.right.fl;
        sensors[index++] = sensor_frame.fsr.right.fr;
        sensors[index++] = sensor_frame.fsr.right.rl;
        sensors[index++] = sensor_frame.fsr.right.rr;
        sensors[index++] = sensor_frame.fsr.left.fl +
                           sensor_frame.fsr.left.fr +
                           sensor_frame.fsr.left.rl +
                           sensor_frame.fsr.left.rr;
        sensors[index++] = sensor_frame.fsr.right.fl +
                           sensor_frame.fsr.right.fr +
                           sensor_frame.fsr.right.rl +
                           sensor_frame.fsr.right.rr;
    }

    void execute()
    {
        //create socket to communicate with LoLA
        io_service io_service;
        local::stream_protocol::socket socket(io_service);
        socket.connect("/tmp/robocup");
        constexpr int max_len = 100000;
        char socketdata[max_len] = {'\0'};
        boost::system::error_code ec;
        LolaFrameHandler frame_handler;

        const LolaSensorFrame &sensor_frame = frame_handler.unpack(socketdata, socket.receive(boost::asio::buffer(socketdata, max_len)));
        std::string headVersion = sensor_frame.robotConfig.HeadVersion;
        std::string bodyVersion = sensor_frame.robotConfig.BodyVersion;
        std::cout << "the bodyId is" << sensor_frame.robotConfig.BodyId << std::endl;
        std::cout << "the bodyversion is" << sensor_frame.robotConfig.BodyVersion << std::endl;
        std::cout << "the Headid is" << sensor_frame.robotConfig.HeadId << std::endl;
        std::cout << "the HeadVersion is" << sensor_frame.robotConfig.HeadVersion << std::endl;
        if (!parseNAOVersion(headVersion, data->headVersion))
        {
            fprintf(stderr, "libbhuman: unknown headVerion: %s!\n", headVersion.c_str());
        }
        if (!parseNAOVersion(bodyVersion, data->bodyVersion))
        {
            fprintf(stderr, "libbhuman: unknown bodyVersion: %s!\n", bodyVersion.c_str());
        }
        data->headType = NAOType::H25;
        data->bodyType = NAOType::H25;

        int headIdSize;
        data->headVersion == NAOVersion::V6 ? headIdSize = 20 : headIdSize = 15;
        std::string headId = sensor_frame.robotConfig.HeadId;
        std::strncpy(data->headId, headId.c_str(), headIdSize);
        data->headId[headIdSize] = 0;

        int bodyIdSize;
        data->bodyVersion == NAOVersion::V6 ? bodyIdSize = 20 : bodyIdSize = 15;
        std::string bodyId = sensor_frame.robotConfig.BodyId;
        std::strncpy(data->bodyId, bodyId.c_str(), bodyIdSize);
        data->bodyId[bodyIdSize] = 0;

        while (true)
        {
            auto calcu_start = high_resolution_clock::now();
            if (lola_shutdown)
            {
                break;
            }
            // auto now = high_resolution_clock::now();
            // duration<double> dtotal = duration_cast<duration<double>>(now - lasttime);
            // std::cout<<"cycle time is "<<dtotal.count()*1000<<"ms"<<std::endl;
            // lasttime = now;
            try
            {
                CycleIndex += 1;
                ///////////////set the sensors
                data->readingActuators = data->newestActuators;
                if (data->readingActuators == lastReadingActuators)
                {
                    if (actuatorDrops == 0)
                        fprintf(stderr, "libbhuman: missed actuator request.\n");
                    ++actuatorDrops;
                }
                else
                    actuatorDrops = 0;
                lastReadingActuators = data->readingActuators;
                // std::cout<<"rear "<<data->actuators[0][rWristYawPositionActuator]<<std::endl;
                // std::cout<<"rear "<<data->actuators[1][rWristYawPositionActuator]<<std::endl;
                // std::cout<<"rear "<<data->actuators[2][rWristYawPositionActuator]<<std::endl;
                float *readingActuators = data->actuators[data->readingActuators];
                float *actuators = handleState(readingActuators);
                if (state != standing)
                {
                    if (frameDrops > 0 || state == shuttingDown)
                    {
                        setEyeLeds(actuators);
                    }
                    else
                    {
                        copyNonServos(readingActuators, actuators);
                    }
                }
                setBatteryLeds(actuators);
                // set position actuators
                for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
                {
                    positionRequest[i] = actuators[i];
                    // std::cout<<"positionRequest is "<<i<<"  "<<positionRequest[i]<<"  "<<positionRequest[rWristYawPositionActuator]<<std::endl;
                }

                auto &joints = frame_handler.actuator_frame.joints;
                auto &leds = frame_handler.actuator_frame.leds;
                // auto &robocup = frame_handler.actuator_frame.robocup;
                
                // set safety to control on/off the acceleration limit
                // writeRobocup(robocup);
                // set stiffness actuators
                bool requestedStiffness = false;
                for (int i = headYawStiffnessActuator; i < headYawStiffnessActuator + lbhNumOfStiffnessActuatorIds; ++i)
                {
                    if (actuators[i] != requestedActuators[i])
                    {
                        for (int j = 0; j < lbhNumOfStiffnessActuatorIds; ++j)
                            stiffnessRequest[j] = requestedActuators[headYawStiffnessActuator + j] = actuators[headYawStiffnessActuator + j];
                        requestedStiffness = true;
                        break;
                    }
                }
                //write the position actuators into socket
                writeJoints(joints);
                //set led
                if (!requestedStiffness)
                {
                    for (int i = 0; i < lbhNumOfLedActuatorIds; ++i)
                    {
                        int index = faceLedRedLeft0DegActuator + ledIndex;
                        if (++ledIndex == lbhNumOfLedActuatorIds)
                            ledIndex = 0;
                        if (actuators[index] != requestedActuators[index])
                        {
                            ledRequest[i] = requestedActuators[index] = actuators[index];
                            // break;
                        }
                    }
                }
                writeLeds(leds);
                gameCtrl->handleOutput(leds);
                // set team info
                // since this should happen very rarely, we don't use a proxy here
                if (data->bhumanStartTime != lastBHumanStartTime)
                {
                    lastBHumanStartTime = data->bhumanStartTime;
                }
                char *buffer;
                size_t size;
                // std::cout<<"the right wrist is "<< joints.arms[RWristYaw].angle<<std::endl;
                tie(buffer, size) = frame_handler.pack();
                socket.send(boost::asio::buffer(buffer, size));
            }
            catch (string &e)
            {
                fprintf(stderr, "libbhuman: %s\n", e.c_str());
                exit(0);
            }
            //read sensors
            // get new sensor values and copy them to the shared memory block
            try
            {
                // copy sensor values into the shared memory block
                const LolaSensorFrame &sensor_frame = frame_handler.unpack(socketdata, socket.receive(boost::asio::buffer(socketdata, max_len)));
                // std::cout<<"touch0 is "<<data->sensors[0][headTouchFrontSensor]<<std::endl;
                // std::cout<<"touch1 is "<<data->sensors[1][headTouchFrontSensor]<<std::endl;
                // std::cout<<"touch2 is "<<data->sensors[2][headTouchFrontSensor]<<std::endl;
                // std::cout<<"data add is "<<data<<std::endl;
                // std::cout<<"front "<<data->actuators[2][rWristYawPositionActuator]<<std::endl;
                int writingSensors = 0;
                if (writingSensors == data->newestSensors)
                    ++writingSensors;
                if (writingSensors == data->readingSensors)
                    if (++writingSensors == data->newestSensors)
                        ++writingSensors;
                assert(writingSensors != data->newestSensors);
                assert(writingSensors != data->readingSensors);
                float *sensors = data->sensors[writingSensors];
                //extract sensors
                extract_sensors(sensors, sensor_frame);
                for (int i = 0; i < lbhNumOfSensorIds; ++i)
                    sensorPtrs[i] = sensors[i];

                memcpy(&data->gameControlData[writingSensors], &gameCtrlDataForGC, sizeof(RoboCupGameControlData));

                data->newestSensors = writingSensors;
                // detect shutdown request via chest-button
                if (sensors[chestButtonSensor] == 0.f)
                    startPressedTime = CycleIndex;
                else if (state != shuttingDown && startPressedTime && CycleIndex - startPressedTime > 300)
                {
                    (void)!system("( /home/nao/bin/bhumand stop && sudo shutdown -h now ) &");
                    state = state == sitting ? preShuttingDownWhileSitting : preShuttingDown;
                }
                gameCtrl->buttons[0] = &(sensor_frame.touch.ChestBtn);
                gameCtrl->buttons[1] = &(sensor_frame.touch.lFootBumper.left);
                gameCtrl->buttons[2] = &(sensor_frame.touch.lFootBumper.right);
                gameCtrl->buttons[3] = &(sensor_frame.touch.rFootBumper.left);
                gameCtrl->buttons[4] = &(sensor_frame.touch.rFootBumper.right);
                gameCtrl->handleInput();
            }
            catch (string &e)
            {
                fprintf(stderr, "libbhuman: %s\n", e.c_str());
                exit(0);
            }
            // raise the semaphore
            if (sem != SEM_FAILED)
            {
                int sval;
                if (sem_getvalue(sem, &sval) == 0)
                {
                    if (sval < 1)
                    {
                        sem_post(sem);
                        frameDrops = 0;
                    }
                    else
                    {
                        if (frameDrops == 0)
                            fprintf(stderr, "libbhuman: dropped sensor data.\n");
                        ++frameDrops;
                    }
                }
            }
            usleep(10000);
            auto calcu_end = high_resolution_clock::now();
            std::chrono::duration<double> calcu_total = std::chrono::duration_cast<std::chrono::duration<double>>(calcu_end - calcu_start);
            // std::cout << "last  ---- >>>> " << calcu_total.count() << " seconds" << std::endl;
        }
    }

    BHuman() : state(sitting)
    {

        //create a socket to communicate with LoLA
        // io_service io_service;
        // local::stream_protocol::socket socket(io_service);
        // socket.connect("/tmp/robocup");

        // constexpr int max_len = 100000;
        // char socketData[max_len] = {'\0'};
        // boost::system::error_code ec;
        // LolaFrameHandler frame_handler;
        memoryHandle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
        if (memoryHandle == -1)
            perror("libbhuman: shm_open");
        else if (ftruncate(memoryHandle, sizeof(LBHData)) == -1)
            perror("libbhuman: ftruncate");
        else
        {
            data = (LBHData *)mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
            if (data == MAP_FAILED)
                perror("libbhuman: mmap");
            else
            {
                memset(data, 0, sizeof(LBHData));
                // open semaphore
                sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
                if (sem == SEM_FAILED)
                    perror("libbhuman: sem_open");
                else
                {
                    try
                    {
                        gameCtrl = new GameCtrl();
                        // const LolaSensorFrame& sensor_frame = frame_handler.unpack(socketData, socket.receive(boost::asio::buffer(socketData, max_len)));
                        // std::string headVersion = sensor_frame.robotConfig.HeadVersion;
                        // std::string bodyVersion = sensor_frame.robotConfig.BodyVersion;
                        // std::cout<<"the bodyId is"<< sensor_frame.robotConfig.BodyId<<std::endl;
                        // std::cout<<"the bodyversion is"<< sensor_frame.robotConfig.BodyVersion<<std::endl;
                        // std::cout<<"the Headid is"<< sensor_frame.robotConfig.HeadId<<std::endl;
                        // std::cout<<"the HeadVersion is"<< sensor_frame.robotConfig.HeadVersion<<std::endl;
                        // if(!parseNAOVersion(headVersion, data->headVersion))
                        // {
                        //     fprintf(stderr, "libbhuman: unknown headVerion: %s!\n", headVersion.c_str());
                        // }
                        // if(!parseNAOVersion(bodyVersion, data->bodyVersion))
                        // {
                        //     fprintf(stderr, "libbhuman: unknown bodyVersion: %s!\n", bodyVersion.c_str());
                        // }
                        CycleIndex = 0;
                        data->headType = NAOType::H25;
                        data->bodyType = NAOType::H25;

                        int headIdSize;
                        data->headVersion == NAOVersion::V6 ? headIdSize = 20 : headIdSize = 15;
                        memset(requestedActuators, 0, sizeof(requestedActuators));
                        for (int i = faceLedRedLeft0DegActuator; i < chestBoardLedRedActuator; ++i)
                        {
                            requestedActuators[i] = -1.f;
                        }
                        // std::string headId = sensor_frame.robotConfig.HeadId;
                        // std::strncpy(data->headId, headId.c_str(), headIdSize);
                        // data->headId[headIdSize] = 0;

                        // int bodyIdSize;
                        // data->bodyVersion == NAOVersion::V6 ? bodyIdSize = 20 : bodyIdSize = 15;
                        // std::string bodyId = sensor_frame.robotConfig.BodyId;
                        // std::strncpy(data->bodyId, bodyId.c_str(), bodyIdSize);
                        // data->bodyId[bodyIdSize] = 0;
                        // socket.close();
                    }
                    catch (string &e)
                    {
                        fprintf(stderr, "libbhuman: %s\n", e.c_str());
                    }
                }
            }
        }
    }
    /** Close all resources acquired. */
    ~BHuman()
    {
        close();
    }
};

void ctrlc_handler(int)
{
    lola_shutdown = true;
}

int main(int, char *[])
{
    // Register some handlers so we can clean-up when we're killed.
    signal(SIGINT, ctrlc_handler);
    signal(SIGTERM, ctrlc_handler);
    lasttime = high_resolution_clock::now();
    BHuman bhuman;
    bhuman.execute();

    return 0;
}
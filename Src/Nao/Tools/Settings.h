#pragma once

#include <string>

class Settings
{
public:
    enum TeamColor
    {
        blue,
        red,
        yellow,
        black,
        white,
        green,
        orange,
        purple,
        brown,
        gray
    };

    std::string headName;
    std::string bodyName;

    static bool recover;

    static constexpr int highestValidPlayerNumber = 6;
    static constexpr int lowestValidPlayerNumber = 1;
    bool isGoalkeeper;
    bool isDropInGame = false;

    static bool loadSucceeded() { return loaded; }

    Settings() { loadsParameters(); }

private:
    static Settings settings;
    static bool loaded;

    Settings(bool master);

    Settings &operator=(const Settings &other)
    {
        teamNumber = other.teamNumber;
        teamColor = other.teamColor;
        playerNumber = other.playerNumber;
        location = other.location.c_str();
        teamPort = other.teamPort;
        headName = other.headName.c_str();
        bodyName = other.bodyName.c_str();
        isGoalkeeper = other.isGoalkeeper;
        isDropInGame = other.isDropInGame;
        return *this;
    }

    /**
     * The function loads the settings from disk
     * @return whether the settings were loaded successfully
     */
    bool load();

public:
    void loadsParameters()
    {
        teamNumber = 20;
        teamColor = blue;
        playerNumber = 2;
        location = "Default";
        scenario = "Default";
        teamPort = 10020;
    }
    int teamNumber;            /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
    TeamColor teamColor;       /**< The color of our team. Use theOwnTeamInfo.teamColor instead. */
    int playerNumber;          /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
    std::string location;      /**< The name of the location. */
    std::string scenario;      /**< The name of the scenario. */
    int teamPort;              /**< The UDP port our team uses for team communication. */
};
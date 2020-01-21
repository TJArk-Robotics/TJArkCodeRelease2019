/**
 * @file MofCompiler.h
 *
 * This file declares a single function to compile the motion net for special actions.
 *
 * @author Uwe Düffert
 * @author Martin Lötzsch
 */

#pragma once

#include "Representations/MotionControl/SpecialActionRequest.h"

#include <vector>
#include <map>

class MofCompiler
{
private:
    static const int MAXLAB = 5000;
    int numOfLabels = 0;
    char *label_motion[MAXLAB];
    char *label_name[MAXLAB];
    short label_number[MAXLAB];

    static const int MAXLIN = 32000;
    int numOfLines = 0;
    char motion[512];
    int actMotionID = -1;
    char *line_data[MAXLIN];
    short line_number[MAXLIN];
    short line_motionID[MAXLIN];

    static const int MAXFIL = 500;
    int numOfFiles = 0;
    char *file_name[MAXFIL];
    short file_startindex[MAXFIL];

    int jumpTable[SpecialActionRequest::NumOfSpecialActionID];

    char *printBuffer;
    size_t printBufferSize;

    std::map<SpecialActionRequest::SpecialActionID, std::string> SpecialActionIDName;
    std::map<std::string, SpecialActionRequest::SpecialActionID> SpecialActionIDValue;

public:
    MofCompiler() 
    {
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::playDead, "playDead"));
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::sitDown, "sitDown"));
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::stand, "stand"));
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::standHigh, "standHigh"));
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::standHighLookUp, "standHighLookUp"));
        SpecialActionIDName.insert(std::pair<SpecialActionRequest::SpecialActionID, std::string>(SpecialActionRequest::getUpEngineDummy, "getUpEngineDummy"));

        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("playDead", SpecialActionRequest::playDead));
        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("sitDown", SpecialActionRequest::sitDown));
        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("stand", SpecialActionRequest::stand));
        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("standHigh", SpecialActionRequest::standHigh));
        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("standHighLookUp", SpecialActionRequest::standHighLookUp));
        SpecialActionIDValue.insert(std::pair<std::string, SpecialActionRequest::SpecialActionID>("getUpEngineDummy", SpecialActionRequest::getUpEngineDummy));
    }

    ~MofCompiler();

    /**
   * The function compiles all mofs.
   * @param buffer A buffer that receives any error message output. It may contain
   *               several lines separated by \n.
   * @param size The length of the buffer.
   * @return Success of compilation.
   */
    bool compileMofs(char *buffer, size_t size, std::vector<float> &motionData);

private:
    /**
   * The function replaces printf so that the output is written into a buffer.
   * @param format Normal printf format string.
   * @return Normal printf return value.
   */
    int myprintf(const char *format, ...);

    /**
   * Generate MotionNetData.cpp
   * @return True if successful
   */
    bool generateMotionNet(std::vector<float> &motionData);

    /**
   * Parse all mof files except extern.mof to generate motion net
   * @return True if successful
   */
    bool parseMofs();

    /**
   * Parse extern.mof to generate jump table from extern into motion net
   * @return True if successful
   */
    bool parseExternMof();
};

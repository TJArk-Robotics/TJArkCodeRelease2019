#pragma once

#define MAX_PACKAGE_SEND_SIZE 6000000
#define MAX_PACKAGE_RECEIVE_SIZE 3000000

#include "Modules/Infrastructure/Adapter.h"

#include "Tools/Communication/TransmissionServer.h"
#include "Platform/Nao/DebugHandler.h"

class Debug : Adapter
{
public:
    Debug(Blackboard* bb) : Adapter(bb) {}
    ~Debug() { server.socketDisconnect(); }
    void tick();

    void JPEGCompress(const cv::Mat &src, std::vector<uchar> &buff, int quality);

private:
    TransmissionServer server;
//     DebugHandler debugHandler;
};
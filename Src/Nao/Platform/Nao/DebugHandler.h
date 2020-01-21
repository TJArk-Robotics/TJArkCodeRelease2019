#pragma once

#include "Tools/Debugging/TcpConnection.h"

class DebugHandler : TcpConnection
{
private:
    void *in;                          /**< Incoming debug data is stored here. */
    void *out;                         /**< Outcoming debug data is stored here. */
    unsigned char *sendData = nullptr; /**< The data to send next. */
    int sendSize = 0;                  /**< The size of the data to send next. */

public:
    DebugHandler(void *in, void *out, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0);

    ~DebugHandler() { if (sendData) delete[] sendData; }

    void communicate(bool send);
};
#include "DebugHandler.h"
#include "Platform/BHAssert.h"

DebugHandler::DebugHandler(void *in, void *out, int maxPackageSendSize, int maxPackageReceiveSize) : TcpConnection(0, 0xA1BD, TcpConnection::receiver, maxPackageSendSize, maxPackageReceiveSize), in(in), out(out)
{
}

void DebugHandler::communicate(bool send)
{
    // if (send && !sendData && !out.isEmpty())
    // {
    //     sendSize = out.getStreamedSize();
    //     OutBinaryMemory memory(sendSize);
    //     memory << out;
    //     sendData = reinterpret_cast<unsigned char *>(memory.obtainData());
    //     out.clear();
    // }

    // unsigned char *receivedData;
    // int receivedSize = 0;

    // if (sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
    // {
    //     delete[] sendData;
    //     sendData = nullptr;
    //     sendSize = 0;
    // }

    // if (receivedSize > 0)
    // {
    //     InBinaryMemory memory(receivedData);
    //     memory >> in;
    //     delete[] receivedData;
    // }
}
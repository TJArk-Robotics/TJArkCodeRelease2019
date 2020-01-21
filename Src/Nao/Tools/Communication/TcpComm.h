#pragma once

#include <netinet/in.h>
#define socket_t int

class TcpComm
{
private:
    socket_t createSocket = 0;
    socket_t transferSocket = 0;
    sockaddr_in address;
    int overallBytesSent = 0;
    int overallBytesReceived = 0;
    int maxPackageSendSize;
    int maxPackageReceiveSize;
    bool wasConnected = false;

    bool checkConnection();

    void closeTransferSocket();

public:
    TcpComm(const char *ip, int port, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0);
    ~TcpComm();

    bool send(const unsigned char *buffer, int size);

    bool receive(unsigned char *buffer, int size, bool wait = true);

    int getOverallBytesSent() const { return overallBytesSent; }

    int getOverallBytesReceived() const { return overallBytesReceived; }

    bool connected() const { return transferSocket > 0; }
};
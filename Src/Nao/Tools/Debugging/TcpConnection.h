#pragma once

#include "Tools/Communication/TcpComm.h"
#include <memory>

#define MAX_PACKAGE_SIZE 67108864 // max package size that can be received. prevent from allocating too much buffer (max ~64 MB)

class TcpConnection
{
public:
    enum Handshake
    {
        noHandshake,
        sender,
        receiver
    };

private:
    std::unique_ptr<TcpComm> tcpComm; /**< The TCP/IP connection. */
    bool ack = false;
    bool client = false;
    Handshake handshake = noHandshake; /**< The handshake mode. */

public:
    TcpConnection() = default;

    TcpConnection(const char *ip, int port, Handshake handshake = noHandshake, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0)
    {
        connect(ip, port, handshake, maxPackageSendSize, maxPackageReceiveSize);
    }

    void connect(const char *ip, int port, Handshake Handshake = noHandshake, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0);
    
    bool sendAndReceive(const unsigned char *dataToSend, int sendSize, unsigned char *&dataRead, int &readSize);

    bool isConnected() const { return tcpComm && tcpComm->connected(); }

    bool isClient() const { return client; }

    int getOverallBytesSent() const { return tcpComm ? tcpComm->getOverallBytesSent() : 0; }

    int getOverallBytesReceived() const { return tcpComm ? tcpComm->getOverallBytesReceived() : 0; }

    bool sendHeartbeat();

private:
    int receive(unsigned char* &buffer);
};
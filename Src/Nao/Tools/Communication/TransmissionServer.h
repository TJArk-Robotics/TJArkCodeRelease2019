#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class TransmissionServer
{
public:
    TransmissionServer() {}
    ~TransmissionServer() { socketDisconnect(); }
    int socketFd;
    int connFd;

private:
    int needRecv;
    int count;

public:
    // open socket
    int socketConnect(int PORT);
    // transmit image
    int transmit(char *buff, int len);
    // close socket
    void socketDisconnect();
};
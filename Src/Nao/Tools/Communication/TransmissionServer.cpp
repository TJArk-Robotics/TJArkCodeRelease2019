#include "TransmissionServer.h"

int TransmissionServer::socketConnect(int PORT)
{
    socketFd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in serverSockAddr;
    serverSockAddr.sin_family = AF_INET;
    serverSockAddr.sin_port = htons(PORT);
    serverSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socketFd, (struct sockaddr*)&serverSockAddr, sizeof(serverSockAddr)) == -1)
    {
        perror("bind");
        return -1;
    }

    if (listen(socketFd, 5) == -1)
    {
        perror("listen");
        return -1;
    }

    struct sockaddr_in clientAddr;
    socklen_t len = sizeof(clientAddr);
    connFd = accept(socketFd, (struct sockaddr*)&clientAddr, &len);

    if (connFd < 0)
    {
        perror("connect\n");
        return -1;
    }
    else
    {
        printf("connect successful!\n");
        return 1;
    }

    close(socketFd);
}

void TransmissionServer::socketDisconnect()
{
    close(socketFd);
    close(connFd);
}

int TransmissionServer::transmit(char *data, int len)
{
        if (send(connFd, data, len, 0) < 0)
            // printf("send image error: %s(errno: %d)\n", strerror(errno), errno);
            return -1;
}
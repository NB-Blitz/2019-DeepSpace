#include "Socket.hpp"

using namespace std;

void Blitz::Socket::Open()
{
    pthread_create(&threads, NULL, Server, (void *)output);
}

void *Blitz::Socket::Server(void * args)
{
    string * output = (string *) args;

    unsigned short port = 5800;
    int sock;
    struct sockaddr_in socketAddress;

    socketAddress.sin_family = AF_INET;
    socketAddress.sin_port = htons(port);
    socketAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    bind(sock, (struct sockaddr *)&socketAddress, sizeof(socketAddress));

    while(true)
    {
        char buffer[256];

        recv(sock, buffer, 256, 0);

        string data(buffer);

        *output = data;
    }

    close(sock);

    pthread_exit(NULL);
}
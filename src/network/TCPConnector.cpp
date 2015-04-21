#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "TCPConnector.h"

namespace   Network
{

TCPConnector::TCPConnector()
{
}

TCPConnector::~TCPConnector()
{
    if (_socket != -1)
        this->disconnect();
}

bool    TCPConnector::isConnected() const
{
    return (_socket != -1);
}

int     TCPConnector::getSocket() const
{
    return (_socket);
}

bool    TCPConnector::initSockAddr(std::string const &ip, unsigned short port)
{
    int n = 1;
    _socket = socket(PF_INET, SOCK_STREAM, 0);
    memset(&_sa, 0, sizeof(struct sockaddr_in));
    _sa.sin_family = PF_INET;
    _sa.sin_port = htons(port);
    _sa.sin_addr.s_addr = ip.empty() ? INADDR_ANY : inet_addr(ip.c_str());
    setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(int));
    return (true);
}

bool    TCPConnector::connectTo(const std::string &ip, unsigned short port)
{
    initSockAddr(ip, port);
    if (connect(_socket, (const sockaddr*)&_sa, sizeof(_sa)) < 0)
        return (false);
    return (true);
}

void TCPConnector::disconnect()
{
    std::cerr << "DISCONNECTED" << std::endl; // THIS IS DEBUG
    ::close(_socket);
    _socket = -1;
}

}

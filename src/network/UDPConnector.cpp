#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "UDPConnector.hh"

namespace   Network
{

UDPConnector::UDPConnector()
{
}

UDPConnector::~UDPConnector()
{
    if (_socket != -1)
        this->disconnect();
}

bool    UDPConnector::isConnected() const
{
    return (_socket != -1);
}

int     UDPConnector::getSocket() const
{
    return (_socket);
}

bool    UDPConnector::initSockAddr(std::string const &ip, unsigned short port)
{
    int n = 1;
    _socket = socket(PF_INET, SOCK_DGRAM, 0);
    memset(&_sa, 0, sizeof(struct sockaddr_in));
    _sa.sin_family = PF_INET;
    _sa.sin_port = htons(port);
    _sa.sin_addr.s_addr = ip.empty() ? INADDR_ANY : inet_addr(ip.c_str());
    setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(int));
    return (true);
}

bool    UDPConnector::connectTo(const std::string &ip, unsigned short port)
{
    initSockAddr(ip, port);
    if (connect(_socket, (const sockaddr*)&_sa, sizeof(_sa)) < 0)
        return (false);
    return (true);
}

void UDPConnector::disconnect()
{
    std::cerr << "DISCONNECTED UDP" << std::endl; // THIS IS DEBUG
    ::close(_socket);
    _socket = -1;
}

}

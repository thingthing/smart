#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "TCPConnector.h"

namespace   Network
{

TCPConnector::TCPConnector() :
    _rxBuffer(MAX_RX_BUFFER_SIZE),
    _txBuffer(MAX_TX_BUFFER_SIZE)
{
}

TCPConnector::~TCPConnector()
{
    if (_socket != -1)
        close(_socket);
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
    _fdset = (struct pollfd){_socket, POLLIN, 0};
    this->dispatch("ConnectedEvent");
    return (true);
}

bool            TCPConnector::send(const std::string &data)
{
    if (data.size() <= _txBuffer.getSpaceLeft())
    {
        _txBuffer.write(&(data[0]), data.size());
        _fdset.events |= POLLOUT;
        return (true);
    }
    return (false);
}

void            TCPConnector::run()
{
    // TODO one day : try to reconnect in case it fails ?
    if (_socket != -1)
    {
        if (poll(&_fdset, 1, 100) > 0)
        {
            if (_fdset.revents & POLLIN)
            {
                if ((_byteRead = _rxBuffer.readFrom(_socket)) > 0)
                {
                    _byteRead = _rxBuffer.getSpaceUsed();
                    for (unsigned int i = 0; i < _rxBuffer.getSpaceUsed(); ++i) // This is horrible. And we have to change that when we will deal with data blobs w/ the server anyway.
                    {
                        if (*(((const char *)_rxBuffer.peek()) + i) == '\n')
                        {
                            _rxBuffer.poke(i, "\0", 1);
                            this->dispatch("ReceivePacketEvent", _rxBuffer); // Horrible, but it will do the trick for now.
                        }
                    }
                    if (_rxBuffer.getSpaceLeft() == 0)              // Some random data, drop it. Should not happen.
                        _rxBuffer.reset();
                }
                else
                {
                    std::cerr << "DISCONNECTED" << std::endl; // THIS IS DEBUG
                    ::close(_socket);
                    _socket = -1;
                    this->dispatch("DisconnectEvent");
                }
            }
            else if (_fdset.revents & POLLOUT)
            {
                if ((_byteWritten = _txBuffer.writeTo(_socket)) > 0)
                {
                    if (_txBuffer.getSpaceUsed() == 0)
                    {
                        _fdset.events &= ~POLLOUT;              // A race condition with send may arise here, but we don't care for now
                        _txBuffer.reset();
                    }
                }
            }
        }
    }
}

}

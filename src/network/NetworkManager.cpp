#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "NetworkManager.hh"

namespace   Network
{

NetworkManager::NetworkManager(IConnector *connector) :
    _rxBuffer(MAX_RX_BUFFER_SIZE),
    _txBuffer(MAX_TX_BUFFER_SIZE),
    _connector(connector)
{
}

NetworkManager::~NetworkManager()
{
    delete _connector;
}

IConnector *NetworkManager::getConnector() const
{
    return (_connector);
}

bool    NetworkManager::connectTo(const std::string &ip, unsigned short port)
{
    if (_connector->connectTo(ip, port) == false)
        return (false);
    _fdset = (struct pollfd){_connector->getSocket(), POLLIN, 0};
    this->dispatch("ConnectedEvent");
    return (true);
}

void    NetworkManager::disconnect()
{
    _connector->disconnect();
    this->dispatch("DisconnectEvent");
}

bool            NetworkManager::send(const std::string &data)
{
    if (data.size() <= _txBuffer.getSpaceLeft())
    {
        _txBuffer.write(&(data[0]), data.size());
        _fdset.events |= POLLOUT;
        return (true);
    }
    return (false);
}

void            NetworkManager::run()
{
    // TODO one day : try to reconnect in case it fails ?
    if (_connector->isConnected() == true)
    {
        if (poll(&_fdset, 1, 100) > 0)
        {
            if (_fdset.revents & POLLIN)
            {
                if ((_byteRead = _rxBuffer.readFrom(_connector->getSocket())) > 0)
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
                    _connector->disconnect();
                }
            }
            else if (_fdset.revents & POLLOUT)
            {
                if ((_byteWritten = _txBuffer.writeTo(_connector->getSocket())) > 0)
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

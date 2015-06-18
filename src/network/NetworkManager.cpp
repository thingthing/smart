#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "NetworkManager.hh"

namespace   Network
{

NetworkManager::NetworkManager()
{
    _packet.getPacketHeader().packetSize = 0;
}

NetworkManager::~NetworkManager()
{
    for (std::map<std::string, IConnector *>::iterator it = _connectors.begin();
         it != _connectors.end(); ++it)
    {
        delete it->second;
    }
}

IConnector *NetworkManager::getConnector(std::string const &connector_id) const
{
    if (_connectors.find(connector_id) != _connectors.end())
        return (_connectors.at(connector_id));
    return (NULL);
}

void NetworkManager::setConnector(std::string const &key, IConnector *connector)
{
    ///@todo: Check if key exists?
    _connectors[key] = connector;
}

bool    NetworkManager::connectTo(const std::string &ip, unsigned short port)
{
    unsigned int fdsetIndex = 0;

    for (std::map<std::string, IConnector *>::iterator it = _connectors.begin();
         it != _connectors.end(); ++it)
    {
        if (it->second->connectTo(ip, port) == false)
        {
            std::cerr << "Connection to " << it->first << " failed" << std::endl;
            return (false);
        }
        _fdset[fdsetIndex] = (struct pollfd) {it->second->getSocket(), POLLIN, 0};
        _fdsetList.insert(std::pair<std::string, pollfd &>(it->first, _fdset[fdsetIndex]));
        ++fdsetIndex;
    }
    this->dispatch("ConnectedEvent");
    return (true);
}

bool    NetworkManager::connectTo(const std::string &ip, unsigned short port, const std::string &connector_id)
{
    IConnector *connector;

    if ((connector = this->getConnector(connector_id)) == NULL)
        return (false);
    if (connector->connectTo(ip, port) == false)
        return (false);

    unsigned int fdsetIndex = _fdsetList.size();
    _fdset[fdsetIndex] = (struct pollfd) {connector->getSocket(), POLLIN, 0};
    _fdsetList.insert(std::pair<std::string, pollfd &>(connector_id, _fdset[fdsetIndex]));

    this->dispatch("ConnectedEvent");
    return (true);
}

void    NetworkManager::disconnect()
{
    for (std::map<std::string, IConnector *>::iterator it = _connectors.begin();
         it != _connectors.end(); ++it)
    {
        this->disconnect(it->first);
    }
}

void    NetworkManager::disconnect(const std::string &connector_id)
{
    IConnector *connector;

    if ((connector = this->getConnector(connector_id)) != NULL)
    {
        if (connector->isConnected())
        {
            connector->disconnect();
            this->dispatch("DisconnectEvent");
        }
    }
}

bool            NetworkManager::send(const Network::APacketBase &packet, const std::string &connector_id)
{
    std::cout << "try to send some data" << std::endl;
    if (_connectors.at(connector_id)->isConnected() == false)
    {
        std::cerr << "Try to send to "<< connector_id << " wich is not connected" << std::endl;
        return (false);
    }
    if (packet.getPacketSize() <= _connectors.at(connector_id)->getWriteBuffer().getSpaceLeft())
    {
        _connectors.at(connector_id)->getWriteBuffer().write(packet.data(), packet.getPacketSize());
        ///@todo: check if fd exists
        (_fdsetList.at(connector_id)).events |= POLLOUT;
        return (true);
    }
    return (false);
}

void            NetworkManager::run()
{
    // TODO one day : try to reconnect in case it fails ?
    //if (_connector->isConnected() == true)
    //{
    if (poll(_fdset, _fdsetList.size(), 100) > 0)
    {
        std::cout << "Poll start" << std::endl;
        for (std::map<std::string, pollfd &>::iterator it = _fdsetList.begin();
             it != _fdsetList.end(); ++it)
        {
            IConnector *connector = _connectors[it->first];
            if (it->second.revents & POLLIN)
            {
                std::cout << "Pollin for is == " << it->first << std::endl;
                if ((_byteRead = connector->getReadBuffer().readFrom(connector->getSocket())) > 0)
                {
                    _byteRead = connector->getReadBuffer().getSpaceUsed();
                    if (_packet.getPacketHeader().packetSize == 0)
                    {
                        if (_byteRead > (int)sizeof(Network::ComPacket))
                            connector->getReadBuffer() >> _packet.getPacketHeader();
                    }
                    else
                    {
                        if (_packet.getPacketHeader().packetSize < _packet.getPacketSize())
                        {
                            const int      bytesMissing = _packet.getPacketHeader().packetSize - _packet.getPacketSize();
                            _packet.appendFromCircularBuffer(connector->getReadBuffer(), (_byteRead > (bytesMissing)) ? bytesMissing : _byteRead);
                        }
                        if (_packet.getPacketHeader().packetSize == _packet.getPacketSize())
                        {
                            this->dispatch("ReceivePacketEvent", std::ref(_packet));
                            _packet.getPacketHeader().packetSize = 0;
                        }
                    }
                    if (connector->getReadBuffer().getSpaceLeft() == 0)                                 // Some random data, drop it. Should never happen.
                        connector->getReadBuffer().reset();
                }
                else
                    connector->disconnect();
            }
            else if ((it->second.revents & POLLOUT))
            {
                std::cout << "Pollout for is == " << it->first << std::endl;
                if (!connector->isConnected())
                    std::cout << "Try to write on socket not connected : " << it->first<< std::endl;
                std::cout << "Writting on socket : " <<  connector->getSocket() << std::endl;
                if ((_byteWritten = connector->getWriteBuffer().writeTo(connector->getSocket())) > 0)
                {
                    std::cout << "Byte written" << std::endl;
                    if (connector->getWriteBuffer().getSpaceUsed() == 0)
                    {
                        std::cout << "Reset buffer" << std::endl;
                        it->second.events &= ~POLLOUT;              // A race condition with send may arise here, but we don't care for now
                        connector->getWriteBuffer().reset();
                    }
                }
            }

        }
        //}
    }
}

}

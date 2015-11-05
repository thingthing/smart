#include <errno.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <iostream>
#include <bitset>
#include "NetworkManager.hh"

namespace   Network
{

NetworkManager::NetworkManager()
{
    _packet.clear();
    // std::cout << "In constructor packet initiated size in header = " << _packet.getPacketHeader().packetSize << " real size == " << _packet.getPacketSize() << std::endl;
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

bool    NetworkManager::connectTo(const std::string &ip, unsigned short port, const std::string &connector_id, IAgent *agent)
{
    IConnector *connector;

    if ((connector = this->getConnector(connector_id)) == NULL)
        return (false);
    if (connector->connectTo(ip, port) == false)
        return (false);

    unsigned int fdsetIndex = _fdsetList.size();
    _fdset[fdsetIndex] = (struct pollfd) {connector->getSocket(), POLLIN, 0};
    _fdsetList.insert(std::pair<std::string, pollfd &>(connector_id, _fdset[fdsetIndex]));

    this->dispatch("ConnectedEvent", agent);
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
    //std::cout << "try to send some packet" << std::endl;
    if (_connectors.at(connector_id)->isConnected() == false)
    {
        std::cerr << "Try to send to " << connector_id << " wich is not connected" << std::endl;
        return (false);
    }
    if (packet.getPacketSize() <= _connectors.at(connector_id)->getWriteBuffer().getSpaceLeft())
    {
        Network::ComPacket packet_cpy;
        struct s_ComPacketHeader header = (*(s_ComPacketHeader *)packet.data());
        std::bitset<16> magic(header.magic);
        std::bitset<16> packetSize(header.packetSize);
        std::bitset<16> version(header.version);
        std::bitset<16> headerSize(header.headerSize);

        packet_cpy.append(packet.data() + sizeof(Network::s_ComPacketHeader), packet.getPacketSize() - sizeof(Network::s_ComPacketHeader));
        packet_cpy.getPacketHeader().magic = htons(header.magic);
        packet_cpy.getPacketHeader().packetSize = htons(header.packetSize);
        packet_cpy.getPacketHeader().version = htons(header.version);
        packet_cpy.getPacketHeader().headerSize = htons(header.headerSize);
        //  std::cout << "sending :: magic = " << magic << " -- packetsize == " << packetSize << " -- version == " << version << " -- header size == " << headerSize << std::endl;
        // std::cout << "Packet size is == " << packet.getPacketSize() << std::endl;
        // std::cout << "Packet size minus header == " << packet.getPacketSize() - sizeof(Network::s_ComPacketHeader) << std::endl;
        _connectors.at(connector_id)->getWriteBuffer().write(packet_cpy.data(), packet_cpy.getPacketSize());
        ///@todo: check if fd exists
        (_fdsetList.at(connector_id)).events |= POLLOUT;
        return (true);
    }
    return (false);
}

bool            NetworkManager::send(const std::string &chunk, const std::string &connector_id)
{
    // std::cout << "try to send some string" << std::endl;
    if (_connectors.at(connector_id)->isConnected() == false)
    {
        std::cerr << "Try to send to " << connector_id << " wich is not connected" << std::endl;
        return (false);
    }
    unsigned int chunk_size = chunk.size();
    // std::cout << "Chunk size in udp is == " << chunk_size << std::endl;
    if (chunk_size <= _connectors.at(connector_id)->getWriteBuffer().getSpaceLeft())
    {
        // std::cout << "Sending chunk" << std::endl;
        _connectors.at(connector_id)->getWriteBuffer().write(chunk.c_str(), chunk_size);
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
    if (poll(_fdset, _fdsetList.size(), 0) > 0)
    {
        for (std::map<std::string, pollfd &>::iterator it = _fdsetList.begin();
                it != _fdsetList.end(); ++it)

        {
            IConnector *connector = _connectors[it->first];

            if (it->second.revents & POLLIN)
            {
                std::cerr << "Pollin for is == " << it->first << std::endl;
                if ((_byteRead = connector->getReadBuffer().readFrom(connector->getSocket())) > 0)
                {
                    _byteRead = connector->getReadBuffer().getSpaceUsed();
                    /// Check if packet has minimum size: packet header size
                    if (_packet.getPacketHeader().packetSize == sizeof(Network::s_ComPacketHeader))
                    {
                        std::cerr << "Packet size is minimum so set header with buffer" << std::endl;
                        if (_byteRead > (int)sizeof(Network::ComPacket))
                        {
                            unsigned short tmp;
                            connector->getReadBuffer() >> tmp;
                            _packet.getPacketHeader().magic = ntohs(tmp);
                            connector->getReadBuffer() >> tmp;
                            _packet.getPacketHeader().packetSize = ntohs(tmp);
                            connector->getReadBuffer() >> tmp;
                            _packet.getPacketHeader().version = ntohs(tmp);
                            connector->getReadBuffer() >> tmp;
                            _packet.getPacketHeader().headerSize = ntohs(tmp);
                        }
                    }
                    //else
                    //{
                    // std::cout << "Packet initiated size in header = " << _packet.getPacketHeader().packetSize << " real size == " << _packet.getPacketSize() << std::endl;
                    if (_packet.getPacketHeader().packetSize > _packet.getPacketSize())
                    {
                        const int      bytesMissing = _packet.getPacketHeader().packetSize - _packet.getPacketSize();
                        // std::cout << "Byte missing in packet add " << bytesMissing << " bytes" << std::endl;
                        _packet.appendFromCircularBuffer(connector->getReadBuffer(), (_byteRead > (bytesMissing)) ? bytesMissing : _byteRead);
                        // std::cout << "After append packet initiated size in header = " << _packet.getPacketHeader().packetSize << " real size == " << _packet.getPacketSize() << std::endl;
                    }
                    if (_packet.getPacketHeader().packetSize == _packet.getPacketSize())
                    {
                        std::cout << "Packet completly recieved, dispatch event" << std::endl;
                        this->dispatch("ReceivePacketEvent", &_packet);
                        // std::cout << "After dispatching" << std::endl;
                        _packet.clear();
                    }
                    //}
                    if (connector->getReadBuffer().getSpaceLeft() == 0)                                 // Some random data, drop it. Should never happen.
                    {
                        std::cerr << "Reseting read buffer" << std::endl;
                        connector->getReadBuffer().reset();
                    }
                }
                else
                    connector->disconnect();
            }

            if ((it->second.revents & POLLOUT))
            {
                // std::cout << "Pollout for is == " << it->first << std::endl;
                if (!connector->isConnected())
                    std::cerr << "Try to write on socket not connected : " << it->first << std::endl;
                // std::cout << "Writting on socket : " <<  connector->getSocket() << std::endl;
                if ((_byteWritten = connector->getWriteBuffer().writeTo(connector->getSocket())) > 0)
                {
                    // std::cout << "Byte written" << std::endl;
                    if (connector->getWriteBuffer().getSpaceUsed() == 0)
                    {
                        std::cerr << "Reseting write buffer" << std::endl;
                        it->second.events &= ~POLLOUT;              // A race condition with send may arise here, but we don't care for now
                        connector->getWriteBuffer().reset();
                    }
                }
            }

        }

    }
}
}



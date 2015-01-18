#include <errno.h>
#include <stdlib.h>

#include <iostream>

#include "TCPConnector.h"

namespace   Network
{

TCPConnector::TCPConnector(AProtocol *protocol) :
    _rxBuffer(MAX_RX_BUFFER_SIZE),
    _txBuffer(MAX_TX_BUFFER_SIZE)
{
    if (protocol)
        this->setProtocol(*protocol);
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
    return (true);
}

bool            TCPConnector::send(const Packet &packet)
{
    if ((packet.getDataSize() + sizeof(int)) <= _txBuffer.getSpaceLeft())
    {
        _txBuffer << packet.getDataSize();
        _txBuffer.write(packet.getData(), packet.getDataSize());
        return (true);
    }
    return (false);
}

void            TCPConnector::run()
{
    int         byteRead;
    int         byteWritten;
    int         tmpRxPacketSize = -1;
    Packet      tmpRxPacket;

    // TODO : try to reconnect in case it fails ?

    while (_socket != -1)
    {
        if (poll(&_fdset, 1, 100) > 0)
        {
            if (_fdset.revents & POLLIN)
            {
                if ((byteRead = _rxBuffer.readFrom(_socket)) > 0)
                {
                    byteRead = _rxBuffer.getSpaceUsed();
                    if ((tmpRxPacketSize == -1) && ((unsigned int)byteRead >= sizeof(int)))
                    {
                        _rxBuffer >> tmpRxPacketSize;
                        byteRead -= sizeof(tmpRxPacketSize);
                        std::cout << "read the size of the pckt:" << tmpRxPacketSize << std::endl;
                    }
                    if ((byteRead) && (tmpRxPacketSize != -1))
                    {
                        const int bytesToWriteInPacket = (byteRead > tmpRxPacketSize) ? tmpRxPacketSize : byteRead;
                        tmpRxPacket.append(_rxBuffer.peek(bytesToWriteInPacket), bytesToWriteInPacket);
                        tmpRxPacketSize -= bytesToWriteInPacket;
                        if (tmpRxPacketSize == 0)                           // packet reception complete
                        {
                            _protocol->receivePacketEvent(tmpRxPacket);
                            tmpRxPacketSize = -1;
                            tmpRxPacket.clear();
                            std::cout << "packet complet" << std::endl;
                        }
                        std::cout << "bytesToWriteInPacket:" << bytesToWriteInPacket << std::endl;
                    }
                    if (_rxBuffer.getSpaceLeft() == 0)              // Some random data, drop it. Should not happen.
                    {
                        _rxBuffer.reset();
                    }
                }
                else
                {
                    std::cerr << "DISCONNECT" << std::endl; // DEBUG
                    ::close(_socket);
                    _socket = -1;
                    _protocol->disconnectEvent();
                    continue;
                }
            }
            else if (_fdset.revents & POLLOUT)
            {
                if ((byteWritten = _txBuffer.writeTo(_socket)) > 0)
                {
                    if (_txBuffer.getSpaceUsed() == 0)
                    {
                        _fdset.events ^= POLLOUT;
                        _txBuffer.reset();
                    }
                }
            }
        }
    }
}

// TODO TODO TODO TODO TODO

void            TCPConnector::start()       // put this in ANetworkAdapter ?
{
    run(); // will be changed
}

void            TCPConnector::stop()
{
    //disconnect
}

}

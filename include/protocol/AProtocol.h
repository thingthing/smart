#ifndef     IPROTOCOL_H_
# define    IPROTOCOL_H_

#include "CircularBuffer.h"
#include "Packet.h"

namespace Network { class ANetworkAdapter; }

class       AProtocol
{
public:
    AProtocol(Network::ANetworkAdapter &networkAdapter) :
        _networkAdapter(&networkAdapter)
    {}
    virtual void        setNetHandler(Network::ANetworkAdapter &handler)
    {
        _networkAdapter = &handler;
    }

    virtual void        connectedEvent() = 0;
    virtual void        receivePacketEvent(Network::Packet &packet) = 0;
    virtual void        disconnectEvent() = 0;

private:
    AProtocol(){}
    Network::ANetworkAdapter          *_networkAdapter;
};

#endif

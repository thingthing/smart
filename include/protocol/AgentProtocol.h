#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "Packet.h"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::ANetworkAdapter &networkAdapter) :
        AProtocol(networkAdapter)
    {}

    enum            e_events
    {
        HANDSHAKE = 0,
        HANDSHAKE_ACK,
        START_ACK,
        e_events_count
    };

    virtual ~AgentProtocol(){}

    virtual void        connectedEvent();
    virtual void        receivePacketEvent(Network::Packet &packet);
    virtual void        disconnectEvent();

protected:
    AgentProtocol();
};

#endif

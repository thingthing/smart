#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "Packet.h"
#include "Agent.hh"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::ANetworkAdapter &networkAdapter, Agent &agent) :
        AProtocol(networkAdapter),
        _agent(agent)
    {}

    virtual ~AgentProtocol(){}

    virtual void        connectedEvent();
    virtual void        receivePacketEvent(Network::CircularBuffer &packet);
    virtual void        disconnectEvent();

    void                sendMovement();

protected:
    AgentProtocol();

    Agent               &_agent;
};

#endif

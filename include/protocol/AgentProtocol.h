#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "network/ComPacket.h"
#include "Agent.hh"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::NetworkManager &networkAdapter);

    virtual ~AgentProtocol();

    virtual void        connectedEvent();
    virtual void        receivePacketEvent(Network::ComPacket &packet);
    virtual void        disconnectEvent();
    virtual void        sendPacketEvent();

    void         setAgent(Agent &agent);

protected:
    AgentProtocol();

    Agent               *_agent;
    Network::ComPacket   _outPacket;
};

#endif

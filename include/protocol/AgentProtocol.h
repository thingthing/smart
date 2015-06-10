#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "Packet.h"
#include "Agent.hh"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::NetworkManager &networkAdapter);

    virtual ~AgentProtocol();

    virtual void        connectedEvent();
    virtual void        receivePacketEvent(Network::CircularBuffer &packet);
    virtual void        disconnectEvent();
    virtual void        sendPacketEvent();

    void         setAgent(Agent &agent);

protected:
    AgentProtocol();

    Agent               *_agent;
};

#endif

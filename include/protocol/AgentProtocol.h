#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "network/ComPacket.h"
#include "Agent.hh"
#include "json/json.h"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::NetworkManager &networkAdapter);

    virtual ~AgentProtocol();

    virtual void        connectedEvent();
    virtual void        receivePacketEvent(Network::ComPacket &packet);
    virtual void        disconnectEvent();
    virtual void        sendPacketEvent();
    void                sendStatusEvent(std::string const &status);

    void         setAgent(Agent &agent);

private:
    pcl::PointXYZ       getPosFromJson(Json::Value const &root);

protected:
    AgentProtocol();

    Agent               *_agent;
    Network::ComPacket   _outPacket;
};

#endif

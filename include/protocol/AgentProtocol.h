#ifndef     AGENT_PROTOCOL_H_
# define    AGENT_PROTOCOL_H_

#include "AProtocol.h"
#include "network/ComPacket.h"
#include "Agent.hh"
#include "Slam.hh"
#include "ChunkFactory.h"
#include "json/json.h"

class       AgentProtocol : public AProtocol
{
public:
    AgentProtocol(Network::NetworkManager &networkAdapter);

    virtual ~AgentProtocol();

    virtual void        connectedEvent(IAgent *);
    virtual void        receivePacketEvent(Network::ComPacket *packet);
    virtual void        disconnectEvent();
    virtual void        sendPacketEvent(IAgent *);
    void                sendDataTcp(Json::Value &root);
    void                sendCloudEvent(pcl::PointCloud<pcl::PointXYZ> const &cloud);
    void                sendNewLandmarkEvent(std::vector<Landmarks::Landmark *> &nl);
    void                sendStatusEvent(std::string const &status);


    void         setAgent(Agent &agent, Slam &slam);

    static const std::string TCP_KEY;
    static const std::string UDP_KEY;

private:
    pcl::PointXYZ       getPosFromJson(Json::Value const &root);

protected:
    AgentProtocol();

    Network::ChunkFactory        _factory;
    Network::ComPacket   _outPacket;
};

#endif

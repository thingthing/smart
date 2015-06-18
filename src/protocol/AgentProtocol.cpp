#include <string>

#include "AgentProtocol.h"
#include "json/json.h"
#include "NetworkManager.hh"
#include "IConnector.hh"
#include "TCPConnector.h"
#include "UDPConnector.hh"

static const std::string TCP_KEY = "TCP";
static const std::string UDP_KEY = "UDP";


AgentProtocol::AgentProtocol(Network::NetworkManager &networkAdapter)
    : AProtocol(networkAdapter)
{
    Network::IConnector *connector = new Network::TCPConnector();
    _networkAdapter.setConnector(TCP_KEY, connector);
}

AgentProtocol::~AgentProtocol()
{
}

void         AgentProtocol::setAgent(Agent &agent)
{
    _agent = &agent;
    this->registerCallback("SetGoalPosEvent", [this](pcl::PointXYZ pos){_agent->setGoalPos(pos);});
    _agent->registerCallback("SendPacketEvent", [this](){sendPacketEvent();});
}

void        AgentProtocol::connectedEvent()
{
    std::cout << "connected event " << std::endl;
    Json::Value     root;
    root["data"]["name"] = _agent->name();
    root["data"]["position"]["x"] = _agent->getPos().x;
    root["data"]["position"]["y"] = _agent->getPos().y;
    root["data"]["position"]["z"] = _agent->getPos().z;
    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    _outPacket.clear();
    _outPacket.append(root.toStyledString().c_str(), root.toStyledString().size());       // TODO : do something to handle strings directly with << in APacket.
    _networkAdapter.send(_outPacket, TCP_KEY);
}

void        AgentProtocol::sendPacketEvent()
{
    Json::Value     root;

    root["data"]["position"]["x"] = _agent->getPos().x;
    root["data"]["position"]["y"] = _agent->getPos().y;
    root["data"]["position"]["z"] = _agent->getPos().z;
    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    _outPacket.append(root.toStyledString().c_str(), root.toStyledString().size());
    std::cout << "Send movement event " << std::endl;
    ///@todo: Really send position
    _networkAdapter.send(_outPacket, TCP_KEY);
    _outPacket.clear();
}

void        AgentProtocol::receivePacketEvent(Network::ComPacket &packet)      // only for test 4 now, will change
{
    Json::Reader        reader;
    Json::Value         root;
    std::string         serverReply((const char *)packet.data() + sizeof(Network::s_ComPacketHeader), packet.getPacketSize() - sizeof(Network::s_ComPacketHeader));

    std::cout << "received message from server " << serverReply << std::endl;
    if (reader.parse(serverReply, root, false) == true)
    {
        std::cout << "received a movement order " << serverReply << std::endl;
        pcl::PointXYZ pos = (pcl::PointXYZ) {
                root["data"]["position"].get("x", 0.0).asFloat(),
                root["data"]["position"].get("y", 0.0).asFloat(),
                root["data"]["position"].get("z", 0.0).asFloat()
    };
    this->dispatch("SetGoalPosEvent", pos); // And the agent should subscribes to the events.
    }
    else
        std::cout << "error while parsing order " << serverReply << ": " << reader.getFormattedErrorMessages() << std::endl;
}

void        AgentProtocol::disconnectEvent()
{

}

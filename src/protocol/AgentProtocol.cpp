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
    Json::Value     reply;
    //reply["name"] = _agent->name();
    _networkAdapter.send("name:" + _agent->name() + "\n", TCP_KEY);
    /*reply["position"]["x"] = _agent->getPos().x;
    reply["position"]["y"] = _agent->getPos().y;
    reply["position"]["z"] = _agent->getPos().z;
    _networkAdapter.send(reply.toStyledString(), TCP_KEY);*/
}

void        AgentProtocol::sendPacketEvent()
{
    std::cout << "Send movement event " << std::endl;
    ///@todo: Really send position
    _networkAdapter.send("position:{\"x\":" + std::to_string(_agent->getPos().x) + ", \"y\":" + std::to_string(_agent->getPos().y) + ", \"z\":" + std::to_string(_agent->getPos().z) + "}\n", TCP_KEY);
}

void        AgentProtocol::receivePacketEvent(Network::CircularBuffer &packet)      // only for test 4 now, will change
{
    Json::Reader        reader;
    Json::Value         root;
    std::string         serverReply((const char *)packet.peek());
    size_t              posColumn = serverReply.find_first_of(":");
    packet.peek(serverReply.size() + 1);
    std::cout << "received message from serveur " << serverReply << std::endl;
    if (posColumn != serverReply.npos)
    {
        if (reader.parse(serverReply.substr(posColumn + 1), root, false) == true)
        {
            std::cout << "received a movement order " << serverReply << std::endl;
            /*_agent.setGoalPos(root.get("x", 0.0).asDouble(),
                              root.get("y", 0.0).asDouble(),
                              root.get("z", 0.0).asDouble());*/
            pcl::PointXYZ pos = (pcl::PointXYZ) {
                root.get("x", 0.0).asFloat(),
                root.get("y", 0.0).asFloat(),
                root.get("z", 0.0).asFloat()
            };
            this->dispatch("SetGoalPosEvent", pos); // And the agent should subscribes to the events.
        }
        else
            std::cout << "error while parsing order " << serverReply << ": " << reader.getFormatedErrorMessages() << std::endl;
    }
}

void        AgentProtocol::disconnectEvent()
{

}
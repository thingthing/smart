#include <string>

#include "AgentProtocol.h"
#include "json/json.h"
#include "NetworkManager.hh"

AgentProtocol::AgentProtocol(Network::NetworkManager &networkAdapter)
    : AProtocol(networkAdapter)
{
}

AgentProtocol::~AgentProtocol()
{
}

void        AgentProtocol::connectedEvent()
{
    std::cout << "connected event " << std::endl;
    Json::Value     reply;
    reply["name"] = _agent->name();
    reply["position"]["x"] = _agent->getPos().x;        // todo : Move this in a "sendAgentInfo" function or something like this
    reply["position"]["y"] = _agent->getPos().y;        // And dispatch a "newConnection" event, or smthn like this to the agent.
    reply["position"]["z"] = _agent->getPos().z;
    _networkAdapter.send(reply.toStyledString());
}

void        AgentProtocol::receivePacketEvent(Network::CircularBuffer &packet)      // only for test 4 now, will change
{
    Json::Reader        reader;
    Json::Value         root;
    std::string         serverReply((const char *)packet.peek());
    size_t              posColumn = serverReply.find_first_of(":");
    packet.peek(serverReply.size() + 1);

    if (posColumn != serverReply.npos)
    {
        if (reader.parse(serverReply.substr(posColumn + 1), root, false) == true)
        {
            std::cout << "received a movement order " << serverReply << std::endl;
            /*_agent.setGoalPos(root.get("x", 0.0).asDouble(),
                              root.get("y", 0.0).asDouble(),
                              root.get("z", 0.0).asDouble());*/
            this->dispatch("AGivenCommand"/*, params*/); // And the agent should subscribes to the events.
        }
        else
            std::cout << "error while parsing order : " << reader.getFormatedErrorMessages()<< std::endl;
    }
}

void        AgentProtocol::disconnectEvent()
{

}

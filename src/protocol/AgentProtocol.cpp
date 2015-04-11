#include <string>

#include "AgentProtocol.h"

#include "json/json.h"
#include "network/ANetworkAdapter.h"

void        AgentProtocol::connectedEvent()
{
    // Send agent infos.
    //std::cout << "connected event " << std::endl;
    //_networkAdapter->send("name:Testouille\n");
    //_networkAdapter->send("position:{\"x\":" + std::to_string(_agent.getPos().x) + ", \"y\":" + std::to_string(_agent.getPos().y) + ", \"z\":" + std::to_string(_agent.getPos().z) + "}\n");
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
  /*      if (reader.parse(serverReply.substr(posColumn + 1), root, false) == true)
        {
            std::cout << "received a movement order " << serverReply << std::endl;
            _agent.setGoalPos(root.get("x", 0.0).asDouble(),
                              root.get("y", 0.0).asDouble(),
                              root.get("z", 0.0).asDouble());
        }
        else
            std::cout << "error while parsing order : " << reader.getFormatedErrorMessages()<< std::endl;
            */
    }
    this->dispatch("AGivenCommand"/*, params*/); // And the agent subscribes to the events.
}
/*
void        AgentProtocol::sendMovement()       // Only for test, do something more generic
{
   // _networkAdapter->send("position:{\"x\":" + std::to_string(_agent.getPos().x) + ", \"y\":" + std::to_string(_agent.getPos().y) + ", \"z\":" + std::to_string(_agent.getPos().z) + "}\n");

}*/

void        AgentProtocol::disconnectEvent()
{

}

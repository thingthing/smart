#include <string>

#include "AgentProtocol.h"
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
    this->registerCallback("SetGoalPosEvent", [this](pcl::PointXYZ pos) {_agent->setGoalPos(pos);});
    this->registerCallback("SetPosEvent", [this](pcl::PointXYZ pos) {_agent->setPos(pos);});
    _agent->registerCallback("SendPacketEvent", [this]() {sendPacketEvent();});
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

void PrintJSONValue( const Json::Value &val )
{
    if( val.isString() ) {
        printf( "string(%s)", val.asString().c_str() ); 
    } else if( val.isBool() ) {
        printf( "bool(%d)", val.asBool() ); 
    } else if( val.isInt() ) {
        printf( "int(%d)", val.asInt() ); 
    } else if( val.isUInt() ) {
        printf( "uint(%u)", val.asUInt() ); 
    } else if( val.isDouble() ) {
        printf( "double(%f)", val.asDouble() ); 
    }
    else 
    {
        printf( "unknown type=[%d]", val.type() ); 
    }
}

bool PrintJSONTree( const Json::Value &root, unsigned short depth /* = 0 */) 
{
    depth += 1;
    printf( " {type=[%d], size=%d}", root.type(), root.size() ); 

    if( root.size() > 0 ) {
        printf("\n");
        for( Json::ValueIterator itr = root.begin() ; itr != root.end() ; itr++ ) {
            // Print depth. 
            for( int tab = 0 ; tab < depth; tab++) {
               printf("-"); 
            }
            printf(" subvalue(");
            PrintJSONValue(itr.key());
            printf(") -");
            PrintJSONTree( *itr, depth); 
        }
        return true;
    } else {
        printf(" ");
        PrintJSONValue(root);
        printf( "\n" ); 
    }
    return true;
}

pcl::PointXYZ AgentProtocol::getPosFromJson(Json::Value const &root)
{
    return ((pcl::PointXYZ) {
        root.get("x", 0.0).asFloat(),
                 root.get("y", 0.0).asFloat(),
                 root.get("z", 0.0).asFloat()
    });
}

void        AgentProtocol::receivePacketEvent(Network::ComPacket &packet)      // only for test 4 now, will change
{
    Json::Reader        reader;
    Json::Value         root;
    std::string         serverReply((const char *)packet.peek());

    std::cout << "received message from serveur " << serverReply << std::endl;
    if (reader.parse(serverReply, root, false) == true)
    {
        std::cout << "received a movement order " << serverReply << std::endl;
        Json::Value data = root["data"];
        if (root.get("status", 0).asInt() == 0 && data.isNull() == false)
        {
            for (Json::ValueIterator it = data.begin(); it != data.end(); ++it)
            {
                std::string command = it.memberName();
                pcl::PointXYZ pos;
                if (command == "order")
                {
                    pos = this->getPosFromJson(*it);
                    this->dispatch("SetGoalPosEvent", pos);
                }
                else if (command == "position")
                {
                    pos = this->getPosFromJson(*it);
                    this->dispatch("SetPosEvent", pos);
                }
                else
                    std::cout << "Command not known: " << command << std::endl;
            }
        }
        else
        {
            std::cout << "Status error recieved" << std::endl;
        }
    }
    else
        std::cout << "error while parsing order " << serverReply << ": " << reader.getFormatedErrorMessages() << std::endl;
}

void        AgentProtocol::disconnectEvent()
{

}

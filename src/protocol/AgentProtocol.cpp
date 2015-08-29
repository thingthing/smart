#include <string>

#include "AgentProtocol.h"
#include "NetworkManager.hh"
#include "IConnector.hh"
#include "TCPConnector.h"
#include "UDPConnector.hh"


const std::string AgentProtocol::TCP_KEY = "TCP";
const std::string AgentProtocol::UDP_KEY = "UDP";


AgentProtocol::AgentProtocol(Network::NetworkManager &networkAdapter)
: AProtocol(networkAdapter)
{
    Network::IConnector *connector = new Network::TCPConnector();
    _networkAdapter.setConnector(AgentProtocol::TCP_KEY, connector);
    _networkAdapter.setConnector(AgentProtocol::UDP_KEY, new Network::UDPConnector());
}

AgentProtocol::~AgentProtocol()
{
}

void         AgentProtocol::setAgent(Agent &agent, Slam &slam)
{
    _agent = &agent;
    this->registerCallback("SetGoalPosEvent", [this](pcl::PointXYZ pos) {_agent->setGoalPos(pos);});
    this->registerCallback("SetPosEvent", [this](pcl::PointXYZ pos) {_agent->setPos(pos);});
    _agent->registerCallback("SendPacketEvent", [this]() {sendPacketEvent();});
    _agent->registerCallback("SendStatusEvent", [this](std::string const &status) {sendStatusEvent(status);});
    ///@todo: Register in the factory (process data)
    slam.registerCallback("SendCloudEvent", [this](pcl::PointCloud<pcl::PointXYZ> const &cloud) {sendCloudEvent(cloud);});
    slam.registerCallback("SendNewLandmarkEvent", [this](std::vector<Landmarks::Landmark *> &nl) {sendNewLandmarkEvent(nl);});
}

void        AgentProtocol::sendCloudEvent(pcl::PointCloud<pcl::PointXYZ> const &cloud)
{
    _factory.processData(cloud);
    while (_factory.isFullChunkReady())
    {
        std::string toSend = _factory.getChunk();
        _networkAdapter.send(toSend, AgentProtocol::UDP_KEY);
    }
}

void        AgentProtocol::sendNewLandmarkEvent(std::vector<Landmarks::Landmark *> &nl)
{
    _factory.processData(nl);
    while (_factory.isFullChunkReady())
    {
        std::string toSend = _factory.getChunk();
        _networkAdapter.send(toSend, AgentProtocol::UDP_KEY);
    }
}

/**
 * @brief Function called when a connector is connected to the server
 * @todo: Change the function to get in param the key of the connector (maybe)
 */
void        AgentProtocol::connectedEvent()
{
    std::cout << "connected event " << std::endl;
    Json::Value     reply;

    reply["data"]["name"] = _agent->name();
    reply["data"]["position"]["x"] = _agent->getPos().x;
    reply["data"]["position"]["y"] = _agent->getPos().y;
    reply["data"]["position"]["z"] = _agent->getPos().z;
    reply["status"]["code"] = 0;
    reply["status"]["message"] = "ok";
        std::cout << "before clear in connected:: magic = " << (char)_outPacket.getPacketHeader().magic << " -- packetsize == " << _outPacket.getPacketHeader().packetSize << " -- version == " << _outPacket.getPacketHeader().version << " -- header size == " << _outPacket.getPacketHeader().headerSize << std::endl;
    _outPacket.clear();
    _outPacket.append(reply.toStyledString().c_str(), reply.toStyledString().size());       // TODO : do something to handle strings directly with << in APacket.
    std::cout << "sending in connected:: magic = " << (char)_outPacket.getPacketHeader().magic << " -- packetsize == " << _outPacket.getPacketHeader().packetSize << " -- version == " << _outPacket.getPacketHeader().version << " -- header size == " << _outPacket.getPacketHeader().headerSize << std::endl;
    _networkAdapter.send(_outPacket, AgentProtocol::TCP_KEY);
}

void        AgentProtocol::sendStatusEvent(std::string const &status)
{
    Json::Value     root;

    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    root["data"]["status"] = status;
    _outPacket.append(root.toStyledString().c_str(), root.toStyledString().size());
    std::cout << "Send status event " << std::endl;
    _networkAdapter.send(_outPacket, AgentProtocol::TCP_KEY);
    _outPacket.clear();
        std::cout << "in send status:: magic = " << (char)_outPacket.getPacketHeader().magic << " -- packetsize == " << _outPacket.getPacketHeader().packetSize << " -- version == " << _outPacket.getPacketHeader().version << " -- header size == " << _outPacket.getPacketHeader().headerSize << std::endl;
}

void        AgentProtocol::sendPacketEvent()
{
    Json::Value     root;

    root["data"]["position"]["x"] = _agent->getPos().x;
    root["data"]["position"]["y"] = _agent->getPos().y;
    root["data"]["position"]["z"] = _agent->getPos().z;
    root["data"]["battery"] = _agent->getBattery();
    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    _outPacket.append(root.toStyledString().c_str(), root.toStyledString().size());
    std::cout << "Send movement event " << std::endl;
    ///@todo: Really send position
    _networkAdapter.send(_outPacket, AgentProtocol::TCP_KEY);
    _outPacket.clear();
            std::cout << "in send packet:: magic = " << (char)_outPacket.getPacketHeader().magic << " -- packetsize == " << _outPacket.getPacketHeader().packetSize << " -- version == " << _outPacket.getPacketHeader().version << " -- header size == " << _outPacket.getPacketHeader().headerSize << std::endl;
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
    std::string         serverReply((const char *)packet.data() + sizeof(Network::s_ComPacketHeader), packet.getPacketSize() - sizeof(Network::s_ComPacketHeader));

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

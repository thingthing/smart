#include <string>

#include "AgentProtocol.h"
#include "NetworkManager.hh"
#include "IConnector.hh"
#include "TCPConnector.h"
#include "UDPConnector.hh"


const std::string AgentProtocol::TCP_KEY = "TCP";
const std::string AgentProtocol::UDP_KEY = "UDP";


AgentProtocol::AgentProtocol(Network::NetworkManager &networkAdapter)
    : AProtocol(networkAdapter), _cloud(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
    Network::IConnector *connector = new Network::TCPConnector();
    _networkAdapter.setConnector(AgentProtocol::TCP_KEY, connector);
    _networkAdapter.setConnector(AgentProtocol::UDP_KEY, new Network::UDPConnector());
}

AgentProtocol::~AgentProtocol()
{
    this->stop();
}

void         AgentProtocol::setAgent(IAgent *agent, Slam &slam)
{
    this->_mode = agent->getMode();
    this->registerCallback("SetGoalPosEvent", [agent](pcl::PointXYZ pos) {dynamic_cast<Agent *>(agent)->setGoalPos(pos);});
    this->registerCallback("DownloadEvent", [agent]() {dynamic_cast<Agent *>(agent)->executeDownload();});
    this->registerCallback("SetPosEvent", [agent](pcl::PointXYZ pos) {agent->setPos(pos);});
    agent->registerCallback("SendPacketEvent", [this](IAgent *agent) {sendPacketEvent(agent);});
    agent->registerCallback("SendStatusEvent", [this](std::string const & status) {sendStatusEvent(status);});
    this->registerCallback("moveAgentEvent", [agent]() {agent->goTowardsGoal();});
    ///@todo: Register in the factory (process data)
    this->registerCallback("StopCaptureEvent", [agent]() {agent->getCapture()->stopCapture();});
    this->registerCallback("StartCaptureEvent", [agent]() {agent->getCapture()->startCapture();});
    agent->registerCallback("SendCloudEvent", [this](pcl::PointCloud<pcl::PointXYZRGBA> const & cloud) {sendCloudEvent(cloud);});
    slam.registerCallback("SendNewLandmarkEvent", [this](std::vector<Landmarks::Landmark *> &nl) {sendNewLandmarkEvent(nl);});
}

void        AgentProtocol::run() {
    // To decomment to see the cloud to be send
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    //     std::cout << "    " << cloud.points[i].x
    //               << " "    << cloud.points[i].y
    //               << " "    << cloud.points[i].z << std::endl;
    //std::cerr << "RUNINNG WITH CLOUD == " << _cloud->size() << std::endl;
    if (!_cloud->empty()) {
        _factory.processData(*_cloud);
        int i = 0;
        bool is_ready = true;
        std::string toSend = "";
        std::cerr << "Start of send pointCloud "<< _cloud->points.size() << std::endl;
        while (_factory.isFullChunkReady())
        {
            if (is_ready) {
                ++i;
                toSend  = _factory.getChunk();
            }
            try {
                is_ready = _networkAdapter.send(toSend, AgentProtocol::UDP_KEY);
            } catch (std::exception *e) {
                std::cerr << "An error occured when trying to send on UDP" << std::endl;
                return ;
            }
            if (_mode == IAgent::DIRECT)
                boost::this_thread::sleep(boost::posix_time::millisec(5));
            else if (_mode == IAgent::DELAYED)
                boost::this_thread::sleep(boost::posix_time::millisec(10));
        }
        std::cerr << "SEND CLOUD event " << i << " packets with " << _cloud->points.size() << " points" << std::endl;
    }
    _cloud->clear();
    this->pause();
    this->dispatch("moveAgentEvent");
    //Cloud sent we can now restart capture
    this->dispatch("StartCaptureEvent");
    //To uncomment if you want to send only one cloud
    // Json::Value     reply;

    // reply["data"]["exit"] = 0;
    // reply["status"]["code"] = 0;
    // reply["status"]["message"] = "ok";
    // this->sendDataTcp(reply);
    // exit(1);
}

void        AgentProtocol::sendCloudEvent(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud)
{
    //Stopping capture before sending
    this->dispatch("StopCaptureEvent");
    if (!_cloud->empty())
      _cloud->clear();
      // pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr ptrCloud(&_cloud);
        //Clean cloud before send
    pcl::copyPointCloud(cloud, *_cloud);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
    vox.setInputCloud(_cloud);
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*_cloud);
    //std::cerr << "Trying to send cloud event" << std::endl;
    this->start();
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

void        AgentProtocol::sendDataTcp(Json::Value &root)
{
    _outPacket.clear();
    std::string buffer = root.toStyledString();
    // buffer.erase(std::remove_if(buffer.begin(),
    //                           buffer.end(),
    //                           [](char x){return std::isspace(x);}),
    //            buffer.end());
    _outPacket.append(buffer.c_str(), buffer.size());
    // std::cout << "data sent == " << buffer << std::endl;
    _networkAdapter.send(_outPacket, AgentProtocol::TCP_KEY);
    // std::cout << "in send data:: magic = " << (char)_outPacket.getPacketHeader().magic << " -- packetsize == " << _outPacket.getPacketHeader().packetSize << " -- version == " << _outPacket.getPacketHeader().version << " -- header size == " << _outPacket.getPacketHeader().headerSize << std::endl;
    _outPacket.clear();
}
/**
 * @brief Function called when a connector is connected to the server
 * @todo: Change the function to get in param the key of the connector (maybe)
 */
void        AgentProtocol::connectedEvent(IAgent *agent)
{
    // std::cout << "connected event " << std::endl;
    Json::Value     reply;

    reply["data"]["name"] = agent->name();
    reply["data"]["position"]["x"] = agent->getPos().x;
    reply["data"]["position"]["y"] = agent->getPos().y;
    reply["data"]["position"]["z"] = agent->getPos().z;
    reply["status"]["code"] = 0;
    reply["status"]["message"] = "ok";
    this->sendDataTcp(reply);
}

void        AgentProtocol::sendStatusEvent(std::string const &status)
{
    Json::Value     root;

    //std::cout << "Send status event " << std::endl;
    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    root["data"]["state"] = status;
    this->sendDataTcp(root);
}

void        AgentProtocol::sendPacketEvent(IAgent *agent)
{
    Json::Value     root;

    //std::cout << "Send movement event " << std::endl;
    root["data"]["position"]["x"] = agent->getPos().x;
    root["data"]["position"]["y"] = agent->getPos().y;
    root["data"]["position"]["z"] = agent->getPos().z;
    root["data"]["battery"] = (agent->getBattery() * 100) / Agent::DEFAULTBATTERY;
    root["status"]["code"] = 0;
    root["status"]["message"] = "ok";
    this->sendDataTcp(root);
}

void PrintJSONValue( const Json::Value &val )
{
    if ( val.isString() ) {
        printf( "string(%s)", val.asString().c_str() );
    } else if ( val.isBool() ) {
        printf( "bool(%d)", val.asBool() );
    } else if ( val.isInt() ) {
        printf( "int(%d)", val.asInt() );
    } else if ( val.isUInt() ) {
        printf( "uint(%u)", val.asUInt() );
    } else if ( val.isDouble() ) {
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

    if ( root.size() > 0 ) {
        printf("\n");
        for ( Json::ValueIterator itr = root.begin() ; itr != root.end() ; itr++ ) {
            // Print depth.
            for ( int tab = 0 ; tab < depth; tab++) {
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

///@todo: Faire un tableau associatif de pointeur sur fonction pour les ordres
void        AgentProtocol::receivePacketEvent(Network::ComPacket *packet)      // only for test 4 now, will change
{
  static bool first_order = true;
    Json::Reader        reader;
    Json::Value         root;
    std::string         serverReply((const char *)packet->data() + sizeof(Network::s_ComPacketHeader), packet->getPacketSize() - sizeof(Network::s_ComPacketHeader));

    //std::cout << "received message from serveur " << serverReply << std::endl;
    if (reader.parse(serverReply, root, false) == true)
    {
        //std::cout << "received a data " << serverReply << std::endl;
        Json::Value data = root["data"];
        Json::Value status = root["status"];
        int status_code = status.get("code", 0).asInt();
        // std::cout << "Status code == " << status_code << std::endl;
        if (status_code == 0 && data.empty() == false)
        {
            //std::cout << "Data found == " << data << std::endl;
            for (Json::ValueIterator it = data.begin(); it != data.end(); ++it)
            {
                std::string command = it.memberName();
                pcl::PointXYZ pos;
                if (command == "order")
                {
                    pos = this->getPosFromJson(*it);
                    std::cout << "Order goal pos got == " << pos << std::endl;
        		    if (first_order)
                    {
        		      first_order = false;
        		      this->dispatch("StartCaptureEvent");
                    }
        		    this->dispatch("SetGoalPosEvent", pos);
                }
                else if (command == "position")
                {
                    pos = this->getPosFromJson(*it);
                    this->dispatch("SetPosEvent", pos);
                }
                else if (command == "download")
                {
                    this->dispatch("DownloadEvent");
                }
                else
                    std::cerr << "Command not known: " << command << std::endl;
            }
        }
        else
        {
            if (status_code != 0)
                std::cerr << "Status error recieved: [" << status_code << "]: " << status.get("message", "").asString() << std::endl;
	       if (status_code == 42) {
                std::cerr << "Disconnecting event found" << std::endl;
                //this->disconnectEvent();
            }
            // else
            //     std::cerr << "No data recieved but good status: " << status.get("message", "").asString() << std::endl;
        }
    }
    else
        std::cerr << "error while parsing order " << serverReply << ": " << reader.getFormatedErrorMessages() << std::endl;
}

void        AgentProtocol::disconnectEvent()
{
    Json::Value     reply;

    reply["data"]["exit"] = 0;
    reply["status"]["code"] = 0;
    reply["status"]["message"] = "ok";
    this->sendDataTcp(reply);
}

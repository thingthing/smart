#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core(AgentProtocol &protocol) :
    _protocol(protocol)
{
    Agent *a = &_agent;
    AgentProtocol *ap = &_protocol;
    _protocol.setAgent(_agent);
    
    _protocol.registerCallback("SetGoalPosEvent", [a](pcl::PointXYZ pos){a->setGoalPos(pos);});
    _agent.registerCallback("SendPacketEvent", [ap](){ap->sendPacketEvent();});
    _slam = new Slam(a);
}

Core::~Core()
{
    delete _slam;
}

void        Core::update()
{
    std::cout << "Updating" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud = _agent.takeData();
    _slam->updateState(cloud, _agent);
    _agent.updateState();
}

void        Core::run()
{
    while (1)
    {
        usleep(1000000);
        this->update();
    }
}


#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core(AgentProtocol &protocol) :
    _protocol(protocol)
{
    _protocol.setAgent(_agent);
    _protocol.registerCallback("SetGoalPosEvent", [this](pcl::PointXYZ pos){setAgentGoal(pos);});
    _agent.registerCallback("SendPacketEvent", [this](){sendPacketEvent();});
    _slam = new Slam(&_agent);
}

Core::~Core()
{
    delete _slam;
}

void        Core::sendPacketEvent()
{
    _protocol.sendPacketEvent();
}

void        Core::setAgentGoal(pcl::PointXYZ const &pos)
{
    _agent.setGoalPos(pos);
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


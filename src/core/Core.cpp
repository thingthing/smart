#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"
#include "Agent.hh"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core(AgentProtocol &protocol) :
    _protocol(protocol)
{
    _agent = new Agent();
    _protocol.setAgent(*(dynamic_cast<Agent*>(_agent)));
    _slam = new Slam(_agent);
}

Core::~Core()
{
    delete _slam;
    delete _agent;
}

void        Core::update()
{
    std::cout << "Updating" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud = _agent->takeData();
    _slam->updateState(cloud, *_agent);
    _agent->updateState();
}

void        Core::run()
{
    while (1)
    {
        usleep(1000000);
        this->update();
    }
}


#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"
#include "Agent.hh"
#include <exception>
#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core(AgentProtocol &protocol) :
    _protocol(protocol)
{
    _agent = new Agent();
    _slam = new Slam(_agent);
    _protocol.setAgent(_agent, *_slam);
}

Core::~Core()
{
    delete _slam;
    delete _agent;
}

void        Core::update()
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud = _agent->getCapture()->getData();
    try {    
      _slam->updateState(cloud, _agent);
    } catch(...) {
      std::cerr << "Error during slam update" << std::endl;
    }
    try {
      _agent->updateState();
    } catch(...) {
      std::cerr << "Error during agent update" << std::endl;
    }
}

void        Core::run()
{
 try {
        while (1)
        {
            usleep(1000000);
            this->update();
        }
    } catch (std::exception &e) {
        std::cout << "Error catch == " << e.what() << std::endl;
    }
}

IAgent      *Core::getAgent() const
{
    return (_agent);
}

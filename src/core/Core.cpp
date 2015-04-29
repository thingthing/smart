#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core(AgentProtocol &protocol) :
    _protocol(protocol)
{
    _protocol.setAgent(_agent);
}

Core::~Core()
{
}

void        Core::run()
{
    while (1)
    {
        usleep(1000000);
        /*  if (_agent.getPos() != _agent.getGoalPos())
            {
                _agent.goTowardsTheGoalFakeTest();
                _protocol.sendMovement();
            }
            */
    }
}


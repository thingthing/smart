#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected

Core::Core() :
    _networkAdapter(*(new Network::TCPConnector(NULL))),
    _protocol(_networkAdapter, _agent)
{
    _networkAdapter.setProtocol(_protocol);
}

Core::~Core()
{
    delete (&_networkAdapter);
}

void        Core::run()
{
    if (_networkAdapter.connectTo("54.148.17.11", 4200) == false)
        std::cout << "failed to connect" << std::endl;
    else
    {
        _networkAdapter.start();
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
}


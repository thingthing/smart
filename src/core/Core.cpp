#include <iostream> // removeme
#include "Core.h"
#include "TCPConnector.h"

#include <thread> // test, to be removed

// TODO : try to reconnect when disconnected
// make the ANetwork thing as thread

Core::Core() :
    _networkAdapter(*(new Network::TCPConnector(NULL))),
    _protocol(_networkAdapter)
{
    _networkAdapter.setProtocol(_protocol);
}

Core::~Core()
{
    delete (&_networkAdapter);
}

void        Core::run()
{
    if (_networkAdapter.connectTo("127.0.0.1", 1337) == false)
        std::cout << "faaaailed to connect" << std::endl;
    else
    {
        std::thread     test([this]()
        {
            _networkAdapter.start();
        });           // THIS IS JUST FOR TESTS. WILL BE CHANGED.

        while (1)
        {
            usleep(100000);
        }
    }
}

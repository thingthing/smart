
#include "Core.h"

int     main(int argc, char **argv)
{
    Network::NetworkManager  networkAdapter;
    AgentProtocol            protocol(networkAdapter);
    Core                     core(protocol);

    if (networkAdapter.connectTo("54.148.17.11", 4200) == false)
    {
        std::cout << "failed to connect" << std::endl;
        return (-1);
    }
    networkAdapter.start();
    core.run();
    return (0);
}

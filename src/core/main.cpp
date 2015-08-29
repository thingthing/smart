#include "Core.h"

int     main(int argc, char **argv)
{
    Network::NetworkManager  networkAdapter;
    AgentProtocol            protocol(networkAdapter);
    Core                     core(protocol);

    /**
     * @todo : Add port in configuration files
     */
    if (networkAdapter.connectTo("86.196.184.254", 4200, AgentProtocol::TCP_KEY) == false)
    {
        std::cout << "failed to connect tcp" << std::endl;
        return (-1);
    }
     if (networkAdapter.connectTo("86.196.184.254", 4300, AgentProtocol::UDP_KEY) == false)
    {
        std::cout << "failed to connect udp" << std::endl;
        return (-1);
    }
    networkAdapter.start();
    core.run();
    return (0);
}

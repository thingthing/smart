#include "Core.h"

int     main(int argc, char **argv)
{
    Network::NetworkManager  networkAdapter;
    AgentProtocol            protocol(networkAdapter);
    Core                     core(protocol);
    std::string              server_ip = "37.59.100.252";

    if (argc > 1)
        server_ip = argv[1];
    std::cout << "Server ip is " << server_ip << std::endl;
    /**
     * @todo : Add port in configuration files
     */
    if (networkAdapter.connectTo(server_ip, 4200, AgentProtocol::TCP_KEY, core.getAgent()) == false)
    {
        std::cout << "failed to connect tcp" << std::endl;
        return (-1);
    }
    if (networkAdapter.connectTo(server_ip, 4300, AgentProtocol::UDP_KEY, core.getAgent()) == false)
    {
        std::cout << "failed to connect udp" << std::endl;
        return (-1);
    }
    std::cout << "Starting networkAdapter" << std::endl;
    networkAdapter.start();
    std::cout << "Runing core" << std::endl;
    core.run();
    //If code is here, you are exited, so send the disconnect info to server
    networkAdapter.disconnect();
    std::cout << "EXITING MAIN" << std::endl;
    return (0);
}

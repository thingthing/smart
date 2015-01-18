#ifndef             CORE_H_
# define            CORE_H_

#include "protocol/AgentProtocol.h"
#include "network/TCPConnector.h"

class           Core
{
public:
    Core();
    ~Core();

    void        run();

protected:
    Network::TCPConnector       &_networkAdapter;
    AgentProtocol               _protocol;
};

#endif

#ifndef             CORE_H_
# define            CORE_H_

#include "protocol/AgentProtocol.h"
#include "network/TCPConnector.h"
#include "core/Agent.hh"

class           Core
{
public:
    Core();
    ~Core();

    void        run();

protected:
    Agent                       _agent;
    Network::TCPConnector       &_networkAdapter;
    AgentProtocol               _protocol;
};

#endif

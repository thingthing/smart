#ifndef             CORE_H_
# define            CORE_H_

#include "AgentProtocol.h"
#include "TCPConnector.h"
#include "Agent.hh"
#include "NonCopyable.h"

class           Core
{
public:
    Core(AgentProtocol &protocol);
    ~Core();

    void        run();

protected:
    Core() = delete;
    NON_COPYABLE(Core)

    Agent                       _agent;
    AgentProtocol               &_protocol;
};

#endif

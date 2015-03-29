#ifndef             CORE_H_
# define            CORE_H_

#include "protocol/AgentProtocol.h"
#include "network/TCPConnector.h"
#include "core/Agent.hh"
#include "utils/NonCopyable.h"

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

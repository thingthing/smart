#ifndef             CORE_H_
# define            CORE_H_

#include "AgentProtocol.h"
#include "TCPConnector.h"
#include "IAgent.hh"
#include "NonCopyable.h"
#include "Slam.hh"

class           Core
{
public:
    Core(AgentProtocol &protocol);
    ~Core();

    void        run();
    void        update();
    IAgent      *getAgent() const;

protected:
    Core() = delete;
    NON_COPYABLE(Core)

    IAgent                      *_agent;
    Slam                        *_slam;
    AgentProtocol               &_protocol;
};

#endif

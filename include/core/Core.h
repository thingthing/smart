#ifndef             CORE_H_
# define            CORE_H_

#include "AgentProtocol.h"
#include "TCPConnector.h"
#include "Agent.hh"
#include "NonCopyable.h"
#include "Slam.hh"

class           Core
{
public:
    Core(AgentProtocol &protocol);
    ~Core();

    void        setAgentGoal(pcl::PointXYZ const &pos);
    void        sendPacketEvent();
    void        run();
    void        update();

protected:
    Core() = delete;
    NON_COPYABLE(Core)

    Agent                       _agent;
    Slam                        *_slam;
    AgentProtocol               &_protocol;
};

#endif

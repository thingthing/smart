#ifndef     IPROTOCOL_H_
# define    IPROTOCOL_H_

#include "CircularBuffer.h"
#include "ComPacket.h"
#include "NonCopyable.h"
#include "event/Dispatcher.h"
#include "NetworkManager.hh"
#include "IAgent.hh"

namespace Network { class NetworkManager; }

class       AProtocol : public Utils::Dispatcher
{
public:
    AProtocol(Network::NetworkManager &networkAdapter);
    virtual ~AProtocol();

    virtual void        connectedEvent(IAgent *) = 0;
    virtual void        receivePacketEvent(Network::ComPacket *packet) = 0;
    virtual void        disconnectEvent() = 0;
    virtual void        sendPacketEvent(IAgent *) = 0;

private:
    AProtocol() = delete;
    NON_COPYABLE(AProtocol)

    Network::NetworkManager          &_networkAdapter;
};

#endif

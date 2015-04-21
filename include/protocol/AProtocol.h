#ifndef     IPROTOCOL_H_
# define    IPROTOCOL_H_

#include "CircularBuffer.h"
#include "Packet.h"
#include "NonCopyable.h"
#include "event/Dispatcher.h"
#include "NetworkManager.hh"

namespace Network { class NetworkManager; }

class       AProtocol : public Utils::Dispatcher
{
public:
    AProtocol(Network::NetworkManager &networkAdapter);
    virtual ~AProtocol();

    virtual void        connectedEvent() = 0;
    virtual void        receivePacketEvent(Network::CircularBuffer &packet) = 0;
    virtual void        disconnectEvent() = 0;

private:
    AProtocol() = delete;
    NON_COPYABLE(AProtocol)

    Network::NetworkManager          &_networkAdapter;
};

#endif

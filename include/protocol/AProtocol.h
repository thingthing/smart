#ifndef     IPROTOCOL_H_
# define    IPROTOCOL_H_

#include "CircularBuffer.h"
#include "Packet.h"
#include "utils/NonCopyable.h"
#include "utils/event/Dispatcher.h"

namespace Network { class ANetworkAdapter; }

class       AProtocol : public Utils::Dispatcher
{
public:
    AProtocol(Network::ANetworkAdapter &networkAdapter);
    virtual ~AProtocol();

    virtual void        connectedEvent() = 0;
    virtual void        receivePacketEvent(Network::CircularBuffer &packet) = 0;
    virtual void        disconnectEvent() = 0;

private:
    AProtocol() = delete;
    NON_COPYABLE(AProtocol)

    Network::ANetworkAdapter          &_networkAdapter;
};

#endif

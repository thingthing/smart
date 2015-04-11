#include <string>

#include "AProtocol.h"
#include "network/ANetworkAdapter.h"


AProtocol::AProtocol(Network::ANetworkAdapter &networkAdapter) :
    _networkAdapter(networkAdapter)
{
    _networkAdapter.registerCallback("ConnectedEvent", [this](){connectedEvent();});
    _networkAdapter.registerCallback("ReceivePacketEvent", [this](Network::CircularBuffer &packet) {receivePacketEvent(packet);});
    _networkAdapter.registerCallback("DisconnectEvent", [this](){disconnectEvent();});
}

AProtocol::~AProtocol()
{
}

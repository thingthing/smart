#include <string>

#include "AProtocol.h"

AProtocol::AProtocol(Network::NetworkManager &networkAdapter) :
    _networkAdapter(networkAdapter)
{
    _networkAdapter.registerCallback("ConnectedEvent", [this](IAgent *agent){connectedEvent(agent);});
    _networkAdapter.registerCallback("ReceivePacketEvent", [this](Network::ComPacket *packet) {receivePacketEvent(packet);});
    _networkAdapter.registerCallback("DisconnectEvent", [this](){disconnectEvent();});
}

AProtocol::~AProtocol()
{
}

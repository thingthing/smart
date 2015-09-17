#ifndef COMPACKET_H
#define COMPACKET_H

#include <network/APacket.h>

namespace   Network
{

/** @brief Communication Packet Header v1.0. According to the communication RFC

    @author Maxime C.
    @date June 2015
    */
struct      s_ComPacketHeader
{
    s_ComPacketHeader()
    {
        magic = 0x42;
        packetSize = sizeof(s_ComPacketHeader) + 1;
        version = 0x01;
        headerSize = sizeof(s_ComPacketHeader);
    }

    byte            magic;
    unsigned short  packetSize;
    unsigned short  version;
    unsigned short  headerSize;
};

class       ComPacket : public APacket<s_ComPacketHeader>
{
public:
    ComPacket();
    virtual ~ComPacket();

protected:
   virtual void        init();
};

}
#endif // COMPACKET_H

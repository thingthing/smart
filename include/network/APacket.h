#ifndef     _PACKET_H_
# define    _PACKET_H_

#include <string>
#include <vector>
#include <stdexcept>

#include "CircularBuffer.h"
#include "utils/NonCopyable.h"
#include "defines.h"

namespace   Network
{

/** @brief Base class to unify all packet-based communications (via the NetworkManager).

    APacketBase is used when the type of the packet header is unknown. Provides basic information about the packet.
    @author Maxime C.
    @date June 2015
    */
class   APacketBase                     // Use the base class to refer to a packet without specifying the template.
{
public:
    APacketBase()
    {
        _data = new byte[2048];
        _bufferSize = 2048;
    }

    virtual ~APacketBase()
    {
        delete[] (_data);
    }

    inline unsigned int        getPacketSize() const
    {
        return (_writeCursor);
    }

    inline const byte         *data() const
    {
        return (_data);
    }

    virtual void        clear() = 0;

protected:
       NON_COPYABLE(APacketBase)
    virtual void        init() = 0;

    byte                *_data;
    unsigned int        _bufferSize;
    unsigned int        _writeCursor;
};



/** @brief Base class to unify all packet-based communications. Inherits from APacketBase and templated with a header structure of a given type of packet

    When creating a new packet-based protocol, inherit from this class and specify the header of each packet in template.
    It then provides the user with convenient append and read functions, regardless of the header size.
    WARNING : a 2-bytes header field called "packetSize" HAVE TO be defined in the header structure and correspond to the size in bytes of the payload + header.
    @author Maxime C.
    @date June 2015
    */
template<typename PacketHeaderType>
class   APacket : public APacketBase
{
public:
    APacket()
    {
        _header = (PacketHeaderType *)_data;
    }
    virtual    ~APacket() {}

    /** @brief Returns the header structure from the packet. */
    inline PacketHeaderType        &getPacketHeader()
    {
        return (*(PacketHeaderType *)_data);
    }

    /** @brief Clears the data (empty packet with only an header). */
    virtual void        clear()
    {
        _writeCursor = sizeof(PacketHeaderType);
        _data[sizeof(PacketHeaderType)] = 0;
    }

    /** @brief Append a data of scalar type T to the packet */
    template<typename T>
    APacket     &operator<<(T const &data)
    {
        append(data);
        return (*this);
    }

    /** @brief Append a data of scalar type T to the packet */
    template<typename T>
    void        append(T const &data)
    {
        append(&data, sizeof(T));
    }

    /** @brief Append size bytes from the circular buffer data at the end of the packet.
     *
     * Note: Consume the data from the CircularBuffer. */
    void        appendFromCircularBuffer(CircularBuffer &data, unsigned int size)
    {
        if (size > data.getSpaceUsed())
            return;
        reallocIfNecessary(size);
        data.read(_data + _writeCursor, size);
        _header->packetSize += size;
        _writeCursor += size;
    }

    /** @brief Append size bytes from the buffer data at the end of the packet.*/
    template<typename T>
    void        append(T const * const data, const unsigned int size)
    {
        reallocIfNecessary(size);
        memcpy(_data + _writeCursor, &data, size);
        _header->packetSize += size;
        _writeCursor += size;
    }

    /** @brief Read data from the packet and store it in a scalar type T.
     *
     * WARNING : Can throw if there is not enough data in the packet. */
    template<typename T>
    APacket     &operator>>(T &data)
    {
        read(data);
        return (*this);
    }


    /** @brief Read data from the packet and store it in a scalar type T.
     *
     * WARNING : Can throw if there is not enough data in the packet. */
    template<typename T>
    void        read(T &data)
    {
        read(&data, sizeof(T));
    }


    /** @brief Read size bytes from the packet and store it in a buffer.
     *
     * WARNING : Can throw if there is not enough data in the packet. */
    template<typename T>
    void        read(T *data, const unsigned int size)
    {
        if ((size > _writeCursor) || (_writeCursor - size < sizeof(PacketHeaderType)))
            throw std::runtime_error("APacket:: Trying to read past the header.");       // Should try to do something else than exception, but it will be good enough for now.
        _writeCursor -= size;
        _header->packetSize -= size;
        memcpy(&data, _data + _writeCursor, size);
    }

protected:
    /** @brief Reallocate the inner data buffer if it's being too small. */
    void                reallocIfNecessary(unsigned int const size)
    {
        if ((_writeCursor + size) > _bufferSize)
        {
            byte *tmp = new byte[_bufferSize * 2];                  // Quite ugly but will do the trick. It's very unlikely that we'll have multi-megabytes single packets.
            memcpy(tmp, _data, _writeCursor);
            delete[] (_data);
            _data = tmp;
            _header = (PacketHeaderType *)_data;
        }
    }

    PacketHeaderType    *_header;
};

}

#endif

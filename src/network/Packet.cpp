////////////////////////////////////////////////////////////
//
// SFML - Simple and Fast Multimedia Library
// Copyright (C) 2007-2014 Laurent Gomila (laurent.gom@gmail.com)
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#include "Packet.h"
#include <cstring>


namespace   Network
{

Packet::Packet() :
    _readPos(0),
    _isValid(true)
{

}

Packet::~Packet()
{

}

void            Packet::append(const void* data, std::size_t sizeInBytes)
{
    if (data && (sizeInBytes > 0))
    {
        std::size_t start = _data.size();
        _data.resize(start + sizeInBytes);
        std::memcpy(&_data[start], data, sizeInBytes);
    }
}

void            Packet::clear()
{
    _data.clear();
    _readPos = 0;
    _isValid = true;
}

const void      *Packet::getData() const
{
    return ((!_data.empty()) ? (&_data[0]) : (NULL));
    }

    std::size_t Packet::getDataSize() const
    {
    return (_data.size());
}

bool        Packet::endOfPacket() const
{
    return _readPos >= _data.size();
}

Packet::operator BoolType() const
{
    return _isValid ? &Packet::checkSize : NULL;
}



Packet      &Packet::operator >>(bool &data)
{
    unsigned char value;
    if (*this >> value)
        data = (value != 0);

    return *this;
}



Packet      &Packet::operator >>(char &data)
{
    if (checkSize(sizeof(data)))
    {
        data = *reinterpret_cast<const char*>(&_data[_readPos]);
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(unsigned char &data)
{
    if (checkSize(sizeof(data)))
    {
        data = *reinterpret_cast<const unsigned char*>(&_data[_readPos]);
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(short &data)
{
    if (checkSize(sizeof(data)))
    {
        data = ntohs(*reinterpret_cast<const short*>(&_data[_readPos]));
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(unsigned short &data)
{
    if (checkSize(sizeof(data)))
    {
        data = ntohs(*reinterpret_cast<const unsigned short*>(&_data[_readPos]));
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(int &data)
{
    if (checkSize(sizeof(data)))
    {
        data = ntohl(*reinterpret_cast<const int*>(&_data[_readPos]));
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(unsigned int &data)
{
    if (checkSize(sizeof(data)))
    {
        data = ntohl(*reinterpret_cast<const unsigned int*>(&_data[_readPos]));
        _readPos += sizeof(data);
    }

    return *this;
}

Packet      &Packet::operator >>(long &data)
{
    if (checkSize(sizeof(data)))
    {
        // Since ntohll is not available everywhere, we have to convert
        // to network byte order (big endian) manually
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&_data[_readPos]);
        data = (static_cast<long long>(bytes[0]) << 56) |
                (static_cast<long long>(bytes[1]) << 48) |
                (static_cast<long long>(bytes[2]) << 40) |
                (static_cast<long long>(bytes[3]) << 32) |
                (static_cast<long long>(bytes[4]) << 24) |
                (static_cast<long long>(bytes[5]) << 16) |
                (static_cast<long long>(bytes[6]) <<  8) |
                (static_cast<long long>(bytes[7])      );
        _readPos += sizeof(data);
    }

    return *this;
}



Packet      &Packet::operator >>(unsigned long &data)
{
    if (checkSize(sizeof(data)))
    {
        // Since ntohll is not available everywhere, we have to convert
        // to network byte order (big endian) manually
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&_data[_readPos]);
        data = (static_cast<unsigned long long>(bytes[0]) << 56) |
                (static_cast<unsigned long long>(bytes[1]) << 48) |
                (static_cast<unsigned long long>(bytes[2]) << 40) |
                (static_cast<unsigned long long>(bytes[3]) << 32) |
                (static_cast<unsigned long long>(bytes[4]) << 24) |
                (static_cast<unsigned long long>(bytes[5]) << 16) |
                (static_cast<unsigned long long>(bytes[6]) <<  8) |
                (static_cast<unsigned long long>(bytes[7])      );
        _readPos += sizeof(data);
    }

    return *this;
}

Packet      &Packet::operator >>(float &data)
{
    if (checkSize(sizeof(data)))
    {
        data = *reinterpret_cast<const float*>(&_data[_readPos]);
        _readPos += sizeof(data);
    }

    return *this;
}

Packet      &Packet::operator >>(double &data)
{
    if (checkSize(sizeof(data)))
    {
        data = *reinterpret_cast<const double*>(&_data[_readPos]);
        _readPos += sizeof(data);
    }

    return *this;
}

Packet      &Packet::operator >>(char *data)
{
    // First extract string length
    unsigned int length = 0;
    *this >> length;

    if ((length > 0) && checkSize(length))
    {
        // Then extract characters
        std::memcpy(data, &_data[_readPos], length);
        data[length] = '\0';

        // Update reading position
        _readPos += length;
    }

    return *this;
}



Packet      &Packet::operator >>(std::string &data)
{
    // First extract string length
    unsigned int length = 0;
    *this >> length;

    data.clear();
    if ((length > 0) && checkSize(length))
    {
        // Then extract characters
        data.assign(&_data[_readPos], length);

        // Update reading position
        _readPos += length;
    }

    return *this;
}


Packet      &Packet::operator <<(bool data)
{
    *this << static_cast<unsigned char>(data);
    return *this;
}



Packet      &Packet::operator <<(char data)
{
    append(&data, sizeof(data));
    return *this;
}



Packet      &Packet::operator <<(unsigned char data)
{
    append(&data, sizeof(data));
    return *this;
}



Packet      &Packet::operator <<(short data)
{
    short toWrite = htons(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(unsigned short data)
{
    unsigned short toWrite = htons(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(int data)
{
    int toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(unsigned int data)
{
    unsigned int toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(long long data)
{
    // Since htonll is not available everywhere, we have to convert
    // to network byte order (big endian) manually
    unsigned char toWrite[] =
    {
        static_cast<unsigned char>((data >> 56) & 0xFF),
        static_cast<unsigned char>((data >> 48) & 0xFF),
        static_cast<unsigned char>((data >> 40) & 0xFF),
        static_cast<unsigned char>((data >> 32) & 0xFF),
        static_cast<unsigned char>((data >> 24) & 0xFF),
        static_cast<unsigned char>((data >> 16) & 0xFF),
        static_cast<unsigned char>((data >>  8) & 0xFF),
        static_cast<unsigned char>((data      ) & 0xFF)
    };
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(unsigned long long data)
{
    // Since htonll is not available everywhere, we have to convert
    // to network byte order (big endian) manually
    unsigned char toWrite[] =
    {
        static_cast<unsigned char>((data >> 56) & 0xFF),
        static_cast<unsigned char>((data >> 48) & 0xFF),
        static_cast<unsigned char>((data >> 40) & 0xFF),
        static_cast<unsigned char>((data >> 32) & 0xFF),
        static_cast<unsigned char>((data >> 24) & 0xFF),
        static_cast<unsigned char>((data >> 16) & 0xFF),
        static_cast<unsigned char>((data >>  8) & 0xFF),
        static_cast<unsigned char>((data      ) & 0xFF)
    };
    append(&toWrite, sizeof(toWrite));
    return *this;
}



Packet      &Packet::operator <<(float data)
{
    append(&data, sizeof(data));
    return *this;
}



Packet      &Packet::operator <<(double data)
{
    append(&data, sizeof(data));
    return *this;
}



Packet      &Packet::operator <<(const char *data)
{
    // First insert string length
    unsigned int length = static_cast<unsigned int>(std::strlen(data));
    *this << length;

    // Then insert characters
    append(data, length * sizeof(char));

    return *this;
}


Packet      &Packet::operator <<(const std::string &data)
{
    // First insert string length
    unsigned int length = static_cast<unsigned int>(data.size());
    *this << length;

    // Then insert characters
    if (length > 0)
        append(data.c_str(), length * sizeof(std::string::value_type));

    return *this;
}


bool        Packet::checkSize(std::size_t size)
{
    _isValid = _isValid && (_readPos + size <= _data.size());
    return _isValid;
}



const void  *Packet::onSend(std::size_t &size)
{
    size = getDataSize();
    return getData();
}


void        Packet::onReceive(const void *data, std::size_t size)
{
    append(data, size);
}

}

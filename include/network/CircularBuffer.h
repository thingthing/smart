#ifndef     CIRCULAR_BUFFER_H_
# define    CIRCULAR_BUFFER_H_

#include <string.h>
#include <unistd.h>

#include "defines.h"

#include <iostream>

namespace   Network
{

class       CircularBuffer
{
public:
    CircularBuffer(size_t size);
    ~CircularBuffer();

    size_t          getSpaceLeft();
    size_t          getSpaceUsed();
    size_t          getBufferSize();
    void            reset();
    const void      *peek(size_t moveReadCursor = 0);                // Get a pointer on the begging of the buffer, to peek data without consuming data.

    // ######################## read ####################
    template<typename T>
    CircularBuffer        &operator>> (T &data)
    {
        this->read(&data, sizeof(T));
        return (*this);
    }

    template<typename T>
    bool        read(T &data)
    {
        return (this->read(&data, sizeof(T)));
    }

    template<typename T>
    bool        read(T *data, const size_t size)
    {
        std::cout << "trying to read : " << size << " actually in buf : " << getSpaceUsed() << std::endl;
        if (size > getSpaceUsed())
            return (false);        // Cannot read that much data now
        memcpy(data, _buffer + _readCursor, size);
        _readCursor += size;
        if (_readCursor >= _bufferSize)
            _readCursor %= _bufferSize;
        return (true);
    }


    int        writeTo(int fd)        // Read from the buffer to the fd
    {
        int     byteWritten;

        byteWritten = ::write(fd, _buffer + _readCursor, getSpaceUsed());
        if (byteWritten > 0)
        {
            _readCursor += byteWritten;
            if (_readCursor >= _bufferSize)
                _readCursor %= _bufferSize;
        }
        return (byteWritten);
    }

    // ######################## write ####################

    template<typename T>
    CircularBuffer        &operator<< (const T &data)
    {
        this->write(&data, sizeof(T));
        return (*this);
    }

    template<typename T>
    bool        write(const T &data)
    {
        return (this->write(&data, sizeof(T)));
    }

    template<typename T>
    bool        write(const T *data, const size_t size)
    {
        if (size > getSpaceLeft())
            return (false);                 // cannot store that much data now
        memcpy(_buffer + _writeCursor, data, size);
        _writeCursor += size;
        if (_writeCursor >= _bufferSize)
            _writeCursor %= _bufferSize;
        return (true);
    }

    int        readFrom(int fd)        // fill the buffer from an fd
    {
        int byteRead;

        byteRead = ::read(fd, _buffer + _writeCursor, getSpaceLeft());
        if (byteRead > 0)
        {
            _writeCursor += byteRead;
            if (_writeCursor >= _bufferSize)
                _writeCursor %= _bufferSize;
        }
        return (byteRead);
    }


private:
    CircularBuffer();
    byte        *_buffer;

    size_t       _bufferSize;
    size_t       _writeCursor;
    size_t       _readCursor;
};

}

#endif

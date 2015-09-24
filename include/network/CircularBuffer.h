#ifndef     CIRCULAR_BUFFER_H_
# define    CIRCULAR_BUFFER_H_

#include <string.h>
#include <unistd.h>

#include "defines.h"

#include <iostream>

namespace   Network
{
/** 
 * @brief Circular Buffer. Convenient for network communications
 *
 * @author Maxime C.
 * @date June 2015
 */
class       CircularBuffer
{
public:
    /** 
     * @brief Constructor. The specified size will be rounded to a multiple of getpagesize()
     * @param size size_t Default size of buffer
     * 
     * The circular buffer is initialized by memory-mapping the same memory area contiguiously. That is,
     * When you write at the last address + 1, it will write on the first byte of the buffer. (avoids modulo
     * operation and data wrapping logic !
     * */
    CircularBuffer(size_t size = 4096);
    ~CircularBuffer();

    size_t          getSpaceLeft();                                     /**< @brief Returns the space left in the buffer in bytes */
    size_t          getSpaceUsed();                                     /**< @brief Returns the space used in the buffer in bytes */
    size_t          getBufferSize();                                    /**< @brief Returns the total buffer size (space left + space used) */
    void            reset();                                            /**< @brief Reset the buffer states (read cursor and write cursor at 0, space used at 0) */
    const void      *peek(size_t moveReadCursor = 0);                   /**< @brief Get a pointer on the begging of the buffer, to peek data without consuming it. */
    void            poke(size_t pos, const void *data, size_t size);    /**< @brief force some data in the buffer. Ugly hack made to make the test app, DEPRECATED, MAY BE REMOVED SOON */

    // ######################## read ####################
    /**< @brief Read some data to the scalar type T */
    template<typename T>
    CircularBuffer        &operator>> (T &data)
    {
        this->read(&data, sizeof(T));
        return (*this);
    }

    /**< @brief Read some data to the scalar type T */
    template<typename T>
    bool        read(T &data)
    {
        return (this->read(&data, sizeof(T)));
    }

    /**< @brief Read size bytes from the inner buffer to the buffer "data" */
    template<typename T>
    bool        read(T *data, const size_t size)
    {
        if (size > getSpaceUsed())
            return (false);        // Cannot read that much data now
        memcpy(data, _buffer + _readCursor, size);
        _readCursor += size;
        if (_readCursor >= _bufferSize)
            _readCursor %= _bufferSize;
        return (true);
    }

    /**< @brief Read from the inner buffer to the fd. Useful to write to sockets */
    int        writeTo(int fd)
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

    /**< @brief Write a scalar type T to the circular buffer */
    template<typename T>
    CircularBuffer        &operator<< (const T &data)
    {
        this->write(&data, sizeof(T));
        return (*this);
    }

    /**< @brief Write a scalar type T to the circular buffer */
    template<typename T>
    bool        write(const T &data)
    {
        return (this->write(&data, sizeof(T)));
    }

    /**< @brief Write size bytes from data to the circular buffer */
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

    /**< @brief Read as much data as possible from the fd. Useful to populate the circular buffer with data from a socket. */
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

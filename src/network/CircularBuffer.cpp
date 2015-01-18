#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdexcept>

#include "CircularBuffer.h"

namespace   Network
{

static int PAGE_SIZE = getpagesize();

CircularBuffer::CircularBuffer(size_t size)
{
    char    path[] = "/dev/shm/ring-buffer-XXXXXX";
    int     fd;
    void    *address;

    reset();
    fd = mkstemp(path);
    if (fd < 0)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to create a temp. file for the buffer's mapping.");
    if (unlink(path) != 0)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to mark the temp. file - to be removed - after close.");
    _bufferSize = size + ((size % PAGE_SIZE) ? (PAGE_SIZE - size) : 0);
    if (ftruncate(fd, _bufferSize) != 0)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to resize the temp file.");
    if ((_buffer = (byte *)mmap(NULL, _bufferSize * 2, PROT_NONE,
                                MAP_ANONYMOUS | MAP_PRIVATE, -1, 0)) == MAP_FAILED)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to map the circular buffer (2*size) in memory.");
    if ((address = mmap(_buffer, _bufferSize, PROT_READ | PROT_WRITE,
                        MAP_FIXED | MAP_SHARED, fd, 0)) != _buffer)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to resize the buffer to 1 time its size.");
    if ((address = mmap(_buffer + _bufferSize,
                        _bufferSize, PROT_READ | PROT_WRITE,
                        MAP_FIXED | MAP_SHARED, fd, 0)) != (_buffer + _bufferSize))
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to remap the buffer after itself (magic buffer).");
    if (close(fd) != 0)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to close the temp file.");
}

CircularBuffer::~CircularBuffer()
{
    if (munmap(_buffer, _bufferSize << 1) != 0)
        throw std::runtime_error(std::string(__FUNCTION__) + ": Unable to unmap the buffer from memory");
}

size_t              CircularBuffer::getBufferSize()
{
    return (_bufferSize);
}

size_t              CircularBuffer::getSpaceUsed()
{
    return ((_writeCursor < _readCursor) ?
                (_readCursor - _writeCursor) : (_writeCursor - _readCursor));
}

size_t              CircularBuffer::getSpaceLeft()
{
    return (_bufferSize - getSpaceUsed());
}

void                CircularBuffer::reset()
{
    _writeCursor = 0;
    _readCursor = 0;
}

const void          *CircularBuffer::peek(size_t moveReadCursor)
{
    const byte      *tmp = _buffer + _readCursor;

    if (moveReadCursor)
    {
        _readCursor += moveReadCursor;
        if (_readCursor >= _bufferSize)
            _readCursor %= _bufferSize;
    }
    return (tmp);
}

}

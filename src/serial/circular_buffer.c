#include "circular_buffer.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>


#ifdef rtos
#include <compiler_settings.h>
#endif

#ifdef __cplusplus
extern "C"
{
 #endif


void        init_circular_buffer(t_circular_buffer *self)
{
    self->availableData = 0;
    self->readIdx = 0;
    self->writeIdx = 0;
}

// WARNING : MAKE SURE TO deactive interrupts before calling these !
// Beware not to read more than there is data available.
void         circular_buffer_read(t_circular_buffer *self, unsigned char *buf, unsigned int size)
{
#ifdef rtos
    __disable_irq();
#endif


    if (self->readIdx + size < CIRCULAR_BUFFER_SIZE)
        memcpy(buf, (const void *)(self->buf + self->readIdx), size);
    else
    {
        memcpy(buf, (const void *)(self->buf + self->readIdx), CIRCULAR_BUFFER_SIZE - self->readIdx);
        memcpy(buf + (CIRCULAR_BUFFER_SIZE - self->readIdx), (const void *)(self->buf), size - (CIRCULAR_BUFFER_SIZE - self->readIdx));
    }
    self->readIdx = (self->readIdx + size) & CIRCULAR_BUFFER_SIZE_MASK;
    self->availableData -= size;


#ifdef rtos
    __enable_irq();
#endif
}


void         circular_buffer_write(t_circular_buffer *self, const unsigned char *buf, unsigned int size)
{
#ifdef rtos
    __disable_irq();
#endif

    if (self->writeIdx + size < CIRCULAR_BUFFER_SIZE)
    {
        memcpy((void *)(self->buf + self->writeIdx), buf, size);
    }
    else
    {
        memcpy((void *)(self->buf + self->writeIdx), buf, CIRCULAR_BUFFER_SIZE - self->writeIdx);
        memcpy((void *)(self->buf), buf + (CIRCULAR_BUFFER_SIZE - self->writeIdx), size - (CIRCULAR_BUFFER_SIZE - self->writeIdx));
    }
    self->writeIdx = (self->writeIdx + size) & CIRCULAR_BUFFER_SIZE_MASK;
    self->availableData += size;

#ifdef rtos
    __enable_irq();
#endif

}


unsigned char        circular_buffer_read_one(t_circular_buffer *self)
{
#ifdef rtos
    __disable_irq();
#endif

    unsigned char c = self->buf[self->readIdx];
    self->readIdx = (self->readIdx + 1) & CIRCULAR_BUFFER_SIZE_MASK;
    --self->availableData;

#ifdef rtos
    __enable_irq();
#endif
    return (c);
}


void        circular_buffer_write_one(t_circular_buffer *self, const unsigned char c)
{
#ifdef rtos
    __disable_irq();
#endif
    self->buf[self->writeIdx] = c;
    self->writeIdx = (self->writeIdx + 1) & CIRCULAR_BUFFER_SIZE_MASK;
    ++self->availableData;
#ifdef rtos
    __enable_irq();
#endif
}

unsigned int        circular_buffer_write_to(t_circular_buffer *self, unsigned int fd)
{
    unsigned int    toRead = self->availableData;

    if (toRead)
    {
        if (self->readIdx + toRead > CIRCULAR_BUFFER_SIZE_MASK)
            toRead = CIRCULAR_BUFFER_SIZE_MASK - self->readIdx;
        int ret = write(fd, self->buf + self->readIdx, toRead);
        if (ret > 0)
        {
            self->readIdx = (self->readIdx + ret) & CIRCULAR_BUFFER_SIZE_MASK;
            self->availableData -= ret;
            return (ret);
        }
    }
    return (0);
}


unsigned int        getAvailableData(t_circular_buffer *self)
{
#ifdef rtos
    __disable_irq();
#endif
    unsigned int ret = self->availableData;
#ifdef rtos
    __enable_irq();
#endif
    return (ret);
}

#ifdef __cplusplus
}
 #endif

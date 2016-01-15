#ifndef     CIRCULAR_BUFFER_H_
# define    CIRCULAR_BUFFER_H_

# define    CIRCULAR_BUFFER_SIZE        (2048) //Should be a power of 2. (here : 2KB)
# define    CIRCULAR_BUFFER_SIZE_MASK     (CIRCULAR_BUFFER_SIZE - 1)


typedef struct      s_circular_buffer
{
    unsigned int    availableData;
    unsigned int    writeIdx;
    unsigned int    readIdx;
    unsigned char   buf[CIRCULAR_BUFFER_SIZE];
}                   t_circular_buffer;

void        init_circular_buffer(volatile t_circular_buffer *self);
void        circular_buffer_read(volatile t_circular_buffer *self, unsigned char *buf, unsigned int size);
unsigned char circular_buffer_read_one(volatile t_circular_buffer *self);
void        circular_buffer_write(volatile t_circular_buffer *self, unsigned  const char *buf, unsigned int size);
void        circular_buffer_write_one(volatile t_circular_buffer *self, const unsigned char buf);
unsigned int getAvailableData(volatile t_circular_buffer *self);
unsigned int        circular_buffer_write_to(volatile t_circular_buffer *self, unsigned int fd);
#endif

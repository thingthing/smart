#include "movement/Movement.h"
#include <stdlib.h>

Movement::Movement()
{
    init_circular_buffer(&_rxBuffer);
    init_circular_buffer(&_txBuffer);
}

Movement::~Movement(){}


// ============== Serial Connection ================

void    Movement::connectArduinoSerial()
{
    int fdSerial = open (serialTTY.c_str(), O_RDWR); // | O_NOCTTY | O_SYNC
    if (fdSerial < 0)
    {
        std::cerr << "error " << errno << " opening fdSerial: " << fdSerial << std::endl;
        return;
    }
    else
    {
        std::cout << "fdSerial Opened!" << std::endl;
        memset(&poll_set, '\0', sizeof(poll_set));
        poll_set.fd = fdSerial;
        poll_set.events = POLLIN;
    }
}



void        Movement::processReceivedData(unsigned int size)
{
    char    buf[64] = {0};
    int     parseCursor = 2;

    circular_buffer_read(&_rxBuffer, (unsigned char *)buf, size);

    float p = atof(buf + parseCursor);
    while (buf[parseCursor] != ':')
        ++parseCursor;
    ++parseCursor;
    float y = atof(buf + parseCursor);
    while (buf[parseCursor] != ':')
        ++parseCursor;
    float r = atof(buf + parseCursor);
    printf("b : %f %f %f\n", p, y, r);
}

void        Movement::updateSerial()
{
    if (poll(&poll_set, 1, 0) > 0) // 1 = numFds
    {
        if (poll_set.revents & POLLIN)
        {
            static unsigned char tmpbuf[64];
            int ret = read (fdSerial, (char*)tmpbuf, sizeof(char) * 64 );
            if (ret > 0)
            {
	      circular_buffer_write(&_rxBuffer, tmpbuf, ret);
                while (_rxBuffer.availableData && _rxBuffer.buf[_rxBuffer.readIdx] != 'p')
                    circular_buffer_read_one(&_txBuffer);

                for (int i = 0; i < _rxBuffer.availableData; ++i)
                    if (_rxBuffer.buf[(_rxBuffer.readIdx + i) & CIRCULAR_BUFFER_SIZE_MASK] == '\n')
                        processReceivedData(i + 1);
            }
        }
        if (poll_set.revents & POLLOUT)
        {
            circular_buffer_write_to(&_txBuffer, fdSerial);
            if (_txBuffer.availableData == 0)
                poll_set.events &= ~POLLOUT;
        }
    }
}

// ================ Movement =======================



void    Movement::updateMotorsSpeed()
{
    char c = 'g';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}


void    Movement::increaseMotorSpeed(uint motorNo)
{
    char c = (motorNo) ? 'q' : 'a';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}

void    Movement::decreaseMotorSpeed(uint motorNo)
{
    char c = (motorNo) ? 's' : 'z';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}


void    Movement::updateStackMovement(float x, float y, float z)
{
}

void    Movement::goForward()
{

}

void    Movement::goBack()
{

}

void    Movement::goLeft()
{

}

void    Movement::goRight()
{

}

void    Movement::goUp() { // not implemented
}

void    Movement::goDown() { // not implemented
}

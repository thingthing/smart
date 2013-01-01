#include "movement/Movement.h"
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <termios.h>
#include <fcntl.h>

Movement::Movement()
{
    init_circular_buffer(&_rxBuffer);
    init_circular_buffer(&_txBuffer);
}

Movement::~Movement(){}


// ============== Serial Connection ================

void    Movement::connectArduinoSerial()
{
    fdSerial = open (serialTTY.c_str(), O_RDWR | O_NONBLOCK); // | O_NOCTTY | O_SYNC
    if (fdSerial <= 0)
    {
        std::cerr << "error " << errno << " opening fdSerial: " << fdSerial << std::endl;
        return; 
    }
    else
    {
      std::cout << "fdSerial Opened! : " << fdSerial << std::endl;
        memset(&poll_set, '\0', sizeof(poll_set));
        poll_set.fd = fdSerial;
        poll_set.events = POLLIN;

        std::cout << "Init Connection" << std::endl;
        struct termios termAttr;
        if (tcgetattr(fdSerial, &termAttr) < 0)
          std::cout << "couldn't get term attributes" << std::endl;
        speed_t brate = B9600;
        cfsetispeed(&termAttr, brate);
        cfsetospeed(&termAttr, brate);

        termAttr.c_cflag &= ~PARENB;
        termAttr.c_cflag &= ~CSTOPB;
        termAttr.c_cflag &= ~CSIZE;
        termAttr.c_cflag |= CS8;

        termAttr.c_cflag &= ~CRTSCTS;

        termAttr.c_cflag |= CREAD | CLOCAL;

        termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);

        termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        termAttr.c_oflag &= ~OPOST;

        termAttr.c_cc[VMIN] = 0;
        termAttr.c_cc[VTIME] = 0;

        if (tcsetattr(fdSerial, TCSANOW, &termAttr) < 0 || 
	    tcsetattr(fdSerial, TCSAFLUSH, &termAttr) < 0)
          std::cout << "problem with tcsetattr" << std::endl;
	sleep(2);
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
    ++parseCursor; 
    float r = atof(buf + parseCursor);
     printf("b : %f %f %f\n", p, y, r); 
    _pitchRollYaw.x = p;
    _pitchRollYaw.y = r;
    _pitchRollYaw.z = y;
}

void        Movement::updateSerial()
{
     if (poll(&poll_set, 1, 0) > 0) // 1 = numFds
    {
      std::cout << "apres poll" << std::endl;
        if (poll_set.revents & POLLIN)
        {
	  std::cout << "dans pollin" << std::endl;
	  static unsigned char tmpbuf[64];
  
	  int ret = read (fdSerial, (char*)tmpbuf, sizeof(char) * 64 );
	  //int nbRead = 100;
	  //char buf[nbRead];
	  //memset(buf, '\0', sizeof(char) * nbRead);
	  //int ret = read(fdSerial, buf, sizeof(char) * nbRead); // le fdSerial passe a 65537 par magie une fois sortie de 
            if (ret > 0)
            {

	      
	      printf("read smth %d available : %d \n", ret, _rxBuffer.availableData );
	      circular_buffer_write(&_rxBuffer, tmpbuf, ret);
                while (_rxBuffer.availableData && _rxBuffer.buf[_rxBuffer.readIdx] != 'p')
		  {
                    circular_buffer_read_one(&_rxBuffer);
		  }

                for (unsigned int i = 0; i < _rxBuffer.availableData; ++i)
                    if (_rxBuffer.buf[(_rxBuffer.readIdx + i) & CIRCULAR_BUFFER_SIZE_MASK] == '\n')
		      {
			printf("nacio \n");
                        processReceivedData(i + 1);
		      }
	      
            }
	    else
	      {

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

void Movement::updateGyro()
{
    unsigned char c = 'g';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}

// ================ Movement =======================

# define MAX_SPEED 200

void	Movement::sendMotorSpeed(uint motorNo, short speed)
{
unsigned char buf[] = {'r', '\x00', '\x00', 'r', '\x00', '\x00'};
buf[1] = motorNo * 2;
buf[4] = motorNo * 2 + 1;

buf[2] = speed & 0xFF;
buf[5] = (speed >> 8) & 0xFF;

circular_buffer_write(&_txBuffer, (unsigned char *)&buf, 6);
}

void    Movement::updateMotorsSpeed()
{
    unsigned char c = 'u';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}


void    Movement::increaseMotorSpeed(uint motorNo)
{
    unsigned char c = (motorNo) ? 'q' : 's';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}

void    Movement::decreaseMotorSpeed(uint motorNo)
{
    unsigned char c = (motorNo) ? 'a' : 'z';
    circular_buffer_write(&_txBuffer, &c, 1);
    poll_set.events |= POLLOUT;
}

# define	FORWARD_SPEED 1570
# define	BACK_SPEED	1430

void    Movement::goForward()
{
sendMotorSpeed(0, FORWARD_SPEED);
sendMotorSpeed(1, BACK_SPEED);
}

void    Movement::goBack()
{
sendMotorSpeed(0, BACK_SPEED);
sendMotorSpeed(1, FORWARD_SPEED);
}

void    Movement::goLeft()
{
sendMotorSpeed(0, FORWARD_SPEED);
sendMotorSpeed(1, FORWARD_SPEED);
}

void    Movement::goRight()
{
sendMotorSpeed(0, BACK_SPEED);
sendMotorSpeed(1, BACK_SPEED);
}

void    Movement::goUp() { // not implemented
}

void    Movement::goDown() { // not implemented
}

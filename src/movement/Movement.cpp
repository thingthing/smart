#include "movement/Movement.h"

Movement::Movement() {}

Movement::~Movement(){}


// ============== Serial Connection ================

void    Movement::connectArduinoSerial()
{
    int fdSerial = open (serialTTY.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fdSerial < 0)
    {
            std::cerr << "error " << errno << " opening fdSerial: " << fdSerial << std::endl;
            return;
    }
    else
        std::cout << "fdSerial Opened!" << std::endl;

    set_interface_attribs (0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (0);   // set no blocking

    sleep(5);

    std::cout << "gonna send character to make the agent run" << std::endl;

    int sizeSent = 2;
    write (fdSerial, "az", sizeSent); // send 2 character greeting

    usleep ((sizeSent + 25) * 100); // sleep enough to transmit the 2 plus
                                    // receive 25:  approx 100 uS per char transmit
    char buf [100];
    int n = read (fdSerial, buf, sizeof buf);  // read up to 100 characters if ready to read
    if (n > 0)
        std::cout << "read: '" << buf << "'" << std::endl;
    else
        std::cout << "nothing sent" << std::endl;
}

int     Movement::set_interface_attribs (int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fdSerial, &tty) != 0)
    {
        std::cerr << "error " << errno << " from tcgetattr" << std::endl;
        return -1;
    }

    cfsetospeed (&tty, speedConnection);
    cfsetispeed (&tty, speedConnection);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fdSerial, TCSANOW, &tty) != 0)
    {
        std::cerr << "error << " << errno << " from tcsetattr" << std::endl;
        return -1;
    }
    return 0;
}

void    Movement::set_blocking (int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fdSerial, &tty) != 0)
    {
        std::cerr << "error " << errno << " from tggetattr" << std::endl;
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr (fdSerial, TCSANOW, &tty) != 0)
            std::cerr << "error " << errno << " setting term attributes" << std::endl;
}

// ================ Movement =======================

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

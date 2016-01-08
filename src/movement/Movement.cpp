#include "movement/Movement.h"

Movement::Movement() {}

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

    std::cout << "Waiting to be ready" << std::endl;
    sleep(3);
    std::cout << "Starting Communication" << std::endl;


    poll(&poll_set, 1, -1); // 1 = numFds // -1 je sais pas pourquoi
    int n;
    while (true)
    {
        if (poll_set.revents & POLLIN)
        {
            int nbRead = 100;
            char buf [nbRead];
            memset(buf, '\0', sizeof(char) * nbRead);
            n = read (fdSerial, buf, sizeof(char) * nbRead );  // read up to 100 characters if ready to read
            int g;
            if (n > 0)
            {
                buf[n] = '\n';
                g = 0;
                std::cout << "We read : " << n << std::endl;
                while (buf[g] != '\n' && g < n)
                {
                    std::cout << buf[g];
                    ++g;
                }
                std::cout << std::endl;
            }
            else
                std::cout << "nothing sent" << std::endl;
        }
        /*
            int sizeSent = 1;
            n = write (fdSerial, "g", sizeSent);

            std::cout << "the number written: " << n << std::endl;
        */
        //    usleep ((sizeSent + 25) * 100); // sleep enough to transmit the 2 plus
                                        // receive 25:  approx 100 uS per char transmit
    }
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

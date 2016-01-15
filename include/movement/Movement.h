#ifndef		_MOVEMENT_H_
# define	_MOVEMENT_H_

// Open include
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iostream>
#include <string>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#include <poll.h>
#include "circular_buffer.h"

//# stty -F /dev/YOUR_DEVICE 9600 # set speed on connection
//# stty -F /dev/YOUR_DEVICE -hupcl # unable auto reset

class Movement
{
public:
    Movement();
    ~Movement();

    // ================= SerialConnection =================

    void connectArduinoSerial();

    // ======================================================

    // ==================== Displacement ====================
    //    void updateStackMovement(pos3d);
    void updateStackMovement(float x, float y, float z);

    void goForward();
    void goBack();

    void goLeft();
    void goRight();

    void goUp();
    void goDown();
    // ======================================================

    void processReceivedData(unsigned int size);
    void updateSerial();
    void sendMotorSpeed(uint motorNo, short speed);
private:

    struct pos3f
    {
        float x;
        float y;
        float z;
    };

    struct angle3f
    {
        float x;
        float y;
        float z;
    };

    int           fdSerial;
    int           speedConnection = 9600; // BAUD
    std::string   serialTTY = "/dev/ttyACM0";
    struct pollfd poll_set;

    t_circular_buffer       _rxBuffer;
    t_circular_buffer       _txBuffer;
};

#endif  /* !_MOVEMENT_H_ */

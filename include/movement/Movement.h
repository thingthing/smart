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

typedef struct  s_vector3f
{
    float x;
    float y;
    float z;
}               t_vector3f;

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

    void goForward();
    void goBack();

    void goLeft();
    void goRight();

    void goUp();
    void goDown();
    // ======================================================

    void processReceivedData(unsigned int size);
    void updateSerial();
    void updateGyro();
    void    updateMotorsSpeed();
    void sendMotorSpeed(uint motorNo, short speed);
    void increaseMotorSpeed(uint motorNo);
    void decreaseMotorSpeed(uint motorNo);

    t_vector3f      &getPitchRollYaw() { return (__pitchRollYaw); }
private:

    t_vector3f    _pitchRollYaw;
    int           fdSerial;
    int           speedConnection = 9600; // BAUD
    std::string   serialTTY = "/dev/ttyACM99";
    struct pollfd poll_set;

    t_circular_buffer       _rxBuffer;
    t_circular_buffer       _txBuffer;
};

#endif  /* !_MOVEMENT_H_ */

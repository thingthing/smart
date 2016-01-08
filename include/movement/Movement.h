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

private:

    void set_blocking (int should_block);
    int  set_interface_attribs (int parity);

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

    int         fdSerial;
    int         speedConnection = B9600; // BAUD
    std::string serialTTY = "/dev/ttyACM0";
};

#endif  /* !_MOVEMENT_H_ */

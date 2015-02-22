#include "AThread.h"
#include <unistd.h>


AThread::AThread()
{
    _threadState = PAUSED;
    _thread = new std::thread([this]()
    {
        while (_threadState != STOP_REQUESTED)
        {
            if (_threadState == PAUSE_REQUESTED)
                _threadState = PAUSED;
            while (_threadState == PAUSED)
                usleep(10000);
            run();
        }
        _threadState = STOPPED;
    });
}


AThread::~AThread()
{
    this->stop();
    while (this->_threadState != STOPPED);
    delete _thread;
}


void            AThread::start()
{
    _threadState = STARTED;
}


void            AThread::stop()
{
    _threadState = STOP_REQUESTED;
}


void AThread::pause()
{
    _threadState = PAUSE_REQUESTED;
}

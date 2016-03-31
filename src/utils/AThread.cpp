#include "AThread.h"
#include <unistd.h>
#include <iostream>

AThread::AThread()
{
    _threadState = PAUSED;
    _thread = new std::thread([this]()
    {
        while (_threadState != STOP_REQUESTED)
        {
            if (_threadState == PAUSE_REQUESTED) {
                _threadState = PAUSED;
                //std::cerr << "SETTING STATE TO PAUSED" << std::endl;
            }
            while (_threadState == PAUSED)
                {
                    usleep(10000);
                    //std::cerr << "We are PAUSED" << std::endl;
                }
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
    //std::cerr << "STARTING THREAD" << std::endl;
    _threadState = STARTED;
}


void            AThread::stop()
{
    _threadState = STOP_REQUESTED;
}


void AThread::pause()
{
    //std::cerr << "PAUSE_REQUESTED" << std::endl;
    _threadState = PAUSE_REQUESTED;
}

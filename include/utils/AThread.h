#ifndef     THREAD_H_
# define    THREAD_H_

#include <thread>

class       AThread
{
public:
    AThread();
    ~AThread();

    void                start();
    void                stop();
    void                pause();            /**< Not instantaneous*/

protected:
    enum        e_thread_state
    {
        STOPPED,
        STOP_REQUESTED,
        PAUSED,
        PAUSE_REQUESTED,
        STARTED
    };

    virtual void        run() = 0;              /**< main thread function*/

    e_thread_state      _threadState;
    std::thread         *_thread;
};

#endif

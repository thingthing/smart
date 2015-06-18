#ifndef     THREAD_H_
# define    THREAD_H_

#include <thread>

/** @brief Base class for all threads.

    Inheriting from this class will make the run() function run in a thread and called in a loop.
    DO NOT make your own infinite loop in the run function. This is already handled by a lambda in the constructor,
    and allows the thread function to be easily stopped or paused.
    Note: if stopped, it won't be possible to start the thread again.
    Stop and pause are not instantaneous. It may at most take one main loop cycle (one call to run).

    @author Maxime C.
    @date June 2015
    */
class       AThread
{
public:
    AThread();
    virtual ~AThread();

    void                start();            /**< @brief Start the thread */
    void                stop();             /**< @brief Stop the thread */
    void                pause();            /**< @brief Pause the thread */

protected:
    enum        e_thread_state
    {
        STOPPED,
        STOP_REQUESTED,
        PAUSED,
        PAUSE_REQUESTED,
        STARTED
    };

    virtual void        run() = 0;              /**< @brief Main thread function */

    e_thread_state      _threadState;
    std::thread         *_thread;
};

#endif

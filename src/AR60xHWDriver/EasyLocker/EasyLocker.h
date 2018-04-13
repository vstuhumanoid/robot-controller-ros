//
// Created by garrus on 14.04.18.
//

#ifndef ROBOT_CONTROLLER_ROS_EASYLOCKER_H
#define ROBOT_CONTROLLER_ROS_EASYLOCKER_H

#include <condition_variable>
#include <mutex>

/**
 * Simplify using of condition_variable
 */
class EasyLocker
{
public:

    EasyLocker();

    /**
     * Notify one thread
     */
    void Notify();

    /**
     * Notify all threads
     */
    void NotifyAll();

    /**
     * Wait for notification
     */
    void Wait();

private:

    void SetFlag();

    bool is_notified_;
    std::mutex mutex_;
    std::condition_variable variable_;
};


#endif //ROBOT_CONTROLLER_ROS_EASYLOCKER_H

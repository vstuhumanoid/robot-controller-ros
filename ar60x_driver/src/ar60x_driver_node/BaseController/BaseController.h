#ifndef ROBOT_CONTROLLER_ROS_BASECONTROLLER_H
#define ROBOT_CONTROLLER_ROS_BASECONTROLLER_H

#include <experimental/optional>
#include <string>
#include <atomic>
#include <thread>
#include <ros/ros.h>
#include <AR60xHWDriver.h>

/**
 * Base class for all driver-ROS communcation controllers.
 * Incapsulate thread-related work.
 */
class BaseController
{
public:
    /**
     * Create new BaseController with automatic update in
     * background thread
     *
     * @param driver Reference to robot driver
     * @param nh  Reference to node handler
     * @param publishing_frequency Publishing frequency (in Hz)
     */
    BaseController(AR60xHWDriver& driver,  ros::NodeHandle& nh, double publishing_frequency);

    /**
     * Create new BaseController with manual updating.
     * You should manually call BaseController::Update() to perform operations
     *
     * @param driver Reference to robot driver
     * @param nh  Reference to node handler
     */
    BaseController(AR60xHWDriver& driver,  ros::NodeHandle& nh);

    ~BaseController();

    /**
     * Start publishing.
     * Do not use this function in manual mode. Use BaseController::Update() instead.
     */
    void Start();

    /**
     * Stop publishing
     * Do not use this function in manual mode. Use BaseController::Update() instead.
     */
    void Stop();

    /**
     * Manual updating. Call this function periodically.
     * Do not use this function in automatic mode.
     */
    void Update();

protected:
    virtual void loop() = 0;
    virtual std::string controller_name() = 0;

    AR60xHWDriver& driver_;
    ros::NodeHandle& nh_;

private:
    void thread_func();
    bool is_auto_;
    std::thread publising_thread_;
    std::atomic_bool is_running_;
    std::experimental::optional<ros::Rate> publishing_rate_;
};


#endif //ROBOT_CONTROLLER_ROS_BASECONTROLLER_H

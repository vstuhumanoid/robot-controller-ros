#ifndef ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H
#define ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

#include <string>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/builtin_bool.h>
#include <AR60xHWDriver.h>
#include <DataTypes/PowerData.h>
#include <robot_controller_ros/robot_supply_state.h>
#include <robot_controller_ros/supply_state.h>
#include <robot_controller_ros/joint_supply_state.h>

using namespace robot_controller_ros;

/**
 * @brief This is a class for ROS-interface for robot power control.
 *
 * Features:
 *
 * Publishing:
 *  - publishing whole robot supply state (voltage, current, power-on?)
 *  - publishing joints' supply state (voltage, current)
 * Commands (subscribe):
 *  - On/off whole robot
 *
 */
class PowerController
{
public:

    /**
     * Create new PowerController
     *
     * Note: don't forget to init
     */
    PowerController();

    /**
     * Create new PowerController
     *
     * If driver and NodeHandler aren't accessible during object
     * initialization, you must pass them later PowerController::Init() before using this
     * object.
     *
     * @param driver Pointer to robot driver
     * @param nh  Pointer to node handler
     * @param publishingFrequency Publishing frequency (in Hz)
     */
    PowerController(AR60xHWDriver *driver,  ros::NodeHandle *nh,  double publishingFrequency);


    /**
     * Initialize driver and NodeHandle if they haven't initialized yet
     * @param driver Pointer to robot driver
     * @param nh  Pointer to node handler
     * @param publishingFrequency Publishing frequency (in Hz)
     */
    void Init(AR60xHWDriver *driver, ros::NodeHandle *nh, double publishingFrequency);

    ~PowerController();

    /**
     * Start publishing
     */
    void Start();

    /**
     * Stop publishing
     */
    void Stop();

private:

    void thread_func();
    void robot_supply_command_cb(std_msgs::Bool msg);
    void power_on();
    void power_off();
    supply_state get_supply_state(PowerData::PowerSupplies supply);

    AR60xHWDriver* driver_;
    ros::NodeHandle* nh_;
    std::thread publising_thread_;
    ros::Rate publishing_rate_;
    bool is_running_;

    ros::Publisher robot_supply_state_publisher_;
    ros::Publisher joints_supply_state_publiser_;
    ros::Subscriber power_commands_subscriber_;         // on/off command
};


#endif //ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

#ifndef ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H
#define ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

#include <string>
#include <thread>
#include <vector>
#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/builtin_bool.h>
#include <AR60xHWDriver.h>
#include <DataTypes/PowerSources.h>
#include <robot_controller_ros/RobotSupplyState.h>
#include <robot_controller_ros/SupplyState.h>
#include <robot_controller_ros/JointsSupplyState.h>
#include <BaseController/BaseController.h>

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
class PowerController : BaseController
{
public:

    /**
     * Create new PowerController
     *
     * @param driver Pointer to robot driver
     * @param nh  Pointer to node handler
     * @param publishingFrequency Publishing frequency (in Hz)
     */
    PowerController(AR60xHWDriver& driver,  ros::NodeHandle& nh,  double publishingFrequency);
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
    SupplyState get_supply_state(PowerSources::PowerSupplies supply);
    void publish_robot_supply_state();
    void publish_joints_supply_state();


    const std::string namespace_ = "power";

    std::thread publising_thread_;
    ros::Rate publishing_rate_;
    bool is_running_;
    std::vector<uint8_t> joints_;

    ros::Publisher robot_supply_state_publisher_;
    ros::Publisher joints_supply_state_publiser_;
    ros::Subscriber power_commands_subscriber_;         // on/off command
};


#endif //ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

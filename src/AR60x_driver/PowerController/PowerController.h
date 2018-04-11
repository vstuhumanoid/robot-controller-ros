#ifndef ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H
#define ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/builtin_bool.h>
#include <DataTypes/PowerSources.h>
#include <robot_controller_ros/SourcesSupplyState.h>
#include <robot_controller_ros/JointsSupplyState.h>
#include <AR60xHWDriver.h>
#include <BaseController/BaseController.h>

using namespace robot_controller_ros;

/**
 * @brief This is a class for ROS-interface for robot power control.
 *
 * Functions:
 *
 * Publishing:
 *  - publishing whole robot supply state (voltage, current, power-on?)
 *  - publishing joints' supply state (voltage, current)
 * Subscribe:
 *  - On/off whole robot
 *
 */
class PowerController : public BaseController
{
public:

    /**
     * Create new PowerController with manual update
     *
     * @param driver Reference to robot driver
     * @param nh  Reference to node handler
     */
    PowerController(AR60xHWDriver& driver,  ros::NodeHandle& nh);

    /**
     * Create new PowerController with automatic update in
     * background thread
     *
     * @param driver Reference to robot driver
     * @param nh  Reference to node handler
     * @param publishingFrequency Publishing frequency (in Hz)
     */
    PowerController(AR60xHWDriver& driver,  ros::NodeHandle& nh,  double publishingFrequency);

    void PowerOn();
    void PowerOff();

private:
    void init_topics();
    void loop() override;
    std::string controller_name() override;
    void robot_supply_command_cb(std_msgs::Bool msg);

    const std::string namespace_ = "power";



    ros::Publisher robot_supply_state_publisher_;
    ros::Publisher joints_supply_state_publiser_;
    ros::Subscriber power_commands_subscriber_;         // on/off command
};


#endif //ROBOT_CONTROLLER_ROS_POWER_CONTROLLER_H

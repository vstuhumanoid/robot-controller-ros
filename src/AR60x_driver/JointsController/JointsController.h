//
// Created by garrus on 28.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_JOINTSCONTROLLER_H
#define ROBOT_CONTROLLER_ROS_JOINTSCONTROLLER_H

#include <ros/ros.h>
#include <BaseController/BaseController.h>
#include <AR60xHWDriver.h>

#include <sensor_msgs/JointState.h>
#include <robot_controller_ros/JointsCommand.h>
#include <robot_controller_ros/JointsParams.h>
#include <robot_controller_ros/JointsMode.h>

/**
 * @brief This is a class for ROS-interface for robot joints control
 *
 * Functions:
 *
 * Publising:
 *  - joint states
 *  - joint params: all joints parameters, such as limits, offset,
 *    mode, pid gains etc
 *
 * Subscribe:
 *  - sending command to joints (position and pid gains)
 *  - setting joint params
 *  - setting joint mode
 *
 */
class JointsController : public BaseController
{
public:

    /**
    * Create new JointsController with manual update
    *
    * @param driver Reference to robot driver
    * @param nh  Reference to node handler
    */
    JointsController(AR60xHWDriver& driver, ros::NodeHandle& nh);
    JointsController(AR60xHWDriver& driver, ros::NodeHandle& nh, double publishingFrequency);

    /**
      * Create new JointsController with automatic update in
      * background thread
      *
      * @param driver Reference to robot driver
      * @param nh  Reference to node handler
      * @param publishingFrequency Publishing frequency (in Hz)
      */
    void PublishJoints();

private:

    void init_topics();
    void loop() override;
    std::string controller_name() override;

    void joint_command_cb(robot_controller_ros::JointsCommand msg);
    void joint_params_cb(robot_controller_ros::JointsParams msg);
    void joint_mode_cb(robot_controller_ros::JointsMode msg);

    const std::string namespace_ = "joints";

    ros::Publisher joint_state_publisher_;
    ros::Publisher joint_params_publisher_;

    ros::Subscriber joint_command_subscriber_;
    ros::Subscriber joint_params_subscriber_;
    ros::Subscriber joint_mode_subscriber_;
};


#endif //ROBOT_CONTROLLER_ROS_JOINTSCONTROLLER_H

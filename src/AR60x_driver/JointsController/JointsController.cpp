//
// Created by garrus on 28.03.18.
//

#include "JointsController.h"

JointsController::JointsController(AR60xHWDriver &driver, ros::NodeHandle& nh) :
    BaseController(driver, nh)
{
    init_topics();
}

JointsController::JointsController(AR60xHWDriver &driver, ros::NodeHandle &nh, double publishingFrequency)
        : BaseController(driver, nh, publishingFrequency)
{
    init_topics();
}

void JointsController::init_topics()
{
    joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>(namespace_ + "/state", 1000);
    joint_params_publisher_ = nh_.advertise<robot_controller_ros::JointsParams>(namespace_ + "/get_params", 100, true);

    joint_command_subscriber_ = nh_.subscribe(namespace_ + "/commands", 1000, &JointsController::joint_command_cb, this);
    joint_params_subscriber_ = nh_.subscribe(namespace_ + "/set_params", 100, &JointsController::joint_params_cb, this);
    joint_mode_subscriber_ = nh_.subscribe(namespace_ + "/set_mode", 100, &JointsController::joint_mode_cb, this);
}


std::string JointsController::controller_name()
{
    return "JointsControler";
}


void JointsController::loop()
{
    auto msg = driver_.JointsGetState();
    joint_state_publisher_.publish(msg);
}


void JointsController::PublishJoints()
{
    driver_.WaitForReceive();
    joint_params_publisher_.publish(driver_.JointsGetParams());
}


void JointsController::joint_command_cb(robot_controller_ros::JointsCommand msg)
{
    driver_.JointsSetCommand(msg);
}

void JointsController::joint_params_cb(robot_controller_ros::JointsParams msg)
{
    // We should re-publish joints params when something is changed
    driver_.JointsSetParams(msg);
    PublishJoints();
}

void JointsController::joint_mode_cb(robot_controller_ros::JointsMode msg)
{
    // We should re-publish joints params when something is changed
    driver_.JointsSetMode(msg);
    PublishJoints();
}

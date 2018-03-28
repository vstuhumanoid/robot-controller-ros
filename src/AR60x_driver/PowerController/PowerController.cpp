//
// Created by garrus on 23.03.18.
//

#include "PowerController.h"

PowerController::PowerController(AR60xHWDriver &driver, ros::NodeHandle &nh) : BaseController(driver, nh)
{
    init_topics();
}

PowerController::PowerController(AR60xHWDriver& driver, ros::NodeHandle& nh, double publishingFrequency) :
    BaseController(driver, nh, publishingFrequency)
{
    init_topics();
}

void PowerController::init_topics()
{
    robot_supply_state_publisher_ = nh_.advertise<RobotSupplyState>(namespace_ + "/sources_state", 100);
    joints_supply_state_publiser_ = nh_.advertise<JointsSupplyState>(namespace_ + "/joints_state", 100);
    power_commands_subscriber_ = nh_.subscribe<std_msgs::Bool>(namespace_ + "/command", 100, &PowerController::robot_supply_command_cb, this);
}


std::string PowerController::controller_name()
{
    return "PowerController";
}

void PowerController::loop()
{
    robot_supply_state_publisher_.publish(driver_.PowerGetSourcesSupplyState());
    joints_supply_state_publiser_.publish(driver_.PowerGetJointsSupplyState());
}


void PowerController::robot_supply_command_cb(std_msgs::Bool msg)
{
    if(msg.data)
        power_on();
    else
        power_off();
}

void PowerController::power_on()
{
    ROS_INFO("Power on command received. Powering on robot...");

    driver_.SupplySetOnOff(PowerSources::Supply12V, true);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply48V, true);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply8V1, true);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply8V2, true);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply6V1, true);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply6V2, true);
    ros::Duration(0.5).sleep();

    ROS_INFO("Power on commands sent");
}

void PowerController::power_off()
{
    ROS_INFO("Power off command received. Powering off robot...");

    driver_.SupplySetOnOff(PowerSources::Supply6V1, false);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply6V2, false);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply8V1, false);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply8V2, false);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply48V, false);
    ros::Duration(0.5).sleep();
    driver_.SupplySetOnOff(PowerSources::Supply12V, false);
    ros::Duration(0.5).sleep();

    ROS_INFO("Power off commands sent");
}



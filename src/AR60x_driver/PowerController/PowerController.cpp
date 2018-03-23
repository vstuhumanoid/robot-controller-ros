//
// Created by garrus on 23.03.18.
//

#include "PowerController.h"


PowerController::PowerController()
    :publishing_rate_(5) // default value
{

}


PowerController::PowerController(AR60xHWDriver *driver, ros::NodeHandle *nh, double publishingFrequency)
    :publishing_rate_(publishingFrequency)
{
    Init(driver, nh, publishingFrequency);
}

void PowerController::Init(AR60xHWDriver *driver, ros::NodeHandle *nh, double publishingFrequency)
{
    driver_ = driver;
    nh_ = nh;
    publishing_rate_ = ros::Rate(publishingFrequency);

    robot_supply_state_publisher_ = nh->advertise<robot_supply_state>("robot_supply_state", 100);
    joints_supply_state_publiser_ = nh->advertise<joint_supply_state>("joint_supply_state", 100);

    power_commands_subscriber_ = nh->subscribe<std_msgs::Bool>("robot_supply_command", 100,
                                                              &PowerController::robot_supply_command_cb, this);
}

PowerController::~PowerController()
{
    Stop();
}


void PowerController::Start()
{
    if(!is_running_)
    {
        ROS_INFO("Starting PowerController...");
        is_running_ = true;
        publising_thread_ = std::thread(&PowerController::thread_func, this);
        publising_thread_.detach();
        ROS_INFO("PowerController started");
    }
    else
    {
        ROS_WARN("PowerController is already running");
    }
}

void PowerController::Stop()
{
    if(is_running_)
    {
        ROS_INFO("Stopping PowerController...");
        publising_thread_.join();
        ROS_INFO("PowerController stopped");
    }
}

void PowerController::thread_func()
{
    while(is_running_)
    {
        robot_supply_state msg;
        msg.S48 = get_supply_state(PowerData::Supply48V);
        msg.S12 = get_supply_state(PowerData::Supply12V);
        msg.S8_1 = get_supply_state(PowerData::Supply8V1);
        msg.S8_2 = get_supply_state(PowerData::Supply8V2);
        msg.S6_1 = get_supply_state(PowerData::Supply6V1);
        msg.S6_2 = get_supply_state(PowerData::Supply6V2);
        robot_supply_state_publisher_.publish(msg);

        // TODO: Publish all joints supply state

        publishing_rate_.sleep();
    }
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

    driver_->SupplySetOnOff(PowerData::Supply12V, true);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply48V, true);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply8V1, true);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply8V2, true);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply6V1, true);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply6V2, true);
    ros::Duration(0.5).sleep();

    ROS_INFO("Power on commands sent");
}

void PowerController::power_off()
{
    ROS_INFO("Power off command received. Powering off robot...");

    driver_->SupplySetOnOff(PowerData::Supply6V1, false);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply6V2, false);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply8V1, false);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply8V2, false);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply48V, false);
    ros::Duration(0.5).sleep();
    driver_->SupplySetOnOff(PowerData::Supply12V, false);
    ros::Duration(0.5).sleep();

    ROS_INFO("Power off commands sent");
}


supply_state PowerController::get_supply_state(PowerData::PowerSupplies supply)
{
    supply_state ros_state;
    auto state = driver_->PowerGetSupplyState(supply);
    ros_state.Current = state.Current;
    ros_state.Voltage = state.Voltage;
    return ros_state;
}




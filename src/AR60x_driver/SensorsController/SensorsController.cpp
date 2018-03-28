//
// Created by garrus on 28.03.18.
//

#include "SensorsController.h"

SensorsController::SensorsController(AR60xHWDriver &driver, ros::NodeHandle &nh) : BaseController(driver, nh)
{
    init_topics();
}

SensorsController::SensorsController(AR60xHWDriver &driver, ros::NodeHandle &nh, double publishing_frequency)
        : BaseController(driver, nh, publishing_frequency)
{
    init_topics();
}

void SensorsController::init_topics()
{
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(namespace_ + "/imu", 1000);
    feet_publisher_ = nh_.advertise<robot_controller_ros::FeetSensors>(namespace_ + "/feet", 1000);
}


std::string SensorsController::controller_name()
{
    return "SensorsController";
}

void SensorsController::loop()
{
    imu_publisher_.publish(driver_.SensorGetImu());
    feet_publisher_.publish(driver_.SensorGetFeet());
}



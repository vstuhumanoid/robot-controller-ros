//
// Created by garrus on 28.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_SENSORSCONTROLLER_H
#define ROBOT_CONTROLLER_ROS_SENSORSCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <robot_controller_ros/FeetSensors.h>
#include <AR60xHWDriver.h>
#include <BaseController/BaseController.h>

/**
 * @brief This is a class for ROS-interface for robot sensors
 *
 * Functions:
 *
 * Publishing:
 *  - publishing data from IMU sensor
 *  - publishing data from feet pressure sensors
 *
 * TODO: Sensors calibration
 */
class SensorsController : public BaseController
{
public:

    SensorsController(AR60xHWDriver &driver, ros::NodeHandle &nh);
    SensorsController(AR60xHWDriver &driver, ros::NodeHandle &nh, double publishing_frequency);

private:
    void init_topics();
    void loop() override;
    std::string controller_name() override;

    std::string namespace_ = "sensors";

    ros::Publisher imu_publisher_;
    ros::Publisher feet_publisher_;
};


#endif //ROBOT_CONTROLLER_ROS_SENSORSCONTROLLER_H

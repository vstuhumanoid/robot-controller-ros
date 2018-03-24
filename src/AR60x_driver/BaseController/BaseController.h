#ifndef ROBOT_CONTROLLER_ROS_BASECONTROLLER_H
#define ROBOT_CONTROLLER_ROS_BASECONTROLLER_H

#include <ros/ros.h>
#include <AR60xHWDriver.h>

class BaseController
{
public:
    /**
     * Create new BaseController
     *
     * @param driver Pointer to robot driver
     * @param nh  Pointer to node handler
     * @param publishingFrequency Publishing frequency (in Hz)
     */
    BaseController(AR60xHWDriver& driver,  ros::NodeHandle& nh);

protected:
    AR60xHWDriver& driver_;
    ros::NodeHandle& nh_;
};


#endif //ROBOT_CONTROLLER_ROS_BASECONTROLLER_H

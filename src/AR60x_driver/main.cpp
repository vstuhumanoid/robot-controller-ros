#include <ros/ros.h>
#include "AR60xHWDriver/AR60xHWDriver.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    ros::Rate rate(1);

    while(ros::ok())
    {
        ROS_INFO("Hello");
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
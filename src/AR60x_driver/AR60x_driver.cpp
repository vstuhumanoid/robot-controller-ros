#include <ros/ros.h>
#include <std_msgs/builtin_float.h>

#include <AR60xHWDriver.h>
#include "PowerController/PowerController.h"

using namespace std;
using namespace robot_controller_ros;


AR60xHWDriver driver;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    PowerController power_controller(&driver, &nh, 5);

    // Load config and connect to robot
    string config_filename;
    nh.getParam("driver_config", config_filename);
    driver.loadConfig(config_filename);
    driver.robotConnect();

    power_controller.Start();

    ros::spin();

    return 0;
}



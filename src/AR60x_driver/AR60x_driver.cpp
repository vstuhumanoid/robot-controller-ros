#include <ros/ros.h>
#include <std_msgs/builtin_float.h>

#include <AR60xHWDriver.h>
#include <PowerController/PowerController.h>
#include <JointsController/JointsController.h>
#include <SensorsController/SensorsController.h>

using namespace std;
using namespace robot_controller_ros;


AR60xHWDriver driver;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    // Load config and connect to robot
    string config_filename;
    nh.getParam("driver_config", config_filename);
    driver.LoadConfig(config_filename);

    PowerController power_controller(driver, nh, 5);
    JointsController jointsController(driver, nh, 50);
    SensorsController sensorsController(driver, nh, 50);

    driver.RobotConnect();
    power_controller.Start();
    jointsController.Start();
    sensorsController.Start();

    jointsController.PublishJoints();

    while(ros::ok())
    {
        ros::spinOnce();
    }


    return 0;
}



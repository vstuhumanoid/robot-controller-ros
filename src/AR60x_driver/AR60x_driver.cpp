#include <ros/ros.h>
#include <std_msgs/builtin_float.h>

#include <AR60xHWDriver.h>
#include <PowerController/PowerController.h>
#include <JointsController/JointsController.h>
#include <SensorsController/SensorsController.h>

using namespace std;
using namespace robot_controller_ros;


void connect_cb(const std_msgs::Bool& msg);

AR60xHWDriver driver;
ros::Subscriber connection_sub;
ros::Publisher connection_pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;
    connection_sub = nh.subscribe("connection/command", 100, connect_cb);
    connection_pub = nh.advertise<std_msgs::Bool>("connection/state", 100);

    // Load config and connect to robot
    string config_filename;
    nh.getParam("driver_config", config_filename);
    driver.LoadConfig(config_filename);

    PowerController power_controller(driver, nh, 5);
    JointsController jointsController(driver, nh);
    SensorsController sensorsController(driver, nh);

    driver.RobotConnect();

    /*power_controller.Start();
    jointsController.Start();
    sensorsController.Start();
    power_controller.PowerOn();
    jointsController.PublishJoints();*/

    ros::Rate rate(0.01); //TODO: from config

    while(ros::ok())
    {
        driver.Read();
        jointsController.Update();
        sensorsController.Update();
        driver.Write();

        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}

void connect_cb(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        // Connect to robot
    }
    else
    {
        // Disconnect
    }
}
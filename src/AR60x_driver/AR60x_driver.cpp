#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/builtin_float.h>
#include <AR60xHWDriver.h>
#include <robot_controller_ros/robot_supply_state.h>

using namespace std;
using namespace robot_controller_ros;

AR60xHWDriver driver;

void powerOn();
void powerOff();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;
    ros::Rate rate(1);

    // Load config and connect to robot
    string config_filename;
    nh.getParam("driver_config", config_filename);
    driver.loadConfig(config_filename);
    driver.robotConnect();


    auto pub = nh.advertise<robot_controller_ros::supply_state>("power48", 100);


    while(ros::ok())
    {
        supply_state msg;
        auto v48 = driver.PowerGetSupplyState(PowerData::Supply48V);
        msg.Current = v48.Current;
        msg.Voltage = v48.Voltage;


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void powerOn()
{
    driver.SupplySetOnOff(PowerData::Supply12V, true);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply48V, true);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply8V1, true);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply8V2, true);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply6V1, true);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply6V2, true);
    ros::Duration(0.5).sleep();
}

void powerOff()
{
    driver.SupplySetOnOff(PowerData::Supply6V1, false);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply6V2, false);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply8V1, false);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply8V2, false);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply48V, false);
    ros::Duration(0.5).sleep();
    driver.SupplySetOnOff(PowerData::Supply12V, false);
    ros::Duration(0.5).sleep();
}
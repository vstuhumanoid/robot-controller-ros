#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/builtin_float.h>
#include <AR60xHWDriver.h>

using namespace std;

AR60xHWDriver driver;

void command_callback(const std_msgs::Float32 angle)
{
    driver.JointSetPosition(10, (int)(angle.data));

    ROS_INFO("Command: %f", angle.data);
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

int main(int argc, char** argv)
{

    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    ros::Rate rate(1);

    string path = ros::package::getPath("robot-controller-ros") + "/config_new.xml";
    driver.loadConfig(path);
    driver.robotConnect();

    powerOn();

    ROS_INFO("Set start pose");
    driver.SetStartPose();
    ros::Duration(1).sleep();

    auto sub = nh.subscribe("command", 1000, command_callback);


    while(ros::ok())
    {
        auto state = driver.PowerGetSupplyState(PowerData::Supply48V);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        state = driver.PowerGetSupplyState(PowerData::Supply12V);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        state = driver.PowerGetSupplyState(PowerData::Supply8V1);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        state = driver.PowerGetSupplyState(PowerData::Supply8V2);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        state = driver.PowerGetSupplyState(PowerData::Supply6V1);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        state = driver.PowerGetSupplyState(PowerData::Supply6V2);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        std::cout<<endl;


        ros::spinOnce();
        rate.sleep();
    }

    powerOff();

    return 0;
}
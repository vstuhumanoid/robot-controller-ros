#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/builtin_float.h>
#include <AR60xHWDriver.h>

using namespace std;

AR60xHWDriver driver;

void command_callback(const std_msgs::Float32 angle)
{
    driver.JointSetPosition(1, (int)(angle.data * 100));

    JointState state;
    state.state = JointState::MotorState::TRACE;
    state.controlType = JointState::ControlType::POSITION_CONTROl;
    driver.JointSetState(1, state);

    ROS_INFO("Command: %f", angle.data);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    ros::Rate rate(1);

    string path = ros::package::getPath("robot-controller-ros") + "/config.xml";
    driver.loadConfig(path);
    driver.robotConnect();

    driver.SupplySetOnOff(PowerData::Supply12V, true);
    rate.sleep();
    driver.SupplySetOnOff(PowerData::Supply6V1, true);
    driver.SupplySetOnOff(PowerData::Supply6V2, true);
    driver.SupplySetOnOff(PowerData::Supply8V1, true);
    driver.SupplySetOnOff(PowerData::Supply8V2, true);
    driver.SupplySetOnOff(PowerData::Supply48V, true);
    rate.sleep();

    //auto sub = nh.subscribe("command", 1000, command_callback);


    while(ros::ok())
    {
        auto state = driver.PowerGetSupplyState(PowerData::Supply48V);
        ROS_INFO("Voltage: %f\t Current: %f", state.Voltage, state.Current);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
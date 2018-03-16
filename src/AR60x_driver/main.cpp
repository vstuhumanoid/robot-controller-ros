#include <ros/ros.h>
#include <ros/package.h>
#include <experimental/filesystem>
#include <fstream>
#include <string>
#include "AR60xHWDriver/AR60xHWDriver.h"
#include <QString>

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char** argv)
{
    QString wtf = QString::fromStdString("WTF");

    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;

    ros::Rate rate(1);

    string path = ros::package::getPath("robot-controller-ros")+"/config.xml";
    AR60xHWDriver driver;
    driver.loadConfig(path);

    driver.initPackets();
    driver.robotConnect();

    driver.SupplySetState(PowerData::Supply12V, true);
    rate.sleep();
    driver.SupplySetState(PowerData::Supply6V1, true);
    driver.SupplySetState(PowerData::Supply6V2, true);
    driver.SupplySetState(PowerData::Supply8V1, true);
    driver.SupplySetState(PowerData::Supply8V2, true);
    driver.SupplySetState(PowerData::Supply48V, true);
    rate.sleep();




    while(ros::ok())
    {
        auto state = driver.PowerGetSupplyState(PowerData::Supply48V);
        ROS_DEBUG("Voltage: %f\t Current: %f", state.Voltage, state.Current);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
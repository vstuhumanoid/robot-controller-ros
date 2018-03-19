#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wtf");
    ros::NodeHandle nh;


    auto str = ros::package::getPath("roscpp");
    cout << str;

    return 0;

}
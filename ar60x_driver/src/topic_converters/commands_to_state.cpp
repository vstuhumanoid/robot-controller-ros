#include <ros/ros.h>
#include <robot_msgs/JointsCommand.h>
#include <sensor_msgs/JointState.h>
#include <robot_msgs/JointsParams.h>
#include <string>

using namespace std;
using namespace ros;

Publisher state_publisher;
robot_msgs::JointsParams params;

void callback(const robot_msgs::JointsCommand& msg);
bool get_reverse(string joint);

sensor_msgs::JointState state_msg;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Commands to state converter");
    NodeHandle nh;

    params = *ros::topic::waitForMessage<robot_msgs::JointsParams>("/joints/get_params");

    auto sub = nh.subscribe("/joints/commands", 1000, callback);
    state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    ros::spin();

    return 0;
}

void callback(const robot_msgs::JointsCommand& msg)
{
    state_msg.name.resize(msg.names.size());
    state_msg.position.resize(msg.positions.size());

    for(int i = 0; i<msg.names.size(); i++)
        state_msg.name[i] = msg.names[i];

    for(int i = 0; i<msg.positions.size(); i++)
    {
        double angle  = msg.positions[i];
        if(get_reverse(state_msg.name[i]))
            angle = -angle;

        state_msg.position[i] = angle;
    }


    state_msg.header.stamp = ros::Time::now();
    state_publisher.publish(state_msg);
}

bool get_reverse(string joint)
{
    /*for(int i = 0; i<params.names.size(); i++)
    {
        if(params.names[i] == joint)
            return params.reverse[i];
    }

    ROS_WARN_STREAM("Topic not found");
    return false;*/

    if(joint == "hip_s_joint_right")
        return true;
    if(joint == "hip_r_joint_right")
        return true;
    if(joint == "shoulder_s_joint_right")
        return true;
    if(joint == "ankle_s_joint_right")
        return true;
    if(joint == "knee_joint_right")
        return true;
    if(joint == "knee_joint_left")
        return true;


    return false;
}
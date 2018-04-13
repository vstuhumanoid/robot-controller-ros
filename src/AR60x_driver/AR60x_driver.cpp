#include <ros/ros.h>
#include <std_msgs/builtin_float.h>

#include <AR60xHWDriver.h>
#include <PowerController/PowerController.h>
#include <JointsController/JointsController.h>
#include <SensorsController/SensorsController.h>

using namespace std;
using namespace robot_controller_ros;


void connect_cb(const std_msgs::Bool& msg);
void check_connection();

AR60xHWDriver driver;
ros::Publisher connection_pub;
bool previous_connection_state;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AR60x_driver");
    ros::NodeHandle nh;
    connection_pub = nh.advertise<std_msgs::Bool>("connection/state", 100, true);
    previous_connection_state = false;

    // Load config and connect to robot
    string config_filename;
    if(!nh.getParam("driver_config", config_filename))
    {
        ROS_ERROR("Parameter \"driver_config\" not set");
        return -1;
    }

    driver.LoadConfig(config_filename);

    PowerController power_controller(driver, nh, 5);
    JointsController jointsController(driver, nh);
    SensorsController sensorsController(driver, nh);

    driver.RobotConnect();

    power_controller.Start();
    power_controller.PowerOn();
    jointsController.PublishJoints();

    ros::Rate rate(1e3 / driver.GetConnectionData().sendDelay); // sendDelay in ms

    while(ros::ok())
    {
        //driver.Write();
        //driver.Read();
        jointsController.Update();
        sensorsController.Update();
        check_connection();
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}

void check_connection()
{
    bool current_state = driver.CheckConnection();
    if(current_state != previous_connection_state)
    {
        std_msgs::Bool msg;
        msg.data = current_state;
        connection_pub.publish(msg);
        previous_connection_state = current_state;

        if(current_state)
            ROS_INFO("Robot connected");
        else
            ROS_INFO("Robot disconnected");
    }
}
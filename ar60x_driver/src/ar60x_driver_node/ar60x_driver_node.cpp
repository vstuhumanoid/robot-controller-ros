#include <thread>
#include <functional>
#include <ros/ros.h>
#include <std_msgs/builtin_float.h>

#include <AR60xHWDriver.h>
#include <PowerController/PowerController.h>
#include <JointsController/JointsController.h>
#include <SensorsController/SensorsController.h>

using namespace std;
using namespace robot_msgs;


void check_connection();
void robot_thread_func(JointsController& jointsController, SensorsController& sensorsController);

AR60xHWDriver driver;
ros::Publisher connection_pub;
bool previous_connection_state;
bool is_running;

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

    // Create controllers
    PowerController power_controller(driver, nh, 5);
    JointsController jointsController(driver, nh);
    SensorsController sensorsController(driver, nh);

    driver.RobotConnect();
    power_controller.Start();

    // Starting main communication thread
    ROS_INFO("Starting main thread...");
    is_running = true;
    std::thread robot_thread(robot_thread_func, ref(jointsController), ref(sensorsController));
    robot_thread.detach();
    ROS_INFO("Started");

    // Power on robot and publishing initial joints params
    jointsController.PublishJoints();
    //driver.WaitForReceive();
    power_controller.PowerOn();

    ros::spin();

    // Stopping main communication thread
    ROS_INFO("Stopping main thread...");
    is_running = false;
    robot_thread.join();
    ROS_INFO("Stopped");

    return 0;
}

// Communication with robot and publishing topics
void robot_thread_func(JointsController& jointsController, SensorsController& sensorsController)
{
    ros::Rate rate(1e3 / driver.GetConnectionData().sendDelay); // sendDelay in ms

    while(is_running)
    {
        driver.Write();
        driver.Read();
        jointsController.Update();
        sensorsController.Update();

        check_connection();
        rate.sleep();
    }
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
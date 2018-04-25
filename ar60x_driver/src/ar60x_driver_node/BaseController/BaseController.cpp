#include "BaseController.h"

BaseController::BaseController(AR60xHWDriver &driver, ros::NodeHandle &nh) :
    driver_(driver),
    nh_(nh)
{
    is_auto_ = false;
}


BaseController::BaseController(AR60xHWDriver& driver,  ros::NodeHandle& nh, double publishing_frequency) :
    BaseController(driver, nh)
{
    publishing_rate_ = ros::Rate(publishing_frequency);
    is_running_.store(false);
    is_auto_ = true;
}


BaseController::~BaseController()
{
    if(is_auto_)
        Stop();
}

void BaseController::Start()
{
    if(!is_auto_)
    {
        ROS_ERROR("This controller in manual mode. Use manual Update() instead");
        throw std::runtime_error("Invalid controller mode");
    }

    if(!is_running_.load())
    {
        ROS_INFO_STREAM("Starting " << controller_name() << "...");
        is_running_.store(true);
        publising_thread_ = std::thread(&BaseController::thread_func, this);
        publising_thread_.detach();
        ROS_INFO_STREAM(controller_name() << " started");
    }
    else
    {
        ROS_WARN_STREAM(controller_name() << " is already running");
    }
}

void BaseController::Stop()
{
    if(!is_auto_)
    {
        ROS_ERROR("This controller in manual mode. Use manual Update() instead");
        throw std::runtime_error("Invalid controller mode");
    }

    if(is_running_.load())
    {
        ROS_INFO_STREAM("Stopping " << controller_name() << "...");
        publising_thread_.join();
        ROS_INFO_STREAM(controller_name() << " stopped");
    }
}


void BaseController::Update()
{
    loop();
}


void BaseController::thread_func()
{
    while(is_running_.load())
    {
        loop();
        publishing_rate_.value().sleep();
    }
}

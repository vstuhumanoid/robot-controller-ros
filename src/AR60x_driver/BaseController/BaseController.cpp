#include "BaseController.h"

BaseController::BaseController(AR60xHWDriver& driver,  ros::NodeHandle& nh) :
    driver_(driver),
    nh_(nh)
{

}

//
// Created by user on 22.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_LEGSDATA_H
#define ROBOT_CONTROLLER_ROS_LEGSDATA_H

struct SensorFeetState
{
    struct FootData
    {
        double fx, fy, fz;
        double uch0, uch1, uch2, uch3;
    };

    FootData left, right;
};

#endif //ROBOT_CONTROLLER_ROS_LEGSDATA_H

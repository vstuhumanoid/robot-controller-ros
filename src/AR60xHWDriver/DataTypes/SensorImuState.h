//
// Created by user on 22.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_IMUDATA_H
#define ROBOT_CONTROLLER_ROS_IMUDATA_H


struct SensorImuState
{
    double yaw, pitch, roll;
    double accX, accY, accZ;
    double barometer;

};

#endif //ROBOT_CONTROLLER_ROS_IMUDATA_H

//
// Created by user on 22.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_LEGSDATA_H
#define ROBOT_CONTROLLER_ROS_LEGSDATA_H

/**
 * Raw measured value from the
 * feet pressure sensors
 */
struct SensorFeetState
{
    /**
     * Raw measured value from pressure sensor
     * in one foot
     */
    struct FootData
    {
        /**
         * Moments
         */
        double fx, fy, fz;

        /**
         * Raw values from measurement units
         */
        double uch0, uch1, uch2, uch3;
    };

    FootData left, right;
};

#endif //ROBOT_CONTROLLER_ROS_LEGSDATA_H

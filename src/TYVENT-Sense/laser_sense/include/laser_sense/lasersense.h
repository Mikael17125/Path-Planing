#ifndef LASER_SENSE_H
#define LASER_SENSE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <vector.h>

class Laser
{
public:
    geometry_msgs::Vector3 divideRegion();
    double foundMinimum();
};

Laser process;
sensor_msgs::LaserScan raw_data;
geometry_msgs::Vector3 msg;

void laserCallback(const sensor_msgs::LaserScan &msg);

#endif
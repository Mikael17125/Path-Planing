#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

class ObstacleAvoidance
{
public:
    enum State
    {
        FORWARD = 0,
        LEFT = 1,
        RIGHT = 2,
    };

    enum WallState{
        FIND_WALL = 0,
        TURN = 1,
        FOLLOW_WALL = 2,
        NORMALIZATION = 3,
    };

    ObstacleAvoidance();
    void setDecision(geometry_msgs::Vector3 const &dist);
    void setWallState(geometry_msgs::Vector3 const &dist);
    void wallFollower();
};

ObstacleAvoidance obAvd;

int roboState;
int wallState;
double lin_x, ang_z;
double th_x,th_y,th_z;

geometry_msgs::Vector3 laser_dist;
geometry_msgs::Twist msg; 

void laserCallback(geometry_msgs::Vector3 const &msg);
void wallFollower(int state);


#endif
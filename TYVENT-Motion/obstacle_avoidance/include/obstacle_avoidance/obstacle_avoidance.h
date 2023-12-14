#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class ObstacleAvoidance
{
public:
    enum MotionState
    {
        ROTATE = 0,
        FORWARD = 1,
    };

    enum WallState
    {
        FIND_WALL = 0,
        TURN = 1,
        FOLLOW_WALL = 2,
        NORMALIZATION = 3,
    };

    ObstacleAvoidance();
    void setDecision(geometry_msgs::Vector3 const &dist);
    void setWallState(geometry_msgs::Vector3 const &dist);
    void gotoDesiredPos(double x, double y);
    void wallFollower();
    bool isObstacle();
    bool isArrived(int x, int y);
    void stop();
};

enum MainState
{
    GO_DESIRED = 0,
    WALL_FOLLOWER = 1,
    STOP = 2,
};

ObstacleAvoidance obAvd;

int ANGLE_TOLERANCE;

int motionState;
int wallState;
int mainState;
double lin_x, ang_z;
double th_x, th_y, th_z;
double pos_x, pos_y;
double ori_yaw;
double res, angle;

geometry_msgs::Twist msg;

void laserCallback(geometry_msgs::Vector3 const &msg);
void odomCallback(nav_msgs::Odometry const &msg);

#endif
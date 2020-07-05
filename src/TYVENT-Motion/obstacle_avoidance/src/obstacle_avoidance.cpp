#include <obstacle_avoidance/obstacle_avoidance.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh_;

    ros::Rate rate(60);

    ros::Subscriber laser_sub = nh_.subscribe("/sensing/laser", 1000, laserCallback);
    ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    while (ros::ok())
    {
        // obAvd.setDecision(laser_dist);
        obAvd.setWallState(laser_dist);
        obAvd.wallFollower();

        cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void laserCallback(geometry_msgs::Vector3 const &msg)
{
    laser_dist = msg;
}

ObstacleAvoidance::ObstacleAvoidance()
{
    roboState = FORWARD;
    wallState = FIND_WALL;
    th_x = 1.5;
    th_y = 1.5;
    th_z = 1.5;
}

void ObstacleAvoidance::setDecision(geometry_msgs::Vector3 const &dist)
{
    switch (roboState)
    {
    case FORWARD:
        lin_x = 0.6;
        ang_z = 0;

        if (dist.x < th_x || (dist.y < th_y && dist.z > th_z))
            roboState = LEFT;
        else if (dist.z < th_z && dist.x > th_x)
            roboState = RIGHT;
        break;

    case LEFT:
        lin_x = 0;
        ang_z = -0.3;

        if (dist.z > th_z && dist.y > th_y && dist.z > th_x)
            roboState = FORWARD;
        else if (dist.z < th_z && dist.x > th_x)
            roboState = RIGHT;

        break;

    case RIGHT:
        lin_x = 0;
        ang_z = 0.3;

        if (dist.x > th_x && dist.y > th_y && dist.z > th_z)
            roboState = FORWARD;
        else if (dist.x < th_x || (dist.y < th_y && dist.z > th_z))
            roboState = LEFT;
        break;
    }

    msg.linear.x = lin_x;
    msg.angular.z = ang_z;
}

void ObstacleAvoidance::setWallState(geometry_msgs::Vector3 const &dist)
{

    if (dist.z < 0.3  || dist.y < 0.3 || dist.y < 0.3)
        wallState = 3;
    else if (dist.z > th_z && dist.y > th_y && dist.y > th_x) // 0 0 0
        wallState = 0;
    else if (dist.z > th_z && dist.y > th_y && dist.x < th_x) // 0 0 1
        wallState = 2;
    else if (dist.z > th_z && dist.y < th_y && dist.x > th_x) // 0 1 0
        wallState = 1;
    else if (dist.z > th_z && dist.y < th_y && dist.x < th_x) // 0 1 1
        wallState = 1;
    else if (dist.z < th_z && dist.y > th_y && dist.x > th_x) // 1 0 0
        wallState = 0;
    else if (dist.z < th_z && dist.y > th_y && dist.x < th_x) // 1 0 1
        wallState = 0;
    else if (dist.z < th_z && dist.y < th_y && dist.x > th_x) // 1 1 0
        wallState = 1;
    else
        wallState = 1;
}

void ObstacleAvoidance::wallFollower()
{

    ROS_INFO("STATE [%d]", wallState);

    switch (wallState)
    {
    case FIND_WALL:
        lin_x = 0.4;
        ang_z = 0.6;
        break;

    case TURN:
        lin_x = 0.1;
        ang_z = -0.5;
        break;

    case FOLLOW_WALL:
        lin_x = 0.6;
        ang_z = 0;
        break;

    case NORMALIZATION:
        lin_x = -0.6;
        ang_z = 0;
        break;
    }

    msg.linear.x = lin_x;
    msg.angular.z = ang_z;
}
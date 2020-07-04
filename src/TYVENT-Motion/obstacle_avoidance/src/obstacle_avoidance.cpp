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
        obAvd.setDecision(laser_dist);

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
    th_x = 0.8;
    th_y = 0.8;
    th_z = 0.8;
}

void ObstacleAvoidance::setDecision(geometry_msgs::Vector3 const &dist)
{
    ROS_INFO("DIST [%f]", dist.x);

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

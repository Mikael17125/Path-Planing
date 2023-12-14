#include <obstacle_avoidance/obstacle_avoidance.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh_;

    ros::Rate rate(20);

    ros::Subscriber laser_sub = nh_.subscribe("/sensing/laser", 1000, laserCallback);
    ros::Subscriber odom_sub = nh_.subscribe("/odom", 1000, odomCallback);
    ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    while (ros::ok())
    {

        ROS_INFO("RES [%f]", res);

        static int x = 5, y = 10;
        switch (mainState)
        {
        case GO_DESIRED:

            ROS_INFO("GOTODES");
            obAvd.gotoDesiredPos(x, y);

            if (obAvd.isArrived(x, y))
                mainState = STOP;
            else if (obAvd.isObstacle())
                mainState = WALL_FOLLOWER;
            break;

        case WALL_FOLLOWER:
            ROS_INFO("WALL_FOLL");
            obAvd.wallFollower();

            if (obAvd.isArrived(x, y))
                mainState = STOP;
            else if (!obAvd.isObstacle() || (std::fabs(res) > 80 && std::fabs(res) < 90))
                mainState = GO_DESIRED;
            break;
        case STOP:
            obAvd.stop();
            break;
        }

        cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void laserCallback(geometry_msgs::Vector3 const &msg)
{
    obAvd.setWallState(msg);
}

void odomCallback(nav_msgs::Odometry const &msg)
{
    pos_x = msg.pose.pose.position.x;
    pos_y = msg.pose.pose.position.y;

    double siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
    ori_yaw = std::atan2(siny_cosp, cosy_cosp);

    // ROS_INFO("ODOM [%f][%f]", pos_x, pos_y);
}

ObstacleAvoidance::ObstacleAvoidance()
{
    ANGLE_TOLERANCE = 10;
    motionState = FORWARD;
    wallState = FIND_WALL;
    mainState = GO_DESIRED;

    th_x = 1.5;
    th_y = 1.5;
    th_z = 1.5;
}

bool ObstacleAvoidance::isObstacle()
{
    if (wallState != FIND_WALL)
        return true;

    return false;
}

void ObstacleAvoidance::stop()
{
    msg.linear.x = 0;
    msg.angular.z = 0;
}

bool ObstacleAvoidance::isArrived(int des_x, int des_y)
{
    double dist = std::sqrt(std::pow(des_x - pos_x, 2) + std::pow(des_y - pos_y, 2));

    // ROS_INFO("DIST [%f]", dist);

    if (dist < 0.5)
        return true;

    return false;
}

void ObstacleAvoidance::setWallState(geometry_msgs::Vector3 const &dist)
{

    // ROS_INFO("DIST [%f][%f][%f]", dist.z, dist.y, dist.x);

    if (dist.x < 0.1 || dist.y < 0.1 || dist.z < 0.1)
        wallState = 3;
    else if (dist.x > th_x && dist.y > th_y && dist.z > th_z) // 0 0 0
        wallState = 0;
    else if (dist.x > th_x && dist.y > th_y && dist.z < th_z) // 0 0 1
        wallState = 2;
    else if (dist.x > th_x && dist.y < th_y && dist.z > th_z) // 0 1 0
        wallState = 1;
    else if (dist.x > th_x && dist.y < th_y && dist.z < th_z) // 0 1 1
        wallState = 1;
    else if (dist.x < th_x && dist.y > th_y && dist.z > th_z) // 1 0 0
        wallState = 0;
    else if (dist.x < th_x && dist.y > th_y && dist.z < th_z) // 1 0 1
        wallState = 0;
    else if (dist.x < th_x && dist.y < th_y && dist.x > th_z) // 1 1 0
        wallState = 1;
    else
        wallState = 1;
}

void ObstacleAvoidance::wallFollower()
{
    switch (wallState)
    {
    case FIND_WALL:
        // ROS_INFO("FIND WALL");
        lin_x = 0.2;
        ang_z = -0.3;
        break;

    case TURN:
        // ROS_INFO("TURN");
        lin_x = 0;
        ang_z = 0.75;
        break;

    case FOLLOW_WALL:
        // ROS_INFO("FOLLOW WALL");
        lin_x = 1;
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

void ObstacleAvoidance::gotoDesiredPos(double des_x, double des_y)
{
    angle = std::atan2(des_y - pos_y, des_x - pos_x);
    angle = angle * 180 / 3.1415926;

    ori_yaw = ori_yaw * 180 / 3.1415926;

    res = angle - ori_yaw;
    ROS_INFO("ANGLE [%f]", angle);

    switch (motionState)
    {
    case ROTATE:

        if (res > 0)
        {
            lin_x = 0.2;
            ang_z = -0.3;
        }
        else
        {
            lin_x = 0.2;
            ang_z = 0.3;
        }

        if (std::fabs(res) < ANGLE_TOLERANCE)
            motionState = FORWARD;
        break;

    case FORWARD:
        lin_x = 0.8;
        ang_z = 0;

        if (std::fabs(res) > ANGLE_TOLERANCE)
            motionState = ROTATE;

        break;
    }

    msg.linear.x = lin_x;
    msg.angular.z = ang_z;
}

// void ObstacleAvoidance::setDecision(geometry_msgs::Vector3 const &dist)
// {
//     switch (roboState)
//     {
//     case FORWARD:
//         lin_x = 0.6;
//         ang_z = 0;

//         if (dist.x < th_x || (dist.y < th_y && dist.z > th_z))
//             roboState = LEFT;
//         else if (dist.z < th_z && dist.x > th_x)
//             roboState = RIGHT;
//         break;

//     case LEFT:
//         lin_x = 0;
//         ang_z = -0.3;

//         if (dist.z > th_z && dist.y > th_y && dist.z > th_x)
//             roboState = FORWARD;
//         else if (dist.z < th_z && dist.x > th_x)
//             roboState = RIGHT;

//         break;

//     case RIGHT:
//         lin_x = 0;
//         ang_z = 0.3;

//         if (dist.x > th_x && dist.y > th_y && dist.z > th_z)
//             roboState = FORWARD;
//         else if (dist.x < th_x || (dist.y < th_y && dist.z > th_z))
//             roboState = LEFT;
//         break;
//     }

//     msg.linear.x = lin_x;
//     msg.angular.z = ang_z;
// }
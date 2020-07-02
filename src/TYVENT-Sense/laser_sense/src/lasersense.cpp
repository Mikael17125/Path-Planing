#include <lasersense.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_sense");

    ros::NodeHandle nh_;

    ros::Publisher laser_pub = nh_.advertise<geometry_msgs::Vector3>("/sensing/laser", 1000);
    ros::Subscriber laser_sub = nh.subscribe("/laser", 1000, laserCallback);

    while (ros::ok())
    {

        msg = process.divideRegion(raw_data);

        laser_pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}

geometry_msgs::Vector3 Laser::divideRegion(sensor_msgs::LaserScan data)
{
    geometry_msgs::Vector3 result;

    double fright[144];
    double center[144];
    double fleft[144];

    for (int i = 0; i < 144; i++)
    {
        fright[i] = data[i + 144];
        center[i] = data[i + 288];
        fleft[i] = data[i + 432];
    }

    result.x = foundMinimum(fright);
    result.y = foundMinimum(center);
    result.z = foundMinimum(fleft);

    return result;
}

double foundMinimum(const double &data)
{
    int tmp = 10;
    for (double x : data)
    {
        if(x<tmp)
            tmp = x;
    }

    return tmp;
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    raw_data = msg;
}
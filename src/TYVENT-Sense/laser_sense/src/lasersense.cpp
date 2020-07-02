#include <laser_sense/lasersense.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_sense");

    ros::NodeHandle nh_;
    ros::Rate rate(20);

    ros::Publisher laser_pub = nh_.advertise<geometry_msgs::Vector3>("/sensing/laser", 1000);
    ros::Subscriber laser_sub = nh_.subscribe("/m2wr/laser/scan", 1000, laserCallback);

    while (ros::ok())
    {
        if (!raw_data.ranges.empty())
            msg = process.divideRegion(raw_data);

        laser_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

geometry_msgs::Vector3 Laser::divideRegion(sensor_msgs::LaserScan data)
{
    geometry_msgs::Vector3 result;

    std::vector<double> fright;
    std::vector<double> center;
    std::vector<double> fleft;

    for (int i = 0; i < 144; i++)
    {
        fright.insert(fright.begin() + i, data.ranges[i + 144]);
        center.insert(center.begin() + i, data.ranges[i + 288]);
        fleft.insert(fleft.begin() + i, data.ranges[i + 432]);
    }

    result.x = foundMinimum(fright);
    result.y = foundMinimum(center);
    result.z = foundMinimum(fleft);

    return result;
}

double Laser::foundMinimum(const std::vector<double> &data)
{
    double tmp = 10;
    for (double x : data)
    {
        if (x < tmp)
            tmp = x;
    }

    return tmp;
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    raw_data = msg;
    // ROS_INFO("RANGES [%f]", raw_data[0]);
}
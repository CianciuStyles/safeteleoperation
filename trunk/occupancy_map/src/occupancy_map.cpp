#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserCallback(const sensor_msgs::LaserScan& msg)
{

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "occupancy_map_generator");
	ros::NodeHandle n;

	ros::Subscriber laser = n.subscribe("base_scan", 1000, laserCallback);

	ros::spin();

	return 0;
}

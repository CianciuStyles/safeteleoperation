#include <ros/ros.h>
#include <distance_map/DistanceMap.h>
#include <gradient_map/GradientMap.h>

static ros::Publisher gradient;

void distanceCallback(const distance_map::DistanceMap& msg) {
	std::vector<double> matrix(msg.size_x * msg.size_y);
	gradient_map::GradientMap grad;
	grad.map = matrix;
	grad.size_x = msg.size_x;
	grad.size_y = msg.size_y;
	double center = msg.map[msg.size_x*(msg.size_y/2 - 1) + msg.size_x/2 - 1];
	center += msg.map[msg.size_x*(msg.size_y/2 - 1) + msg.size_x/2];
	center += msg.map[msg.size_x*(msg.size_y/2) + msg.size_x/2 - 1];
	center += msg.map[msg.size_x*(msg.size_y/2) + msg.size_x/2];
	center /= 4;
	
	for (int i = 0; i < msg.size_y*msg.size_x; i++) {
		grad.map[i] = msg.map[i] - center;
	}
	
	gradient.publish(grad);
	ros::Rate loop_rate(20);
	loop_rate.sleep();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "gradient_map_generator");
	ros::NodeHandle n;

	ros::Subscriber occupancy = n.subscribe("distance_map", 1, distanceCallback);
	gradient = n.advertise<gradient_map::GradientMap>("gradient_map", 1);

	ros::spin();

	return 0;
}

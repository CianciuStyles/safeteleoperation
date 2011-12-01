#include <ros/ros.h>
#include <distance_map/DistanceMap.h>
#include <gradient_map/GradientMap.h>

static ros::Publisher gradient;

#ifndef SQUARE_2
#define SQUARE_2 1.414213562     
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool robot(int x, int y, int size_x, int size_y)
{
	if (
		
		(x == (size_y/2 -2) && y == (size_x/2 -2)) ||
		(x == (size_y/2 -2) && y == (size_x/2 -1)) ||
		(x == (size_y/2 -2) && y == (size_x/2)) ||
		(x == (size_y/2 -2) && y == (size_x/2 + 1)) ||
		
		(x == (size_y/2 -1) && y == (size_x/2 -2)) ||
		(x == (size_y/2 -1) && y == (size_x/2 -1)) ||
		(x == (size_y/2 -1) && y == (size_x/2)) ||
		(x == (size_y/2 -1) && y == (size_x/2 + 1)) ||
		
		(x == (size_y/2) && y == (size_x/2 -2)) ||
		(x == (size_y/2) && y == (size_x/2 -1)) ||
		(x == (size_y/2) && y == (size_x/2)) ||
		(x == (size_y/2) && y == (size_x/2 + 1)) ||
		
		(x == (size_y/2 +1) && y == (size_x/2 -2)) ||
		(x == (size_y/2 +1) && y == (size_x/2 -1)) ||
		(x == (size_y/2 +1) && y == (size_x/2)) ||
		(x == (size_y/2 +1) && y == (size_x/2 + 1)) 
		)
		return true;
	
	return false;
}

void distanceCallback(const distance_map::DistanceMap& msg)
{
	return;
	
	gradient_map::GradientMap grad;
	std::vector<double> matrix_scalar(msg.size_x * msg.size_y);
	std::vector<double> matrix_theta(msg.size_x * msg.size_y);
	grad.map_scalar = matrix_scalar;
	grad.map_theta = matrix_theta;
	grad.size_x = msg.size_x;
	grad.size_y = msg.size_y;
	
	for (int x = 0; x < msg.size_y; x++) {
		for (int y = 0; y < msg.size_x; y++) {
			
			double center = msg.map[x*msg.size_x + y];
			if (!(center == 1 || (center > 1.40 && center < 1.42)))
				continue;
			
			double scalar = 1000;
			double theta = -1;
			
			for (int i = x-1; i < x+2; x++) {
				for (int k = y-1; k < y+2; k++) {
					
					/* Invalid indexes */
					if (i > 0 && i < msg.size_y && k > 0 && k < msg.size_y)
						continue;
					/* the center... */
					if (i == x && k == y)
						continue;
					
					double near = msg.map[i*msg.size_x + k];
					double delta = 1;
					if (x != i && y != k) delta = SQUARE_2;
					
					double grad = (near - center) / delta;
					if (grad < scalar) {
						
						scalar = grad;
						if (i < x) {
							if (k < y) theta = 135;
							else if (k == y) theta = 90;
							else theta = 45;
						} else if (i > x) {
							if (k < y) theta = 225;
							else if (k == y) theta = 270;
							else theta = 315;
						} else {
							if (k < y) theta = 180;
							else theta = 0;
						}
						
					}
					
				}
				
				if (theta == -1) 
					ROS_INFO("Invalid gradient");
					
				grad.map_scalar[x*msg.size_x + y] = scalar;
				
				/* convert in radiant */
				//theta = theta * (M_PI / 180);
				grad.map_theta[x*msg.size_x + y] = theta;
				
			}

		}
	}
	
	gradient.publish(grad);
	ros::Rate loop_rate(20);
	loop_rate.sleep();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "gradient_map_generator");
	ros::NodeHandle n;

	ros::Subscriber occupancy = n.subscribe("/distance_map", 1, distanceCallback);
	gradient = n.advertise<gradient_map::GradientMap>("gradient_map", 1);

	ros::spin();

	return 0;
}

#include <ros/ros.h>
#include <distance_map/DistanceMap.h>
#include <gradient_map/GradientMap.h>
#include <math.h>
#include <stdlib.h>

static ros::Publisher gradient;

#ifndef SQUARE_2
#define SQUARE_2 1.414213562     
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
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
*/

void distanceCallback(const distance_map::DistanceMap& msg)
{
	//ROS_INFO("GRADIENT");
	//double map[9] = { 1, 1.41, 2.23, 0, 1, 2, 1, 1.41, 2.23 };
	//double map[9] = { 1, 1.41, 1, 0, 1, 0, 1, 1.41, 1 };
	//double map[9] = { 0, 1, 2, 1, 1.41, 2.23, 2, 2.23, 2.82 };
	gradient_map::GradientMap grad;
	grad.size_x = msg.size_x;
	grad.size_y = msg.size_y;
	std::vector<double> matrix_scalar(grad.size_x * grad.size_y);
	std::vector<double> matrix_theta(grad.size_x * grad.size_y);
	grad.map_scalar = matrix_scalar;
	grad.map_theta = matrix_theta;
	
	
	std::vector<double> map(0);
	for (int x = grad.size_y - 1; x >=0; x--) {
		for (int y = 0; y < grad.size_x; y++) {
			map.push_back(msg.map[x*grad.size_x + y]);
			//ROS_INFO("%f", msg.map[x*grad.size_x + y]);
		}
	}
	
	//std::vector<double> map = msg.map;
	
	int k = 0;
	
	for (int x = 0; x < grad.size_y; x++) {
		for (int y = 0; y < grad.size_x; y++) {
			
			double center = map[x*grad.size_x + y];
			if (center == 0) {
				grad.map_scalar[x*grad.size_x + y] = 0;
				grad.map_theta[x*grad.size_x + y] = -1;
				continue;
			}
			/*
			else if (center > 1) {
				grad.map_scalar[x*grad.size_x + y] = 100;
				grad.map_theta[x*grad.size_x + y] = 0;
				continue;
			}*/
			//if (x > 2) break;
			//ROS_INFO("(%d, %d): %f", x, y, center);
			
			//if (!(center == 1 && x > 0 && y > 0)) continue;
			
			double scalar = 1000;
			double theta = -1;
			double comp_x = 0;
			double comp_y = 0;
			
			for (int i = x-1; i <= x+1; i++) {
				for (int k = y-1; k <= y+1; k++) {
					
					/* Invalid indexes */
					if (i < 0 || i >= grad.size_y || k < 0 || k >= grad.size_y)
						continue;
					if (i == x && y == k) continue;
					
					double near = map[i*grad.size_x + k];
					double delta = 1;
					if (x != i && y != k) delta = SQUARE_2;
					
					//ROS_INFO("i:%d - j:%d - near:%f - delta:%f", i, k, near, delta);
					
					double grad = (near - center) / delta;
					double angle = 0;
					if (i < x) {
						if (k < y) angle = 135;
						else if (k == y) angle = 90;
						else angle = 45;
					} else if (i > x) {
						if (k < y) angle = 225;
						else if (k == y) angle = 270;
						else angle = 315;
					} else {
						if (k < y) angle = 180;
						else angle = 0;
					}
					double a, b;
					a = grad * cos(angle * (M_PI/180));
					b = grad * sin(angle * (M_PI/180));
					
					comp_x += a;
					comp_y += b;
					
					//ROS_INFO(" %f, %f ) : ( %f , %f )", 
					//		grad, angle, a, b);
					
				}
				
			}
			
			//ROS_INFO("COMP ( %f , %f )", comp_x, comp_y);
			scalar = sqrt((comp_x * comp_x) + (comp_y * comp_y));
			if (comp_x < 0)
				theta = acos((-comp_x) / scalar); // radiant
			else
				theta = acos(comp_x / scalar); // radiant
			theta = theta * (180 / M_PI); // degree
			
			//ROS_INFO("ORIGINAL ( %f , %f )", scalar, theta);
			
			if (comp_x < -0.01 && comp_y < -0.01) theta += 180;
			else if (comp_x < -0.01 && theta >= 90) theta += 180;
			else if (comp_x < -0.01 && theta < 90) theta = 180 - theta;
			else if (comp_y < -0.01 && theta >= 90) theta += 180;
			else if (comp_y < -0.01 && theta < 90) theta = 360 - theta; 
			
			//ROS_INFO("FINAL ( %f , %f )", scalar, theta);
			
			if (scalar < 1) scalar = 1000;
			else scalar = center;
			
			/* convert to radiant */
			grad.map_scalar[x*grad.size_x + y] = scalar;
			grad.map_theta[x*grad.size_x + y] = theta;
			
			//if (k++ > 20)
			//	exit(0);
			//return;
			
		}
	}
	
	gradient.publish(grad);
	//exit(0);
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

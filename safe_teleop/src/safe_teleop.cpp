#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "distance_map/DistanceMap.h"
#include <math.h>

using namespace std;
using namespace ros;

Subscriber distance_sub, vel_sub;
Publisher vel_pub;
double near_cells[20], decremento[4];
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;
		twist.linear.x = msg->linear.x - decremento[0];
		twist.linear.y = msg->linear.y;
		twist.linear.z = msg->linear.z;
		twist.angular.x = msg->angular.x; 
		twist.angular.y = msg->angular.y; 
		twist.angular.z = msg->angular.z;
		
		vel_pub.publish(twist);
}

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {
	int size_x = msg->size_x, size_y = msg->size_y, n = msg->size_x*msg->size_y;
	std::vector<double> map = msg->map;
	near_cells[0] = map[(size_y/2 - 3)*size_x + (size_y/2 - 3)];
	near_cells[1] = map[(size_y/2 - 3)*size_x + (size_y/2 - 2)];
	near_cells[2] = map[(size_y/2 - 3)*size_x + (size_y/2 - 1)];
	near_cells[3] = map[(size_y/2 - 3)*size_x + (size_y/2)];
	near_cells[4] = map[(size_y/2 - 3)*size_x + (size_y/2 + 1)];
	near_cells[5] = map[(size_y/2 - 3)*size_x + (size_y/2 + 2)];
	
	near_cells[6] = map[(size_y/2 - 2)*size_x + (size_y/2 + 2)];
	near_cells[7] = map[(size_y/2 - 1)*size_x + (size_y/2 + 2)];
	near_cells[8] = map[(size_y/2)*size_x + (size_y/2 + 2)];
	near_cells[9] = map[(size_y/2 + 1)*size_x + (size_y/2 + 2)];
	near_cells[10] = map[(size_y/2 + 2)*size_x + (size_y/2 + 2)];
	
	near_cells[11] = map[(size_y/2 + 2)*size_x + (size_y/2 + 1)];
	near_cells[12] = map[(size_y/2 + 2)*size_x + (size_y/2)];
	near_cells[13] = map[(size_y/2 + 2)*size_x + (size_y/2 - 1)];
	near_cells[14] = map[(size_y/2 + 2)*size_x + (size_y/2 - 2)];
	near_cells[15] = map[(size_y/2 + 2)*size_x + (size_y/2 - 3)];
	
	near_cells[16] = map[(size_y/2 + 1)*size_x + (size_y/2 - 3)];
	near_cells[17] = map[(size_y/2)*size_x + (size_y/2 - 3)];
	near_cells[18] = map[(size_y/2 - 1)*size_x + (size_y/2 - 3)];
	near_cells[19] = map[(size_y/2 - 2)*size_x + (size_y/2 - 3)];
	
	decremento[0] = (near_cells[3] >= 6) ? 0 : exp(near_cells[3]-1)-1; 
	decremento[1] = (near_cells[8] >= 6) ? 0 : exp(near_cells[8]-1)-1; 
	decremento[2] = (near_cells[13] >= 6) ? 0 : exp(near_cells[13]-1)-1; 
	decremento[3] = (near_cells[18] >= 6) ? 0 : exp(near_cells[18]-1)-1; 
}

int main(int argc, char **argv) {
	init(argc, argv, "safe_teleop");
	NodeHandle n;
	distance_sub = n.subscribe("distance_map", 1, distanceCallback);
	vel_sub = n.subscribe("vel", 1, velCallback); 
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    spin(); 

	return 0;
}

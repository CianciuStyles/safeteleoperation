#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "distance_map/DistanceMap.h"
#include <math.h>

using namespace std;
using namespace ros;

Subscriber distance_sub, vel_sub;
Publisher vel_pub;

//Cells just around the robot
double near_cells[20];
	
//Array of new velocities	
double new_vel[4];


void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;
		
		if(new_vel[0] < 0)
				if(msg->linear.x < 0) twist.linear.x = 0;
				else twist.linear.x = msg->linear.x;
		else
				twist.linear.x = new_vel[0];

    
		twist.linear.y = msg->linear.y;
		twist.linear.z = msg->linear.z;
		twist.angular.x = msg->angular.x; 
		twist.angular.y = msg->angular.y; 
		twist.angular.z = msg->angular.z;
		
		vel_pub.publish(twist);
}

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {
	
	int size_x = msg->size_x;
	int size_y = msg->size_y;
	
	//Vector of cells of the distance map just received.
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
	
	double min_back;
	double min = 100;
	int index = 0;
	for (int i = 1; i < 5; i ++) {
		if (near_cells[i] < min) {
			min = near_cells[i];
			index = i;
			min_back = map[(size_y/2 - 2)*size_x + (size_y/2 - 3) + i];
		}
	}

	//printf("min is %f at %d. min_back is %f. \n", min,index,min_back);
  //&& min - min_back > 0
		
	for (int i = 0; i < size_y; i++) {
		printf("\n");
		for (int j = 0; j < size_x; j++) {
			printf ("%.2f ", msg->map[i*size_y+j]);
		}
	}
	printf("--------------------------------------------\n");
	printf("--------------------------------------------\n");
	printf("--------------------------------------------\n");
	//printf("min is %f\n", min);
	//printf("min_back is %f\n", min_back);

	//printf("%f - %f\n", min, min_back);


		if (min <= 8){
			if (min_back > min) new_vel[0] = 0.5; 			
			else new_vel[0] = 0;
		}		
		
		else if (min < 12) new_vel[0] = 1; 

		else new_vel[0] = -1;

	//printf(" new_vel is %f\n", new_vel[0]);
		 

	//new_vel[1] = (near_cells[8] >= 6) ? 0 : 1/20*exp(near_cells[8]-1)-1; 
	//new_vel[2] = (near_cells[13] >= 6) ? 0 : 1/20*exp(near_cells[13]-1)-1; 
	//new_vel[3] = (near_cells[18] >= 6) ? 0 : 1/20*exp(near_cells[18]-1)-1; 
}

int main(int argc, char **argv) {
	
	init(argc, argv, "safe_teleop");
	
	NodeHandle n;
	
	//Distance Map received from ""distance_map"" node.
	distance_sub = n.subscribe("distance_map", 1, distanceCallback);
	//Commands of velocity received from "teleop_joy" node.
	vel_sub = n.subscribe("vel", 1, velCallback); 
	//Twist message about linear and angular velocity published.
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    spin(); 

	return 0;
}

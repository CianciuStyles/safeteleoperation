#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "distance_map/DistanceMap.h"
#include <math.h>

#define MAX_ANG 1.8

using namespace std;
using namespace ros;

Subscriber distance_sub, vel_sub;
Publisher vel_pub, vel2_pub;

bool enabled = false;

double convert(int x, int y, std::vector<double> map, int size_y) {
	return map[size_y*(size_y - 1 - x) + y];
}
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;
		
		
		vel2_pub.publish(msg);
}

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {
	int size_x = msg->size_x, size_y = msg->size_y;
	vector<double> map = msg->map;
	//printf("conv: %f\n", convert(0, 0, map, size_y)); 

	int manhattan[size_x][size_y];
	manhattan[5][25] = 0;
	for (int i = 24; i >= 0; i--)
		manhattan[5][i] = manhattan[5][i+1] + 1;
		
	for (int i = 26; i < size_y; i++)
		manhattan[5][i] = manhattan[5][i-1] + 1;
		
	for (int j = 4; j >= 0; j--)
		for (int i = 0; i < size_y; i++)
			manhattan[j][i] = manhattan[j+1][i] + 1;
			
	for (int j = 6; j < size_x; j++)
		for (int i = 0; i < size_y; i++)
			manhattan[j][i] = manhattan[j-1][i] + 1;
			
	/*for (int i = 0; i < size_x; i++) {
		printf("\n");
		for (int j = 0; j < size_y; j++)
			printf("%2d ", manhattan[i][j]);
	}*/
}

int main(int argc, char **argv) {
	init(argc, argv, "safe_teleop");
	NodeHandle n;
	distance_sub = n.subscribe("distance_map", 1, distanceCallback);
	vel_sub = n.subscribe("vel_gate", 1, velCallback); 
	vel_pub = n.advertise<geometry_msgs::Twist>("vel", 1);
	vel2_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    spin(); 

        return 0;
}

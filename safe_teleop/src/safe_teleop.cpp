#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gradient_map/GradientMap.h"
#include <math.h>

using namespace std;
using namespace ros;

Subscriber gradient_sub, vel_sub;
Publisher vel_pub;

double new_velocity;
double near_cells_scalar[20], near_cells_theta[20];

/*
void vectorialSum(double firstVector_m, double firstVector_a, double secondVector_m, double secondVector_a, double* resultantVector_m, double* resultantVector_a) {
	double firstVector_x = firstVector_m * cos(firstVector_a);
	double firstVector_y = firstVector_m * sin(firstVector_a);
	double secondVector_x = secondVector_m * cos(secondVector_a);
	double secondVector_y = secondVector_m * sin(secondVector_a);
	
	double resultantVector_x = firstVector_x + secondVector_x;
	double resultantVector_y = firstVector_y + secondVector_y;
	
	*resultantVector_m = sqrt(pow(resultantVector_x, 2.0) + pow(resultantVector_y, 2.0));
	*resultantVector_a = (double)((int)(firstVector_a + ((secondVector_a - firstVector_a) / 2)) % 360);
	
	if(resultantVector_m < 0){
		*resultantVector_m = -*resultantVector_m;
		*resultantVector_a = (double)((int)(*resultantVector_a + 180) % 360);
	}
}
*/
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;

		if(msg->linear.x == 0)
			twist.linear.x = msg->linear.x;
		else {
			twist.linear.x = new_velocity;
			ROS_INFO("Actual velocity: %.2f", twist.linear.x);
		}

		twist.linear.y = msg->linear.y;
		twist.linear.z = msg->linear.z;
		twist.angular.x = msg->angular.x; 
		twist.angular.y = msg->angular.y; 
		twist.angular.z = msg->angular.z;
		
		vel_pub.publish(twist);
}

void gradientCallback(const gradient_map::GradientMap::ConstPtr& msg) {
        int size_x = msg->size_x, size_y = msg->size_y;
        std::vector<double> map_scalar = msg->map_scalar;
        std::vector<double> map_theta = msg->map_theta;
        
        near_cells_scalar[0] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 3)]; near_cells_theta[0] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 3)];
        near_cells_scalar[1] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 2)]; near_cells_theta[1] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 2)];
        near_cells_scalar[2] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 - 1)]; near_cells_theta[2] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 - 1)];
        near_cells_scalar[3] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2)];     near_cells_theta[3] = map_theta[(size_y/2 - 3)*size_x + (size_y/2)];
        near_cells_scalar[4] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 + 1)]; near_cells_theta[4] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 + 1)];
        near_cells_scalar[5] = map_scalar[(size_y/2 - 3)*size_x + (size_y/2 + 2)]; near_cells_theta[5] = map_theta[(size_y/2 - 3)*size_x + (size_y/2 + 2)];
        
        /*
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
        */
        
        //Find the maximum repulsive force
        double max_scalar = near_cells_scalar[0];
        double max_theta = near_cells_theta[0];
        for(int i=1; i<6; i++){
			if(near_cells_scalar[i] > max_scalar){
				max_scalar = near_cells_scalar[i];
				max_theta = near_cells_theta[i];
			}
		}
		
		ROS_INFO("%.2f, %.2f", max_scalar, max_theta);
		
		
		//Calculate the smoothed deceleration
		if(max_scalar == 1000.0)
			new_velocity = 0.0;
		else if(max_scalar < 8){	//further than 8 steps there's no deceleration
			if(max_scalar <= 2.5) new_velocity = 0.0; //if we are close to the obstacle, there's no movement on the X axis
			else if(max_scalar <= 3.5) new_velocity = 0.1;
			else if(max_scalar <= 4.5) new_velocity = 0.4;
			else if(max_scalar <= 5.5) new_velocity = 0.9;
			else if(max_scalar <= 6.5) new_velocity = 1.4;
			else if(max_scalar <= 7.0) new_velocity = 1.8;
		}
		else
			new_velocity = 2.0;
}

int main(int argc, char **argv) {
	init(argc, argv, "safe_teleop");
	NodeHandle n;
	gradient_sub = n.subscribe("gradient_map", 1, gradientCallback);
	vel_sub = n.subscribe("vel", 1, velCallback); 
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    spin(); 

        return 0;
}

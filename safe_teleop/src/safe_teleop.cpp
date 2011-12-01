#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gradient_map/GradientMap.h"
#include <math.h>

using namespace std;
using namespace ros;

Subscriber gradient_sub, vel_sub;
Publisher vel_pub;
//double linear_velocity, angular_velocity;
double total_vector_modulus, total_vector_angle;
double near_cells_scalar[20], near_cells_theta[20];// new_vel[4];

double Degrees2Radians(double angle) {
	return (angle*3.141592653589793238)/180.0;
}

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
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		geometry_msgs::Twist twist;
		double movement_vector_modulus = 0;
		double movement_vector_angle = 0;
			
		if(msg->linear.x != 0){
			vectorialSum(msg->linear.x, msg->angular.z, total_vector_modulus, total_vector_angle, &movement_vector_modulus, &movement_vector_angle);
			
			twist.linear.x = movement_vector_modulus * cos(movement_vector_angle);
			twist.angular.z = (movement_vector_angle - 90)/20;
			                 //movement_vector_modulus * sin(movement_vector_angle);
		}
		else {
			twist.linear.x = msg->linear.x;
			twist.angular.z = msg->angular.z;
		}

		twist.linear.y = msg->linear.y;
		twist.linear.z = msg->linear.z;
		twist.angular.x = msg->angular.x; 
		twist.angular.y = msg->angular.y; 
		
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
	
	/*
	double min_back;
	double min = 100;
	for (int i = 1; i < 5; i ++) {
		if (near_cells[i] < min) {
			min = near_cells[i];
			min_back = map[(size_y/2 - 2)*size_x + (size_y/2 - 3) + i];
		}
	}
	//printf("%f - %f\n", min, min_back);
	*/
	
	//Calculation of the total repulsive force applied to robot
	//double total_vector_modulus = 0.0;
	//double total_vector_angle = 0.0;
	for(int i = 0; i < 6; i++) {
		printf("%f, %f", total_vector_modulus, total_vector_angle);
		vectorialSum(total_vector_modulus, total_vector_angle, near_cells_scalar[i], near_cells_theta[i], &total_vector_modulus, &total_vector_angle);
		printf("%f, %f", total_vector_modulus, total_vector_angle);
	}
	
	
	/*
		if (min >= 16)
			new_vel[0] = -1;
		else if (min <= 8 && min - min_back > 0)
			new_vel[0] = 0;
		else new_vel[0] = 1;//(near_cells[3]); 
	
	new_vel[1] = (near_cells[8] >= 6) ? 0 : 1/20*exp(near_cells[8]-1)-1; 
	new_vel[2] = (near_cells[13] >= 6) ? 0 : 1/20*exp(near_cells[13]-1)-1; 
	new_vel[3] = (near_cells[18] >= 6) ? 0 : 1/20*exp(near_cells[18]-1)-1;
	*/
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

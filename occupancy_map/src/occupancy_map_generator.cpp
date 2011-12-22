#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <occupancy_map/OccupancyMap.h>
#include "tf/transform_listener.h"

#include <stdio.h>
#include <math.h>
#include <unistd.h>

class Wall
{
	public:
		double row;
		double col;
		double tx;
		double ty;
	
		Wall(double xx, double yy)
		{
			row = xx;
			col = yy;
			
			tx = col - 25;
			ty = 25 - row;
		}
};

#define CELL_RESOLUTION 10 /* centimeters */
#define X_SIZE          50
#define Y_SIZE          50

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ros::Publisher obstacle;
tf::TransformListener * listener = 0;
std::list<Wall*> walls;
double start_x, start_y, start_z;

int k = 0;

void laserCallback(const sensor_msgs::LaserScan& msg)
{
	
	double current_x = 0, current_y = 0, current_z = 0;
	tf::StampedTransform laserPose;
	ros::Time t = msg.header.stamp;
	std::string error = "Error :(";
	if (listener->waitForTransform ("/odom", "base_link", t, ros::Duration(0.5), ros::Duration(0.01), &error)){
		
		listener->lookupTransform("/odom", "/base_link", t, laserPose);
		double yaw,pitch,roll;
		btMatrix3x3 mat =  laserPose.getBasis();
		mat.getEulerZYX(yaw, pitch, roll);
		
		current_x = (laserPose.getOrigin().x()) * CELL_RESOLUTION;
		current_y = (laserPose.getOrigin().y()) * CELL_RESOLUTION;
		current_z = yaw;
		
	} else {
		std::cerr << error << std::endl; 
	}
	
	double dx = sqrt( (current_x-start_x)*(current_x-start_x) + (current_y-start_y)*(current_y-start_y) );
	double dy = 0; 
	double dz = current_z - start_z;
	
	//ROS_INFO("Angle: %.1f", current_z);
	
	if (current_z > -1.57 && current_z <= 0) {
		if ((current_x - start_x < 0 && current_y - start_y >= 0))
			dx = -dx;
	} else if (current_z < -1.57 && current_z >= -3.14) {
		if ((current_x - start_x > 0 && current_y - start_y > 0))
			dx = -dx;
	} else if (current_z > 0 && current_z <= 1.57) {
		if ((current_x - start_x < 0 && current_y - start_y < 0))
			dx = -dx;
	} else {
		if ((current_x - start_x > 0 && current_y - start_y < 0))
			dx = -dx;
	}
	
	if (!(dx == 0 && dy == 0 && dz == 0)) {
		
		//ROS_INFO("Di: %.1f %.1f %.1f", dx, dy, dz);
		
		dz = -dz;
		std::list<Wall*> temp;
		std::list<Wall*>::iterator it;
		for(it = walls.begin(); it != walls.end(); it++) {
			
			//ROS_INFO("Wall: %.1f %.1f", (*it)->row, (*it)->col);
			
			double t_x = ((*it)->col-25)*cos(dz) - (25-(*it)->row)*sin(dz);
			double t_y = ((*it)->col-25)*sin(dz) + (25-(*it)->row)*cos(dz);
			
			(*it)->row = (*it)->row + ((*it)->ty - t_y); //row
			(*it)->col = (*it)->col + (t_x - (*it)->tx); //col
			
			(*it)->row = (*it)->row + dx;
			(*it)->col = (*it)->col + dy;
			(*it)->tx = t_x - dy;
			(*it)->ty = t_y - dx;
			
			//ROS_INFO("Wall2: R %.1f  C %.1f TX %.1f TY %.1f DZ %.1f ", (*it)->row, (*it)->col, t_x, t_y, dz);
		
			//if ((*it)->row < -10 || (*it)->row > 59 || (*it)->col < 10 || (*it)->col > 59) {
			if ((*it)->row < 0 || (*it)->row > 49 || (*it)->col < 0 || (*it)->col > 49) {
				temp.push_back(*it);
			}

		}
		
		for(it = temp.begin(); it != temp.end(); it++) {
			walls.remove(*it);
			//ROS_INFO("Remove wall...");
			fflush(stdout);
			delete *it;
		}
		
		//ROS_INFO("Finished traslation");
	}
	
	start_x = current_x;
	start_y = current_y;
	start_z = current_z;
	
	/* Number of scan samples */
	int n = (msg.angle_max - msg.angle_min) / msg.angle_increment;
	
	//double angle = msg.angle_min * (360/2*(M_PI)); // degree
	//double increment = msg.angle_increment * (360/2*(M_PI)); // degree
	double angle = msg.angle_min;
	
	for (int i = 0; i < n; i++) {
		
		/*
		if (k > 0) break;
		
		walls.push_back(new Wall(20, 20));
		walls.push_back(new Wall(20, 25));
		walls.push_back(new Wall(20, 30));
		walls.push_back(new Wall(25, 20));
		walls.push_back(new Wall(25, 30));
		walls.push_back(new Wall(30, 20));
		walls.push_back(new Wall(30, 25));
		walls.push_back(new Wall(30, 30));
		break;
		*/
		double x_rel = -(msg.ranges[i] * sin(angle)) * CELL_RESOLUTION; /* centimetres */
		double y_rel = (msg.ranges[i] * cos(angle)) * CELL_RESOLUTION; /* centimetres */
		double row = 25-y_rel;
		double col = 25+x_rel;
		
		angle += msg.angle_increment;
		if (row < 0 || row > 49 || col < 0 || col > 49) continue;
		//if (row < -20 || row > 69 || col < -20 || col > 69) continue;
		
		//ROS_INFO("Wall at %f %f -- %f %f -- %f %f", x_rel, y_rel, row, col, angle, msg.ranges[i]);
		
		bool skip = false;
		std::list<Wall*>::iterator it;
		for(it = walls.begin(); it != walls.end(); it++) {
			double dr = row - (*it)->row;
			double dc = col - (*it)->col;
			if (dr < 0.8 && dr > -0.8 && dc < 0.8 && dc > -0.8) {
				/*
				(*it)->row = row;
				(*it)->col = col;
				(*it)->tx = col - 25;
				(*it)->ty = 25 - row;
				*/
				skip = true;
				break;
			}
		}
		if (skip) continue;
		
		//ROS_INFO("Add wall");
		
		Wall* new_wall = new Wall(row, col);
		//TODO: verificare se cella già esiste
		walls.push_back(new_wall);
		
	}
	
	k++;
	
	std::vector<uint8_t> matrix(X_SIZE*Y_SIZE);
	std::list<Wall*>::iterator it;
	for(it = walls.begin(); it != walls.end(); it++) {
		int row = round((*it)->row), col = round((*it)->col);
		//ROS_INFO("Wall2: %d %d", row, col);
		if (row < 0 || row > 49 || col < 0 || col > 49) {
			//ROS_INFO("Invalid index");
			//fflush(stdout);
			//exit(1);
			continue;
		}
		matrix[(Y_SIZE -1 - row) * X_SIZE + col] = true;
	}
	
	occupancy_map::OccupancyMap msg_o;
	msg_o.size_x = X_SIZE;
	msg_o.size_y = Y_SIZE;
	msg_o.map = matrix;
	obstacle.publish(msg_o);
	
	ros::Rate loop_rate(20);
	loop_rate.sleep();

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "occupancy_map_generator");
	ros::NodeHandle n;

	ros::Subscriber laser = n.subscribe("base_scan", 1, laserCallback);
	obstacle = n.advertise<occupancy_map::OccupancyMap>("occupancy_map", 1);
	listener = new tf::TransformListener;

	ros::spin();

	return 0;
}

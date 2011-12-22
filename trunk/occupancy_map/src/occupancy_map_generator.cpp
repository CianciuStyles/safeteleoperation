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
			tx = 25 - row;
			ty = 25 - col; 
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
std::vector<Wall*> walls;
double start_x, start_y, start_z;

#if 0

/* cell (0,0) is top-left corner of the matrix */
void set_abs_cell(int x, int y, bool obstacle)
{
	if (x < 0 || x >= X_SIZE) return;
	if (y < 0 || y >= Y_SIZE) return;
	map[y*X_SIZE+x] = obstacle;
}

/* cell (0,0) is the center of the matrix */
void set_rel_cell(int x, int y, bool obstacle)
{
	set_abs_cell(x+(X_SIZE/2), y+(Y_SIZE/2), obstacle);
}

#endif

void laserCallback(const sensor_msgs::LaserScan& msg)
{
	
	double current_x, current_y, current_z;
	tf::StampedTransform laserPose;
	std::string error = "Error :(";
	if (listener->waitForTransform ("/odom", "base_link", ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error)){
		
		listener->lookupTransform("/odom", "/base_link", ros::Time(0), laserPose);
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
	
	if (false && dx == 0 && dy == 0 && dz == 0) {
	
		dz = -dz;
		for (int i = 0; i < walls.size(); i++) {
			
			double t_x = (walls[i]->col-25)*cos(dz) - (25-walls[i]->row)*sin(dz);
			double t_y = (walls[i]->col-25)*sin(dz) + (25-walls[i]->row)*cos(dz);
			
			walls[i]->row = walls[i]->row + (walls[i]->ty - t_y); //row
			walls[i]->col = walls[i]->col + (t_x - walls[i]->tx); //col
			
			walls[i]->row = walls[i]->row + dx;
			walls[i]->col = walls[i]->col + dy;
			walls[i]->tx = t_x - dy;
			walls[i]->ty = t_y - dx;
			
		}
		
		start_x = current_x;
		start_y = current_y;
		start_z = current_z;
		
	}
	
	/* Number of scan samples */
	int n = (msg.angle_max - msg.angle_min) / msg.angle_increment;
	
	//double angle = msg.angle_min * (360/2*(M_PI)); // degree
	//double increment = msg.angle_increment * (360/2*(M_PI)); // degree
	double angle = msg.angle_min;
	
	for (int i = 0; i < n; i++) {
		
		double x_rel = -(msg.ranges[i] * sin(angle)) * CELL_RESOLUTION; /* centimetres */
		double y_rel = (msg.ranges[i] * cos(angle)) * CELL_RESOLUTION; /* centimetres */
		double row = 25-y_rel;
		double col = 25+x_rel;
		
		angle += msg.angle_increment;
		if (row < 0 || row > 49 || col < 0 || col > 49) continue;
		
		//ROS_INFO("Wall at %f %f -- %f %f -- %f %f", x_rel, y_rel, row, col, angle, msg.ranges[i]);
		
		bool skip = false;
		for(int i = 0; i < walls.size(); i++) {
			double dr = row - walls[i]->row;
			double dc = col - walls[i]->col;
			if (dr < 0.1 && dr > -0.1 && dc < 0.1 && dc > -0.1) {
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
	
	std::vector<uint8_t> matrix(X_SIZE*Y_SIZE);
	for(int i = 0; i < walls.size(); i++) {
		int row = round(walls[i]->row), col = round(walls[i]->col);
		//ROS_INFO("Wall2: %d %d", row, col);
		if (row < 0 || row > 49 || col < 0 || col > 49) {
			ROS_INFO("Invalid index");
			//fflush(stdout);
			exit(1);
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

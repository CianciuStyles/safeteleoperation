#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "distance_map/DistanceMap.h"
#include "occupancy_map/OccupancyMap.h"
#include "safe_teleop/TrajectoryMap.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <unistd.h>
#include <queue>

#define MAX_ANG 1.8

using namespace std;
using namespace ros;

int size_x = 50;
int size_y = 50;
/* Odom value when pressed button */
double start_x, start_y, start_z;

/* Current odom value */
double current_x, current_y, current_z;

/* Arrival point (approximation) */
int target_x, target_y;

Subscriber distance_sub, occupancy_sub, vel_sub, button_sub, odom_sub;
Publisher vel_pub, vel2_pub, traj_pub;
tf::TransformListener * listener = 0;

bool enabled = false;

int occupancy[50][50];

class Cell
{
	private:
		
		int row;
		int col;
		double f; /* Total */ 
		double g; /* Already paid */
		double h; /* Manhattan && distance value */
		Cell * parent;
	
	public:
	
		Cell(int r, int c, double gg = 0, double hh = 0) : g(gg), h(hh)
		{
			row = r;
			col = c;
			parent = NULL;
			f = g + h;
			
			return;
		}
		
		double get_f() {return f;}
		double get_g() {return g;}
		double get_h() {return h;}
		
		void set_parent(Cell * p)
		{
			if (p == NULL) printf("Passed an invalid parent\n");
			parent = p;
		}
		
		Cell * get_parent() 
		{
			return parent;
		}
		
		void update_g(double g1)
		{
			g = g1;
			f = h + g1;
		}
		
		int get_row() 
		{
			return row;
		}
		
		int get_col()
		{
			return col;
		}
		
		bool get_neighbour(int index, int* x, int* y, double* dist) {
			switch(index) {
				case 0:
					if(row > 0 && col > 0) {
						*x = row - 1;
						*y = col - 1;
						*dist = 1.41;
						return true;
					} else return false;
				
				case 1:
					if(row > 0) {
						*x = row - 1;
						*y = col;
						*dist = 1;
						return true;
					} else return false;
				
				case 2:
					if(row > 0 && col < (size_y - 1)) {
						*x = row - 1;
						*y = col + 1;
						*dist = 1.41;
						return true;
					} else return false;

				case 3:
					if(col < (size_y - 1)) {
						*x = row;
						*y = col + 1;
						*dist = 1;
						return true;
					} else return false;
				
				case 4:
					if(row < (size_x -1) && col < (size_y - 1)) {
						*x = row + 1;
						*y = col + 1;
						*dist = 1.41;
						return true;
					} else return false;
				
				case 5:
					if(row < (size_x -1)) {
						*x = row + 1;
						*y = col;
						*dist = 1;
						return true;
					} else return false;
				
				case 6:
					if(row < (size_x -1) && col > 0) {
						*x = row + 1;
						*y = col - 1;
						*dist = 1.41;
						return true;
					} else return false;
				
				case 7:
					if(col > 0) {
						*x = row;
						*y = col - 1;
						*dist = 1;
						return true;
					} else return false;

				default:
					return false;
			}
		} 
};

void path_generator(Cell * curr)
{
	
	//printf("\n\nPATH: ");
	Cell * next_move = curr;
	while (curr != NULL) {
		occupancy[curr->get_col()][curr->get_row()] = 10;
		//printf("[%d,%d] ", curr->get_row(), curr->get_col());
		curr = curr->get_parent();
		if (curr != NULL && curr->get_parent() != NULL)
			next_move = curr;
	}
	//printf("\n\n");
	
	ROS_INFO("Next move: %d %d", next_move->get_row(), next_move->get_col());
	
	std::vector<int> matrix(size_x*size_y);
	for (int j = 0; j < size_y; j++)
		for (int i = 0; i < size_x; i++)
			matrix[j*size_x+i] = occupancy[j][i];
	
	safe_teleop::TrajectoryMap msg;
	msg.size_x = size_x;
	msg.size_y = size_y;
	msg.map = matrix;
	traj_pub.publish(msg);
	
	/* genera comando */
	
	
}

class CellComparator
{
	public:
		bool operator()(Cell * c1, Cell * c2)
		{
			if(c1->get_f() > c2->get_f())
				return true;
			else if((c1->get_f() == c2->get_f()) && (c1->get_h() > c2->get_h()))
				return true;
			
			return false;
		}
};

double convert(int x, int y, std::vector<double> map, int size_y) {
	return map[size_y*(size_y - 1 - x) + y];
}
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	//geometry_msgs::Twist twist;
	vel2_pub.publish(msg);
}

int i = 0;
#define SAFE_DISTANCE 4
#define MAX_PENALITY 12
#define CELL_RESOLUTION 0.1

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {

	if (!enabled) return;
	
	double dx = current_x - start_x;
	double dy = current_y - start_y;
	double dz = current_z - start_z;
	
	if (dx == 0 && dy == 0 && dz == 0) return;
	
	double t_x = (25-target_x)*cos(dz) - (25-target_y)*sin(dz);
	double t_y = (25-target_x)*sin(dz) + (25-target_y)*cos(dz);
	
	target_x = 25 - t_x;
	target_y = 25 + t_y;
	//target_x = target_x + (dx / CELL_RESOLUTION);
	//target_y = target_y + (dy / CELL_RESOLUTION);
	
	start_x = current_x;
	start_y = current_y;
	start_z = current_z;
	
	ROS_INFO("Di: %f %f %f - TARGET: %d %d", dx, dy, dz, target_x, target_y);
	
	if (target_x < 0 || target_x > 49 || target_y < 0 || target_y > 49)
		exit(1);
	
	return;
	
	int size_x = msg->size_x, size_y = msg->size_y;
	vector<double> map = msg->map;
	//printf("conv: %f\n", convert(0, 0, map, size_y)); 

	int manhattan[size_x][size_y];
	manhattan[target_x][target_y] = 0;
	for (int i = target_y -1; i >= 0; i--) {
		manhattan[target_x][i] = manhattan[5][i+1] + 1;
	}
	
	for (int i = target_y + 1; i < size_y; i++) {
		manhattan[target_x][i] = manhattan[target_x][i-1] + 1;
	}
	
	for (int j = target_x - 1; j >= 0; j--)
		for (int i = 0; i < size_y; i++) 
			manhattan[j][i] = manhattan[target_x][i] + (target_x-j);
			
	for (int j = target_x + 1; j < size_x; j++)
		for (int i = 0; i < size_y; i++)
			manhattan[j][i] = manhattan[target_x][i] + (j-target_x);
	
	/*
	for (int i = 0; i < size_x; i++) {
		printf("\n");
		for (int j = 0; j < size_y; j++)
			printf("%2d ", manhattan[i][j]);
	}
	*/

	priority_queue<Cell *, vector<Cell *>, CellComparator> open_set[2];
	vector<Cell *> closed_set;

	int pq = 0;

	double h_s = manhattan[25][25];
	if (convert(25, 25, map, size_y) < SAFE_DISTANCE) 
		h_s += SAFE_DISTANCE - convert(25, 25, map, size_y);
	Cell * start_cell = new Cell(25, 25, 0, h_s);
	open_set[pq].push(start_cell);
	
	int k = 0;
	
	while(!open_set[pq].empty()) {
		
		Cell * current_cell = open_set[pq].top();
		
		/*
		printf("Current cell: ( %d , %d ) : F = %.2f - G = %.2f - H = %.2f", 
			current_cell->get_row(), current_cell->get_col(), 
			current_cell->get_f(), current_cell->get_g(),
			current_cell->get_h());
		if (current_cell == start_cell) printf(" - ORIGIN\n");
		else if (current_cell->get_parent() == NULL) printf(" - Invalid parent\n");
		else printf(" - PARENT = ( %d , %d )\n", 
						current_cell->get_parent()->get_row(),
						current_cell->get_parent()->get_col());
		*/
		if(current_cell->get_h() == 0) {
			
			path_generator(current_cell);
			
			/* deallocate objects */
			while (closed_set.empty()) {
				Cell * c = closed_set.back();
				closed_set.pop_back();
				delete c;
			}
			while (open_set[pq].empty()) {
				Cell * c = open_set[pq].top();
				open_set[pq].pop();
				delete c;
			}
			
			return;
		}
				
		open_set[pq].pop();
		closed_set.push_back(current_cell);
		
		for (int i = 0; i < 8; i++) {
			int r;
			int c;
			double dist;
			if (!current_cell->get_neighbour(i, &r, &c, &dist)) continue;
			
			bool in_closed = false;
			for(int j = 0; j < closed_set.size(); j++)
				if (closed_set[j]->get_col() == c && closed_set[j]->get_row() == r) {
					in_closed = true;
					break;
				}
			if (in_closed) continue;
			
			double tentative_g = current_cell->get_g() + dist;
			bool better = false;
			bool exists = false;
			
			Cell * y = NULL;

			while (!open_set[pq].empty()) {
				Cell * e = open_set[pq].top();
				open_set[pq].pop();
				if (pq == 0)
					open_set[1].push(e);
				else 
					open_set[0].push(e);
				
				if (e->get_col() == c && e->get_row() == r) {
					y = e;
					exists = true;
				}
			}

			if (pq == 0)
				pq = 1;
			else pq = 0;
			
			// 
			if (!exists) {
				double h = manhattan[r][c];
				if (convert(r, c, map, size_y) < 2) h += 10000000;
				else if (convert(r, c, map, size_y) < SAFE_DISTANCE) 
					h += MAX_PENALITY - convert(r, c, map, size_y);
				y = new Cell(r, c, tentative_g, h);
				better = true;
				open_set[pq].push(y);
				y->set_parent(current_cell);
				if (y->get_parent() == NULL) printf("Invalid setted parent\n");

			} else if (tentative_g < y->get_g()) {
				better = true;
			} else better = false;
			
			if (better) {
				y->set_parent(current_cell);
				if (y->get_parent() != current_cell)
					printf("Wrong set parent\n");
				y->update_g(tentative_g);
			}
			closed_set.push_back(y);
		}
		
	}
	
}

void occupancyCallback(const occupancy_map::OccupancyMap::ConstPtr& msg) {
	
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			if (msg->map[i*msg->size_x+j])
				occupancy[j][msg->size_y -1 - i] = 1;
			else occupancy[j][msg->size_y -1 - i] = 0;
		}
	
	if (!enabled) {
		
		std::vector<int> matrix(size_x*size_y);
		for (int j = 0; j < size_y; j++)
			for (int i = 0; i < size_x; i++)
				matrix[j*size_x+i] = occupancy[j][i];
		
		safe_teleop::TrajectoryMap msg;
		msg.size_x = size_x;
		msg.size_y = size_y;
		msg.map = matrix;

		traj_pub.publish(msg);
			
	}

}

void updateOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::string frame_id = msg->header.frame_id;
	ros::Time t = msg->header.stamp;
	tf::StampedTransform laserPose;
	std::string error = "Error :(";
	if (listener->waitForTransform ("/odom", "base_link", t, ros::Duration(0.5), ros::Duration(0.01), &error)){
		
		listener->lookupTransform("/odom", "/base_link", t, laserPose);
		double yaw,pitch,roll;
		btMatrix3x3 mat =  laserPose.getBasis();
		mat.getEulerZYX(yaw, pitch, roll);
		
		current_x = laserPose.getOrigin().x();
		current_y = laserPose.getOrigin().y();
		current_z = yaw;
		
  } else {
    std::cerr << error << std::endl; 
  }
}

void buttonCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[2] > 0) {
		
		printf("Enabled autopilot\n");
		enabled = true;
		
		std::string frame_id = msg->header.frame_id;
		ros::Time t = msg->header.stamp;
		tf::StampedTransform laserPose;
		std::string error = "Error :(";
		if (listener->waitForTransform ("/odom", "base_link", t, ros::Duration(0.5), ros::Duration(0.01), &error)){
			
			listener->lookupTransform("/odom", "/base_link", t, laserPose);
			double yaw,pitch,roll;
			btMatrix3x3 mat =  laserPose.getBasis();
			mat.getEulerZYX(yaw, pitch, roll);
			
			/* set relative origin */
			start_x = laserPose.getOrigin().x();
			start_y = laserPose.getOrigin().y();
			start_z = yaw;
			
			current_x = laserPose.getOrigin().x();
			current_y = laserPose.getOrigin().y();
			current_z = yaw;
			
			/* set target */
			target_x = 5;
			target_y = 25;
			
			ROS_INFO("Position: %f %f %f", start_x, start_y, start_z);
			
		} else {
			std::cerr << error << std::endl; 
		}
		
	} 
}

int main(int argc, char **argv) {
	init(argc, argv, "safe_teleop");
	NodeHandle n;
	distance_sub = n.subscribe("distance_map", 1, distanceCallback);
	occupancy_sub = n.subscribe("occupancy_map", 1, occupancyCallback);
	vel_sub = n.subscribe("vel_gate", 1, velCallback); 
	vel_pub = n.advertise<geometry_msgs::Twist>("vel", 1);
	vel2_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	traj_pub = n.advertise<safe_teleop::TrajectoryMap>("trajectory_map", 1);
	button_sub = n.subscribe("joy", 1, buttonCallback); 
	odom_sub = n.subscribe("/base_scan", 100, updateOdomCallback);
	
	listener = new tf::TransformListener;
	
	spin(); 

	return 0;
}

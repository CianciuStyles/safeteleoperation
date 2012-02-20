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
double old_tx = 0, old_ty = 20;

/* Arrival point (approximation) */
double target_row, target_col;

/* Robot frame id */
string robot_frame; 

Subscriber distance_sub, occupancy_sub, vel_sub, button_sub, odom_sub;
Publisher vel_pub, vel2_pub, traj_pub;
tf::TransformListener * listener = 0;

bool enabled = false;
int occupancy[50][50];

#define SAFE_DISTANCE 5
#define MAX_PENALITY 12
#define CELL_RESOLUTION 10

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int i = 0;

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
	if (curr == NULL) return;
	
	//printf("\n\nPATH: ");
	Cell * next_move = curr;
	Cell * last = NULL;
	while (curr != NULL) {
		last = curr;
		occupancy[curr->get_col()][curr->get_row()] = 10;
		//printf("[%d,%d] ", curr->get_row(), curr->get_col());
		curr = curr->get_parent();
		if (curr != NULL && curr->get_parent() != NULL && curr->get_parent()->get_parent() != NULL)
			next_move = curr;
	}
	printf("\n\n");
	
	if (last->get_h() <= 1) {
		ROS_INFO("Autopilot terminated [2]");
		enabled = false;
		i = 0;
		old_tx = 0;
		old_ty = 20;
		return;
	}
	
	
	ROS_INFO("Next move: %d %d", next_move->get_row(), next_move->get_col());
	
	geometry_msgs::Twist cmd;
	geometry_msgs::Twist cmd2;
	bool double_msg = false;
	
	// diff col/row
	int dr = 25 - next_move->get_row();
	int dc = 25 - next_move->get_col();
	
	if ((dr == 2 && (dc == 1 || dc == 2)) || (dr == 1 && dc == 2)) {
		
		cmd.angular.z = 3.95;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else if (dr == 2 && dc == 0) {
		
		cmd.linear.x = 1.0;
		
	} else if ((dr == 2 && (dc == -1 || dc == -2)) || (dr == 1 && dc == -2)) {
		
		cmd.angular.z = -3.95;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else if (dc == 2 && dr == 0) {
		
		cmd.angular.z = 7.85;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else if (dc == -2 && dr == 0) {
		
		cmd.angular.z = -7.85;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else if ((dr == -2 && (dc == 1 || dc == 2)) || (dr == -1 && dc == 2)) {
		
		cmd.angular.z = 11.8;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else if (dc == 0 && dr == -2) {
		
		cmd.angular.z = 15.7;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else  if ((dr == -2 && (dc == -1 || dc == -2)) || (dr == -1 && dc == -2)) {
		
		cmd.angular.z = -11.8;
		cmd2.linear.x = 1.0;
		double_msg = true;
		
	} else {
		
		//ROS_INFO("Error with next move diff: %d %d", dr, dc);
		//exit(1);

		ROS_INFO("Autopilot terminated [2]");
		enabled = false;
		i = 0;
		old_tx = 0;
		old_ty = 20;
		return;
		
	}
	
	vel2_pub.publish(cmd);
	sleep(1);
	if (double_msg) {
		vel2_pub.publish(cmd2);
		sleep(1);
	}

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
    if (!enabled) // Autopilot?
		vel_pub.publish(msg);
}

int skipped = 0;
bool last_skipped = false;
void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {

	if (!enabled) return;

	double dx = sqrt( (current_x-start_x)*(current_x-start_x) + (current_y-start_y)*(current_y-start_y) );
	double dy = 0; 
	double dz = current_z - start_z;
	
	if (last_skipped && skipped > 20) {
		
		enabled = false;
		i = 0;
		old_tx = 0;
		old_ty = 20;
		ROS_INFO("Autopilot failed[4] :(");
		return;
		
	}
	
	if (i++ > 0 && dx == 0 && dy == 0 && dz == 0) {
		skipped++;
		last_skipped = true;
		return;
	}
	dz = -dz;
	last_skipped = false;

	//ROS_INFO("Current: %f %f %f - START: %f %f %f", 
	//				current_x, current_y, current_z,
	//				start_x, start_y, start_z);
	
	
	double t_x = (target_col-25)*cos(dz) - (25-target_row)*sin(dz);
	double t_y = (target_col-25)*sin(dz) + (25-target_row)*cos(dz);
	
	//ROS_INFO("TARGET INITIAL: %f %f %f %f", (target_col-25), (25-target_row), target_col, target_row);
	//ROS_INFO("T_x: %f = %f - %f", t_x, (target_col-25)*cos(dz), (25-target_row)*sin(dz));
	//ROS_INFO("T_y: %f = %f + %f", t_y, (target_col-25)*sin(dz), (25-target_row)*cos(dz));
	
	target_row = target_row + (old_ty - t_y); //row
	target_col = target_col + (t_x - old_tx); //col
	
	target_row = target_row + dx;
	target_col = target_col + dy;
	old_tx = t_x - dy;
	old_ty = t_y - dx;
	
	start_x = current_x;
	start_y = current_y;
	start_z = current_z;
	
	//ROS_INFO("Di: %f %f %f - TARGET: %.1f %.1f - DISTANCE %.1f", dx, dy, dz, target_row, target_col, 
	//			sqrt((25-target_row)*(25-target_row)+(25-target_col)*(25-target_col)));
	
	int a_row = (int) round(target_row);
	int a_col = (int) round(target_col);
	
	if (target_row < 0 || target_row > 49 || target_col < 0 || target_col > 49) {
		//exit(1);
		enabled = false;
		i = 0;
		old_tx = 0;
		old_ty = 20;
		ROS_INFO("Autopilot failed[5] :(");
		return;
	}
	
	
	int size_x = msg->size_x, size_y = msg->size_y;
	vector<double> map = msg->map;
	//printf("conv: %f\n", convert(0, 0, map, size_y)); 

	int manhattan[size_x][size_y];
	manhattan[a_row][a_col] = 0;
	for (int i = a_col -1; i >= 0; i--) {
		manhattan[a_row][i] = manhattan[5][i+1] + 1;
	}
	
	for (int i = a_col + 1; i < size_y; i++) {
		manhattan[a_row][i] = manhattan[a_row][i-1] + 1;
	}
	
	for (int j = a_row - 1; j >= 0; j--)
		for (int i = 0; i < size_y; i++) 
			manhattan[j][i] = manhattan[a_row][i] + (a_row-j);
			
	for (int j = a_row + 1; j < size_x; j++)
		for (int i = 0; i < size_y; i++)
			manhattan[j][i] = manhattan[a_row][i] + (j-a_row);
	
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
	
	if (manhattan[25][25] <= 1) {
		enabled = false;
		i = 0;
		old_tx = 0;
		old_ty = 20;
		ROS_INFO("Autopilot terminated [1]");
		return;
	}
	
	open_set[pq].push(start_cell);
	
	int k = 0;
	int curr_iter = 0;
	int max_iter = 15000;
	
	while(!open_set[pq].empty()) {
		
		ROS_INFO("A* iteration: %d", ++curr_iter);
		
		if (curr_iter >= max_iter) {
			enabled = false;
			i = 0;
			old_tx = 0;
			old_ty = 20;
			ROS_INFO("Autopilot failed :(");
			return;
		} 
		
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
                                if (convert(r, c, map, size_y) <= 2) h += 10000000;
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
	
	/*
	enabled = false;
	i = 0;
	old_tx = 0;
	old_ty = 20;
	ROS_INFO("Autopilot failed[2] :(");
	return;
	*/
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
	robot_frame = msg->header.frame_id;
	ros::Time t = msg->header.stamp;
	tf::StampedTransform laserPose;
	std::string error = "Error :(";
	if (listener->waitForTransform ("/odom", "base_link", t, ros::Duration(0.5), ros::Duration(0.01), &error)){
		
		listener->lookupTransform("/odom", "/base_link", t, laserPose);
		double yaw,pitch,roll;
		btMatrix3x3 mat =  laserPose.getBasis();
		mat.getEulerZYX(yaw, pitch, roll);
		
		current_x = (laserPose.getOrigin().x()) * CELL_RESOLUTION;
		current_y = (laserPose.getOrigin().y()) * CELL_RESOLUTION;
		current_z = yaw;
		
		//ROS_INFO("Position: %f %f %f", current_x, current_y, current_z);
		
	} else {
		std::cerr << error << std::endl; 
	}
}

void buttonCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

	if (msg->buttons[2] > 0 && enabled)
		return;
	
	if (msg->buttons[2] > 0) {
		
		printf("Enabled autopilot\n");
		enabled = true;
		skipped = 0;
		last_skipped = false;
		
		std::string frame_id = robot_frame;
		ros::Time t = msg->header.stamp;
		tf::StampedTransform laserPose;
		std::string error = "Error :(";
		if (listener->waitForTransform ("/odom", "base_link", ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error)){
			
			listener->lookupTransform("/odom", "/base_link", ros::Time(0), laserPose);
			double yaw,pitch,roll;
			btMatrix3x3 mat =  laserPose.getBasis();
			mat.getEulerZYX(yaw, pitch, roll);
			
			/* set relative origin */
			start_x = (laserPose.getOrigin().x()) * CELL_RESOLUTION;
			start_y = (laserPose.getOrigin().y()) * CELL_RESOLUTION;
			start_z = yaw;
			
			current_x = (laserPose.getOrigin().x()) * CELL_RESOLUTION;
			current_y = (laserPose.getOrigin().y()) * CELL_RESOLUTION;
			current_z = yaw;
			
			/* set target */
			target_row = 5;
			target_col = 25;
			
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

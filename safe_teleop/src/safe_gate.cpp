#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "distance_map/DistanceMap.h"
#include "occupancy_map/OccupancyMap.h"
#include "safe_teleop/TrajectoryMap.h"
#include <math.h>
#include <queue>

#define MAX_ANG 1.8

using namespace std;
using namespace ros;

int size_x = 50;
int size_y = 50;

Subscriber distance_sub, occupancy_sub, vel_sub;
Publisher vel_pub, vel2_pub, traj_pub;

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
	printf("\n\nPATH: ");
	while (curr != NULL) {
		occupancy[curr->get_col()][curr->get_row()] = 10;
		printf("[%d,%d] ", curr->get_row(), curr->get_col());
		curr = curr->get_parent();
	}
	printf("\n\n");
	
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
	//geometry_msgs::Twist twist;
	vel2_pub.publish(msg);
}

int i = 0;
#define SAFE_DISTANCE 8
#define MAX_PENALITY 20

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {

	if (i++ % 300 != 0) {
		printf(".");
		fflush(stdout);
		return;
	} else printf("\n");
	
	int size_x = msg->size_x, size_y = msg->size_y;
	vector<double> map = msg->map;
	//printf("conv: %f\n", convert(0, 0, map, size_y)); 

	int manhattan[size_x][size_y];
	manhattan[5][25] = 0;
	for (int i = 24; i >= 0; i--) {
		manhattan[5][i] = manhattan[5][i+1] + 1;
	}
	
	for (int i = 26; i < size_y; i++) {
		manhattan[5][i] = manhattan[5][i-1] + 1;
	}
	
	for (int j = 4; j >= 0; j--)
		for (int i = 0; i < size_y; i++) 
			manhattan[j][i] = manhattan[5][i] + (5-j);
			
	for (int j = 6; j < size_x; j++)
		for (int i = 0; i < size_y; i++)
			manhattan[j][i] = manhattan[5][i] + (j-5);
	
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
		
		printf("Current cell: ( %d , %d ) : F = %.2f - G = %.2f - H = %.2f", 
			current_cell->get_row(), current_cell->get_col(), 
			current_cell->get_f(), current_cell->get_g(),
			current_cell->get_h());
		if (current_cell == start_cell) printf(" - ORIGIN\n");
		else if (current_cell->get_parent() == NULL) printf(" - Invalid parent\n");
		else printf(" - PARENT = ( %d , %d )\n", 
						current_cell->get_parent()->get_row(),
						current_cell->get_parent()->get_col());
		
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

	spin(); 

	return 0;
}

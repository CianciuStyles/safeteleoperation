#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "distance_map/DistanceMap.h"
#include <math.h>
#include <queue>

#define MAX_ANG 1.8

using namespace std;
using namespace ros;

int size_x = 50;
int size_y = 50;

class Cell
{
	private:
		
		int row;
		int col;
		double f, g, h;
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
	printf("%d - %d\n", curr->get_row(), curr->get_col());
	path_generator(curr->get_parent());
}

class CellComparator
{
	public:
		bool operator()(Cell& c1, Cell& c2)
		{
			if(c1.get_f() < c2.get_f())
				return true;
			else if((c1.get_f() == c2.get_f()) && (c1.get_h() < c2.get_h()))
				return true;
			
			return false;
		}
};

Subscriber distance_sub, vel_sub;
Publisher vel_pub, vel2_pub;

bool enabled = false;

double convert(int x, int y, std::vector<double> map, int size_y) {
	return map[size_y*(size_y - 1 - x) + y];
}
	
void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	//geometry_msgs::Twist twist;
	vel2_pub.publish(msg);
}

void distanceCallback(const distance_map::DistanceMap::ConstPtr& msg) {

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
	
	priority_queue<Cell, vector<Cell>, CellComparator> open_set[2];
	vector<Cell> closed_set;

	int pq = 0;

	Cell start_cell(25, 25, 0, manhattan[25][25]);
	open_set[pq].push(start_cell);
	
	int i = 0;
	
	while(!open_set[pq].empty()) {
		
		if (i++ > 1) return;
		
		Cell current_cell = open_set[pq].top();
		
		if(current_cell.get_h() == 0) {
			path_generator(&current_cell);
			return;
		}
		
		open_set[pq].pop();
		closed_set.push_back(current_cell);
		
		for (int i = 0; i < 8; i++) {
			
			int r;
			int c;
			double dist;
			if (!current_cell.get_neighbour(i, &r, &c, &dist)) continue;
			
			for(int j = 0; j < closed_set.size(); j++)
				if (closed_set[j].get_col() == c && closed_set[j].get_row() == r)
					continue;
			
			double tentative_g = current_cell.get_g() + dist;
			bool better = false;
			bool exists = false;
			
			Cell * y = NULL;

			while (!open_set[pq].empty()) {
				
				Cell e = open_set[pq].top();
				open_set[pq].pop();
				open_set[pq % 1].push(e);
				
				if (e.get_col() == c && e.get_row() == r) {
					y = &e;
					exists = true;
				}
			}
			
			pq = pq % 1;
			
			// 
			if (!exists) {
				y = new Cell(r, c, tentative_g, convert(r, c, map, size_y) - manhattan[r][c]);
				better = true;
				open_set[pq].push(*y);
			} else if (tentative_g < y->get_g()) {
				better = true;
			}
			
			
			
			if (better) {
				
				y->set_parent(&current_cell);
				y->update_g(tentative_g);
				
			}
			
		}
		
	}
	
	
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

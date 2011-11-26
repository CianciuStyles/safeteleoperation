#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>

#define CELL_RESOLUTION 5
#define X_SIZE          50
#define Y_SIZE          50

static bool map[X_SIZE*Y_SIZE];

void laserCallback(const sensor_msgs::LaserScan& msg)
{

	

}

void init_map() 
{
	for (int i = 0; i < X_SIZE*Y_SIZE; i++) map[i] = false;
}

/* cell (0,0) is top-left corner of the matrix */
void set_abs_cell(int x, int y, bool obstacle)
{
	if (x < 0 || x > X_SIZE) return;
	if (y < 0 || y > Y_SIZE) return;
	map[y*X_SIZE+x] = obstacle;
}

/* cell (0,0) is the center of the matrix */
void set_rel_cell(int x, int y, bool obstacle)
{
	set_abs_cell(x+(X_SIZE/2), y+(Y_SIZE/2), obstacle);
}

void print_map()
{
	for (int i = 0; i < X_SIZE +1; i++) {
		printf("---");
	}
	printf("\n");
	for (int j = Y_SIZE-1; j >= 0; j--) {
		printf("|");
		for (int i = 0; i < X_SIZE; i++) {
			if (map[j*X_SIZE+i]) printf(" 1 ");
			else printf("   ");
		}
		printf("|\n");
	}
	for (int i = 0; i < X_SIZE+1; i++) {
		printf("---");
	}
	printf("\n");
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "occupancy_map_generator");
	ros::NodeHandle n;

	ros::Subscriber laser = n.subscribe("base_scan", 1000, laserCallback);
	init_map();
	/*
	set_abs_cell(10, 10, true);
	set_abs_cell(20, 20, true);
	set_abs_cell(30, 30, true);
	set_abs_cell(40, 40, true);
	set_rel_cell(0, 0, true); 
	*/
	print_map();

	ros::spin();

	return 0;
}

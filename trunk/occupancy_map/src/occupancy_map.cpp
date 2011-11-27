#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define CELL_RESOLUTION 10 /* centimeters */
#define X_SIZE          50
#define Y_SIZE          50

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool map[X_SIZE*Y_SIZE];
static int version_map = 0;

void init_map() 
{
	for (int i = 0; i < X_SIZE*Y_SIZE; i++) map[i] = false;
}

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

void print_map()
{
	printf("\nMap version: %d\n", version_map++);
	for (int i = 0; i < X_SIZE +1; i++) {
		printf("---");
	}
	printf("\n");
	for (int j = Y_SIZE-1; j >= 0; j--) {
		printf("|");
		for (int i = 0; i < X_SIZE; i++) {
			if (j == Y_SIZE/2 && i == X_SIZE /2) printf(" O ");
			else if (map[j*X_SIZE+i]) printf(" 1 ");
			else printf("   ");
		}
		printf("|\n");
	}
	for (int i = 0; i < X_SIZE+1; i++) {
		printf("---");
	}
	printf("\n");
}

void laserCallback(const sensor_msgs::LaserScan& msg)
{
	/* Clear map */
	init_map();
	
	/* Number of scan samples */
	int n = (msg.angle_max - msg.angle_min) / msg.angle_increment;
	
	//double angle = msg.angle_min * (360/2*(M_PI)); // degree
	//double increment = msg.angle_increment * (360/2*(M_PI)); // degree
	double angle = msg.angle_min;
	
	for (int i = 0; i < n; i++) {
		
		double x_rel = (msg.ranges[i] * sin(angle)) * 100;
		double y_rel = (msg.ranges[i] * cos(angle)) * 100;
		
		/*
		if (angle < 0 && x_rel >= 0) 
			printf("(%f, %f) : (%f, %f)\n", x_rel, y_rel, angle, msg.ranges[i]); 
		*/
		
		/* scale (x,y) to cell resolution */
		x_rel /= CELL_RESOLUTION;
		y_rel /= CELL_RESOLUTION;
		
		int x = - ((int) x_rel);
		int y = (int) y_rel;
		
		/*
		if (x > -50 && x < 50 && y > -50 && y < 50) 
			printf("(%d, %d) : (%f, %f)\n", x, y, angle, msg.ranges[i]); 
		*/
		
		/* Insert it in the map */
		set_rel_cell(x, y, true);
		
		//angle += increment; // degree
		angle += msg.angle_increment;
		
	}
	
	print_map();
	//exit(0);
	
	ros::Rate loop_rate(0.03);
	loop_rate.sleep();

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "occupancy_map_generator");
	ros::NodeHandle n;

	ros::Subscriber laser = n.subscribe("base_scan", 1, laserCallback);
	init_map();
	
	/*
	set_abs_cell(10, 10, true);
	set_abs_cell(20, 20, true);
	set_abs_cell(30, 30, true);
	set_abs_cell(40, 40, true);
	set_rel_cell(0, 0, true); 
	set_rel_cell(-10, -10, true); 
	print_map();
	*/

	ros::spin();

	return 0;
}

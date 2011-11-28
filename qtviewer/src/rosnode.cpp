/* Module that handles ros functions, extends QThread */

#include <ros/ros.h>
#include <string>
#include <occupancy_map/OccupancyMap.h>
#include <cstdlib> 

#include "mappa.h"
#include "rosnode.h"
#include "rosnode.moc"

RosNode::RosNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

RosNode::~RosNode() {
	ros::shutdown();
	wait();
}

void RosNode::init() {
	ros::init(init_argc, init_argv, "qtviewer");
	ros::start();
    ros::NodeHandle n;
	sub = n.subscribe("obstacle_map", 1000, &RosNode::positionCallback, this);
	start();
}

void RosNode::run() {
	ros::spin();
}

void RosNode::positionCallback(const occupancy_map::OccupancyMap::ConstPtr &msg) {
	/*int tmp[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
	for (int i = 0; i < MAP_WIDTH/PIXEL_SIZE; i++)
		for (int j = 0; j < MAP_HEIGHT/PIXEL_SIZE; j++) {
			tmp[i][j] = rand()%2; //for now we generate a matrix with random 0s and 1s for test purposes
			if (tmp[i][j] == 1)
				emit setPixel(i, j);
			else
				emit unsetPixel(i, j);
		}*/
	//ROS_INFO("received");
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			if (msg->map[i*msg->size_x+j])
				emit setPixel(j, msg->size_y -1 - i);
			else
				emit unsetPixel(j, msg->size_y -1 -i);
		}
}

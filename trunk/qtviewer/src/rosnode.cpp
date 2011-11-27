#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
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

void RosNode::positionCallback(const std_msgs::Int32::ConstPtr &msg) {
	int tmp[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
	for (int i = 0; i < MAP_WIDTH/PIXEL_SIZE; i++)
		for (int j = 0; j < MAP_HEIGHT/PIXEL_SIZE; j++) {
			tmp[i][j] = rand()%2;
			if (tmp[i][j] == 1)
				emit setPixel(i, j);
			else
				emit unsetPixel(i, j);
		}
	//ROS_INFO("CIAO!\n");
}
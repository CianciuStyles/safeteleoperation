/* Module that handles ros functions, extends QThread */

#include <ros/ros.h>
#include <string>
#include <occupancy_map/OccupancyMap.h>
#include <distance_map/DistanceMap.h>
#include <cstdlib> 

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
	occ_sub = n.subscribe("occupancy_map", 1, &RosNode::occupancyCallback, this);
	dist_sub = n.subscribe("distance_map", 1, &RosNode::distanceCallback, this);
	start();
}

void RosNode::run() {
	ros::spin();
}

void RosNode::occupancyCallback(const occupancy_map::OccupancyMap::ConstPtr &msg) {
	return;
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			if (msg->map[i*msg->size_x+j])
				emit setOccupancyPixel(j, msg->size_y -1 - i);
			else
				emit unsetOccupancyPixel(j, msg->size_y -1 -i);
		}
}

void RosNode::distanceCallback(const distance_map::DistanceMap::ConstPtr &msg) {
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			emit setDistancePixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
		}
}

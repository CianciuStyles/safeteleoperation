/* Module that handles ros functions, extends QThread */

#include <ros/ros.h>
#include <string>
#include <cstdlib> 
#include <stdio.h>

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
	grad_sub = n.subscribe("gradient_map", 1, &RosNode::gradientCallback, this);
	traj_sub = n.subscribe("trajectory_map", 1, &RosNode::trajectoryCallback, this);
	start();
}

void RosNode::run() {
	ros::spin();
}

void RosNode::setOccMap(OccupancyMap * map)
{
	occ_map = map;
}

void RosNode::setDistMap(DistanceMap * map)
{
	dist_map = map;
}

void RosNode::setGradMap(GradientMap * map)
{
	grad_map = map;
}

void RosNode::setTrajMap(TrajectoryMap * map)
{
	traj_map = map;
}

void RosNode::occupancyCallback(const occupancy_map::OccupancyMap::ConstPtr &msg) {
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			if (msg->map[i*msg->size_x+j]) 
				occ_map->setPixel(j, msg->size_y -1 - i, true);
				//emit setOccupancyPixel(j, msg->size_y -1 - i);
			else
				//emit unsetOccupancyPixel(j, msg->size_y -1 -i);
				occ_map->setPixel(j, msg->size_y -1 - i, false);
		}
	occ_map->update();
}

void RosNode::distanceCallback(const distance_map::DistanceMap::ConstPtr &msg) {
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			dist_map->setPixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
			//emit setDistancePixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
		}
	dist_map->update();
}

void RosNode::gradientCallback(const gradient_map::GradientMap::ConstPtr &msg) {
	/*
	//printf("G=%d\n", d++);
	grad_map->resetMin();
	grad_map->resetMax();
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			grad_map->setPixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
			//emit setDistancePixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
		}*/
	double max = 0;
	double min = 100;
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			//grad_map->setIntensity(j, msg->size_y -1 - i, msg->map_scalar[i*msg->size_x+j]);
			if (msg->map_scalar[i*msg->size_x+j] < 99) {
				grad_map->setIntensity(j, i, msg->map_scalar[i*msg->size_x+j]);
				if (msg->map_scalar[i*msg->size_x+j] > max) max = msg->map_scalar[i*msg->size_x+j];
				if (msg->map_scalar[i*msg->size_x+j] < min) min = msg->map_scalar[i*msg->size_x+j];
				grad_map->setAngle(j, i, msg->map_theta[i*msg->size_x+j]);
			} else {
				grad_map->setIntensity(j, i, 100);
				grad_map->setAngle(j, i, 0);
			}
			//emit setDistancePixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
		}
	//ROS_INFO("Min: %f", min);
	//ROS_INFO("Max: %f", max);
	grad_map->update();
}

void RosNode::trajectoryCallback(const safe_teleop::TrajectoryMap::ConstPtr &msg) {
	for (int i = 0; i < msg->size_y; i++)
		for (int j = 0; j < msg->size_x; j++) {
			traj_map->setPixel(i, j, msg->map[i*msg->size_x+j]);
			//emit setDistancePixel(j, msg->size_y -1 - i, msg->map[i*msg->size_x+j]);
		}
	traj_map->update();
}

#ifndef _ROSNODE_
#define _ROSNODE_

#include <ros/ros.h>
#include <occupancy_map/OccupancyMap.h>
#include <distance_map/DistanceMap.h>
#include <QThread>
#include <QObject>

class RosNode : public QThread {
	Q_OBJECT
	public:
		RosNode(int argc, char **argv);
		~RosNode();
		void init();
		void run();
		void occupancyCallback(const occupancy_map::OccupancyMap::ConstPtr &msg);
		void distanceCallback(const distance_map::DistanceMap::ConstPtr &msg);
	private:
		int init_argc;
		char **init_argv;
		ros::Subscriber occ_sub;
		ros::Subscriber dist_sub;
	signals:
		void setOccupancyPixel(int x, int y);
		void unsetOccupancyPixel(int x, int y);
		void setDistancePixel(int x, int y, double color);
		void setGradientPixel(int x, int y);
		void unsetGradientPixel(int x, int y);
};

#endif

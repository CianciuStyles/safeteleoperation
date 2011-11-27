#ifndef _ROSNODE_
#define _ROSNODE_

#include <ros/ros.h>
#include <occupancy_map/OccupancyMap.h>
#include <QThread>
#include <QObject>

class RosNode : public QThread {
	Q_OBJECT
	public:
		RosNode(int argc, char **argv);
		~RosNode();
		void init();
		void run();
		void positionCallback(const occupancy_map::OccupancyMap::ConstPtr &msg);
	private:
		int init_argc;
		char **init_argv;
		ros::Subscriber sub;
	signals:
		void setPixel(int x, int y);
		void unsetPixel(int x, int y);
};

#endif

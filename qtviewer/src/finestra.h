#ifndef _FINESTRA_
#define _FINESTRA_

#include <QWidget>
#include "settings.h"
#include "trajectory_map.h"
#include "occupancy_map.h"
#include "distance_map.h"
#include "gradient_map.h"

class Finestra: public QWidget {
	Q_OBJECT
	public:
		Finestra(QWidget *parent = 0, Qt::WFlags f = 0, TrajectoryMap *tm = 0, OccupancyMap* om = 0, DistanceMap* dm = 0,
			GradientMap *gm = 0);
	private:
		TrajectoryMap *trajectoryMap;
		OccupancyMap *occupancyMap;
		DistanceMap *distanceMap;
		GradientMap *gradientMap;
	public slots:
		void updateMaps();
};

#endif

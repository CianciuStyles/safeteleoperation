#include <QtGui>
#include <QApplication>
#include <QProcess>
#include <QTimer>
#include <signal.h>
#include "finestra.h"
#include "occupancy_map.h"
#include "distance_map.h"
#include "gradient_map.h"
#include "trajectory_map.h"
#include "rosnode.h"

static RosNode *rn;
static TrajectoryMap *trajectoryMap;
static OccupancyMap *occupancyMap;
static DistanceMap *distanceMap;
static GradientMap *gradientMap;

int main(int argc, char **argv) {
	QApplication app(argc, argv);
	Finestra *f;

	trajectoryMap = new TrajectoryMap(f, 0);
	occupancyMap = new OccupancyMap(f, 0);
	distanceMap = new DistanceMap(f, 0);
	gradientMap = new GradientMap(f, 0);
	
	rn = new RosNode(argc, argv);
	rn->init();
	rn->setTrajMap(trajectoryMap);
	rn->setOccMap(occupancyMap);
	rn->setDistMap(distanceMap);
	rn->setGradMap(gradientMap);
	
	f = new Finestra(0, 0, trajectoryMap, occupancyMap, distanceMap, gradientMap);
	f->setGeometry(200, 20, MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f->setMinimumSize(MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f->setMaximumSize(MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f->show();

	signal(SIGINT, SIG_DFL);
	
	QTimer *timer = new QTimer(NULL);
	QObject::connect(timer, SIGNAL(timeout()), f, SLOT(updateMaps()));
	timer->start(100); //refresh map every 100 milliseconds

	return app.exec();
}

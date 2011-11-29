#include <QtGui>
#include <QApplication>
#include <QProcess>
#include <QGridLayout>
#include <signal.h>
#include "occupancy_map.h"
#include "distance_map.h"
#include "gradient_map.h"
#include "rosnode.h"

static RosNode *rn;
//static Mappa *mappa1;
static OccupancyMap *occupancyMap;
static DistanceMap *distanceMap;
static GradientMap *gradientMap;

class Finestra: public QWidget {
	public:
		Finestra(QWidget *parent = 0, Qt::WFlags f = 0);
};

Finestra::Finestra(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	//QPushButton *quit = new QPushButton("Quit", this);
	//mappa1 = new Mappa(this, 0);
	occupancyMap = new OccupancyMap(this, 0);
	distanceMap = new DistanceMap(this, 0);
	gradientMap = new GradientMap(this, 0);

	//connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
	connect(rn, SIGNAL(setOccupancyPixel(int, int)), occupancyMap, SLOT(drawPixel(int, int)));
	connect(rn, SIGNAL(unsetOccupancyPixel(int, int)), occupancyMap, SLOT(undrawPixel(int, int)));
	connect(rn, SIGNAL(setDistancePixel(int, int, double)), distanceMap, SLOT(drawDistancePixel(int, int, double)));
	//connect(rn, SIGNAL(unsetDistancePixel(int, int)), distanceMap, SLOT(undrawPixel(int, int)));
	//connect(rn, SIGNAL(setGradientPixel(int, int)), gradientMap, SLOT(drawPixel(int, int)));
	//connect(rn, SIGNAL(unsetGradientPixel(int, int)), gradientMap, SLOT(undrawPixel(int, int)));

	QGridLayout *grid = new QGridLayout(this);

	//grid->addWidget(quit, 0, 0);
	//grid->addWidget(mappa1, 0, 0 );
	grid->addWidget(occupancyMap, 0, 1 );
	grid->addWidget(distanceMap, 1, 0 );
	grid->addWidget(gradientMap, 1, 1 );
	//grid->setColumnStretch(0, 10 );
	//grid->setColumnStretch(1, 10 );
	grid->setSpacing(10);
}

int main(int argc, char **argv) {
	rn = new RosNode(argc, argv);
	rn->init();
	
	QApplication app(argc, argv);
	Finestra f;
	f.setGeometry(200, 20, MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f.setMinimumSize(MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f.setMaximumSize(MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
	f.show();
	
	rn->setOccMap(occupancyMap);
	rn->setDistMap(distanceMap);
	rn->setGradMap(gradientMap);

	signal(SIGINT, SIG_DFL);

	return app.exec();
}

#include <QGridLayout>
#include "finestra.h"
#include "finestra.moc"

Finestra::Finestra(QWidget *parent, Qt::WFlags f, TrajectoryMap *tm, OccupancyMap *om, DistanceMap *dm, GradientMap *gm) 
		: QWidget(parent, f) {
	trajectoryMap = tm;
	occupancyMap = om;
	distanceMap = dm;
	gradientMap = gm;

	QGridLayout *grid = new QGridLayout(this);

	grid->addWidget(trajectoryMap, 0, 0);
	grid->addWidget(occupancyMap, 0, 1 );
	grid->addWidget(distanceMap, 1, 0 );
	grid->addWidget(gradientMap, 1, 1 );
	//grid->setColumnStretch(0, 10 );
	//grid->setColumnStretch(1, 10 );
	grid->setSpacing(10);
}

void Finestra::updateMaps() {
	trajectoryMap->update();
	occupancyMap->update();
	distanceMap->update();
	gradientMap->update();
}

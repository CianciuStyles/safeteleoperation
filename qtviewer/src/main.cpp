#include <QtGui>
#include <QApplication>
#include <QGridLayout>
#include "occupancy_map.h"
#include "distance_map.h"
#include "gradient_map.h"
#include "rosnode.h"

static RosNode *rn;
//static Mappa *mappa1;
static OccupancyMap *mappa2;
static DistanceMap *mappa3;
static GradientMap *mappa4;

class Finestra: public QWidget {
	public:
		Finestra(QWidget *parent = 0, Qt::WFlags f = 0);
};

Finestra::Finestra(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	//QPushButton *quit = new QPushButton("Quit", this);
	//mappa1 = new Mappa(this, 0);
	mappa2 = new OccupancyMap(this, 0);
	mappa3 = new DistanceMap(this, 0);
	mappa4 = new GradientMap(this, 0);

	//connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
	connect(rn, SIGNAL(setPixel(int, int)), mappa2, SLOT(drawPixel(int, int)));
	connect(rn, SIGNAL(unsetPixel(int, int)), mappa2, SLOT(undrawPixel(int, int)));
    
	QGridLayout *grid = new QGridLayout(this);

    //grid->addWidget(quit, 0, 0);
    //grid->addWidget(mappa1, 0, 0 );
    grid->addWidget(mappa2, 0, 1 );
    grid->addWidget(mappa3, 1, 0 );
    grid->addWidget(mappa4, 1, 1 );
    //grid->setColumnStretch(0, 10 );
    //grid->setColumnStretch(1, 10 );
    grid->setSpacing(10);
}

int main(int argc, char **argv) {
	rn = new RosNode(argc, argv);
	rn->init();
    QApplication app(argc, argv);
    Finestra f;
    f.setGeometry(100, 10, MAP_WIDTH*2 + 20, MAP_HEIGHT*2 + 20);
    f.show();
 
    int result = app.exec();
	return result;
}

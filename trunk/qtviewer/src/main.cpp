#include <QtGui>
#include <QApplication>
#include <QGridLayout>
#include "mappa.h"
#include "rosnode.h"

static RosNode *rn;
static Mappa *mappa;

class Finestra: public QWidget {
	public:
		Finestra(QWidget *parent = 0, Qt::WFlags f = 0);
};

Finestra::Finestra(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	QPushButton *quit = new QPushButton("Quit", this);
	mappa = new Mappa(this, 0);

	connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
	connect(rn, SIGNAL(setPixel(int, int)), mappa, SLOT(drawPixel(int, int)));
	connect(rn, SIGNAL(unsetPixel(int, int)), mappa, SLOT(undrawPixel(int, int)));
    
	QGridLayout *grid = new QGridLayout(this);

    grid->addWidget(quit, 0, 0);
    grid->addWidget(mappa, 1, 1 );
    grid->setColumnStretch(1, 10 );
}

int main(int argc, char **argv) {
	rn = new RosNode(argc, argv);
	rn->init();
    QApplication app(argc, argv);
    Finestra f;
    f.setGeometry(100, 100, MAP_WIDTH + 150, MAP_HEIGHT + 100);
    f.show();
 
    int result = app.exec();
	return result;
}

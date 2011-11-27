#include <QtGui>
#include <QApplication>
#include <QGridLayout>
#include "std_msgs/String.h"
#include "mappa.h"
#include "rosnode.h"

RosNode *rn;
Mappa *mappa;

typedef struct thread_arguments {
    int arg_c;
    char **arg_v;
} thread_arguments;

class Finestra: public QWidget {
	public:
		Finestra(QWidget *parent = 0, Qt::WFlags f = 0);
};

Finestra::Finestra(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	QPushButton *quit = new QPushButton("Quit", this);
	//QPushButton *print = new QPushButton("Print", this);
	mappa = new Mappa(this, 0);

	connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
	connect(rn, SIGNAL(setPixel(int, int)), mappa, SLOT(drawPixel(int, int)));
	connect(rn, SIGNAL(unsetPixel(int, int)), mappa, SLOT(undrawPixel(int, int)));

	//mappa->setPixel(13, 2);
    
	QGridLayout *grid = new QGridLayout(this);

    grid->addWidget(quit, 0, 0);
    grid->addWidget(mappa, 1, 1 );
    grid->setColumnStretch(1, 10 );
}

int main(int argc, char **argv) {
	/*int i;
	void *status;
	pthread_t tid;
	thread_arguments *targs;
    
    targs = (thread_arguments*)malloc(sizeof(thread_arguments));
    targs->arg_c = argc;
    targs->arg_v = argv;
    i = pthread_create(&tid, NULL, rosFunction, (void*)targs);*/
	rn = new RosNode(argc, argv);
	rn->init();
    QApplication app(argc, argv);
    Finestra f;
    f.setGeometry(100, 100, MAP_WIDTH + 150, MAP_HEIGHT + 100);
    f.show();
    //mappa->setPixel(12, 2);
 
    int result = app.exec();
	//pthread_cancel(tid);
	//pthread_join(tid, &status);
	return result;
}

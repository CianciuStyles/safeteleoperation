#ifndef _TRAJECTORY_MAP_
#define _TRAJECTORY_MAP_

#include <QWidget>
#include "settings.h"

class TrajectoryMap : public QWidget {
	Q_OBJECT
	public:
		TrajectoryMap(QWidget *parent = 0, Qt::WFlags f = 0);
		int pixels[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawTrajectoryPixel(int x, int y, int value);
		void setPixel(int x, int y, int value);
		//void undrawPixel(int x, int y);
	private:
		std::vector<QPainter*> painters;
};

#endif

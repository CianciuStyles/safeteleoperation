#ifndef _DISTANCE_MAP_
#define _DISTANCE_MAP_

#include <QWidget>
#include "settings.h"

class DistanceMap : public QWidget {
	Q_OBJECT
	public:
		DistanceMap(QWidget *parent = 0, Qt::WFlags f = 0);
		double pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawDistancePixel(int x, int y, double color);
		//void undrawPixel(int x, int y);
};

#endif

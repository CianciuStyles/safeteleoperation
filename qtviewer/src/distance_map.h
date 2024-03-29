#ifndef _DISTANCE_MAP_
#define _DISTANCE_MAP_

#include <QWidget>
#include "settings.h"

class DistanceMap : public QWidget {
	//Q_OBJECT
	public:
		DistanceMap(QWidget *parent = 0, Qt::WFlags f = 0);
		double pixels[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
		void drawDistancePixel(int x, int y, double color);
		void setPixel(int x, int y, double value);
	protected:
		void paintEvent(QPaintEvent *event);
	private:
		std::vector<QPainter*> painters;
};

#endif

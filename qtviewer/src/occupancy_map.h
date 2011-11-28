#ifndef _OCCUPANCY_MAP_
#define _OCCUPANCY_MAP_

#include <QWidget>
#include "settings.h"

class OccupancyMap : public QWidget {
	Q_OBJECT
	public:
		OccupancyMap(QWidget *parent = 0, Qt::WFlags f = 0);
		bool pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawPixel(int x, int y);
		void undrawPixel(int x, int y);
		
};

#endif
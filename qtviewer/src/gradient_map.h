#ifndef _GRADIENT_MAP_
#define _GRADIENT_MAP_

#include <QWidget>
#include "settings.h"

class GradientMap : public QWidget {
	//Q_OBJECT
	public:
		GradientMap(QWidget *parent = 0, Qt::WFlags f = 0);
		double intensity[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
		double angle[MAP_WIDTH/PIXEL_SIZE][MAP_HEIGHT/PIXEL_SIZE];
		QPolygon getArrow(int x, int y, double angle);
		void setIntensity(int x, int y, double value);
		void setAngle(int x, int y, double value);
	protected:
		void paintEvent(QPaintEvent *event);
	//public slots:
		
	private:
		std::vector<QPainter*> painters_red;
		std::vector<QPainter*> painters_green;
};

#endif

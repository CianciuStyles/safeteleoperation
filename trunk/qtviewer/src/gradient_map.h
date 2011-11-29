#ifndef _GRADIENT_MAP_
#define _GRADIENT_MAP_

#include <QWidget>
#include "settings.h"

class GradientMap : public QWidget {
	Q_OBJECT
	public:
		GradientMap(QWidget *parent = 0, Qt::WFlags f = 0);
		double pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void setPixel(int x, int y, double value);
		void resetMin();
		void resetMax();
	private:
		std::vector<QPainter*> painters_red;
		std::vector<QPainter*> painters_green;
};

#endif

#ifndef _GRADIENT_MAP_
#define _GRADIENT_MAP_

#include <QWidget>
#include "settings.h"

class GradientMap : public QWidget {
	Q_OBJECT
	public:
		GradientMap(QWidget *parent = 0, Qt::WFlags f = 0);
		bool pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawPixel(int x, int y);
		void undrawPixel(int x, int y);
};

#endif
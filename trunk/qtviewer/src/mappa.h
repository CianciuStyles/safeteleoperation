#ifndef _MAPPA_
#define _MAPPA_

#include <QWidget>

#define MAP_WIDTH 520
#define MAP_HEIGHT 520
#define PIXEL_SIZE 20

class Mappa : public QWidget {
	Q_OBJECT
	public:
		Mappa(QWidget *parent = 0, Qt::WFlags f = 0);//, int h = 0, int w = 0);
		int pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawPixel(int x, int y);
		void undrawPixel(int x, int y);
		
};

#endif

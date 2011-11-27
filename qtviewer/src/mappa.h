#ifndef _MAPPA_
#define _MAPPA_

#include <QWidget>

#define MAP_WIDTH 500
#define MAP_HEIGHT 500
#define PIXEL_SIZE 10

class Mappa : public QWidget {
	Q_OBJECT
	public:
		Mappa(QWidget *parent = 0, Qt::WFlags f = 0);//, int h = 0, int w = 0);
		bool pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawPixel(int x, int y);
		void undrawPixel(int x, int y);
		
};

#endif

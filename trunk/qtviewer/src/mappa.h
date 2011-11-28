#ifndef _MAPPA_
#define _MAPPA_

#include <QWidget>

#define MAP_WIDTH 350
#define MAP_HEIGHT 350
#define PIXEL_SIZE 7

class Mappa : public QWidget {
	Q_OBJECT
	public:
		Mappa(QWidget *parent = 0, Qt::WFlags f = 0);
		bool pixels[MAP_WIDTH][MAP_HEIGHT];
	protected:
		void paintEvent(QPaintEvent *event);
	public slots:
		void drawPixel(int x, int y);
		void undrawPixel(int x, int y);
		
};

#endif

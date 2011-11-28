#include <QPainter>
#include <QColor>
#include <stdio.h>
#include "distance_map.h"
#include "distance_map.moc"
#include "settings.h"
int qwerty = 0;
DistanceMap::DistanceMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k][w] = 0;
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

void DistanceMap::paintEvent(QPaintEvent *event) {
	QPainter p(this);
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
		
	/* draw the robot in the center */
	p.setBrush(Qt::red);
	QRect robot = QRect((rows/2)*PIXEL_SIZE, (cols/2)*PIXEL_SIZE, 2*PIXEL_SIZE, 2*PIXEL_SIZE);
	p.drawRect(robot);
	printf("%d ", qwerty++);
	/* draw pixels according to distances */
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++) {
			QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
			double c = pixels[k][w];
			int g = 256;
			for (g = 0; c > 0; g+=25)
				c-=0.25;
			
			if (g == 0)
				p.setBrush(Qt::black);
			else if (g >= 255)
				p.setBrush(Qt::white);
			else
				p.setBrush(QColor(255, g, 0));
			p.drawRect(rect);
		}	
}

void DistanceMap::drawDistancePixel(int x, int y, double color) {
	pixels[x][y] = color;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

/*void DistanceMap::undrawPixel(int x, int y) {
	if (!pixels[x][y])
		return;
	pixels[x][y] = false;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}*/

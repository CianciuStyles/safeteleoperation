#include <QPainter>
#include <stdio.h>
#include "occupancy_map.h"
#include "occupancy_map.moc"

static int qwerty = 0;
OccupancyMap::OccupancyMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k] = {false};
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

void OccupancyMap::paintEvent(QPaintEvent *event) {
	QPainter p(this);
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	/* draw the robot in the center */
	p.setBrush(Qt::red);
	QRect robot = QRect((rows/2)*PIXEL_SIZE, (cols/2)*PIXEL_SIZE, 2*PIXEL_SIZE, 2*PIXEL_SIZE);
	p.drawRect(robot);
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	printf("O %d ", qwerty++);
	/* draw pixels with obstacles */
	p.setBrush(Qt::black);
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			if (pixels[k][w]) {
				QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
				p.drawRect(rect);
			}	
}

void OccupancyMap::drawPixel(int x, int y) {
	if (pixels[x][y])
		return;
	pixels[x][y] = true;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

void OccupancyMap::undrawPixel(int x, int y) {
	if (!pixels[x][y])
		return;
	pixels[x][y] = false;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

#include <QPainter>
#include <QColor>
#include <stdio.h>
#include "trajectory_map.h"
#include "trajectory_map.moc"
#include "settings.h"

TrajectoryMap::TrajectoryMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT/PIXEL_SIZE; k++)
		for (int w = 0; w < MAP_WIDTH/PIXEL_SIZE; w++)
			pixels[k][w] = 0;
	setPalette(QPalette(QColor(255, 255, 255)));
}

void TrajectoryMap::paintEvent(QPaintEvent *event) {
	QPainter p(this);
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
			p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
			p.drawLine (0, j, MAP_WIDTH, j);
	
	p.setPen(Qt::gray);
	
	/* draw pixels */
	for (int k = 0; k < rows; k++)
		for (int w = 0; w < cols; w++) {
			QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
			int c = pixels[k][w];
			if (c <= 0)
				p.setBrush(Qt::white);
			else if (c > 0 && c < 10)
				p.setBrush(Qt::black);
			else p.setBrush(Qt::cyan);
			
			p.drawRect(rect);
		}
	
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	QRect robot = QRect((rows/2 - 2)*PIXEL_SIZE, (cols/2 -2)*PIXEL_SIZE, 4*PIXEL_SIZE, 4*PIXEL_SIZE);
	p.drawRect(robot);
}

void TrajectoryMap::drawTrajectoryPixel(int x, int y, int value) {
	pixels[x][y] = value;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

void TrajectoryMap::setPixel(int x, int y, int value) {
	pixels[x][y] = value;
}

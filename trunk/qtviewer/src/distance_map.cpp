#include <QPainter>
#include <QColor>
#include <stdio.h>
#include "distance_map.h"
//#include "distance_map.moc"
#include "settings.h"

int qwerty = 0;

DistanceMap::DistanceMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT/PIXEL_SIZE; k++)
		for (int w = 0; w < MAP_WIDTH/PIXEL_SIZE; w++)
			pixels[k][w] = -1;
	setPalette(QPalette(QColor(255, 255, 255)));
}

void DistanceMap::paintEvent(QPaintEvent *event) {
	
	if (painters.size() == 0) {
			//printf("Init painters...\n");
			for (int i = 0; i < 255/10; i++) {
					painters.push_back(new QPainter(this));
					painters[i]->setBrush(QColor(255, 10*i, 0));
					painters[i]->setPen(Qt::gray);
			}
			painters.push_back(new QPainter(this));
			painters[255/10]->setBrush(QColor(255, 255, 0));
			painters[255/10]->setPen(Qt::gray);
	}
	
	//printf("Redraw...\n");
	QPainter p(this);
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
			p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
			p.drawLine (0, j, MAP_WIDTH, j);
	
	QPainter pb(this);
	pb.setBrush(Qt::black);
	pb.setPen(Qt::gray);
	
	QPainter pw(this);
	pw.setBrush(Qt::transparent);
	pw.setPen(Qt::gray);
	
	/* draw pixels according to distances */
	for (int k = 0; k < rows; k++) {
		for (int w = 0; w < cols; w++) {
			QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
			double c = pixels[k][w];
			if (c < 0) continue;
			
			int g = 256;
			for (g = 0; c > 0; g+=10)
				c-=0.25;
			
			if (g == 0)
				pb.drawRect(rect);
			else if (g >= 255)
				pw.drawRect(rect);
			else {
				painters[g/10]->drawRect(rect);
			}
		}
	}
		
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	QRect robot = QRect((rows/2 - 2)*PIXEL_SIZE, (cols/2 -2)*PIXEL_SIZE, 4*PIXEL_SIZE, 4*PIXEL_SIZE);
	p.drawRect(robot);
	
	//printf("Draw robot");
	
}

void DistanceMap::drawDistancePixel(int x, int y, double color) {
	pixels[x][y] = color;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

void DistanceMap::setPixel(int x, int y, double value) {
	if (value > 7)
		pixels[x][y] = -1;
	else pixels[x][y] = value;
}

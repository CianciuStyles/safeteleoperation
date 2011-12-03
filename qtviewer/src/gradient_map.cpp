#include <QPainter>
#include <QColor>
#include <QPolygon>
#include "gradient_map.h"
#include "gradient_map.moc"
#include "settings.h"
#include <stdio.h>

GradientMap::GradientMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT/PIXEL_SIZE; k++)
		for (int w = 0; w < MAP_WIDTH/PIXEL_SIZE; w++) {
			intensity[k][w] = 100;
			angle[k][w] = 0;
		}
	/*
	/*intensity[0][0] = -1;
	angle[0][0] = -1;
	intensity[1][0] = -0.75;
	angle[1][0] = 270;
	intensity[2][0] = 255;
	angle[2][0] = 270;
	intensity[3][0] = 255;
	angle[3][0] = 270;
	intensity[0][1] = 255;
	angle[0][1] = 0;
	intensity[1][1] = 170;
	angle[1][1] = 315;
	intensity[2][1] = 170;
	angle[2][1] = 270;
	intensity[3][1] = 170;
	angle[3][1] = 270;
	intensity[2][2] = 100;
	angle[2][2] = 315;
	intensity[0][2] = 255;
	angle[0][2] = 0;
	intensity[1][2] = 170;
	angle[1][2] = 0;
	intensity[2][1] = 170;*/
	
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

#define RGB_DIVISOR 10

void GradientMap::paintEvent(QPaintEvent *event) {
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	double random;
	
	QPainter p(this);
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	
	int increment_red = 1;
	
	/* Init painters */
	if (painters_red.size() == 0) {
		for (int i = 0; i < 255/10; i++) {
			painters_red.push_back(new QPainter(this));
			painters_red[i]->setBrush(QColor(255, 10*i, 0));
			painters_red[i]->setPen(QColor(255, 10*i, 0));
		}
		painters_red.push_back(new QPainter(this));
		painters_red[255/10]->setBrush(QColor(255, 255, 0));
		painters_red[255/10]->setPen(QColor(255, 255, 0));
		/*
		for (int i = 0; i < 255/increment_red; i++) {
			painters_red.push_back(new QPainter(this));
			painters_red[i]->setBrush(QColor(255, increment_red*i, 0));
			painters_red[i]->setPen(QColor(255, increment_red*i, 0));
		}
		painters_red.push_back(new QPainter(this));
		painters_red[255/increment_red]->setBrush(QColor(255, 255, 255));
		painters_red[255/increment_red]->setPen(QColor(255, 255, 255));
		painters_red.push_back(new QPainter(this));
		painters_red[255/increment_red + 1]->setBrush(QColor(255, 0, 0));//for drawing a QRect instead of an arrow
		painters_red[255/increment_red + 1]->setPen(Qt::gray);
		*/
		painters_red.push_back(new QPainter(this));
		painters_red[255/10 +1]->setBrush(QColor(255, 0, 0));//for drawing a QRect instead of an arrow
		painters_red[255/10 +1]->setPen(Qt::gray);
	}
	
	//p.setBrush(Qt::red);
	for (int k = 0; k < rows; k++) {
		for (int w = 0; w < cols; w++) {
			double c = intensity[k][w];
			int g = 0;
			if (!(angle[k][w] < 0)) {
				
				if (c > 5) continue;
				QPolygon triangle = getArrow(k, w, angle[k][w]);
				
				int g = 256;
				c -= 1;
				for (g = 0; c > 0; g+=10)
					c-=0.25;
				
				if (g > 255)
					g = 255;
				
				painters_red[g/10]->drawPolygon(triangle);
			}
			else {
				painters_red[255/10 +1]->setPen(Qt::gray);
				painters_red[255/10 +1]->drawRect(QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE));
			}
		}
	}
	
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	p.setPen(Qt::NoPen);

	QRect robot = QRect((rows/2 - 2)*PIXEL_SIZE, (cols/2 -2)*PIXEL_SIZE, 4*PIXEL_SIZE, 4*PIXEL_SIZE);
	p.drawRect(robot);
}


QPolygon GradientMap::getArrow(int x, int y, double angle) {
	QPolygon triangle;
	
	if ((angle >= 0 && angle < 22.5) || angle >= 337.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+5,y*PIXEL_SIZE+3, x*PIXEL_SIZE+2,y*PIXEL_SIZE+0, x*PIXEL_SIZE+2,y*PIXEL_SIZE+6);
	}
	else if (angle >= 22.5 && angle < 67.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+5,y*PIXEL_SIZE+2, x*PIXEL_SIZE+5,y*PIXEL_SIZE+6, x*PIXEL_SIZE+1,y*PIXEL_SIZE+2);
	}
	else if (angle >= 67.5 && angle < 112.5){
		triangle.setPoints(3, x*PIXEL_SIZE+3,y*PIXEL_SIZE+1, x*PIXEL_SIZE+0,y*PIXEL_SIZE+4, x*PIXEL_SIZE+6,y*PIXEL_SIZE+4);
	}
	else if (angle >= 112.5 && angle < 157.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+2,y*PIXEL_SIZE+2, x*PIXEL_SIZE+6,y*PIXEL_SIZE+2, x*PIXEL_SIZE+2,y*PIXEL_SIZE+6);
	}
	else if (angle >= 157.5 && angle < 202.5){
		triangle.setPoints(3, x*PIXEL_SIZE+1,y*PIXEL_SIZE+3, x*PIXEL_SIZE+4,y*PIXEL_SIZE+0, x*PIXEL_SIZE+4,y*PIXEL_SIZE+6);
	}
	else if (angle >= 202.5 && angle < 247.5){
		triangle.setPoints(3, x*PIXEL_SIZE+1,y*PIXEL_SIZE+5, x*PIXEL_SIZE+5,y*PIXEL_SIZE+5, x*PIXEL_SIZE+1,y*PIXEL_SIZE+1);
	}
	else if (angle >= 247.5 && angle < 292.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+3,y*PIXEL_SIZE+5, x*PIXEL_SIZE+0,y*PIXEL_SIZE+2, x*PIXEL_SIZE+6,y*PIXEL_SIZE+2);
	}
	else if (angle >= 292.5 && angle < 337.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+5,y*PIXEL_SIZE+5, x*PIXEL_SIZE+5,y*PIXEL_SIZE+1, x*PIXEL_SIZE+1,y*PIXEL_SIZE+5);
	}
	
	return triangle;
}

void GradientMap::setIntensity(int x, int y, double i) {
	intensity[x][y] = i;
}

void GradientMap::setAngle(int x, int y, double a) {
	angle[x][y] = a;
}

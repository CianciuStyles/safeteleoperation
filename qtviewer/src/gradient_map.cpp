#include <QPainter>
#include <QColor>
#include <QPolygon>
#include "gradient_map.h"
#include "gradient_map.moc"
#include "settings.h"
#include <stdio.h>


double min = 1000;
double max = 0;

GradientMap::GradientMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	double random;
	for (int k = 0; k < MAP_HEIGHT/PIXEL_SIZE; k++)
		for (int w = 0; w < MAP_WIDTH/PIXEL_SIZE; w++) {
			pixels[k][w] = 0;
			intensity[k][w] = 0;
			random = rand()%360;
			angle[k][w] = 0;
		}
	
	intensity[0][0] = 255;
	angle[0][0] = 315;
	intensity[1][0] = 255;
	angle[1][0] = 270;
	intensity[0][1] = 255;
	angle[0][1] = 0;
	intensity[1][1] = 170;
	angle[1][1] = 315;
	intensity[2][2] = 100;
	angle[2][2] = 315;
	
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

#define RGB_DIVISOR 10

void GradientMap::paintEvent(QPaintEvent *event) {
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	double random;
	/*for (int k = 0; k < MAP_HEIGHT/PIXEL_SIZE; k++)
		for (int w = 0; w < MAP_WIDTH/PIXEL_SIZE; w++) {
			random = rand()%360;
			angle[k][w] = random;
		}
	*/
	QPainter p(this);
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	
	int increment_red = 10;//(-min) / 255;
	/*if (increment_red <= 1) increment_red = 1;
	int increment_green = max / 255;
	if (increment_green <= 1) increment_green = 1;*/
	
	/* Init painters */
	if (painters_red.size() == 0) {
		for (int i = 0; i < 255/increment_red; i++) {
			painters_red.push_back(new QPainter(this));
			painters_red[i]->setBrush(QColor(255, increment_red*i, 0));
			painters_red[i]->setPen(QColor(255, increment_red*i, 0));
		}
		painters_red.push_back(new QPainter(this));
		painters_red[255/increment_red]->setBrush(QColor(255, 255, 255));
		painters_red[255/increment_red]->setPen(QColor(255, 255, 255));
	}
	
	p.setBrush(Qt::red);
	for (int k = 0; k < rows; k++) {
		for (int w = 0; w < cols; w++) {
			double c = intensity[k][w];
			QPolygon triangle = getArrow(k, w, angle[k][w]);

			int g = 0;
			
			if (c > 0) {
				while(c < 255) {
					g++;
					c += increment_red;
					if (g > 255/increment_red) {
						g = 255/increment_red;
						break;
					}
				}
				/*if (g*increment_red < 0 || g*increment_red > 255)
					printf("Invalid RGB\n");
				QPainter pp(this);
				pp.setBrush(QColor(255, increment_red*g, increment_red*g));
				pp.setPen(Qt::gray);
				pp.drawPolygon(triangle);*/
				if (g < 0 || g > painters_red.size() - 1) 
					printf("Invalid index\n");
				//p.setBrush(Qt::red);
				painters_red[g]->drawPolygon(triangle);
			} 
			else {
				/*QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
				p.setBrush(Qt::transparent);
				p.drawRect(rect);*/
				
			}
		}
	}
	
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	p.setPen(Qt::NoPen);

	QRect robot = QRect((rows/2 - 2)*PIXEL_SIZE, (cols/2 -2)*PIXEL_SIZE, 4*PIXEL_SIZE, 4*PIXEL_SIZE);
	p.drawRect(robot);
}

void GradientMap::resetMax() {
	max = 0;
}

void GradientMap::resetMin() {
	min = 1000;
}

void GradientMap::setPixel(int x, int y, double value) {
	if (value < min) min = value;
	if (value > max) max = value;
	pixels[x][y] = value;
}

QPolygon GradientMap::getArrow(int x, int y, double angle) {
	QPolygon triangle;
	
	if ((angle >= 0 && angle < 22.5) || angle >= 337.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+5,y*PIXEL_SIZE+3, x*PIXEL_SIZE+2,y*PIXEL_SIZE+0, x*PIXEL_SIZE+2,y*PIXEL_SIZE+6);
	}
	else if (angle >= 22.5 && angle < 67.5) {
		triangle.setPoints(3, x*PIXEL_SIZE+5,y*PIXEL_SIZE+1, x*PIXEL_SIZE+5,y*PIXEL_SIZE+5, x*PIXEL_SIZE+1,y*PIXEL_SIZE+5);
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

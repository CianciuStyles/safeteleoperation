#include <QPainter>
#include <QColor>
#include "gradient_map.h"
#include "gradient_map.moc"
#include "settings.h"
#include <stdio.h>

double min = 1000;
double max = 0;

GradientMap::GradientMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k][w] = 0;
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

#define RGB_DIVISOR 10

void GradientMap::paintEvent(QPaintEvent *event) {
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	QPainter p(this);
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	
	int increment_red = (-min) / 255;
	if (increment_red <= 1) increment_red = 1;
	
	/* Init painters */
	if (painters_red.size() == 0) {
		for (int i = 0; i < 255/increment_red; i++) {
			painters_red.push_back(new QPainter(this));
			painters_red[i]->setBrush(QColor(255, increment_red*i, increment_red*i));
			painters_red[i]->setPen(Qt::gray);
		}
		painters_red.push_back(new QPainter(this));
		painters_red[255/increment_red]->setBrush(QColor(255, 255, 255));
		painters_red[255/increment_red]->setPen(Qt::gray);
	}
	
	int increment_green = max / 255;
	if (increment_green <= 1) increment_green = 1;
	
	if (painters_green.size() == 0) {
		for (int i = 0; i < 255/increment_green; i++) {
			painters_green.push_back(new QPainter(this));
			painters_green[i]->setBrush(QColor(i*increment_green, 255, i*increment_green));
			painters_green[i]->setPen(Qt::gray);
		}
		painters_green.push_back(new QPainter(this));
		painters_green[255/increment_green]->setBrush(QColor(255, 255, 255));
		painters_green[255/increment_green]->setPen(Qt::gray);
	}
	
	/* draw pixels with obstacles */
	p.setBrush(Qt::black);
	for (int k = 0; k < rows; k++) {
		for (int w = 0; w < cols; w++) {
			QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
			double c = pixels[k][w];

			int g = 0;
			
			if (c <= 0) {
				while(c > min) {
					g++;
					c += min / 255;
					if (g > 255/increment_red) {
						g = 255/increment_red;
						break;
					}
				}
				painters_red[g]->drawRect(rect);
			} else {
				while(c > 0) {
					g++;
					c -= max / 255;
					if (g > 255/increment_green) {
						g = 255/increment_green;
						break;
					}
				}
				if (g < 0 || g > painters_green.size() - 1) printf("Invalid index\n");
				painters_green[255/increment_green - g]->drawRect(rect);
			}

		}
	}
	
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	QRect robot = QRect((rows/2)*PIXEL_SIZE, (cols/2)*PIXEL_SIZE, 2*PIXEL_SIZE, 2*PIXEL_SIZE);
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

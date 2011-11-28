#include <QPainter>
#include "gradient_map.h"
#include "gradient_map.moc"
#include "settings.h"

GradientMap::GradientMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k][w] = false;
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

void GradientMap::paintEvent(QPaintEvent *event) {
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
	
	/* draw pixels with obstacles */
	p.setBrush(Qt::black);
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			if (pixels[k][w]) {
				QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
				p.drawRect(rect);
			}	
}

void GradientMap::drawPixel(int x, int y) {
	if (pixels[x][y])
		return;
	pixels[x][y] = true;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

void GradientMap::undrawPixel(int x, int y) {
	if (!pixels[x][y])
		return;
	pixels[x][y] = false;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

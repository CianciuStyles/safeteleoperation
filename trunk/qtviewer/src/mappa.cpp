#include <QPainter>
#include "mappa.h"
#include "mappa.moc"

Mappa::Mappa(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k] = {0};
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

void Mappa::paintEvent(QPaintEvent *event) {
	QPainter p(this);
	
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	p.setBrush(Qt::red);
	QRect robot = QRect((rows/2)*PIXEL_SIZE, (cols/2)*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	p.drawRect(robot);
	
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	
	p.setBrush(Qt::black);
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			if (pixels[k][w] == 1) {
				QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
				p.drawRect(rect);
			}
		
			
}

void Mappa::drawPixel(int x, int y) {
	if (pixels[x][y] == 1)
		return;
	pixels[x][y] = 1;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

void Mappa::undrawPixel(int x, int y) {
	if (pixels[x][y] == 0)
		return;
	pixels[x][y] = 0;
	QRect rect = QRect(x*PIXEL_SIZE, y*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
	repaint(rect);
}

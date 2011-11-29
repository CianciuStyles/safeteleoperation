#include <QPainter>
#include <QColor>
#include "gradient_map.h"
#include "gradient_map.moc"
#include "settings.h"

GradientMap::GradientMap(QWidget *parent, Qt::WFlags f) : QWidget(parent, f) {
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++)
			pixels[k][w] = 256;
	setPalette(QPalette(QColor(255, 255, 255)));
	
}

void GradientMap::paintEvent(QPaintEvent *event) {
	QPainter p(this);
	
	if (painters.size() == 0) {
                //printf("Init painters...\n");
                for (int i = 0; i < 255/30; i++) {
                        painters.push_back(new QPainter(this));
                        painters[i]->setBrush(QColor(30*i, 30*i, 30*i));
                        painters[i]->setPen(Qt::gray);
                }
                painters.push_back(new QPainter(this));
                painters[255/30]->setBrush(QColor(255, 255, 255));
                painters[255/30]->setPen(Qt::gray);
        }
	int rows = MAP_WIDTH/PIXEL_SIZE;
	int cols = MAP_HEIGHT/PIXEL_SIZE;
	
	/* draw the grid */
	p.setPen(Qt::gray);
	for (int i = 0; i <= MAP_WIDTH; i+=PIXEL_SIZE)
		p.drawLine (i, 0, i, MAP_HEIGHT);
	for (int j = 0; j <= MAP_HEIGHT; j+=PIXEL_SIZE)
		p.drawLine (0, j, MAP_WIDTH, j);
	
        
        QPainter pw(this);
        pw.setBrush(Qt::transparent);
        pw.setPen(Qt::gray);
        
	/* draw pixels with obstacles */
	p.setBrush(Qt::black);
	for (int k = 0; k < MAP_HEIGHT; k++)
		for (int w = 0; w < MAP_WIDTH; w++) {
			QRect rect = QRect(k*PIXEL_SIZE, w*PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
                        double c = pixels[k][w];
                        //if (c < 0) continue;
                        
                        int g = 0;
                        if (c == -1)
							g = 0;
						else {
							for (g = 60; c > 0; g+=30)
                                c-=0.41;
						}
                        
                        if (g == 0)
                                painters[0]->drawRect(rect);
                        else if (g >= 255)
                               painters[255/30 - 1]->drawRect(rect);
                        else {
                                painters[g/30]->drawRect(rect);
                        }
		}
	/* draw the robot in the center */
	p.setBrush(Qt::blue);
	QRect robot = QRect((rows/2)*PIXEL_SIZE, (cols/2)*PIXEL_SIZE, 2*PIXEL_SIZE, 2*PIXEL_SIZE);
	p.drawRect(robot);
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

void GradientMap::setPixel(int x, int y, double value) {
	if (value > 5)
		pixels[x][y] = -1;
	else pixels[x][y] = value;
}

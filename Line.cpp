#include "Line.h"

Line::Line(double a, double b, Point pt)
{
    this->pt_up = pt;
    this->pt_down = pt;
    this->exist = true;
    this->a = a;
    this->b = b;
    this->c = -(a * pt.x + b * pt.y);
}


Line::Line(double x_down, double y_down, double x_up, double y_up)
{
//    this->x1 = x1;
//    this->y1 = y1;
//    this->x2 = x2;
//    this->y2 = y2;
    this->pt_down.x = x_down;
    this->pt_down.y = y_down;
    this->pt_up.x = x_up;
    this->pt_up.y = y_up;
    this->exist = true;
    generateLine(this->pt_up, this->pt_down);
}

Line::Line(Point pt_down, Point pt_up)
{
    this->pt_up = pt_up;
    this->pt_down = pt_down;
    this->exist = true;
    generateLine(this->pt_up, this->pt_down);
}

Line::Line(const Line& ln)
{
//    this->x1 = ln.x1;
//    this->y1 = ln.y1;
//    this->x2 = ln.x2;
//    this->y2 = ln.y2;
    this->pt_down = ln.pt_down;
    this->pt_up = ln.pt_up;

    this->a = ln.a;
    this->b = ln.b;
    this->c = ln.c;
    this->exist = ln.exist;

}

bool Line::getk() const
{
    if(abs(a) > abs(b))
		return false;//|k|>1
	else
		return true;//|k|<=1
}

double Line::getX(double y) const
{
    double x;
	x = (c + b*y)/(-1 * a);
	return x;
}

double Line::getY(double x) const
{
    double y;
	y = (c + a*x)/(-1 *b);
	return y;
}

void Line::setLine(Point p1, Point p2)
{
    this->pt_down = p1;
    this->pt_up = p2;
    generateLine(this->pt_up, this->pt_down);
}

void Line::generateLine(Point p1, Point p2)
{
    double delta_x, delta_y;
	delta_x = p2.x - p1.x;
	delta_y = p2.y - p1.y;
	//cout<<"delta_x = "<<delta_x<<endl;
	//cout<<"delta_y = "<<delta_y<<endl;
	if(abs(delta_x) >= abs(delta_y))
	{
		double k = delta_y / delta_x;
		double t = p1.y - k * p1.x;
		//cout<<"y = "<<k<<" * x + "<<t<<endl;
		a = k;
		b = -1;
		c = t;
		//cout<<a<<" * x + "<<b<<" * y + "<<c<<" = 0"<<endl;
	}
	else
	{
        double kk = delta_x / delta_y;
        double tt = p1.x - kk * p1.y;
        //cout<<"x = "<<kk<<" *y + "<<tt<<endl;
        a = 1;
        b = -1 * kk;
        c = -1 * tt;
        //cout<<a<<" * x + "<<b<<" * y + "<<c<<" = 0"<<endl;
	}
}

double Line::getPointDist(Point pt, Point ugv_pt) const
{
    double tmp1, tmp2;
    double dist;
    tmp1 = a * pt.x + b * pt.y + c;
    tmp2 = a * ugv_pt.x + b * ugv_pt.y + c;
    if(tmp1 * tmp2 >= 0 )
    {
        dist = abs(tmp1) / sqrt(a * a + b * b);
    }
    else
        dist = -1 * ( abs(tmp1) / sqrt(a * a + b * b) );

    return dist;
}

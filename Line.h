#ifndef LINE_H_INCLUDED
#define LINE_H_INCLUDED

#include "Point.h"

class Line
{
public:
//functions
    Line(double a, double b, Point pt);

    Line(double x_down = 0, double y_down = 0, double x_up = 0, double y_up = 0);
    Line(Point pt_down, Point pt_up);
    Line(const Line& ln);

    //void generateLine(Point p1, Point p2);
    void generateLine(Point p1, Point p2);
    void setLine(Point p1, Point p2);
    double getPointDist(Point pt, Point ugv_pt) const;

    bool getk() const;    //if k<=1, return true, else return false;
    double getX(double y) const;
    double getY(double x) const;


//variables
    //double x1, y1;
    //double x2, y2;
    Point pt_down, pt_up;

    bool exist;
    double a, b, c; //ax + by + c = 0;
};

#endif // LINE_H_INCLUDED

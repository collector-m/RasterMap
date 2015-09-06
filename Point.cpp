#include "Point.h"

Point::Point(double x, double y, double dist)
{
    this->x = x;
    this->y = y;
    this->dist = dist;
    //cout << "Point's constructor called!" << endl;
    //cout << "x: " << x << " y:" << y << endl;
}

//Point::Point(const Point& p)
//{
//    x = p.x;
//    y = p.y;
//    dist = p.dist;
//}
//
//Point& Point::operator=(const Point& p)
//{
//    x = p.x;
//    y = p.y;
//    dist = p.dist;
//}
//
//Point::~Point()
//{
//
//}

void Point::setPoint(double x, double y)
{
    this->x = x;
    this->y = y;
    this->dist = sqrt(pow(x,2) + pow(y,2));
}

bool Point::IsInDetectBox() const
{
    if((this->x < detect_right) && (this->x > detect_left) && (this->y < detect_up) && (this->y > detect_down))
        return true;
    else
        return false;
}

bool Point::IsInDetectBox()
{
    if((this->x < detect_right) && (this->x > detect_left) && (this->y < detect_up) && (this->y > detect_down))
        return true;
    else
        return false;
}

bool Point::IsInHorDetectBox() const
{
    if((this->x < hor_detect_right) && (this->x > hor_detect_left) && (this->y < hor_detect_up) && (this->y > hor_detect_down))
        return true;
    else
        return false;
}

bool Point::IsInHorDetectBox()
{
    if((this->x < hor_detect_right) && (this->x > hor_detect_left) && (this->y < hor_detect_up) && (this->y > hor_detect_down))
        return true;
    else
        return false;
}

void Point::transToFrozen(module::Pose_t vehicle)
{
    double tx = this->x;
    double ty = this->y;
//    this->x = cos(vehicle.eulr) * tx + sin(vehicle.eulr) * ty + vehicle.x;
//    this->y = -sin(vehicle.eulr) * tx + cos(vehicle.eulr) * ty + vehicle.y;
    this->x = cos(vehicle.eulr) * tx + sin(vehicle.eulr) * ty;
    this->y = -sin(vehicle.eulr) * tx + cos(vehicle.eulr) * ty;
}

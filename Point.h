#ifndef POINT_H_INCLUDED
#define POINT_H_INCLUDED

#include <module/shm.h>
#include <module/module.h>

#include <cmath>
#include <iostream>

//const float detect_left = -1.2;
//const float detect_right = 1.2;
//const float detect_up = 80.0;
//const float detect_down = 0.0;

const float detect_left = -1.0;
const float detect_right = 1.0;
const float detect_up = 80.0;
const float detect_down = 0.0;

//const float detect_left = -10.0;
//const float detect_right = 10.0;
//const float detect_up = 80.0;
//const float detect_down = 0.0;

const float hor_detect_left = -20.0;
const float hor_detect_right = 20.0;
const float hor_detect_up = 20.0;
const float hor_detect_down = 0.0;

using namespace std;

class Point
{
public:
    Point(double x = 0, double y = 0, double dist = 0);

    //Point(const Point& p);
    //Point& operator=(const Point& p);
    //~Point();

    void setPoint(double x, double y);
    //void setDist(double dist);
    //void setX(double x);
    //void setY(double y);

    bool IsInDetectBox();
    bool IsInDetectBox() const;

    bool IsInHorDetectBox();
    bool IsInHorDetectBox() const;

    void transToFrozen(module::Pose_t vehicle);
    //double getDist() const;
    //double getX() const;
    //double getY() const;

    //variables
    double dist;
    double x, y;

private:

};

#endif // POINT_H_INCLUDED

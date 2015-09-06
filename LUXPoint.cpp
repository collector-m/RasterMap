#include "LUXPoint.h"

//LUXPoint::LUXPoint(int channel, double dist, double x, double y, double z, double hAngle, double vAngle)
//{
//    this->channel = channel;
//    this->z = z;
//    this->hAngle = hAngle, this->vAngle = vAngle;
//    this->x = -y;
//    this->y = x;
//}

LUXPoint::LUXPoint(int channel, double dist, double x, double y)
{
    this->channel = channel;
    this->x = -y;
    this->y = x;
}

bool LUXPoint::IsInFilterBox()
{
	if((this->x <= filter_right) && (this->x >= filter_left) && (this->y <= filter_front) && (this->y >= filter_back))
        return true;
    else
        return false;
}

bool LUXPoint::IsInFilterBox() const 
{
	if((this->x <= filter_right) && (this->x >= filter_left) && (this->y <= filter_front) && (this->y >= filter_back))
        return true;
    else
        return false;
}

//LUXPoint::LUXPoint(const LUXPoint& lux_pt):Point(lux_pt)
//{
//    channel = lux_pt.channel;
//    z = lux_pt.z;
//}
//
//
//
//LUXPoint::~LUXPoint()
//{
//
//}

//void LUXPoint::setChannel(int channel)
//{
//    this->channel = channel;
//}
//
//void LUXPoint::setZ(double z)
//{
//    this->z = z;
//}
//
//void LUXPoint::setAngle(double hAngle, double vAngle)
//{
//    this->hAngle = hAngle;
//    this->vAngle = vAngle;
//}
//
//void LUXPoint::setHorizonalAngle(double hAngle)
//{
//    this->hAngle = hAngle;
//}
//
//void LUXPoint::setVerticalAngle(double vAngle)
//{
//    this->vAngle = vAngle;
//}
//
//int LUXPoint::getChannel() const
//{
//    return channel;
//}
//
//double LUXPoint::getZ() const
//{
//    return z;
//}
//
//double LUXPoint::getHorizonalAngle() const
//{
//    return hAngle;
//}
//
//double LUXPoint::getVerticalAngle() const
//{
//    return vAngle;
//}

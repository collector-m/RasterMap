#include "HDLPoint.h"

HDLPoint::HDLPoint(int x, int y, int z, unsigned short dist, unsigned short rot, unsigned char i, unsigned char c)
{
    this->x = x / 1000.0;
    this->y = y / 1000.0;
    this->z = z / 1000.0;
    this->dist = dist / 1000.0;
    this->rot = rot;
    this->i = i;
    this->c = c;
}

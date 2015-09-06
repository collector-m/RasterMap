#ifndef PERIMETER_H
#define PERIMETER_H

#include "waypoint.h"

class Perimeter
{
public:
	Perimeter();
	~Perimeter();
	bool Read(stringstream &ifs);
	
public:
	string perimeter_id;
	int num_perimeterpoints;
	map < string,Waypoint > waypoints;
};

#endif // PERIMETER_H

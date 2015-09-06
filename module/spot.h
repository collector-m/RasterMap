#ifndef SPOT_H
#define SPOT_H

#include "waypoint.h"

class Spot
{
public:
	Spot();
	~Spot();
	bool Read(stringstream &ifs);
	
public:
	string spot_id;
	map <string, Waypoint > waypoints;
};

#endif // SPOT_H

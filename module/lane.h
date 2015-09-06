#ifndef LANE_H
#define LANE_H

#include "waypoint.h"
#include "charpoint.h"

class Lane
{
public:
	Lane();
	~Lane();
	bool Read(stringstream &ifs);
	
public:
	string lane_id;
	int num_waypoints;
	map < string,Waypoint > waypoints;
	int num_charpoints;
	map < string,Charpoint > charpoints;
};

#endif // LANE_H

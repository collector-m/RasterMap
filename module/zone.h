#ifndef ZONE_H
#define ZONE_H

#include "perimeter.h"
#include "spot.h"

class Zone
{
public:
	Zone();
	~Zone();
	bool Read(stringstream &ifs);
	
public:
	string zone_id;
	string zone_name;
	Perimeter perimeter;
	int num_spots;
	map < string,Spot > spots;
};

#endif // ZONE_H

#include "waypoint.h"

Waypoint::Waypoint()
{
	waypoint_id = "";
	lateral = 0;
	horizontal = 0;
}

Waypoint::~Waypoint()
{
}

bool Waypoint::Read(stringstream &ifs){
	string tmp;
	getline(ifs,tmp);
	stringstream ss;
	ss << tmp;
	ss >> waypoint_id >> lateral >> horizontal >> p1 >> p2 >> obj;
	return true;
}

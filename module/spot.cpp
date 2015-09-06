#include "spot.h"

Spot::Spot()
{
	spot_id = "";
	//Waypoint waypoint;
	//waypoints.push_back(waypoint);
}

Spot::~Spot()
{
}

bool Spot::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "spot") {
			ss >> spot_id;
			for(int i = 0; i < 2; i++) {
				Waypoint waypoint;
				if(!waypoint.Read(ifs))
					return false;
				//waypoints.push_back(waypoint);
				waypoints.insert(pair<string,Waypoint>(waypoint.waypoint_id,waypoint));
			}
		} else if(key == "end_spot")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}

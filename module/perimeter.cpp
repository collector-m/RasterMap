#include "perimeter.h"

Perimeter::Perimeter()
{
	perimeter_id = "";
	num_perimeterpoints = 0;
	//Waypoint waypoint;
	//waypoints.push_back(waypoint);
}

Perimeter::~Perimeter()
{
}

bool Perimeter::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "perimeter")
			ss >> perimeter_id;
		else if(key == "num_perimeterpoints") {
			ss >> num_perimeterpoints;
			for(int i = 0; i < num_perimeterpoints; i++) {
				Waypoint waypoint;
				if(!waypoint.Read(ifs))
					return false;
				//waypoints.push_back(waypoint);
				waypoints.insert(pair<string,Waypoint>(waypoint.waypoint_id,waypoint));
			}
		} else if(key == "end_perimeter")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}

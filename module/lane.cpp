#include "lane.h"

Lane::Lane()
{
	string lane_id = "";
	num_waypoints = 0;
	num_charpoints = 0;
	//Waypoint waypoint;
	//waypoints.push_back(waypoint);
	//Charpoint charpoint;
	//charpoints.push_back(charpoint);
}

Lane::~Lane()
{
}

bool Lane::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "lane")
			ss >> lane_id;
		else if(key == "num_waypoints")
			ss >> num_waypoints;
		else if(key == "waypoint") {
			for(int i = 0; i < num_waypoints; i++) {
				Waypoint waypoint;
				if(!waypoint.Read(ifs))
					return false;
				//waypoints.push_back(waypoint);
				waypoints.insert(pair<string,Waypoint>(waypoint.waypoint_id,waypoint));
			}
			ifs >> key;
			if(key == "end_waypoint")
				continue;
			cout << "Read waypoints Error:" << key << endl;
		} else if(key == "num_charpoints")
			ss >> num_charpoints;
		else if(key == "charpoint") {
			for(int i = 0; i < num_charpoints; i++) {
				Charpoint charpoint;
				if(!charpoint.Read(ifs))
					return false;
				//charpoints.push_back(charpoint);
				charpoints.insert(pair<string,Charpoint>(charpoint.charpoint_id,charpoint));
			}
			ifs >> key;
			if(key == "end_charpoint")
				continue;
			cout << "Read waypoints Error:" << key << endl;
		} else if(key == "end_lane")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}

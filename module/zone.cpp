#include "zone.h"

Zone::Zone()
{
	zone_id = "";
	zone_name = "";
	num_spots = 0;
	//Spot spot;
	//spots.push_back(spot);
}

Zone::~Zone()
{
}

bool Zone::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "zone")
			ss >> zone_id;
		else if(key == "zone_name")
			ss >> zone_name;
		else if(key == "perimeter") {
			ifs.seekg(lastpos,ios::beg);
			if(!perimeter.Read(ifs))
				return false;
		} else if(key == "num_spots")
			ss >> num_spots;
		else if(key == "spot") {
			ifs.seekg(lastpos,ios::beg);
			for(int i = 0; i < num_spots; i++) {
				Spot spot;
				if(!spot.Read(ifs))
					return false;
				//spots.push_back(spot);
				spots.insert(pair<string,Spot>(spot.spot_id,spot));
			}
		} else if(key == "end_zone")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}

#include "segment.h"

Segment::Segment()
{
	segment_id = "";
	segment_name = "";
	num_lanes = 0;
	//Lane lane;
	//lanes.push_back(lane);
}

Segment::~Segment()
{
}

bool Segment::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "segment")
			ss >> segment_id;
		else if(key == "segment_name")
			ss >> segment_name;
		else if(key == "num_lanes")
			ss >> num_lanes;
		else if(key == "lane") {
			ifs.seekg(lastpos,ios::beg);
			Lane lane;
			if(!lane.Read(ifs))
				return false;
			//lanes.push_back(lane);
			lanes.insert(pair<string,Lane>(lane.lane_id,lane));
		} else if(key == "end_segment")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}

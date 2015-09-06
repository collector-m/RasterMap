#include "rndf.h"

double RNDF::getx(int segment,int lane,int waypoint){
	string segment_id = int2str(segment);
	string lane_id = segment_id + "." + int2str(lane);
	string waypoint_id = lane_id + "." + int2str(waypoint);
	//cout << segment_id << endl;
	//cout << lane_id << endl;
	//cout << waypoint_id << endl;
	return segments[segment_id].lanes[lane_id].waypoints[waypoint_id].lateral;
}
double RNDF::gety(int segment,int lane,int waypoint){
	string segment_id = int2str(segment);
	string lane_id = segment_id + "." + int2str(lane);
	string waypoint_id = lane_id + "." + int2str(waypoint);
	return segments[segment_id].lanes[lane_id].waypoints[waypoint_id].horizontal;
}

bool RNDF::Read(stringstream &ifs)
{
	string line;
	int lastpos = 0;
	while(getline(ifs,line)) {
		int n = line.find("//",0);
		line = line.substr(0,n);
		stringstream ss;
		ss << line;

		string key;
		ss >> key;
		if(key == "RNDF_name")
			ss >> RNDF_name;
		else if(key == "num_segments")
			ss >> num_segments;
		else if(key == "num_zones")
			ss >> num_zones;
		else if(key == "format_version")
			ss >> format_version;
		else if(key == "creation_date")
			ss >> creation_date;
		else if(key == "segment") {
			ifs.seekg(lastpos,ios::beg);
			for(int i = 0; i < num_segments; i++) {
				Segment segment;
				if(!segment.Read(ifs))
					return false;
				//segments.push_back(segment);
				segments.insert(pair<string,Segment>(segment.segment_id,segment));
			}
		} else if(key == "zone") {
			ifs.seekg(lastpos,ios::beg);
			for(int i = 0; i < num_zones; i++) {
				Zone zone;
				if(!zone.Read(ifs))
					return false;
				//zones.push_back(zone);
				zones.insert(pair<string,Zone>(zone.zone_id,zone));
			}
		} else if(key == "connection") {
			Connection connection;
			ss >> connection.waypoint_id1 >> connection.waypoint_id2;
			connections.push_back(connection);
		} else if(key == "end_file")
			break;

		lastpos = ifs.tellg();
	}
	return true;
}
bool RNDF::ReadRNDF(string rndf_name)
{

	return true;
}


bool RNDF::Initialize()
{
	if(!ReadRNDF())
		exit(-1);
	return true;
}

void RNDF::Destroy()
{
}

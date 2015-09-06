#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
using namespace std;

class Waypoint
{
public:
	Waypoint();
	~Waypoint();
	bool Read(stringstream &ifs);
	
public:
	string waypoint_id;
	double lateral;
	double horizontal;
	int p1;
	int p2;
	int obj;

};

#endif // WAYPOINT_H

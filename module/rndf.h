#pragma once

#include "segment.h"
#include "zone.h"
#include "connection.h"

#include <sstream>
#include <stdlib.h>
#include <fstream>
#include <semaphore.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>    /* For mode constants */
#include <fcntl.h>       /* For O_* constants */
#include <sys/time.h>
#include <unistd.h>
#include <sys/shm.h>
#include <map>

class RNDF
{
private:
	/*
	 * class-static-variable lazy initialize
	 */
	bool ReadRNDF(string rndf_name="/home/denggroup/UGV/bin/RNDF.txt");
	
	void Destroy();


public:

	RNDF() {
		RNDF_name = "";
		num_segments = 0;
		num_zones = 0;
		format_version = "";
		creation_date = "";
		//Segment segment;
		//segments.push_back(segment);
		//Zone zone;
		//zones.push_back(zone);
		Connection connection;
		connections.push_back(connection);
	}
	~RNDF() {
		Destroy();
	}
	bool Read(stringstream &ifs);
	bool Initialize();
	double getx(int segment,int lane,int waypoint);
	double gety(int segment,int lane,int waypoint);
	string int2str(int n){
		stringstream ss;
		ss << n;
		string str;
		ss >> str;
		return str;
	}

	string RNDF_name;
	int num_segments;
	int num_zones;
	string format_version;
	string creation_date;

	map < string, Segment> segments;
	map < string, Zone> zones;
	vector <Connection> connections;

};

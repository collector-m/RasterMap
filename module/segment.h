#ifndef SEGMENT_H
#define SEGMENT_H

#include "lane.h"

class Segment
{
public:
	Segment();
	~Segment();
	bool Read(stringstream &ifs);

public:
	string segment_id;
	string segment_name;
	int num_lanes;
	map < string, Lane > lanes;

};

#endif // SEGMENT_H

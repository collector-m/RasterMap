#ifndef CHARPOINT_H
#define CHARPOINT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;


class Charpoint
{
public:
	Charpoint();
	~Charpoint();
	bool Read(stringstream &ifs);
	
public:
	string charpoint_id;
	double lateral;
	double horizontal;
	int type;

};

#endif // CHARPOINT_H

#ifndef CONNECTION_H
#define CONNECTION_H

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;




class Connection
{
public:
	Connection();
	~Connection();

public:
	string waypoint_id1;
	string waypoint_id2;

};

#endif // CONNECTION_H

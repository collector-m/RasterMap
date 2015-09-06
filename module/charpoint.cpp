#include "charpoint.h"

Charpoint::Charpoint()
{
	charpoint_id = "";
	lateral = 0;
	horizontal = 0;
	type = -1;
}

Charpoint::~Charpoint()
{
}

bool Charpoint::Read(stringstream &ifs)
{
	string tmp;
	getline(ifs,tmp);
	stringstream ss;
	ss << tmp;
	ss >> charpoint_id >> lateral >> horizontal >> type;
	return true;
}

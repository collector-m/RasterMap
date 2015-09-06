#ifndef RADARMAP_H
#define RADARMAP_H

#include "shm.h"
#include "module.h"

using namespace module;
using namespace module::shm;
#define RADARMAP_MANAGER RadarMap::GetInstance()

class RadarMap
{
private:
	static RadarMap m_instance;
	RadarMap();
	~RadarMap();
public:
	static RadarMap& GetInstance() {
		return m_instance;
	}
	bool getLocalMap(LocalMap_t &lm);

};

#endif // RADARMAP_H

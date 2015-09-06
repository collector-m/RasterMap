#include "radar_map.h"

RadarMap RadarMap::m_instance;

RadarMap::RadarMap()
{
}

RadarMap::~RadarMap()
{
}

bool RadarMap::getLocalMap(LocalMap_t &lm){
	/*
	MetaData_t metaData;
	metaData.type = MetaData_t::META_NAVIGATION;
	SHARED_OBJECTS.getMetaData(&metaData);
	MetaNavigation_t m_gps = metaData.value.v_navi; // m_gps.ENU[0], m_gps.ENV[1], m_gps.EULR[2]
	metaData.type = MetaData_t::META_LOCAL_NAVIGATION;
	SHARED_OBJECTS.getMetaData(&metaData);
	MetaLocalNavigation_t m_local_gps = metaData.value.v_navi_local; // m_local_gps.UGVtoFCF[0], m_local_gps.UGVtoFCF[1], m_local_gps.UGVtoFCF[2]
	*/
	
	PointT_t pt;
	Grid_t g;
	lm.carpos.num = 50;
	pt.x = 111;pt.y = 222;
	lm.carpos.pts[0]=pt;
	pt.x = 555;pt.y = 888;
	lm.carpos.pts[49] = pt;
	
	pt.x = 22;pt.y = 33;
	g.p = 123;
	g.gt = GridType_t(2);
	lm.LocalGridMap.insert(make_pair(pt,g));
	pt.x = 123;pt.y = 456;
	g.p = 33;
	g.gt = Boundary;
	lm.LocalGridMap.insert(make_pair(pt,g));
	return true;
}
#pragma once

#include "../types.h"
#include "../shm.h"
#include "../module.h"
#include <semaphore.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>    /* For mode constants */
#include <fcntl.h>       /* For O_* constants */
#include <sys/time.h>
#include <unistd.h>
#include <sys/shm.h>

namespace module
{

namespace shm
{
// shm
#define SHM_NAME "/ugv"

struct SharedDecision {
	Decision_t s_decision;
	bool isValid;
	pthread_spinlock_t lock;
};

struct SharedMetaData {
	MetaNavigation_t s_navi;
	MetaLocalNavigation_t s_navi_local;
	MetaLaserHdl_t s_laserHdl;
	MetaCameraBW_t s_cameraBW[3];
	MetaCameraC_t s_cameraC;
	MetaLUXPoints_t s_luxPts;
	MetaLUXObjects_t s_luxObjs;
	//MetaBeforeRadr s_beforeRadr;
	//MetaAfterRadrL s_afterRadrL;
	//MetaAfterRadrR s_afterRadrR;
	MetaMMWPoints s_mmwPts;
	MetaLUXPoints s_roadPts;
	MetaMap_t s_map;
	//MetaLUXObjects s_roadObjs;
//	MetaHokuyoPoints_t s_hokuyoPts[2];
//	MetaHokuyoObjects_t s_hokuyoObjs[2];
	bool isValid[13];
	bool isNew[13];
	struct timeval timestamps[13];
	pthread_spinlock_t locks[13];
};

struct SharedRecoData {
	RecoStopLine_t s_stopline;
	RecoTrafficSign_t s_ts;
	RecoTrafficLight_t s_tl;
	//RecoTrackLdAd_t s_trackLdAd;
	RecoSlowDown_t s_slowdown;
	RecoEmergency_t s_emergency;
	RecoSideObs_t s_sideObs;
	RecoLaneMark_t s_lanemark;
	RecoLaneMark_t s_curb;
	RecoLaneMark_t s_rail;
	RecoLaneMark_t s_hdl_lane;
	RecoLaneMark_t s_gps;
	RecoSCurve_t s_scurve;
	Pose_t s_guide_pt;
	RecoTrackLocalAndPreview_t s_trackLocalAndPreview;
	RecoLaneChangeLaneMarks_t s_laneChange_lanemarks;
	RecoObjectLaneMarks_t s_object_lanemarks;
	//RecoBeforeRadr s_beforeRadr;
	//RecoAfterRadrL s_afterRadrL;
	//RecoAfterRadrR s_afterRadrR;
	RecoCross_t s_cross;
	RecoUturn_t s_uturn;
	//RecoObjects_t s_objects;
	RecoGoodHDL_t s_goodhdl;
	RecoPathPoints_t s_pathpoints;
	//RecoObjects_2_t s_objects_2;
	RecoLocalPts_t s_localpts;
	RecoGlobalPts_t s_globalpts;
	RecoCollide_t s_collide;
	RecoVel_t s_vel;
	RecoFusionPts_t s_fusion_pts;
	RecoMapDBoundary_t s_map_dboundary;
	RecoSubMap_t s_sub_map;
	RecoOccupyGrids_t s_occupy_grids;
	RecoObstacleList_t s_obstacle_list;
	RecoLanePts_t s_lane_pts;
	RecoHDLPts_t s_hdl_pts;
	RecoCurbPts_t s_curb_pts;
	bool isNew[RecoData::RT_MAX - 1];
	bool isValid[RecoData::RT_MAX - 1];
	struct timeval timestamps[RecoData::RT_MAX - 1];
	double belief[RecoData::RT_MAX - 1];
	pthread_spinlock_t locks[RecoData::RT_MAX - 1];
};

struct SharedMarkers {
	MarkerNavi_t s_navi;
	MarkerIntersection_t s_intersection;
//	MarkerTaillightImage_t s_taillightImage;
//	MarkerHokuyoObs_t s_hokuyoobs;
//	MarkerVelocityDec_t s_velocityDec;
	MarkerVelocityDecLux_t s_velocityDecLux;
	MarkerLaneChange_t s_lanechange;
	MarkerClothoid_t s_clothoid;
	MarkerLaneChangeObstacle_t s_lanechangeobstacle;
//	MarkerLaneChangeSide_t s_lanechangeside;
//	MarkerObstacle_t s_obstacle;
	MarkerObstacleLux_t s_obstacleLux;
	MarkerTrafficLight_t s_tl;
//	MarkerInitSensor_t s_initsensor;
	MarkerRoadTracking_t s_roadtracking;
	MarkerRefreeze_t s_refreeze_cam;
	MarkerRefreeze_t s_refreeze_lux;
	MarkerRefreeze_t s_refreeze_hdl;
	MarkerRefreeze_t s_refreeze_hdll;
	MarkerCamLost s_cam_lost;
	MarkerCameraTmp_t s_camera_tmp;
	MarkerGoodHDL_t s_goodhdl;
	MarkerCanLaneChange_t s_can_lanechange;
	MarkerLaneChangeOver_t s_lanechange_over;
	MarkerActivePoint_t s_activepoint;
	MarkerActivePoint_t s_activepointinrndf;
	MarkerActivePoint_t s_activepointinroute;
	MarkerNavStopline_t s_nav_stopline;
	MarkerRNDF_t s_rndf;
	MarkerRefreeze_t s_refreeze_map;
	MarkerRefreeze_t s_refreeze_object;
	MarkerInSturn_t s_sturn;
	MarkerSturnDec_t s_sturndec;
	MarkerDynamicObj_t s_dynamic_obj;
	MarkerDynamicObj_t s_dynamic_obj_1;
	MarkerDynamicObjOver_t s_dynamic_obj_over;
	MarkerDynamicObjOver_t s_dynamic_obj_over_1;
	MarkerDevide_t s_devide;
	MarkerIsUturn_t s_isuturn;
	MarkerStartObj_t s_start_obj;
	//MarkerObActivated_t s_ob_activated;
	bool isValid[MarkerData::MARKER_MAX - 1];
	pthread_spinlock_t locks[MarkerData::MARKER_MAX - 1];
};

struct SharedSmartMarkers {
	bool isValid[SHM_SMART_MARKER_NUM];
	bool isNew[SHM_SMART_MARKER_NUM];
	pthread_spinlock_t locks[10];
};

struct SharedMemory {
	struct SharedDecision shm_decision;
	struct SharedMetaData shm_metaData;
	struct SharedRecoData shm_recoData;
	struct SharedMarkers shm_markers;
	struct SharedSmartMarkers shm_smartMarkers;
};

class SharedObjectsImpl : public SharedObjects
{
protected:
	struct SharedMemory* m_addr;

	static inline void Lock(pthread_spinlock_t* lock);
	static inline void Unlock(pthread_spinlock_t* lock);

public:
	bool GetDecision(Decision_t* decision);
	bool SetDecision(const Decision_t& decision);

	bool SetMetaData(const MetaData_t& data, int index = 0);
	bool GetMetaData(MetaData* data, int index = 0, bool isGettingNewData = false);

	bool SetRecoData(const RecoData_t& data);
	bool GetRecoData(RecoData_t* data, bool isGettingNewData = false);

	bool SetMarker(const MarkerData_t& data);
	bool GetMarker(MarkerData_t* data);

	bool SetSmartMarker(int index);
	bool GetSmartMarker(int index, bool isNew = false);
};

}
}

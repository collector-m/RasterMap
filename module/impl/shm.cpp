#include "shm.h"
#include "../log.h"
#include <signal.h>

using namespace module::logger;

namespace module
{

namespace shm
{

// Decision
bool SharedObjectsImpl::GetDecision(Decision_t* decision)
{
	if(m_addr == NULL || decision == NULL)
		return false;

	bool result = true;
	Lock(&m_addr->shm_decision.lock);
	if(m_addr->shm_decision.isValid)
		memcpy(decision, &m_addr->shm_decision.s_decision, sizeof(Decision_t));
	else
		result = false;
	Unlock(&m_addr->shm_decision.lock);
	return result;
}

bool SharedObjectsImpl::SetDecision(const Decision_t& decision)
{
	if(m_addr == NULL)
		return false;

	Lock(&m_addr->shm_decision.lock);
	memcpy(&m_addr->shm_decision.s_decision, &decision, sizeof(Decision_t));
	m_addr->shm_decision.isValid = true;
	Unlock(&m_addr->shm_decision.lock);
	return true;
}

// Meta Data
bool SharedObjectsImpl::SetMetaData(const MetaData_t& data, int index)
{
	if(m_addr == NULL)
		return false;

	switch(data.type) {
	case MetaData::META_NAVIGATION:
		Lock(&m_addr->shm_metaData.locks[0]);
		memcpy(&m_addr->shm_metaData.s_navi, &data.value.v_navi, sizeof(MetaNavigation_t));
		m_addr->shm_metaData.isValid[0] = true;
		m_addr->shm_metaData.isNew[0] = true;
		m_addr->shm_metaData.timestamps[0] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[0]);
		return true;
	case MetaData::META_LASER_HDL:
		Lock(&m_addr->shm_metaData.locks[1]);
		memcpy(&m_addr->shm_metaData.s_laserHdl, &data.value.v_laserHdl, sizeof(MetaLaserHdl_t));
		m_addr->shm_metaData.isValid[1] = true;
		m_addr->shm_metaData.isNew[1] = true;
		m_addr->shm_metaData.timestamps[1] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[1]);
		return true;
	case MetaData::META_CAMERA_BW:
		if(index >= 0 && index <= 2) {
			Lock(&m_addr->shm_metaData.locks[2 + index]);
			memcpy(&m_addr->shm_metaData.s_cameraBW[index], &data.value.v_cameraBW, sizeof(MetaCameraBW_t));
			m_addr->shm_metaData.isValid[2 + index] = true;
			m_addr->shm_metaData.isNew[2 + index] = true;
			m_addr->shm_metaData.timestamps[2 + index] = data.timestamp;
			Unlock(&m_addr->shm_metaData.locks[2 + index]);
			return true;
		} else
			return false;
	case MetaData::META_CAMERA_C:
		Lock(&m_addr->shm_metaData.locks[5]);
		memcpy(&m_addr->shm_metaData.s_cameraC, &data.value.v_cameraC, sizeof(MetaCameraC_t));
		m_addr->shm_metaData.isValid[5] = true;
		m_addr->shm_metaData.isNew[5] = true;
		m_addr->shm_metaData.timestamps[5] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[5]);
		return true;
	case MetaData::META_LUX_POINTS:
		Lock(&m_addr->shm_metaData.locks[6]);
		memcpy(&m_addr->shm_metaData.s_luxPts, &data.value.v_luxPts, sizeof(MetaLUXPoints_t));
		m_addr->shm_metaData.isValid[6] = true;
		m_addr->shm_metaData.isNew[6] = true;
		m_addr->shm_metaData.timestamps[6] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[6]);
		return true;
	case MetaData::META_LUX_OBJECTS:
		Lock(&m_addr->shm_metaData.locks[7]);
		memcpy(&m_addr->shm_metaData.s_luxObjs, &data.value.v_luxObjs, sizeof(MetaLUXObjects_t));
		m_addr->shm_metaData.isValid[7] = true;
		m_addr->shm_metaData.isNew[7] = true;
		m_addr->shm_metaData.timestamps[7] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[7]);
		return true;
	case MetaData::META_LOCAL_NAVIGATION:
		Lock(&m_addr->shm_metaData.locks[8]);
		memcpy(&m_addr->shm_metaData.s_navi_local, &data.value.v_navi_local, sizeof(MetaLocalNavigation_t));
		m_addr->shm_metaData.isValid[8] = true;
		m_addr->shm_metaData.isNew[8] = true;
		m_addr->shm_metaData.timestamps[8] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[8]);
		return true;
	case MetaData::META_MMW_POINTS:
		Lock(&m_addr->shm_metaData.locks[9]);
		memcpy(&m_addr->shm_metaData.s_mmwPts, &data.value.v_mmwPts, sizeof(MetaMMWPoints_t));
		m_addr->shm_metaData.isValid[9] = true;
		m_addr->shm_metaData.isNew[9] = true;
		m_addr->shm_metaData.timestamps[9] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[9]);
		return true;
	case MetaData_t::META_ROAD_POINTS:
		Lock(&m_addr->shm_metaData.locks[10]);
		memcpy(&m_addr->shm_metaData.s_roadPts, &data.value.v_roadPts, sizeof(MetaLUXPoints_t));
		m_addr->shm_metaData.isValid[10] = true;
		m_addr->shm_metaData.isNew[10] = true;
		m_addr->shm_metaData.timestamps[10] = data.timestamp;
		Unlock(&m_addr->shm_metaData.locks[10]);
		return true;
//	case MetaData::META_ROAD_OBJECTS:
//		Lock(&m_addr->shm_metaData.locks[14]);
//		memcpy(&m_addr->shm_metaData.s_roadObjs, &data.value.v_roadObjs, sizeof(MetaLUXObjects_t));
//		m_addr->shm_metaData.isValid[14] = true;
//		m_addr->shm_metaData.isNew[14] = true;
//		m_addr->shm_metaData.timestamps[14] = data.timestamp;
//		Unlock(&m_addr->shm_metaData.locks[14]);
//		return true;
//	case MetaData::META_HOKUYO_POINTS:
//		if(index >= 0 && index <= 1)
//		{
//			Lock(&m_addr->shm_metaData.locks[10 + index]);
//			memcpy(&m_addr->shm_metaData.s_hokuyoPts[index], &data.value.v_hokuyoPts, sizeof(MetaHokuyoPoints_t));
//			m_addr->shm_metaData.isValid[10 + index] = true;
//			m_addr->shm_metaData.isNew[10 + index] = true;
//			m_addr->shm_metaData.timestamps[10 + index] = data.timestamp;
//			Unlock(&m_addr->shm_metaData.locks[10 + index]);
//			return true;
//		}
//		else
//			return false;
//	case MetaData::META_HOKUYO_OBJECTS:
//		if(index >= 0 && index <= 1)
//		{
//			Lock(&m_addr->shm_metaData.locks[12 + index]);
//			memcpy(&m_addr->shm_metaData.s_hokuyoObjs[index], &data.value.v_hokuyoObjs, sizeof(MetaHokuyoObjects_t));
//			m_addr->shm_metaData.isValid[12 + index] = true;
//			m_addr->shm_metaData.isNew[12 + index] = true;
//			m_addr->shm_metaData.timestamps[12 + index] = data.timestamp;
//			Unlock(&m_addr->shm_metaData.locks[12 + index]);
//			return true;
//		}
//		else
//			return false;
	default:
		return false;
	};
}

bool SharedObjectsImpl::GetMetaData(MetaData_t* data, int index, bool isGettingNewData)
{
	if(m_addr == NULL || data == NULL)
		return false;

	bool result = false;

	switch(data->type) {
	case MetaData::META_NAVIGATION:
		Lock(&m_addr->shm_metaData.locks[0]);
		if(m_addr->shm_metaData.isValid[0]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[0]))) {
			memcpy(&data->value.v_navi, &m_addr->shm_metaData.s_navi, sizeof(MetaNavigation_t));
			data->timestamp = m_addr->shm_metaData.timestamps[0];

			if(isGettingNewData && m_addr->shm_metaData.isNew[0])
				m_addr->shm_metaData.isNew[0] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[0]);
		break;
	case MetaData::META_LASER_HDL:
		Lock(&m_addr->shm_metaData.locks[1]);
		if(m_addr->shm_metaData.isValid[1]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[1]))) {
			memcpy(&data->value.v_laserHdl, &m_addr->shm_metaData.s_laserHdl, sizeof(MetaLaserHdl_t));
			data->timestamp = m_addr->shm_metaData.timestamps[1];

			if(isGettingNewData && m_addr->shm_metaData.isNew[1])
				m_addr->shm_metaData.isNew[1] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[1]);
		break;
	case MetaData::META_CAMERA_BW:
		if(index >= 0 && index <= 2) {
			Lock(&m_addr->shm_metaData.locks[2 + index]);
			if(m_addr->shm_metaData.isValid[2 + index]
			   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[2 + index]))) {
				memcpy(&data->value.v_cameraBW, &m_addr->shm_metaData.s_cameraBW[index], sizeof(MetaCameraBW_t));
				data->timestamp = m_addr->shm_metaData.timestamps[2 + index];

				if(isGettingNewData && m_addr->shm_metaData.isNew[2 + index])
					m_addr->shm_metaData.isNew[2 + index] = false;

				result = true;
			}
			Unlock(&m_addr->shm_metaData.locks[2 + index]);
		}
		break;
	case MetaData::META_CAMERA_C:
		Lock(&m_addr->shm_metaData.locks[5]);
		if(m_addr->shm_metaData.isValid[5]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[5]))) {
			memcpy(&data->value.v_cameraC, &m_addr->shm_metaData.s_cameraC, sizeof(MetaCameraC_t));
			data->timestamp = m_addr->shm_metaData.timestamps[5];

			if(isGettingNewData && m_addr->shm_metaData.isNew[5])
				m_addr->shm_metaData.isNew[5] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[5]);
		break;
	case MetaData::META_LUX_POINTS:
		Lock(&m_addr->shm_metaData.locks[6]);
		if(m_addr->shm_metaData.isValid[6]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[6]))) {
			memcpy(&data->value.v_luxPts, &m_addr->shm_metaData.s_luxPts, sizeof(MetaLUXPoints_t));
			data->timestamp = m_addr->shm_metaData.timestamps[6];

			if(isGettingNewData && m_addr->shm_metaData.isNew[6])
				m_addr->shm_metaData.isNew[6] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[6]);
		break;
	case MetaData::META_LUX_OBJECTS:
		Lock(&m_addr->shm_metaData.locks[7]);
		if(m_addr->shm_metaData.isValid[7]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[7]))) {
			memcpy(&data->value.v_luxObjs, &m_addr->shm_metaData.s_luxObjs, sizeof(MetaLUXObjects_t));
			data->timestamp = m_addr->shm_metaData.timestamps[7];
			if(isGettingNewData && m_addr->shm_metaData.isNew[7])
				m_addr->shm_metaData.isNew[7] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[7]);
		break;
	case MetaData::META_LOCAL_NAVIGATION:
		Lock(&m_addr->shm_metaData.locks[8]);
		if(m_addr->shm_metaData.isValid[8]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[8]))) {
			memcpy(&data->value.v_navi_local, &m_addr->shm_metaData.s_navi_local, sizeof(MetaLocalNavigation_t));
			data->timestamp = m_addr->shm_metaData.timestamps[8];
			if(isGettingNewData && m_addr->shm_metaData.isNew[8])
				m_addr->shm_metaData.isNew[8] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[8]);
		break;
	case MetaData::META_MMW_POINTS:
		Lock(&m_addr->shm_metaData.locks[9]);
		if(m_addr->shm_metaData.isValid[9]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[9]))) {
			memcpy(&data->value.v_mmwPts, &m_addr->shm_metaData.s_mmwPts, sizeof(MetaMMWPoints_t));
			data->timestamp = m_addr->shm_metaData.timestamps[9];
			if(isGettingNewData && m_addr->shm_metaData.isNew[9])
				m_addr->shm_metaData.isNew[9] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[9]);
		break;
	case MetaData::META_ROAD_POINTS:
		Lock(&m_addr->shm_metaData.locks[10]);
		if(m_addr->shm_metaData.isValid[10]
		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[10]))) {
			memcpy(&data->value.v_roadPts, &m_addr->shm_metaData.s_roadPts, sizeof(MetaLUXPoints_t));
			data->timestamp = m_addr->shm_metaData.timestamps[10];
			if(isGettingNewData && m_addr->shm_metaData.isNew[10])
				m_addr->shm_metaData.isNew[10] = false;

			result = true;
		}
		Unlock(&m_addr->shm_metaData.locks[10]);
		break;
//	case MetaData::META_ROAD_OBJECTS:
//		Lock(&m_addr->shm_metaData.locks[14]);
//		if(m_addr->shm_metaData.isValid[14]
//		   && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[14]))) {
//			memcpy(&data->value.v_roadObjs, &m_addr->shm_metaData.s_roadObjs, sizeof(MetaLUXObjects_t));
//			data->timestamp = m_addr->shm_metaData.timestamps[14];
//			if(isGettingNewData && m_addr->shm_metaData.isNew[14])
//				m_addr->shm_metaData.isNew[14] = false;
//
//			result = true;
//		}
//		Unlock(&m_addr->shm_metaData.locks[14]);
//		break;
//	case MetaData::META_HOKUYO_POINTS:
//		if(index >= 0 && index <= 1)
//		{
//			Lock(&m_addr->shm_metaData.locks[10 + index]);
//			if(m_addr->shm_metaData.isValid[10 + index]
//			        && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[10 + index])))
//			{
//				memcpy(&data->value.v_hokuyoPts, &m_addr->shm_metaData.s_hokuyoPts[index], sizeof(MetaHokuyoPoints_t));
//				data->timestamp = m_addr->shm_metaData.timestamps[10 + index];
//
//				if(isGettingNewData && m_addr->shm_metaData.isNew[10 + index])
//					m_addr->shm_metaData.isNew[10 + index] = false;
//
//				result = true;
//			}
//			Unlock(&m_addr->shm_metaData.locks[10 + index]);
//		}
//		break;
//	case MetaData::META_HOKUYO_OBJECTS:
//		if(index >= 0 && index <= 1)
//		{
//			Lock(&m_addr->shm_metaData.locks[12 + index]);
//			if(m_addr->shm_metaData.isValid[12 + index]
//			        && (!isGettingNewData || (isGettingNewData && m_addr->shm_metaData.isNew[12 + index])))
//			{
//				memcpy(&data->value.v_hokuyoObjs, &m_addr->shm_metaData.s_hokuyoObjs[index], sizeof(MetaHokuyoObjects_t));
//				data->timestamp = m_addr->shm_metaData.timestamps[12 + index];
//
//				if(isGettingNewData && m_addr->shm_metaData.isNew[12 + index])
//					m_addr->shm_metaData.isNew[12 + index] = false;
//
//				result = true;
//			}
//			Unlock(&m_addr->shm_metaData.locks[12 + index]);
//		}
//		break;
	default:
		break;
	};
	return result;
}

// RecoData
bool SharedObjectsImpl::SetRecoData(const RecoData_t& data)
{
	if(m_addr == NULL || data.type <= 0 || data.type >= RecoData::RT_MAX)
		return false;

	int index = data.type - 1;
	Lock(&m_addr->shm_recoData.locks[index]);

	switch(data.type) {
	case RecoData::RT_STOPLINE:
		memcpy(&m_addr->shm_recoData.s_stopline, &data.value.v_stopline, sizeof(RecoStopLine_t));
		break;
	case RecoData::RT_TRAFFICSIGN:
		memcpy(&m_addr->shm_recoData.s_ts, &data.value.v_ts, sizeof(RecoTrafficSign_t));
		break;
	case RecoData::RT_TRAFFICLIGHT:
		memcpy(&m_addr->shm_recoData.s_tl, &data.value.v_tl, sizeof(RecoTrafficLight_t));
		break;
		//case RecoData::RT_TRACK_LDAD:
		//memcpy(&m_addr->shm_recoData.s_trackLdAd, &data.value.v_trackLdAd, sizeof(RecoTrackLdAd_t));
		//break;
	case RecoData::RT_TRACK_LOCALANDPREVIEW:
		memcpy(&m_addr->shm_recoData.s_trackLocalAndPreview, &data.value.v_trackLocalAndPreview, sizeof(RecoTrackLocalAndPreview_t));
		break;
	case RecoData::RT_SLOWDOWN:
		memcpy(&m_addr->shm_recoData.s_slowdown, &data.value.v_slowdown, sizeof(RecoSlowDown_t));
		break;
	case RecoData::RT_EMERGENCY:
		memcpy(&m_addr->shm_recoData.s_emergency, &data.value.v_emergency, sizeof(RecoEmergency_t));
		break;
	case RecoData::RT_SIDEOBS:
		memcpy(&m_addr->shm_recoData.s_sideObs, &data.value.v_sideObs, sizeof(RecoSideObs_t));
		break;
	case RecoData::RT_LANEMARK:
		memcpy(&m_addr->shm_recoData.s_lanemark, &data.value.v_lanemark, sizeof(RecoLaneMark_t));
		break;
	case RecoData::RT_CURB:
		memcpy(&m_addr->shm_recoData.s_curb, &data.value.v_curb, sizeof(RecoLaneMark_t));
		break;
	case RecoData::RT_RAIL:
		memcpy(&m_addr->shm_recoData.s_rail, &data.value.v_rail, sizeof(RecoLaneMark_t));
		break;
	case RecoData::RT_HDL_LANE:
		memcpy(&m_addr->shm_recoData.s_hdl_lane, &data.value.v_hdl_lane, sizeof(RecoLaneMark_t));
		break;
	case RecoData::RT_GPS:
		memcpy(&m_addr->shm_recoData.s_gps, &data.value.v_gps, sizeof(RecoLaneMark_t));
		break;
	case RecoData::RT_SCURVE:
		memcpy(&m_addr->shm_recoData.s_scurve, &data.value.v_scurve, sizeof(RecoSCurve_t));
		break;
	case RecoData::RT_GUIDE_PT:
		memcpy(&m_addr->shm_recoData.s_guide_pt, &data.value.v_guide_pt, sizeof(Pose_t));
		break;
	case RecoData::RT_LANECHANGE_LANEMARKS:
		memcpy(&m_addr->shm_recoData.s_laneChange_lanemarks, &data.value.v_laneChange_lanemarks, sizeof(RecoLaneChangeLaneMarks_t));
		break;
	case RecoData::RT_OBJECT_LANEMARKS:
		memcpy(&m_addr->shm_recoData.s_object_lanemarks, &data.value.v_object_lanemarks, sizeof(RecoObjectLaneMarks_t));
		break;
	case RecoData::RT_CROSS:
		memcpy(&m_addr->shm_recoData.s_cross, &data.value.v_cross, sizeof(RecoCross_t));
		break;
	case RecoData::RT_UTURN:
		memcpy(&m_addr->shm_recoData.s_uturn, &data.value.v_uturn, sizeof(RecoUturn_t));
		break;
		/*case RecoData::RT_OBJECTS:
			memcpy(&m_addr->shm_recoData.s_objects, &data.value.v_objects, sizeof(RecoObjects_t));
			break;*/
	case RecoData::RT_GOODHDL:
		memcpy(&m_addr->shm_recoData.s_goodhdl, &data.value.v_goodhdl, sizeof(RecoGoodHDL_t));
		break;
	case RecoData::RT_PathPoints:
		memcpy(&m_addr->shm_recoData.s_pathpoints, &data.value.v_pathpoints, sizeof(RecoPathPoints_t));
		break;
		/*case RecoData::RT_OBJECTS_2:
			memcpy(&m_addr->shm_recoData.s_objects_2, &data.value.v_objects_2, sizeof(RecoObjects_2_t));
			break;*/
	case RecoData::RT_LOCALPTS:
		memcpy(&m_addr->shm_recoData.s_localpts, &data.value.v_localpts, sizeof(RecoLocalPts_t));
		break;
	case RecoData::RT_GLOBALPTS:
		memcpy(&m_addr->shm_recoData.s_globalpts, &data.value.v_globalpts, sizeof(RecoGlobalPts_t));
		break;
	case RecoData::RT_COLLIDE:
		memcpy(&m_addr->shm_recoData.s_collide, &data.value.v_collide, sizeof(RecoCollide_t));
		break;
	case RecoData::RT_VEL:
		memcpy(&m_addr->shm_recoData.s_vel, &data.value.v_vel, sizeof(RecoVel_t));
		break;
	case RecoData::RT_FUSION_PTS:
		memcpy(&m_addr->shm_recoData.s_fusion_pts, &data.value.v_fusion_pts, sizeof(RecoPts_t));
		break;
	case RecoData::RT_MAP_DBOUNDARY:
		memcpy(&m_addr->shm_recoData.s_map_dboundary, &data.value.v_map_dboundary, sizeof(RecoMapDBoundary_t));
		break;
	case RecoData::RT_SUB_MAP:
		memcpy(&m_addr->shm_recoData.s_sub_map, &data.value.v_sub_map, sizeof(RecoSubMap_t));
		break;
	case RecoData::RT_OCCUPY_GRIDS:
		memcpy(&m_addr->shm_recoData.s_occupy_grids, &data.value.v_occupy_grids, sizeof(RecoOccupyGrids_t));
		break;
	case RecoData::RT_OBSTACLE_LIST:
		memcpy(&m_addr->shm_recoData.s_obstacle_list, &data.value.v_obstacle_list, sizeof(RecoObstacleList_t));
		break;
	case RecoData::RT_LANE_PTS:
		memcpy(&m_addr->shm_recoData.s_lane_pts, &data.value.v_lane_pts, sizeof(RecoPts_t));
		break;
	case RecoData::RT_HDL_PTS:
		memcpy(&m_addr->shm_recoData.s_hdl_pts, &data.value.v_hdl_pts, sizeof(RecoPts_t));
		break;
	case RecoData::RT_CURB_PTS:
		memcpy(&m_addr->shm_recoData.s_curb_pts, &data.value.v_curb_pts, sizeof(RecoPts_t));
		break;
	default:
		break;
	};
	m_addr->shm_recoData.isValid[index] = true;
	m_addr->shm_recoData.isNew[index] = true;
	if(data.timestamp.tv_sec == 0 && data.timestamp.tv_usec == 0)
		gettimeofday(&m_addr->shm_recoData.timestamps[index], NULL);
	else
		m_addr->shm_recoData.timestamps[index] = data.timestamp;
	m_addr->shm_recoData.belief[index] = data.belief;
	Unlock(&m_addr->shm_recoData.locks[index]);
	return true;
}

bool SharedObjectsImpl::GetRecoData(RecoData_t* data, bool isGettingNewData)
{
	if(m_addr == NULL || data == NULL || data->type <= 0 || data->type >= RecoData::RT_MAX)
		return false;

	int index = data->type - 1;
	bool result = false;

	Lock(&m_addr->shm_recoData.locks[index]);
	if(m_addr->shm_recoData.isValid[index]
	   && (!isGettingNewData || (isGettingNewData && m_addr->shm_recoData.isNew[index]))) {
		switch(data->type) {
		case RecoData::RT_STOPLINE:
			memcpy(&data->value.v_stopline, &m_addr->shm_recoData.s_stopline, sizeof(RecoStopLine_t));
			break;
		case RecoData::RT_TRAFFICSIGN:
			memcpy(&data->value.v_ts, &m_addr->shm_recoData.s_ts, sizeof(RecoTrafficSign_t));
			break;
		case RecoData::RT_TRAFFICLIGHT:
			memcpy(&data->value.v_tl, &m_addr->shm_recoData.s_tl, sizeof(RecoTrafficLight_t));
			break;
			//case RecoData::RT_TRACK_LDAD:
			//memcpy(&data->value.v_trackLdAd, &m_addr->shm_recoData.s_trackLdAd, sizeof(RecoTrackLdAd_t));
			//break;
		case RecoData::RT_TRACK_LOCALANDPREVIEW:
			memcpy(&data->value.v_trackLocalAndPreview, &m_addr->shm_recoData.s_trackLocalAndPreview, sizeof(RecoTrackLocalAndPreview_t));
			break;
		case RecoData::RT_SLOWDOWN:
			memcpy(&data->value.v_slowdown, &m_addr->shm_recoData.s_slowdown, sizeof(RecoSlowDown_t));
			break;
		case RecoData::RT_EMERGENCY:
			memcpy(&data->value.v_emergency, &m_addr->shm_recoData.s_emergency, sizeof(RecoEmergency_t));
			break;
		case RecoData::RT_SIDEOBS:
			memcpy(&data->value.v_sideObs, &m_addr->shm_recoData.s_sideObs, sizeof(RecoSideObs_t));
			break;
		case RecoData::RT_LANEMARK:
			memcpy(&data->value.v_lanemark, &m_addr->shm_recoData.s_lanemark, sizeof(RecoLaneMark_t));
			break;
		case RecoData::RT_CURB:
			memcpy(&data->value.v_curb, &m_addr->shm_recoData.s_curb, sizeof(RecoLaneMark_t));
			break;
		case RecoData::RT_RAIL:
			memcpy(&data->value.v_rail, &m_addr->shm_recoData.s_rail, sizeof(RecoLaneMark_t));
			break;
		case RecoData::RT_HDL_LANE:
			memcpy(&data->value.v_hdl_lane, &m_addr->shm_recoData.s_hdl_lane, sizeof(RecoLaneMark_t));
			break;
		case RecoData::RT_GPS:
			memcpy(&data->value.v_gps, &m_addr->shm_recoData.s_gps, sizeof(RecoLaneMark_t));
			break;
		case RecoData::RT_SCURVE:
			memcpy(&data->value.v_scurve, &m_addr->shm_recoData.s_scurve, sizeof(RecoSCurve_t));
			break;
		case RecoData::RT_GUIDE_PT:
			memcpy(&data->value.v_guide_pt, &m_addr->shm_recoData.s_guide_pt, sizeof(Pose_t));
			break;
		case RecoData::RT_LANECHANGE_LANEMARKS:
			memcpy(&data->value.v_laneChange_lanemarks, &m_addr->shm_recoData.s_laneChange_lanemarks, sizeof(RecoLaneChangeLaneMarks_t));
			break;
		case RecoData::RT_OBJECT_LANEMARKS:
			memcpy(&data->value.v_object_lanemarks, &m_addr->shm_recoData.s_object_lanemarks, sizeof(RecoObjectLaneMarks_t));
			break;
		case RecoData::RT_CROSS:
			memcpy(&data->value.v_cross, &m_addr->shm_recoData.s_cross, sizeof(RecoCross_t));
			break;
		case RecoData::RT_UTURN:
			memcpy(&data->value.v_uturn, &m_addr->shm_recoData.s_uturn, sizeof(RecoUturn_t));
			break;
			/*case RecoData::RT_OBJECTS:
				memcpy(&data->value.v_objects, &m_addr->shm_recoData.s_objects, sizeof(RecoObjects_t));
				break;*/
		case RecoData::RT_GOODHDL:
			memcpy(&data->value.v_goodhdl, &m_addr->shm_recoData.s_goodhdl, sizeof(RecoGoodHDL_t));
			break;
		case RecoData::RT_PathPoints:
			memcpy(&data->value.v_pathpoints, &m_addr->shm_recoData.s_pathpoints, sizeof(RecoPathPoints_t));
			break;
			/*case RecoData::RT_OBJECTS_2:
				memcpy(&data->value.v_objects_2, &m_addr->shm_recoData.s_objects_2, sizeof(RecoObjects_2_t));
				break;*/
		case RecoData::RT_LOCALPTS:
			memcpy(&data->value.v_localpts, &m_addr->shm_recoData.s_localpts, sizeof(RecoLocalPts_t));
			break;
		case RecoData::RT_GLOBALPTS:
			memcpy(&data->value.v_globalpts, &m_addr->shm_recoData.s_globalpts, sizeof(RecoGlobalPts_t));
			break;
		case RecoData::RT_COLLIDE:
			memcpy(&data->value.v_collide, &m_addr->shm_recoData.s_collide, sizeof(RecoCollide_t));
			break;
		case RecoData::RT_VEL:
			memcpy(&data->value.v_vel, &m_addr->shm_recoData.s_vel, sizeof(RecoVel_t));
			break;
		case RecoData::RT_FUSION_PTS:
			memcpy(&data->value.v_fusion_pts, &m_addr->shm_recoData.s_fusion_pts, sizeof(RecoPts_t));
			break;
		case RecoData::RT_MAP_DBOUNDARY:
			memcpy(&data->value.v_map_dboundary, &m_addr->shm_recoData.s_map_dboundary, sizeof(RecoMapDBoundary_t));
			break;
		case RecoData::RT_SUB_MAP:
			memcpy(&data->value.v_sub_map, &m_addr->shm_recoData.s_sub_map, sizeof(RecoSubMap_t));
			break;
		case RecoData::RT_OCCUPY_GRIDS:
			memcpy(&data->value.v_occupy_grids, &m_addr->shm_recoData.s_occupy_grids, sizeof(RecoOccupyGrids_t));
			break;
		case RecoData::RT_OBSTACLE_LIST:
			memcpy(&data->value.v_obstacle_list, &m_addr->shm_recoData.s_obstacle_list, sizeof(RecoObstacleList_t));
			break;
		case RecoData::RT_LANE_PTS:
			memcpy(&data->value.v_lane_pts, &m_addr->shm_recoData.s_lane_pts, sizeof(RecoPts_t));
			break;
		case RecoData::RT_HDL_PTS:
			memcpy(&data->value.v_hdl_pts, &m_addr->shm_recoData.s_hdl_pts, sizeof(RecoPts_t));
			break;
		case RecoData::RT_CURB_PTS:
			memcpy(&data->value.v_curb_pts, &m_addr->shm_recoData.s_curb_pts, sizeof(RecoPts_t));
			break;
		};

		data->timestamp = m_addr->shm_recoData.timestamps[index];
		data->belief = m_addr->shm_recoData.belief[index];
		if(isGettingNewData && m_addr->shm_recoData.isNew[index])
			m_addr->shm_recoData.isNew[index] = false;

		result = true;
	}
	Unlock(&m_addr->shm_recoData.locks[index]);
	return result;
}

// Marker
bool SharedObjectsImpl::SetMarker(const MarkerData_t& data)
{
	//LOG_INFO("m_addr=%p, type=%d", m_addr, data.type - 1);
	if(m_addr == NULL || data.type <= 0 || data.type >= MarkerData::MARKER_MAX)
		return false;

	int index = data.type - 1;
	Lock(&m_addr->shm_markers.locks[index]);
	switch(data.type) {
	case MarkerData::MARKER_NAVIGATION:
		memcpy(&m_addr->shm_markers.s_navi, &data.value.v_navi, sizeof(MarkerNavi_t));
		break;
	case MarkerData::MARKER_INTERSECTION:
		memcpy(&m_addr->shm_markers.s_intersection, &data.value.v_intersection, sizeof(MarkerIntersection_t));
		break;
//	case MarkerData::MARKER_TAILLIGHT_IMAGE:
//		memcpy(&m_addr->shm_markers.s_taillightImage, &data.value.v_taillightImage, sizeof(MarkerTaillightImage_t));
//		break;
//	case MarkerData::MARKER_HOKUYO_OBS:
//		memcpy(&m_addr->shm_markers.s_hokuyoobs, &data.value.v_hokuyoobs, sizeof(MarkerHokuyoObs_t));
//		break;
//	case MarkerData::MARKER_VELOCITY_DEC:
//		memcpy(&m_addr->shm_markers.s_velocityDec, &data.value.v_velocityDec, sizeof(MarkerVelocityDec_t));
//		break;
	case MarkerData::MARKER_VELOCITY_DEC_LUX:
		memcpy(&m_addr->shm_markers.s_velocityDecLux, &data.value.v_velocityDecLux, sizeof(MarkerVelocityDecLux_t));
		break;
	case MarkerData::MARKER_LANECHANGE:
		memcpy(&m_addr->shm_markers.s_lanechange, &data.value.v_lanechange, sizeof(MarkerLaneChange_t));
		break;
	case MarkerData::MARKER_CLOTHOID:
		memcpy(&m_addr->shm_markers.s_clothoid, &data.value.v_clothoid, sizeof(MarkerClothoid_t));
		break;
	case MarkerData::MARKER_LANECHANGE_OBSTACLE:
		memcpy(&m_addr->shm_markers.s_lanechangeobstacle, &data.value.v_lanechangeobstacle, sizeof(MarkerLaneChangeObstacle_t));
		break;
//	case MarkerData::MARKER_LANECHANGE_SIDE:
//		memcpy(&m_addr->shm_markers.s_lanechangeside, &data.value.v_lanechangeside, sizeof(MarkerLaneChangeSide_t));
//		break;
//	case MarkerData::MARKER_OBSTACLE:
//		memcpy(&m_addr->shm_markers.s_obstacle, &data.value.v_obstacle, sizeof(MarkerObstacle_t));
//		break;
	case MarkerData::MARKER_OBSTACLE_LUX:
		memcpy(&m_addr->shm_markers.s_obstacleLux, &data.value.v_obstacleLux, sizeof(MarkerObstacleLux_t));
		break;
	case MarkerData::MARKER_TL:
		memcpy(&m_addr->shm_markers.s_tl, &data.value.v_tl, sizeof(MarkerTrafficLight_t));
		break;
		//case MarkerData::MARKER_INITSENSOR:
		//	memcpy(&m_addr->shm_markers.s_initsensor, &data.value.v_initsensor, sizeof(MarkerInitSensor_t));
		//	break;
	case MarkerData::MARKER_ROADTRACKING:
		memcpy(&m_addr->shm_markers.s_roadtracking, &data.value.v_roadtracking, sizeof(MarkerRoadTracking_t));
		break;
	case MarkerData::MARKER_REFREEZE_CAM:
		memcpy(&m_addr->shm_markers.s_refreeze_cam, &data.value.v_refreeze_cam, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_REFREEZE_LUX:
		memcpy(&m_addr->shm_markers.s_refreeze_lux, &data.value.v_refreeze_lux, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_REFREEZE_HDL:
		memcpy(&m_addr->shm_markers.s_refreeze_hdl, &data.value.v_refreeze_hdl, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_REFREEZE_HDLL:
		memcpy(&m_addr->shm_markers.s_refreeze_hdll, &data.value.v_refreeze_hdll, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_CAM_LOST:
		memcpy(&m_addr->shm_markers.s_cam_lost, &data.value.v_cam_lost, sizeof(MarkerCamLost_t));
		break;
	case MarkerData::MARKER_CAMERATMP:
		memcpy(&m_addr->shm_markers.s_camera_tmp, &data.value.v_camera_tmp, sizeof(MarkerCameraTmp_t));
		break;
	case MarkerData::MARKER_GOODHDL:
		memcpy(&m_addr->shm_markers.s_goodhdl, &data.value.v_goodhdl, sizeof(MarkerGoodHDL_t));
		break;
	case MarkerData::MARKER_CANLANECHANGE:
		memcpy(&m_addr->shm_markers.s_can_lanechange, &data.value.v_can_lanechange, sizeof(MarkerCanLaneChange_t));
		break;
	case MarkerData::MARKER_LANECHANGEOVER:
		memcpy(&m_addr->shm_markers.s_lanechange_over, &data.value.v_lanechange_over, sizeof(MarkerLaneChangeOver_t));
		break;
	case MarkerData::MARKER_ACTIVEPOINT:
		memcpy(&m_addr->shm_markers.s_activepoint, &data.value.v_activepoint, sizeof(MarkerActivePoint_t));
		break;
	case MarkerData::MARKER_ACTIVEPOINTINRNDF:
		memcpy(&m_addr->shm_markers.s_activepointinrndf, &data.value.v_activepointinrndf, sizeof(MarkerActivePoint_t));
		break;
	case MarkerData::MARKER_ACTIVEPOINTINROUTE:
		memcpy(&m_addr->shm_markers.s_activepointinroute, &data.value.v_activepointinroute, sizeof(MarkerActivePoint_t));
		break;
	case MarkerData::MARKER_NAV_STOPLINE:
		memcpy(&m_addr->shm_markers.s_nav_stopline, &data.value.v_nav_stopline, sizeof(MarkerNavStopline_t));
		break;
	case MarkerData::MARKER_RNDF:
		memcpy(&m_addr->shm_markers.s_rndf, &data.value.v_rndf, sizeof(MarkerRNDF_t));
		break;
	case MarkerData::MARKER_REFREEZE_MAP:
		memcpy(&m_addr->shm_markers.s_refreeze_map, &data.value.v_refreeze_map, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_REFREEZE_OBJECT:
		memcpy(&m_addr->shm_markers.s_refreeze_object, &data.value.v_refreeze_object, sizeof(MarkerRefreeze_t));
		break;
	case MarkerData::MARKER_INSTURN:
		memcpy(&m_addr->shm_markers.s_sturn, &data.value.v_sturn, sizeof(MarkerInSturn_t));
		break;
	case MarkerData::MARKER_STURN_DEC:
		memcpy(&m_addr->shm_markers.s_sturndec, &data.value.v_sturndec, sizeof(MarkerSturnDec_t));
		break;
	case MarkerData::MARKER_DYNAMIC_OBJ:
		memcpy(&m_addr->shm_markers.s_dynamic_obj, &data.value.v_dynamic_obj, sizeof(MarkerDynamicObj_t));
		break;
	case MarkerData::MARKER_DYNAMIC_OBJ_1:
		memcpy(&m_addr->shm_markers.s_dynamic_obj_1, &data.value.v_dynamic_obj_1, sizeof(MarkerDynamicObj_t));
		break;
	case MarkerData::MARKER_DYNAMIC_OBJ_OVER:
		memcpy(&m_addr->shm_markers.s_dynamic_obj_over, &data.value.v_dynamic_obj_over, sizeof(MarkerDynamicObjOver_t));
		break;
	case MarkerData::MARKER_DYNAMIC_OBJ_OVER_1:
		memcpy(&m_addr->shm_markers.s_dynamic_obj_over_1, &data.value.v_dynamic_obj_over_1, sizeof(MarkerDynamicObjOver_t));
		break;
	case MarkerData::MARKER_DEVIDE:
		memcpy(&m_addr->shm_markers.s_devide, &data.value.v_devide, sizeof(MarkerDevide_t));
		break;
	case MarkerData::MARKER_ISUTURN:
		memcpy(&m_addr->shm_markers.s_isuturn, &data.value.v_isuturn, sizeof(MarkerIsUturn_t));
		break;
	case MarkerData::MARKER_START_OBJ:
		memcpy(&m_addr->shm_markers.s_start_obj, &data.value.v_start_obj, sizeof(MarkerStartObj_t));
		break;
	default:
		break;
	};

	m_addr->shm_markers.isValid[index] = true;
	//LOG_INFO("index=%d, value=%d", index, m_addr->shm_markers.isValid[index]);
	Unlock(&m_addr->shm_markers.locks[index]);
	return true;
}

bool SharedObjectsImpl::GetMarker(MarkerData_t* data)
{
	if(m_addr == NULL || data == NULL || data->type <= 0 || data->type >= MarkerData::MARKER_MAX)
		return false;

	int index = data->type - 1;
	bool result = true;

	Lock(&m_addr->shm_markers.locks[index]);
	//LOG_INFO("index=%d value=%d", index, m_addr->shm_markers.isValid[index]);
	if(m_addr->shm_markers.isValid[index]) {
		switch(data->type) {
		case MarkerData::MARKER_NAVIGATION:
			memcpy(&data->value.v_navi, &m_addr->shm_markers.s_navi, sizeof(MarkerNavi_t));
			break;
		case MarkerData::MARKER_INTERSECTION:
			memcpy(&data->value.v_intersection, &m_addr->shm_markers.s_intersection, sizeof(MarkerIntersection_t));
			break;
//		case MarkerData::MARKER_TAILLIGHT_IMAGE:
//			memcpy(&data->value.v_taillightImage, &m_addr->shm_markers.s_taillightImage, sizeof(MarkerTaillightImage_t));
//			break;
//		case MarkerData::MARKER_HOKUYO_OBS:
//			memcpy(&data->value.v_hokuyoobs, &m_addr->shm_markers.s_hokuyoobs, sizeof(MarkerHokuyoObs_t));
//			break;
//		case MarkerData::MARKER_VELOCITY_DEC:
//			memcpy(&data->value.v_velocityDec, &m_addr->shm_markers.s_velocityDec, sizeof(MarkerVelocityDec_t));
//			break;
		case MarkerData::MARKER_VELOCITY_DEC_LUX:
			memcpy(&data->value.v_velocityDecLux, &m_addr->shm_markers.s_velocityDecLux, sizeof(MarkerVelocityDecLux_t));
			break;
		case MarkerData::MARKER_LANECHANGE:
			memcpy(&data->value.v_lanechange, &m_addr->shm_markers.s_lanechange, sizeof(MarkerLaneChange_t));
			break;
		case MarkerData::MARKER_CLOTHOID:
			memcpy(&data->value.v_clothoid, &m_addr->shm_markers.s_clothoid, sizeof(MarkerClothoid_t));
			break;
		case MarkerData::MARKER_LANECHANGE_OBSTACLE:
			memcpy(&data->value.v_lanechangeobstacle, &m_addr->shm_markers.s_lanechangeobstacle, sizeof(MarkerLaneChangeObstacle_t));
			break;
//		case MarkerData::MARKER_LANECHANGE_SIDE:
//			memcpy(&data->value.v_lanechangeside, &m_addr->shm_markers.s_lanechangeside, sizeof(MarkerLaneChangeSide_t));
//			break;
//		case MarkerData::MARKER_OBSTACLE:
//			memcpy(&data->value.v_obstacle, &m_addr->shm_markers.s_obstacle, sizeof(MarkerObstacle_t));
//			break;
		case MarkerData::MARKER_OBSTACLE_LUX:
			memcpy(&data->value.v_obstacleLux, &m_addr->shm_markers.s_obstacleLux, sizeof(MarkerObstacleLux_t));
			break;
		case MarkerData::MARKER_TL:
			memcpy(&data->value.v_tl, &m_addr->shm_markers.s_tl, sizeof(MarkerTrafficLight_t));
			break;
			//case MarkerData::MARKER_INITSENSOR:
			//	memcpy(&data->value.v_initsensor, &m_addr->shm_markers.s_initsensor, sizeof(MarkerInitSensor_t));
			//	break;
		case MarkerData::MARKER_ROADTRACKING:
			memcpy(&data->value.v_roadtracking, &m_addr->shm_markers.s_roadtracking, sizeof(MarkerRoadTracking_t));
			break;
		case MarkerData::MARKER_REFREEZE_CAM:
			memcpy(&data->value.v_refreeze_cam, &m_addr->shm_markers.s_refreeze_cam, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_REFREEZE_LUX:
			memcpy(&data->value.v_refreeze_lux, &m_addr->shm_markers.s_refreeze_lux, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_REFREEZE_HDL:
			memcpy(&data->value.v_refreeze_hdl, &m_addr->shm_markers.s_refreeze_hdl, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_REFREEZE_HDLL:
			memcpy(&data->value.v_refreeze_hdll, &m_addr->shm_markers.s_refreeze_hdll, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_CAM_LOST:
			memcpy(&data->value.v_cam_lost, &m_addr->shm_markers.s_cam_lost, sizeof(MarkerCamLost_t));
			break;
		case MarkerData::MARKER_CAMERATMP:
			memcpy(&data->value.v_camera_tmp, &m_addr->shm_markers.s_camera_tmp, sizeof(MarkerCameraTmp_t));
			break;
		case MarkerData::MARKER_GOODHDL:
			memcpy(&data->value.v_goodhdl, &m_addr->shm_markers.s_goodhdl, sizeof(MarkerGoodHDL_t));
			break;
		case MarkerData::MARKER_CANLANECHANGE:
			memcpy(&data->value.v_can_lanechange, &m_addr->shm_markers.s_can_lanechange, sizeof(MarkerCanLaneChange_t));
			break;
		case MarkerData::MARKER_LANECHANGEOVER:
			memcpy(&data->value.v_lanechange_over, &m_addr->shm_markers.s_lanechange_over, sizeof(MarkerLaneChangeOver_t));
			break;
		case MarkerData::MARKER_ACTIVEPOINT:
			memcpy(&data->value.v_activepoint, &m_addr->shm_markers.s_activepoint, sizeof(MarkerActivePoint_t));
			break;
		case MarkerData::MARKER_ACTIVEPOINTINRNDF:
			memcpy(&data->value.v_activepointinrndf, &m_addr->shm_markers.s_activepointinrndf, sizeof(MarkerActivePoint_t));
			break;
		case MarkerData::MARKER_ACTIVEPOINTINROUTE:
			memcpy(&data->value.v_activepointinroute, &m_addr->shm_markers.s_activepointinroute, sizeof(MarkerActivePoint_t));
			break;
		case MarkerData::MARKER_NAV_STOPLINE:
			memcpy(&data->value.v_nav_stopline, &m_addr->shm_markers.s_nav_stopline, sizeof(MarkerNavStopline_t));
			break;
		case MarkerData::MARKER_RNDF:
			memcpy(&data->value.v_rndf, &m_addr->shm_markers.s_rndf, sizeof(MarkerRNDF_t));
			break;
		case MarkerData::MARKER_REFREEZE_MAP:
			memcpy(&data->value.v_refreeze_map, &m_addr->shm_markers.s_refreeze_map, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_REFREEZE_OBJECT:
			memcpy(&data->value.v_refreeze_object, &m_addr->shm_markers.s_refreeze_object, sizeof(MarkerRefreeze_t));
			break;
		case MarkerData::MARKER_INSTURN:
			memcpy(&data->value.v_sturn, &m_addr->shm_markers.s_sturn, sizeof(MarkerInSturn_t));
			break;
		case MarkerData::MARKER_STURN_DEC:
			memcpy(&data->value.v_sturndec, &m_addr->shm_markers.s_sturndec, sizeof(MarkerSturnDec_t));
			break;
		case MarkerData::MARKER_DYNAMIC_OBJ:
			memcpy(&data->value.v_dynamic_obj, &m_addr->shm_markers.s_dynamic_obj, sizeof(MarkerDynamicObj_t));
			break;
		case MarkerData::MARKER_DYNAMIC_OBJ_1:
			memcpy(&data->value.v_dynamic_obj_1, &m_addr->shm_markers.s_dynamic_obj_1, sizeof(MarkerDynamicObj_t));
			break;
		case MarkerData::MARKER_DYNAMIC_OBJ_OVER:
			memcpy(&data->value.v_dynamic_obj_over, &m_addr->shm_markers.s_dynamic_obj_over, sizeof(MarkerDynamicObjOver_t));
			break;
		case MarkerData::MARKER_DYNAMIC_OBJ_OVER_1:
			memcpy(&data->value.v_dynamic_obj_over_1, &m_addr->shm_markers.s_dynamic_obj_over_1, sizeof(MarkerDynamicObjOver_t));
			break;
		case MarkerData::MARKER_DEVIDE:
			memcpy(&data->value.v_devide, &m_addr->shm_markers.s_devide, sizeof(MarkerDevide_t));
			break;
		case MarkerData::MARKER_ISUTURN:
			memcpy(&data->value.v_isuturn, &m_addr->shm_markers.s_isuturn, sizeof(MarkerIsUturn_t));
			break;
		case MarkerData::MARKER_START_OBJ:
			memcpy(&data->value.v_start_obj, &m_addr->shm_markers.s_start_obj, sizeof(MarkerStartObj_t));
			break;
		default:
			result = false;
			break;
		};
	} else
		result = false;

	Unlock(&m_addr->shm_markers.locks[index]);
	return result;
}

bool SharedObjectsImpl::SetSmartMarker(int index)
{
	if(m_addr == NULL || index < 1 || index > 10)
		return false;

	Lock(&m_addr->shm_smartMarkers.locks[index - 1]);
	m_addr->shm_smartMarkers.isValid[index - 1] = true;
	m_addr->shm_smartMarkers.isNew[index - 1] = true;
	Unlock(&m_addr->shm_smartMarkers.locks[index - 1]);
	return true;
}

bool SharedObjectsImpl::GetSmartMarker(int index, bool isGettingNewData)
{
	if(m_addr == NULL || index < 1 || index > 10)
		return false;

	bool result = false;
	Lock(&m_addr->shm_smartMarkers.locks[index - 1]);
	if(m_addr->shm_smartMarkers.isValid[index - 1]
	   && (!isGettingNewData || (isGettingNewData && m_addr->shm_smartMarkers.isNew[index - 1]))) {
		if(isGettingNewData && m_addr->shm_smartMarkers.isNew[index - 1])
			m_addr->shm_smartMarkers.isNew[index - 1] = false;

		result = true;
	}
	Unlock(&m_addr->shm_smartMarkers.locks[index - 1]);
	return result;
}

inline void SharedObjectsImpl::Lock(pthread_spinlock_t* lock)
{
	sigset_t newset;
	sigemptyset(&newset);
	sigaddset(&newset, SIGINT);
	sigprocmask(SIG_BLOCK, &newset, NULL);

	pthread_spin_lock(lock);
}

inline void SharedObjectsImpl::Unlock(pthread_spinlock_t* lock)
{
	pthread_spin_unlock(lock);

	sigset_t newset;
	sigemptyset(&newset);
	sigaddset(&newset, SIGINT);
	sigprocmask(SIG_UNBLOCK, &newset, NULL);
}
}
}

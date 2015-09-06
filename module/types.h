#ifndef __TYPES_H_
#define __TYPES_H_
#include <sys/time.h>
#include <stdio.h>
//#include <semaphore.h>
#include <vector>
#include <math.h>
#include <opencv/cv.h>
#include <map>
using namespace std;

namespace module
{
/****************************************************
 * device meta data macro
 */
//Ibeo(LUX) meta data size
#define LUX_MAX_POINTS         11360
#define LUX_MAX_OBJECTS        256
#define LUX_MAX_CONTOURPOINTS  256
// HDL meta point number
#define HDL_LASER_NUMBER       64
#define HDL_MAX_POINT_LASER    4000
#define HDL_MAX_POINT_NUMBER HDL_MAX_POINT_LASER * HDL_LASER_NUMBER
// Black-White(BW) camera meta data size
#define BWCAM_IMAGE_WIDTH      780
#define BWCAM_IMAGE_HEIGHT     582
#define BWCAM_IMAGE_SIZE BWCAM_IMAGE_WIDTH * BWCAM_IMAGE_HEIGHT
// Color(C) camera meta data size
#define CCAM_IMAGE_WIDTH       1292
#define CCAM_IMAGE_HEIGHT      964
#define ROI_X                0.25
#define ROI_Y                0
#define ROI_W                0.5
#define ROI_H                0.6
#define C_WIDTH                3
#define CCAM_IMAGE_SIZE CCAM_IMAGE_WIDTH * CCAM_IMAGE_HEIGHT * C_WIDTH
// decision guide pts num
#define MAX_GUIDE_PTS          10
#define MAX_TARGET_PTS			200


//动态栅格子地图
#define SUB_MAP_LENGTH 801
#define SUB_MAP_WIDTH  801

//占用栅格集合

#define MAX_OCCUPY_NUM 6000

//栅格结构
typedef struct mapGrid
{
	unsigned char p[3];			//使用3个字节表示
} MapGrid_t;

//东北冻结坐标系下栅格结构的定义
typedef struct gridFXY
{
	float x, y;
} gridFXY_t;

//动态栅格子地图结构
typedef struct RecoSubMap
{
	gridFXY_t left_down_grid;			//子地图中左下角栅格的全局坐标
	MapGrid_t submap[SUB_MAP_LENGTH][SUB_MAP_WIDTH];
} RecoSubMap_t;




//全局地图中占用栅格结构的定义
typedef struct occupy_grid
{
	int row;
	int col;
} occupy_grid_t;

//占用栅格集合结构
typedef struct RecoOccupyGrids
{
	bool interactive_flag;
	unsigned int num;
	occupy_grid_t occupy_grids[MAX_OCCUPY_NUM];
} RecoOccupyGrids_t;



//障碍物结构
#define MAX_OBSTACLE_NUM 5

typedef struct obstacle
{
	unsigned char type;			//类型
	gridFXY_t anchor_pt;			//锚点
	gridFXY_t bounding_box[4];		//包络盒的四个顶点
	gridFXY_t velocity;				//速度
} obstacle_t;

typedef struct RecoObstacleList
{
	bool interactive_flag;
	unsigned int num;			//障碍物的个数
	obstacle_t obs[MAX_OBSTACLE_NUM];	//障碍物数组
} RecoObstacleList_t;

// Ibeo(ASL) meta data size
//#define ASL_MAX_POINTS 8648
//#define ASL_MAX_CONTOURPOINTS	16
//#define ASL_MAX_OBJECTS		64
// Hokuyo meta data size
//#define HOKUYO_MAX_POINTS 1081
//#define HOKUYO_MAX_OBJECTS  20
//#define HOKUYO_MAX_CONTOURPOINTS 150
/****************************************************
 * Decision Structure (controller <--> decision)
 */
typedef struct GuidePts {
	double x;
	double y;
	double z;
} GuidePts_t;

typedef struct RoutePts {
	int segment;
	int lane;
	int waypoint;
} RoutePts_t;

typedef struct Pose {
	double x;
	double y;
	double eulr;
} Pose_t;

typedef struct Decision {
	bool isOverride;
	int currGuidePtIndex;
	bool inCurrGuidePt;
	bool outCurrGuidePt;
	double LDevi;
	double AzimuthDevi;
	Pose_t localpt;
	Pose_t previewpt;
	double DesiredVelocity;
	double PlanningVelocity;
	double VehicleVelocity;
	double Acceleration;
	char Shift;
	bool Leftlight;
	bool Rightlight;
	enum {
	    VS_START     = 1,
	    VS_STOP      = 2,
	    VS_PAUSE     = 3,
	    VS_EMERGENCY = 4,
	    VS_RUNNING   = 5
	} State;
	enum BehaviorType {
	    BEHAVIOR_ROADTRACKING   = 1,
	    BEHAVIOR_CROSS          = 2,
	    BEHAVIOR_LANE_CHANGE    = 3,
	    BEHAVIOR_STOPLINE_PAUSE = 4,
	    BEHAVIOR_CONSTRUCT 		= 5,
	    BEHAVIOR_MAX            = 6
	} Behavior;
	enum BehaviorProperty {
	    INTERSECTION_UNKNOWN = 0,
	    INTERSECTION_LEFT    = 1,
	    INTERSECTION_RIGHT   = 2,
	    INTERSECTION_FORWARD = 3,
	    INTERSECTION_UTURN   = 4
	} Options;
	struct BehaviorData {
		GuidePts_t guidePts[MAX_GUIDE_PTS];
		RoutePts_t routePts[MAX_GUIDE_PTS];
		int num;
	} Data;
	struct timeval timestamp;
	void SetTimeStamp() {
		gettimeofday(&timestamp, NULL);
	}
} Decision_t;
/****************************************************
 * Shared Device Meta Data Structures
 */
// GPS Meta Data
#pragma pack (4)
typedef struct MetaNavigation {
	long int ID;
	double ENU[3];
	double Velocity[3];
	double Eulr[3];
	double AngularVelocity[3];
	double V;
	double A;
	double COV[3];
	double TMbr[4][4];
	double org_xyz[3];
	unsigned int count1;
	unsigned int count2;
	double GetVelocityIMU() {
		return sqrt(pow(Velocity[0], 2) + pow(Velocity[1], 2));
	}
	double GetVelocityEncoder() {
		return V;
	}
} MetaNavigation_t;
#pragma pack ()

typedef struct MetaLocalNavigation {
	double T[3][3];
	Pose_t original_pt;
	Pose_t last_local_pt;
	double new_original_eulr;
	double UGVtoFCF[3];
	int counter;
} MetaLocalNavigation_t;

typedef struct LUXPoint {
	float m_x;
	float m_y;
	float m_z;
	float m_dist;
	float m_hAngle;
	float m_vAngle;
	float m_echoWidth;
	unsigned short m_flags;
	unsigned char m_deviceID;
	unsigned char m_channel;
	unsigned char m_subchannel;
	unsigned char m_millisec;
	unsigned short m_microsec;
	unsigned short m_segmentId;
} LUXPoint_t;

// Ibeo(LUX) MetaPoints
typedef struct MetaLUXPoints {
	float m_startAngle;
	float m_endAngle;
	unsigned int m_scannerStatus;
	unsigned int m_flags;
	unsigned int m_scanNumber;
	unsigned char m_deviceID;
	unsigned char m_scannerType;
	unsigned int m_uPoints;
	LUXPoint_t m_pts[LUX_MAX_POINTS];
} MetaLUXPoints_t;

typedef struct LUXObject {
	struct LUXPoint2D {
		float m_x;
		float m_y;
	};
	enum ObjectClassification {
	    Unclassified = 0, ///< = 0: No classification determined (yet)
	    UnknownSmall = 1, ///< = 1: Relatively small structure like pole, tree, single bush, etc. which does not fit to other classes.
	    UnknownBig = 2, ///< = 2: Bigger structure which does not fit other classes.
	    Pedestrian = 3, ///< = 3: Pedestrian, usually determined by moving behaviour.
	    Bike = 4, ///< = 4: bike, motor bike
	    Car = 5, ///< = 5: Passenger car.
	    Truck = 6, ///< = 6: Big vehicle, bus, truck.
	    Structure_Pylon = 7, ///< = 7: Only enhanced, special classification (staf).
	    Structure_Beacon = 8, ///< = 8: Only enhanced, special classification (staf).
	    Structure_GuardRail = 9, ///< = 9: Only enhanced, special classification (staf).
	    Structure_ConcreteBarrier = 10, ///< = 10: Only enhanced, special classification (staf).
	    NumClasses, ///< Number of currently defined classes except Unknown.
	    Unknown = 15 ///< = 15: Compatibility to earlier software which does not determine between UnknownBig or -Small or Unclassified.
	};
	unsigned short m_objectId;
	unsigned short  m_flags; ///< reserved
// Data from the tracking/classification:
	unsigned int  m_objectAge;            ///< number of scans in which this object has been tracked, or instead time?
	unsigned short m_hiddenStatusAge;      ///< Counts how long the object has not been observed but only predicted.
	ObjectClassification m_classification; ///< The object class that is most likely for this object.
	unsigned int  m_classificationAge;           ///< Counts how long the object has been classified in the current classification.
	float   m_classificationQuality;       ///< The quality of the current classification.
	LUXPoint2D m_centerPoint;      ///< Center point of object rectangle, given in Vehicle coordinate system.
	LUXPoint2D m_centerPointSigma;
	float   m_courseAngle;      ///< named by ISO 8855; also called Orientation or Heading [rad]
	float   m_courseAngleSigma; // in [rad]
	LUXPoint2D m_relativeVelocity; ///< Velocity of this object [meter/seconds], relative to the vehicle coordinate system.
	LUXPoint2D m_relativeVelocitySigma;
	LUXPoint2D m_absoluteVelocity; ///< Velocity of this object [meter/seconds] as absolute velocity; the orientation is relative to the vehicle coordinate system.
	LUXPoint2D m_absoluteVelocitySigma;
	LUXPoint2D m_objectBox;         ///< The object's length and width as a rectangle, relative to the object's coordinate system.
	LUXPoint2D m_objectBoxSigma;
	LUXPoint2D m_boundingBoxCenter; ///< Center of the bounding box.
	LUXPoint2D m_boundingBox;       ///< A rectangle in parallel to the vehicle coordinate system (a paraxial rectangle) that contains (bounds) all of this object's points
// These components are also proposed
	LUXPoint2D m_closestPoint; ///< The point of this object that is closest to the origin of the vehicle coordinate system.
// This can also be calculated
	LUXPoint2D m_contourPoints[LUX_MAX_CONTOURPOINTS]; ///< A poly-line that describes the outline of the current object measurement.
	unsigned long long m_vehicleWLANid;    ///< An identifier to be used by WLAN fusion algorithms.
	float m_objectHeight;      ///< The height of this object in [m] (most probably received through WLAN data).
	float m_objectHeightSigma; ///< The standard deviation of the height of this object in [m] (most probably received through WLAN data).
	float m_objectMass; ///< The mass of this object in [kilogram] (as received e.g. through WLAN data)
	/** True, if this object is valid. */
	bool m_isValid;
} LUXObject_t;

// Ibeo(LUX) MetaObjects
typedef struct MetaLUXObjects {
	unsigned int m_uObjects;
	LUXObject_t obj[LUX_MAX_OBJECTS];
} MetaLUXObjects_t;

// HDL Laser Meta Data
typedef struct LPoint {
	int x;
	int y;
	int z;
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
} LPoint_t;

typedef struct MetaLaserHdl {
	int pts_count;
	LPoint_t pts[HDL_MAX_POINT_NUMBER];
} MetaLaserHdl_t;

// Black-White(BW) Camera Meta Data
typedef struct MetaCameraBW {
	unsigned char mdata[BWCAM_IMAGE_SIZE];
} MetaCameraBW_t;

// Color(C) Camera Meta Data
typedef struct MetaCameraC {
	unsigned char mdata[CCAM_IMAGE_SIZE];
} MetaCameraC_t;

typedef struct MMWPoint {
	double x;
	double y;
} MMWPoint_t;

typedef struct MetaMMWPoints {
	MMWPoint_t m_pts_front[64];
	MMWPoint_t m_pts_back1[64];
	MMWPoint_t m_pts_back2[64];
	unsigned long mmw_stamp;
} MetaMMWPoints_t;

typedef struct MetaMap {
	unsigned char p[2501][2501];
	int x_offset,y_offset;
	bool runmark;
} MetaMap_t;

// Hokuyo Meta Data
//typedef struct MetaHokuyoPoints
//{
//	long range[HOKUYO_MAX_POINTS];
//	long intensity[HOKUYO_MAX_POINTS];
//} MetaHokuyoPoints_t;
//typedef struct MetaHokuyoObjects
//{
//	unsigned int uNumberOfObjects; //Specified the number of objects which were transmitted.
//	unsigned int uSensorStatus; // Specified the status of the Laserscanner. {0: Error and; 1: already}
//
//	unsigned int uaObjectAge[HOKUYO_MAX_OBJECTS]; // Age of an object [Scans]
//	unsigned int uaTrackingNumber[ASL_MAX_OBJECTS]; // For the life of an object constant number.
//
//	/**
//	 * Shows the status of an object.
//	 *
//	 * There are 3 possible states:
//	 *	- 0: known object (the object is older than 1 scan),
//	 *	- 1: unknown object (the object was seen the 1st time) and
//	 *	- 255: the information isn't available.
//	 */
//	unsigned int uaTrackingStatus[HOKUYO_MAX_OBJECTS];
//
//	unsigned int uaNumberOfPoints[HOKUYO_MAX_OBJECTS]; // Number of contour-points which describe the outline of an object.
//
//	double daPointNumberPosX[HOKUYO_MAX_OBJECTS][HOKUYO_MAX_CONTOURPOINTS]; // Relative position in x-direction [m].
//	double daPointNumberPosY[HOKUYO_MAX_OBJECTS][HOKUYO_MAX_CONTOURPOINTS]; // Relative position in Y-direction [m].
//
//	bool baVelocityXValid[HOKUYO_MAX_OBJECTS];
//	double daVelocityX[HOKUYO_MAX_OBJECTS]; // Relative velocity in x-direction [m/s]
//
//	bool baVelocityYValid[HOKUYO_MAX_OBJECTS];
//	double daVelocityY[HOKUYO_MAX_OBJECTS]; // Relative velocity in y-direction [m/s]
//
//	bool baPositionXsigmaValid[HOKUYO_MAX_OBJECTS];
//	double daPositionXsigma[HOKUYO_MAX_OBJECTS]; // Standard deviation of value daPositionX
//
//	bool baPositionYsigmaValid[HOKUYO_MAX_OBJECTS];
//	double daPositionYsigma[HOKUYO_MAX_OBJECTS]; // Standard deviation of value daPositionY
//
//	bool baVelocityXsigmaValid[HOKUYO_MAX_OBJECTS];
//	double daVelocityXsigma[HOKUYO_MAX_OBJECTS]; // Standard deviation of value daVelocityX
//
//	bool baVelocityYsigmaValid[HOKUYO_MAX_OBJECTS];
//	double daVelocityYsigma[HOKUYO_MAX_OBJECTS]; // Standard deviation of value daVelocityY
//
//} MetaHokuyoObjects_t;
typedef struct MetaData {
	enum {
	    META_LASER_HDL      = 1,
	    META_CAMERA_BW      = 2,
	    META_CAMERA_C       = 3,
	    META_NAVIGATION     = 4,
	    META_LOCAL_NAVIGATION = 5,
	    META_LUX_POINTS       = 6,
	    META_LUX_OBJECTS    = 7,
	    //META_BEFORE_RADR   = 8,
	    //META_BACK_RADR_L   = 9,
	    //META_BACK_RADR_R   = 10,
	    META_MMW_POINTS    = 8,
	    META_ROAD_POINTS       = 9,
		//META_MAP 	= 10,
	                             //META_ROAD_OBJECTS    = 13

//	    META_HOKUYO_POINTS  = 7,
//	    META_HOKUYO_OBJECTS = 8
	} type;
	union {
		MetaNavigation_t v_navi;
		MetaLocalNavigation_t v_navi_local;
		MetaLaserHdl_t v_laserHdl;
		MetaCameraBW_t v_cameraBW;
		MetaCameraC v_cameraC;
		MetaLUXPoints v_luxPts;//
		MetaLUXObjects v_luxObjs;//
		//MetaBeforeRadr v_beforeRadr;
		//MetaAfterRadrL v_afterRadrL;
		//MetaAfterRadrR v_afterRadrR;
		MetaMMWPoints v_mmwPts;//
		MetaLUXPoints v_roadPts;//
		//MetaMap_t v_map;
		//MetaLUXObjects v_roadObjs;

//		MetaHokuyoPoints_t v_hokuyoPts;
//		MetaHokuyoObjects_t v_hokuyoObjs;
	} value;
	struct timeval timestamp;
	MetaData() {
		SetTimeStamp();
	}
	// set timestamp
	void SetTimeStamp() {
		gettimeofday(&timestamp, NULL);
	}
} MetaData_t;
/*****************************************************
 * Shared recognition data structures
 */
//typedef struct RecoSpotLine
//{
//	double ld;
//	double ad;
//	double distance;
//} RecoSpotLine_t;
// stopline
typedef struct RecoStopLine { // RecoType=RT_STOPLINE
	double angle;
	double distance;
} RecoStopLine_t;

// Traffic Light
typedef struct RecoTrafficLight { // RecoType=RT_TRAFFICLIGHT
	enum TrafficLightType {
	    TRL_NONE   = 0,
	    TRL_RED    = 1,
	    TRL_YELLOW = 2,
	    TRL_GREEN  = 3
	} type;
} RecoTrafficLight_t;

// traffic sign 1
typedef struct RecoTrafficSign { // RecoType=RT_TRAFFICSIGN
	enum {
	    TRS_LIM10          = 1,
	    TRS_CANCLE10         = 1>>2,
	    TRS_CHILD               = 1>>3,
	    TRS_CROSSYELLOW    = 1>>4,
	    TRS_CONSTRUCT          = 1>>5,
	    TRS_STOP         = 1>>6,
	    TRS_UTURN              = 1>>7,
	    TRS_UNKNOWNSIGN           = 0
	} type;
	int result;
	int position;
} RecoTrafficSign_t;

//// road tracking (ld&ad)
//typedef struct RecoTrackLdAd   // RecoType=RT_TRACK_LDAD
//{
//double ld;
//double ad;
//double lanewidth;
//double curbwidth;
//double curvature;
//} RecoTrackLdAd_t;

typedef struct RecoTrackLocalAndPreview { // RecoType=RT_TRACK_LOCALANDPREVIEW
	Pose_t localpt;
	Pose_t previewpt;
	double lanewidth;
	double curbwidth;
	double curvature;
	double curvature_valid;
} RecoTrackLocalAndPreview_t;

// lux car following & lane change
typedef struct RecoSlowDown {
	enum {
	    SL_NONE   = 0,
	    SL_FOLLOW = 1,
	    SL_CHANGE = 2,
	    SL_CHANGE_LEFT = 3,
	    SL_CHANGE_RIGHT = 4,
	    SL_CHANGE_OVER = 5
	} state;
	double dv;
} RecoSlowDown_t;

// emergcy
typedef struct RecoEmergency {
	int level;
} RecoEmergency_t;

// obstacle detection
typedef struct RecoSideObs {
	bool hasLeftObstacle;
	bool hasRightObstacle;
} RecoSideObs_t;


typedef struct LaneModule {
	double x0,x1,x2,x3;
	// y = 1.0/6 * x3*x^3 + 1.0/2 * x2*x^2 + x1*x + x0;
	double w;
	//lane change start set isLaneOver = false
	//lane change over middle lane isLaneOver = true
	bool isLaneOver;
} LaneModule_t;

typedef struct LaneMark {
	CvScalar lane;
	// x = lane.val[0]*y^3 + lane.va[1]*y^2 + lane.val[2]*y + lane.val[3];
	bool valid;
	// 1 is ok,0 is not ok;
} LaneMark_t;

// lane mark detection
typedef struct RecoLaneMark {
	/*
	Pose_t pt_left_preview;
	Pose_t pt_right_preview;
	Pose_t pt_center_preview;
	Pose_t target_points_mid_vehicle[MAX_TARGET_PTS];
	int pts_num;
	double curvature;
	double belief_left;
	double belief_right;
	double dist_left;
	double dist_right;
	double lane_width;
	LaneModule_t lane_module;
	*/
	Pose_t preview;
	double belief;
	double lanewidth;
	CvPoint2D32f left[400];
	CvPoint2D32f right[400];
	LaneMark_t ll_v,l_v,r_v,rr_v;//vehicle
	LaneMark_t ll_f,l_f,r_f,rr_f;//freeze
} RecoLaneMark_t;


typedef struct RecoSCurve {
	Pose_t pt_preview;
	double curvature;
	double belief;
	double width;
} RecoSCurve_t;

typedef struct RecoLaneChangeLaneMarks {
	LaneMark_t ll,l,r,rr;
} RecoLaneChangeLaneMarks_t;

typedef struct RecoObjectLaneMarks {
	LaneMark_t ll,l,r,rr;
} RecoObjectLaneMarks_t;

typedef struct RecoCross {
	bool isvalid;
	bool isFinish;
	bool isStraight; // 1:straight 0:cross
	Pose_t guidept;
	Pose_t localpt;
	Pose_t startpt;
} RecoCross_t;
typedef struct RecoUturn {
	bool isvalid; // 1有效 0无效
	Pose_t guidept;
	Pose_t startpt;
	Pose_t localpt;
	bool isStraight; // 1:straight 0:uturn
} RecoUturn_t;

/*typedef struct object {
	double dis;
	double center_x, center_y;
	double width;
	double vx, vy;
	int belief;
} object_t;

typedef struct RecoObjects {
	object_t objOnCurLane_front;
	object_t objOnLeftLane_front, objOnLeftLane_back;
	object_t objOnRightLane_front, objOnRightLane_back;
} RecoObjects_t;
*/
typedef struct RecoGoodHDL {
	bool isGood;
} RecoGoodHDL_t;

typedef struct RecoPathPoints {
	Pose_t pts[1024];
	int num;
} RecoPathPoints_t;
/*
typedef struct Object_2 {
	double x,y;
	double length,width;
	double v_x,v_y;
	double a_x,a_y;
	int belief;
} Object_2_t;
typedef struct RecoObjects_2 {
	unsigned int num;
	Object_2 objs[64];
} RecoObjects_2_t;
 * */
typedef struct RecoLocalPts{
	int num;
	enum  {
		LocalPtsNum = 200
	};
	Pose_t pts[LocalPtsNum];
} RecoLocalPts_t;
typedef struct RecoGlobalPts{
	int num;
	enum{
		GlobalPtsNum = 5000
	};
	Pose_t pts[GlobalPtsNum];
}RecoGlobalPts_t;

typedef struct RecoPts{
	bool valid;
	int num;
	enum{
		NUM = 500
	};
	Pose_t pts[NUM];
}RecoPts_t;

typedef struct RecoFusionPts{
	bool valid;
	int num;
	//int end;
	enum{
		NUM = 500
	};
	Pose_t pts[NUM];
}RecoFusionPts_t;

typedef struct RecoLanePts{
	bool valid;
	int num;
	//int end;
	enum{
		NUM = 500
	};
	Pose_t pts[NUM];
}RecoLanePts_t;

typedef struct RecoHDLPts{
	bool valid;
	int num;
	//int end;
	enum{
		NUM = 500
	};
	Pose_t pts[NUM];
}RecoHDLPts_t;

typedef struct RecoCurbPts{
	bool valid;
	int num;
	//int end;
	enum{
		NUM = 500
	};
	Pose_t pts[NUM];
}RecoCurbPts_t;

typedef struct GridPos {
	int row, col;
} GridPos_t;

typedef struct Object {
	bool IsMoving;

	int row, col;									//central grid
	double x, y;									//central coordinate
	int up_row, down_row, right_col, left_col;		//bounding box
	double ymax, ymin, xmax, xmin;					//bounding box

	Pose_t edge[4];

	double vx, vy;									//velocity
	double length, width;							//size
	unsigned int belief;							//belief
	int closest_row, closest_col;
	double closest_x, closest_y;

	unsigned int occupy_num;						//occupied grid number
	GridPos_t occupy_grid[MAX_OCCUPY_NUM];
} Object_t;

typedef struct Path{
	double belief;
	double s;
	double x_u[6],y_u[6];
	// x = x_u[5]*u^5 + x_u[4]*u^4 + ... + x_u[1]*u + x_u[0];
	// y = y_u[5]*u^5 + y_u[4]&u^4 + ... + y_u[1]*u + y_u[0];
	double s_pre;
	Pose_t pt_pre;
	double mc;
	bool is_collided;
	double dp;
	//double thetab;
} Path_t;

typedef struct RecoCollide {
	bool marker;	    // chen set 1,check for 0, xiao set 0
	bool ob_activated; // chen pass
	Path_t path_candidates[5][25];

	int startIndex;

	int globalpathNum;
	Pose_t globalpath[5000];

} RecoCollide_t;

typedef struct RecoVel {
	double PlanningVel;
} RecoVel_t;

typedef struct RecoMapDBoundary {
	bool valid;
	Pose_t pt1,pt2;
} RecoMapDBoundary_t;

typedef struct RecoData {
	enum RecoDataType {
	    RT_STOPLINE     = 1,
	    RT_TRAFFICSIGN  = 2,
	    RT_TRAFFICLIGHT = 3,
	    //RT_TRACK_LDAD   = 4,
	    RT_SLOWDOWN     = 5,
	    RT_EMERGENCY    = 6,
	    RT_SIDEOBS      = 7,
	    RT_LANEMARK     = 8,
	    RT_CURB         = 9,
	    RT_RAIL         = 10,
	    RT_HDL_LANE		= 11,
	    RT_GPS			= 12,
	    RT_SCURVE       = 13,
	    RT_GUIDE_PT     = 14,
	    RT_TRACK_LOCALANDPREVIEW = 15,
	    RT_LANECHANGE_LANEMARKS = 16,
	    RT_OBJECT_LANEMARKS = 17,
	    RT_CROSS		= 18,
	    RT_UTURN		= 19,
	    RT_GOODHDL		= 20,
	    RT_PathPoints   = 21,
		RT_LOCALPTS 	=22,
		RT_GLOBALPTS 	= 23,
		RT_COLLIDE		= 24,
		RT_VEL	= 25,
		RT_FUSION_PTS = 26,
		RT_MAP_DBOUNDARY = 27,
		RT_SUB_MAP = 28,
		RT_OCCUPY_GRIDS = 29,
		RT_OBSTACLE_LIST = 30,
		RT_LANE_PTS = 31,
		RT_HDL_PTS = 32,
		RT_CURB_PTS = 33,
	    RT_MAX          = 34

	} type;
	union RecoDataValue {
		RecoStopLine_t v_stopline;
		RecoTrafficSign_t v_ts;
		RecoTrafficLight_t v_tl;
		//RecoTrackLdAd_t v_trackLdAd;
		RecoSlowDown_t v_slowdown;//
		RecoEmergency_t v_emergency;//
		RecoSideObs_t v_sideObs;
		RecoLaneMark_t v_lanemark;
		RecoLaneMark_t v_curb;
		RecoLaneMark_t v_rail;
		RecoLaneMark_t v_hdl_lane;
		RecoLaneMark_t v_gps;
		RecoSCurve_t v_scurve;
		Pose_t v_guide_pt;
		RecoTrackLocalAndPreview_t v_trackLocalAndPreview;
		RecoLaneChangeLaneMarks_t v_laneChange_lanemarks;
		RecoObjectLaneMarks_t v_object_lanemarks;
		//RecoBeforeRadr v_beforeRadr;
		//RecoAfterRadrL v_afterRadrL;
		//RecoAfterRadrR v_afterRadrR;
		RecoCross_t v_cross;
		RecoUturn_t v_uturn;
		RecoGoodHDL_t v_goodhdl;
		RecoPathPoints_t v_pathpoints;
		//RecoObjects_2_t v_objects_2;
		RecoLocalPts_t v_localpts;
		RecoGlobalPts_t v_globalpts;
		RecoCollide_t v_collide;
		RecoVel_t v_vel;
		//RecoFusionPts_t v_fusion_pts;
		RecoPts_t v_fusion_pts;
		RecoMapDBoundary_t v_map_dboundary;
		RecoSubMap_t v_sub_map;
		RecoOccupyGrids_t v_occupy_grids;
		RecoObstacleList_t v_obstacle_list;
		//RecoLanePts_t v_lane_pts;
		//RecoHDLPts_t v_hdl_pts;
		//RecoCurbPts_t v_curb_pts;
		RecoPts_t v_lane_pts;
		RecoPts_t v_hdl_pts;
		RecoPts_t v_curb_pts;

	} value;
	struct timeval timestamp;
	double belief;
	RecoData() {
		SetTimeStamp();
	}
	// set time stamp
	void SetTimeStamp() {
		gettimeofday(&timestamp, NULL);
	}
} RecoData_t;
/*****************************************************
 * Shared Markers
 */
// navigation marker
typedef struct MarkerNavigation { // MarkerType=MARKER_NAVIGATION
	int mark;
	//int mapmark; // zhuch
	//int runmark;
	//int mark_refreeze;
	//int mark_refreeze_lux;
	//int mark_refreeze_hdl;
	double distance;
	double theta;
} MarkerNavi_t;

// refreeze marker
typedef struct MarkerRefreeze { // MarkerType=MARKER_NAVIGATION
	int isRefreeze;
} MarkerRefreeze_t;
// cam lost marker
typedef struct MarkerCamLost { // MarkerType=MARKER_NAVIGATION
	int isCamLost;
} MarkerCamLost_t;

// intersection state: roadtracking <=> decision
typedef struct MarkerIntersection {
	bool isFinished;
	bool start;
} MarkerIntersection_t;

// taillight image: HDL => TailLight
//typedef struct MarkerTaillightImage
//{
//	int x;
//	int y;
//	int width;
//	int height;
//} MarkerTaillightImage_t;

// emergency
//typedef struct MarkerObstacle
//{
//	int level;
//} MarkerObstacle_t;

typedef struct MarkerObstacleLux {
	int level;
} MarkerObstacleLux_t;

// hokuyo->HDL
//typedef struct MarkerHokuyoObs // MarkerType=MARKER_HOKUYO_OBS
//{
//	bool left;
//	bool right;
//	bool back;
//} MarkerHokuyoObs_t;

// velocity dec: ibeo => decision
//typedef struct MarkerVelocityDec
//{
//	double delta_v; // alway be no greater than 0
//	int flag; // 1 = slow down, 0 = do nothing
//} MarkerVelocityDec_t;

typedef struct MarkerVelocityDecLux {
	double delta_v; // alway be no greater than 0
	int flag; // 1 = slow down, 0 = do nothing
} MarkerVelocityDecLux_t;

// obstacle: track => obstacle
typedef struct TrajectoryClothoid {
	double a;
	double b;
	double c;
	double d;
	bool valid;
	// x = a*y^3 + b*y^2 + c*y + d;
} TrajectoryClothoid_t;

typedef struct MarkerClothoid {
	TrajectoryClothoid_t trajectory[10];
	double curvature;
	CvScalar l_clothoid,r_clothoid;
	CvScalar l_ugv,r_ugv;
	bool valid;
} MarkerClothoid_t;

typedef struct MarkerLaneChange {
	enum LaneChangeState {
	    LCS_START    = 1,
	    LCS_FINISHED = 2,
	    LCS_RUNNING  = 3,
	    LCS_FAILED   = 4
	} state;

	enum LaneChangeSide {
	    LC_LEFT    = 1,
	    LC_RIGHT   = 2,
	    LC_UNKNOWN = 3
	} side;
} MarkerLaneChange_t;

typedef struct MarkerLaneChangeObstacle {
	bool hasLeftObstacle;
	bool hasRightObstacle;
	double vLeft;
	double vRight;
} MarkerLaneChangeObstacle_t;

typedef struct MarkerTrafficLight {
	int TrafficLights[10][4];
	int result_length;
	unsigned char pattern[10][5400];
} MarkerTrafficLight_t;

typedef struct MarkerRoadTracking {
	enum RoadTrackingState {
	    RTS_NORMAL =       1,
	    RTS_LANECHANGE =   2,
	    RTS_INTERSECTION = 3
	} state;
} MarkerRoadTracking_t;

typedef struct MarkerCameraTmp {
	unsigned char mdata[4*400*600];
} MarkerCameraTmp_t;

typedef struct MarkerGoodHDL {
	bool isGood;
} MarkerGoodHDL_t;

typedef struct MarkerCanLaneChange_t {
	bool canLaneChange;
} MarkerCanLaneChange_t;

typedef struct MarkerLaneChangeOver_t {
	bool isLaneChangeOver;
} MarkerLaneChangeOver_t;
typedef struct MarkerActivePoint {
	int pre_num;
	int pre_segment;
	int pre_lane;
	int pre_waypoint;
	int pre_p1,pre_p2;
	Pose_t pre_pt;
	int num;
	int segment;
	int lane;
	int waypoint;
	int p1,p2;
	Pose_t pt;
	int next_num;
	int next_segment;
	int next_lane;
	int next_waypoint;
	int next_p1,next_p2;
	Pose_t next_pt;
} MarkerActivePoint_t;
typedef struct MarkerNavStopline {
	bool isupdate;
} MarkerNavStopline_t;
typedef struct MarkerRNDF {
	void*     m_pData;
	size_t    m_uSize;
	int       m_nFile;
} MarkerRNDF_t;
typedef struct MarkerInSturn {
	bool mark;
	double dis;
} MarkerInSturn_t;
typedef struct MarkerSturnDec {
	bool mark;
} MarkerSturnDec_t;
typedef struct MarkerDynamicObj {
	bool hasobj;
	double dis;
	bool mark;
} MarkerDynamicObj_t;

typedef struct MarkerDynamicObjOver {
	bool hasobj;
} MarkerDynamicObjOver_t;

typedef struct MarkerDevide {
	bool mark;
	int loadtype;
} MarkerDevide_t;

typedef struct MarkerIsUturn {
	bool isuturn;
} MarkerIsUturn_t;

typedef struct MarkerStartObj {
	bool mark;
} MarkerStartObj_t;

typedef struct MarkerData {
	enum MarkerDataType {
	    MARKER_NAVIGATION          = 1,
	    MARKER_INTERSECTION        = 2,
//	    MARKER_TAILLIGHT_IMAGE 	   = 3,
//	    MARKER_HOKUYO_OBS          = 4,
//	    MARKER_VELOCITY_DEC        = 5,
	    MARKER_VELOCITY_DEC_LUX    = 3,
	    MARKER_LANECHANGE          = 4,
	    MARKER_CLOTHOID            = 5,
	    MARKER_LANECHANGE_OBSTACLE = 6,
//	    MARKER_LANECHANGE_SIDE     = 10,
//	    MARKER_OBSTACLE            = 11,
	    MARKER_OBSTACLE_LUX        = 7,
	    MARKER_TL                  = 8,
	    MARKER_ROADTRACKING        = 9,
	    MARKER_REFREEZE_CAM		   = 10,
	    MARKER_REFREEZE_LUX		   = 11,
	    MARKER_REFREEZE_HDL        = 12,
	    MARKER_REFREEZE_HDLL   	   = 13,
	    MARKER_CAM_LOST            = 14,
	    MARKER_CAMERATMP            = 15,
	    MARKER_GOODHDL				= 16,
	    MARKER_CANLANECHANGE		= 17,
	    MARKER_LANECHANGEOVER		= 18,
	    MARKER_ACTIVEPOINT 		   = 19,
	    MARKER_ACTIVEPOINTINRNDF 		   = 20,
	    MARKER_ACTIVEPOINTINROUTE 		   = 21,
	    MARKER_NAV_STOPLINE		   = 22,
	    MARKER_RNDF				= 23,
		MARKER_REFREEZE_MAP = 24,
		MARKER_REFREEZE_OBJECT = 25,
		MARKER_INSTURN = 26,
		MARKER_STURN_DEC = 27,
		MARKER_DYNAMIC_OBJ = 28,
		MARKER_DYNAMIC_OBJ_1 = 29,
		MARKER_DYNAMIC_OBJ_OVER = 30,
		MARKER_DYNAMIC_OBJ_OVER_1 = 31,
		MARKER_DEVIDE = 32,
		MARKER_ISUTURN = 33,
		MARKER_START_OBJ = 34,
		//MARKER_OBACTIVATED		= 24,
	    MARKER_MAX                 = 35
	} type;
	union {
		MarkerNavi_t v_navi;
		MarkerIntersection_t v_intersection;
//		MarkerTaillightImage_t v_taillightImage;
//		MarkerHokuyoObs_t v_hokuyoobs;
//		MarkerVelocityDec_t v_velocityDec;
		MarkerVelocityDecLux_t v_velocityDecLux;
		MarkerLaneChange_t v_lanechange;
		MarkerClothoid_t v_clothoid;
		MarkerLaneChangeObstacle_t v_lanechangeobstacle;
//		MarkerLaneChangeSide_t v_lanechangeside;
//		MarkerObstacle_t v_obstacle;
		MarkerObstacleLux_t v_obstacleLux;
		MarkerTrafficLight_t v_tl;
		MarkerRoadTracking v_roadtracking;
		MarkerRefreeze_t v_refreeze_cam;
		MarkerRefreeze_t v_refreeze_lux;
		MarkerRefreeze_t v_refreeze_hdl;
		MarkerRefreeze_t v_refreeze_hdll;
		MarkerCamLost_t v_cam_lost;
		MarkerCameraTmp_t v_camera_tmp;
		MarkerGoodHDL_t v_goodhdl;
		MarkerCanLaneChange_t v_can_lanechange;
		MarkerLaneChangeOver_t v_lanechange_over;
		MarkerActivePoint_t v_activepoint;
		MarkerActivePoint_t v_activepointinrndf;
		MarkerActivePoint_t v_activepointinroute;
		MarkerNavStopline_t v_nav_stopline;
		MarkerRNDF_t v_rndf;
		MarkerRefreeze_t v_refreeze_map;
		MarkerRefreeze_t v_refreeze_object;
		MarkerInSturn_t v_sturn;
		MarkerSturnDec_t v_sturndec;
		MarkerDynamicObj_t v_dynamic_obj;
		MarkerDynamicObj_t v_dynamic_obj_1;
		MarkerDynamicObjOver_t v_dynamic_obj_over;
		MarkerDynamicObjOver_t v_dynamic_obj_over_1;
		MarkerDevide_t v_devide;
		MarkerIsUturn_t v_isuturn;
		MarkerStartObj_t v_start_obj;
		//MarkerObActivated_t v_ob_activated;
	} value;
} MarkerData_t;

typedef struct PointT {
	int x,y;
	bool operator < (const PointT &pt) const{
	//	return x+y < pt.x+pt.y;
		if(x!=pt.x)
			return x<pt.x;
		return y<pt.y;

	}
} PointT_t;
typedef struct Carpos{
	int num;
	PointT_t pts[200*5];
}Carpos_t;
typedef enum GridType {
	Unknown = 0,
    Car_track = 1,
    Boundary = 2
} GridType_t;
typedef struct Grid {
	//unsigned char p;
	float p;
	unsigned short delta_z;
	GridType_t gt;
} Grid_t;
typedef struct LocalMap {
	Carpos_t carpos;
	map <PointT_t, Grid_t> LocalGridMap;
} LocalMap_t;

}
#endif // __TYPES_H_

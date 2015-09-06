#pragma once

namespace module
{
/*
 * modules enum
 */
enum {
    // controller
    MODULE_CONTROLLER                = 0,
    // decision
    MODULE_DECISION                  = 1,
    // emergency
//	MODULE_EMERGENCY                 = 2,
    MODULE_EMERGENCY_LUX             = 2,
    // slowdown
//	MODULE_SLOWDOWN                  = 4,
    MODULE_SLOWDOWN_LUX              = 3,
    // device capture meta data
    MODULE_CAPTURE_HDL               = 4,
    MODULE_CAPTURE_PYLON             = 5,
    MODULE_CAPTURE_PYLON_MONO        = 6,
    MODULE_CAPTURE_MMW          	 = 7,
    MODULE_CAPTURE_LUX               = 8,

    // navigation
    MODULE_NAV_GPS                   = 9,
    MODULE_NAV_GPS_SHOW              = 10,
    MODULE_NAV_LOCAL                 = 11,

    // basic functions (without decision)
    MODULE_TRAFFICSIGN_RECOGNITION   = 12,
    MODULE_TRAFFICLIGHT_1            = 13,
    MODULE_TRAFFICLIGHT_2            = 14,
    /*MODULE_ROAD_TRACKING             = 15,*/

    MODULE_FUSION            		 = 15,
    MODULE_LANEMARK_SCANNER			 = 16,//
    MODULE_CURB_SCANNER			     = 17,//马路鸭子
    MODULE_RAIL_SCANNER				 = 18,//护栏
    //MODULE_INTERSECTION              = 19,
    //MODULE_LANECHANGE                = 20,
    MODULE_GPS_SCANNER               = 19,
    MODULE_HDLLANESCANNER			 = 20,


    //MODULE_MMWCAP					 = 23,
    MODULE_RADARMAP					= 21,
    MODULE_LUXOBJDT					 = 22,
    MODULE_UNSTRUCTUREDROAD 		 = 23,
    MODULE_CROSSSCANNER 			 = 24,
	MODULE_LOCALPLANNING 			= 25,
    MODULE_MAX 						 =26
                                     //MODULE_CHANGE_PATH				 = 27
};

// utils macro
#define MODULE_INDEX_MIN MODULE_CONTROLLER
#define MODULE_INDEX_MAX MODULE_MAX
#define MODULE_NUM (MODULE_INDEX_MAX - MODULE_INDEX_MIN)

/*
 * modules name
 * Note: users could use these names to generate the module execute file
 */
#define MODULE_CONTROLLER_NAME                "./ModuleController"
#define MODULE_DECISION_NAME                  "./ModuleDecision"
//#define MODULE_EMERGENCY_NAME                 "./ModuleEmergency"
#define MODULE_EMERGENCY_LUX_NAME             "./ModuleEmergencyBrake"
//#define MODULE_SLOWDOWN_NAME                  "./ModuleSlowDown"
#define MODULE_SLOWDOWN_LUX_NAME                  "./ModuleSlowDownLux"

#define MODULE_CAPTURE_HDL_NAME               "./ModuleCapHdl"
#define MODULE_CAPTURE_PYLON_NAME             "./ModuleCapPylon"
#define MODULE_CAPTURE_PYLON_MONO_NAME        "./ModuleCapPylonMono"
#define MODULE_CAPTURE_LUX_NAME               "./ModuleCapLux"
#define MODULE_CAPTURE_MMW_NAME            "./ModuleCapMMW"

#define MODULE_NAV_GPS_NAME                   "./ModuleNavGps"
#define MODULE_NAV_GPS_SHOW_NAME              "./ModuleNavGpsShow"
#define MODULE_NAV_LOCAL_NAME                 "./ModuleNavLocal"

#define MODULE_TRAFFICSIGN_RECOGNITION_NAME   "./ModuleTS"
#define MODULE_TRAFFICLIGHT_1_NAME            "./ModuleSFTL"
#define MODULE_TRAFFICLIGHT_2_NAME            "./ModuleTrackingTL"
//#define MODULE_ROAD_TRACKING_NAME           "./ModuleRoadTracking"
#define MODULE_FUSION_NAME             		  "./ModuleFusion"
#define MODULE_LANEMARK_SCANNER_NAME          "./ModuleLaneMarkScanner"
#define MODULE_CURB_SCANNER_NAME              "./ModuleCurbScanner"
#define MODULE_RAIL_SCANNER_NAME              "./ModuleRailScanner"
//#define MODULE_INTERSECTION_NAME              "./ModuleIntersection"
//#define MODULE_LANECHANGE_NAME                "./ModuleLaneChange"
#define MODULE_GPSPLANNING_NAME               "./ModuleGPSScanner"
#define MODULE_HDLLANESCANNER_NAME			  "./ModuleHDLLaneScanner"
//#define MODULE_MMWCAP_NAME			  		"./ModuleMMWCap"
#define MODULE_RADARMAP_NAME			  		"./ModuleRadarMap"
#define MODULE_LUXOBJDT_NAME			  "./ModuleObjectDetect"
//#define MODULE_INTERSECTION_NAME              "./ModuleIntersection"
//#define MODULE_VEHICLE_FOLLOWING_NAME         "./ModuleVF"
//#define MODULE_S_CURVE_NAME                   "./ModuleSC"
//#define MODULE_PARKING_1_NAME                 "./ModuleP1"
//#define MODULE_SIDE_PARKING_NAME              "./ModuleSideParking"
//#define MODULE_PARKING_2_NAME                 "./ModuleP2"
//#define MODULE_CHANGE_PATH_NAME               "./ModuleCP"
#define MODULE_UNSTRUCTUREDROAD_NAME		"./ModuleUnstructuredRoad"
#define MODULE_CROSSSCANNER_NAME		"./ModuleCrossScanner"
#define MODULE_LOCALPLANNING_NAME		"./ModuleLocalPlanning"
/*
 * define a cleanup handler called when interrupted or abort
 * by the Main Control
 */
typedef void (*ModuleCleanup)(void);

/*
 * users should register a cleanup handler if some devices may
 * need to destroy specific resources or release system memories.
 *
 * Note: only the last one handler is registered if this method
 *   is called more than once.
 */
void RegisterCleanup(ModuleCleanup cleanup);
void UnRegisterCleanup();

}

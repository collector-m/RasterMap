#ifndef RASTERMAP_H_INCLUDED
#define RASTERMAP_H_INCLUDED

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <deque>
#include <queue>

#include <signal.h>
#include <fcntl.h>
//#include <sys/mman.h>

#include "HDLPoint.h"
#include "LUXPoint.h"
#include "Grid.h"
#include "pos.h"
#include "Line.h"

//#include <opencv/cv.h>
//#include <opencv/cxcore.h>
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

using namespace std;

//global grid
//kunchenghu
// #define GLOBAL_MAP_ROWS 11480
// #define GLOBAL_MAP_COLS 17902

//#define GLOBAL_MAP_ROWS 11468
//#define GLOBAL_MAP_COLS 17898

//jiugongge
//#define GLOBAL_MAP_ROWS 4338
//#define GLOBAL_MAP_COLS 5794

//yuanboyuan
#define GLOBAL_MAP_ROWS 8669
#define GLOBAL_MAP_COLS 7137

#define OUTPUT_MAP_ROWS 801
#define OUTPUT_MAP_COLS 801

const double pi = 3.1415926535;

//const double map_cx = -12655.2;
//const double map_cy = -13972.2;

typedef struct Grid_tp
{
	unsigned char p[3];
	//unsigned char type;
} Gridtp_t;

typedef struct _boundingBox
{
    Point vertex[4];
} boundingBox;

typedef struct object
{
    //double xmin, ymin, xmax, ymax;
    boundingBox bbx;
} object_t;

typedef struct _click_pt
{
    float x, y;
} click_pt;

class RasterMap
{
public:
    RasterMap();
    ~RasterMap();

//    struct timeval tv[11], tz;
//    struct timeval tbeg, tend;
//    ofstream timelog;
//    void writeTimeLog();

    Gridtp_t g_map[GLOBAL_MAP_ROWS][GLOBAL_MAP_COLS];
    Gridtp_t out_map[OUTPUT_MAP_ROWS][OUTPUT_MAP_COLS];
    Grid g[OUTPUT_MAP_ROWS][OUTPUT_MAP_COLS];

    //double center_x, center_y;
    module::Pose_t dgps_pose;

    inline int getCol(double x, double y) const;
    inline int getRow(double x, double y) const;
    inline double getx(int row, int col) const;
    inline double gety(int row, int col) const;

    inline int getLocalRow(int global_row, int global_col);
    inline int getLocalCol(int global_row, int global_col);
//    inline int getGlobalRow(int local_row, int local_col);
//    inline int getGlobalCol(int local_row, int local_col);

    inline double getfx(int global_row, int global_col);
    inline double getfy(int global_row, int global_col);

    double getProbability(unsigned char p);
    int getObstacleAttr(unsigned char p);
    inline unsigned char getGridChar(bool IsObject, bool IsDynamic, unsigned char p);
	inline unsigned char getGlobalGridChar(bool IsObject, bool IsDynamic, unsigned char p);

    queue<vector<pos> > pos_queue;
    map<pos, unsigned int> pos_counter;

    void process(bool online = true, bool interact_obstacleList = true);

    bool readGlobalMap();
        bool readGlobalMap_old();
        bool readGlobalMapPng();
		bool readGlobalMap_3b();
        ifstream inGlobalMap;

    bool getSHMNaviData();
        module::MetaData_t m_data_navi;
        module::MetaData_t m_data_local_navi;
        module::Pose_t current_pose, delta_pose;

    bool getSHMGlobalPoints();
		module::RecoData_t m_recodata_global_points;
		module::RecoGlobalPts_t m_global_points;
		module::Pose_t closest_global_pt;
		double getPoseDist(module::Pose_t p1, module::Pose_t p2);

    bool getLocation();
        bool getLocation_v1();
        pos ugv_pos, global_pos;
        double diff_x, diff_y;		//the difference of x and y between UGV and closest global point
		int  diff_rows, diff_cols;	//the difference of row and col between UGV and closest global point

    bool setGrids();
        bool setGrids_v1();

    bool setDetectGrids();
        //functions
        int getBiggestRow(int row1, int row2, int row3, int row4);
        int getSmallestRow(int row1, int row2, int row3, int row4);
        int getBiggestCol(int col1, int col2, int col3, int col4);
        int getSmallestCol(int col1, int col2, int col3, int col4);
        //variables
        Point box_pt, ugv_pt; //detect box and ugv
        Point box_ld, box_rd, box_lu, box_ru;   //4 vertex of detect box
        Point ugv_ld, ugv_rd, ugv_lu, ugv_ru;   //4 vertex of ugv
        int arr_lc, arr_rc, arr_dr, arr_ur; //scan range
        Line left_line, right_line, down_line, up_line; //4 line of detect box

    bool setWhiteGrids(float end_y = 80.0, float stretch_width = 1.0); //1.0);
        Line setLeadLine(float end_y, float rotateAngle = 0);//1.7878);
        void setStretchLines(const Line& leadLine, vector<Line>& stretch_lines);
        void getWhiteGridsOnLine(const Line& strch_line, vector<pos>& white_pos, float stretch_width);
        float getPointsDist(const Point& pt1, const Point& pt2);
        inline bool collide(pos tp);
        //declare some global variables for test
        Line lead_line;
        vector<Line> stretch_lines;
        vector<Point> lead_line_pts;
        vector<Point> stretch_pts;

    bool getWhiteGrids_v1();
        Line getsbLine();
            Line sb;
            vector<Point> line_pts;
            Point left_point, right_point;
        Point getTailPoint(Line ln);
        bool collide(float global_p)
        {
            if(global_p > 0.8)
                return true;
            else
                return false;
        }

    bool getSHMPoints();
        //functions
		bool getSHMPoints_v1();
        bool getSHMLUXPoints();
            bool getSHMLUXPoints_v1();
            module::MetaData_t m_data_lux_points;       //LUX points
            module::MetaLUXPoints_t m_lux_points;       //LUX points
        bool getSHMHDLPoints();
            bool getSHMHDLPoints_v1();
            module::MetaData_t m_data_hdl_points;       //HDL points
            module::MetaLaserHdl_t  m_hdl_points;       //HDL points
        bool IsOnRoad(Point pt);
		bool filterLUXPoints();
		bool HaveDifferentColors(int red, int blue, int green, int yellow);
		//variables
		map<unsigned int, vector<LUXPoint> > lux_filter;
        vector<LUXPoint> lux_pts;
        vector<HDLPoint> hdl_pts;

    bool transPoints();
		module::Pose_t modify_delta_pose;

    bool loadPoints();
        bool loadLUXPoints();
        bool loadHDLPoints();
        //vector<pos_t> lux_pos, hdl_pos;
        vector<pos> lux_pos, hdl_pos;

    bool scanGrids();
        vector<pos> current_pos;
        vector<cluster_pos_t> convex_pos;
        bool updatePosQueue(vector<pos>& current_pos, vector<cluster_pos_t>& convex_pos, unsigned int queue_size = 10);
        bool scanPosQueue(vector<cluster_pos_t>& convex_pos, unsigned int occupy_times = 2);
        //Here, the row and the col in cluster_pos_t is array position

    bool getSHMBasePoint();
        module::MarkerData_t m_data_active_pt;
		module::Pose_t base_pose, frozen_pose;

	bool setSHMOccupyGrids();
		module::RecoData_t m_recodata_occupyGrids;

    bool getSHMObstacles();
        module::RecoData_t m_recodata_obstacleList;
        module::RecoObstacleList_t m_obstacleList;
        vector<module::obstacle_t> obs;

    bool setSHMSubmap();
		module::RecoData_t m_recodata_submap;

    bool writeObstacles();
		map<unsigned int, set<pos> > box_grids;
    bool writeObstacles_v1();
		void generateLines(boundingBox& bbx, vector<Line>& box_lines);
		void getLineGrids(Line grid_line, set<pos>& line_grids);
		//void modifyBoxGrids(set<pos>& box_grids, int diff_rows = 0, int diff_cols = 0);
		Point getLocalPoint(Point global_pt);
		boundingBox getLocalBoundingBox(const boundingBox& global_bbx);
        //void transGlobalToOutput(Point& pt);
        //void transGlobalToLocal(Point& pt);
        //void transBoundingBox_v2(boundingBox& bbx);

    bool writeOccupyGrids();

	void refreshGrids();
		void refreshGrids_bigMap();
		void clearGlobalAttr(int global_row, int global_col);

    bool clearGrids();

    bool clusterOccpuyGrids();
        map<unsigned int, vector<cluster_pos_t> > clusters;
        bool IsAdjacent(const cluster_pos_t& pos1, const cluster_pos_t& pos2);

    bool setObjectList();
        map<unsigned int, vector<Point> > cluster_pts;
        map<unsigned int, object_t> objs;
        object_t setNewObject(const vector<Point>& pts, int line_pairs = 6);
        Point getIntersectPoint(double a1, double b1, double c1, double a2, double b2, double c2);
        boundingBox getBoxVertex(int long_line, Point long_pt1, Point long_pt2, Point short_pt1, Point short_pt2, int line_pairs = 6);
        double getPointDist(const Point& pt1, const Point& pt2);

    bool writeObstaclesTest();
        vector<Line> box_lines;;
    bool clearGridsTest(bool my_mode = true);
    bool processTest();
    bool scanGridsTest();
        vector<click_pt> pts;

};


#endif // RASTERMAP_H_INCLUDED

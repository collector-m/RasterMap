#include "RasterMap.h"


RasterMap::RasterMap()
{
    ugv_pos.row = GLOBAL_MAP_ROWS / 2;
    ugv_pos.col = GLOBAL_MAP_COLS / 2;

    delta_pose.x = 0, delta_pose.y = 0, delta_pose.eulr = 0;

    //jiugongge
    //dgps_pose.x = 422172.4;
    //dgps_pose.y = -918693.6;
    //dgps_pose.eulr = 0;

    //kunchenghu
    //dgps_pose.x = 418758.0;
    //dgps_pose.y = -917173.0;
    //dgps_pose.eulr = 0;

	dgps_pose.x = 2102.7;
    dgps_pose.y = 331.9;
    dgps_pose.eulr = 0;

	//dgps_pose.x = 418758.4;
    //dgps_pose.y = -917174.2;
    //dgps_pose.eulr = 0;


    base_pose.x = 0;
    base_pose.y = 0;
    base_pose.eulr = 0;

    frozen_pose.x = dgps_pose.x - base_pose.x;
    frozen_pose.y = dgps_pose.y - base_pose.y;
    frozen_pose.eulr = dgps_pose.eulr - base_pose.eulr;

    current_pose.x = dgps_pose.x;
    current_pose.y = dgps_pose.y;
    current_pose.eulr = dgps_pose.eulr;

	modify_delta_pose.x = 0.0;
	modify_delta_pose.y = 0.0;
	modify_delta_pose.eulr = 0.0;
//    string timelogFile = "RasterMap_time_log";
//    timelog.open(timelogFile.c_str(), ofstream::app);
}

RasterMap::~RasterMap()
{
    //timelog.close();
}

void RasterMap::process(bool online, bool interact_obstacleList)
{
    if(online) {
        clearGrids();
        getSHMNaviData();
		getSHMGlobalPoints();
        getLocation();
        setGrids();
        setDetectGrids();
		setWhiteGrids();

		//getWhiteGrids_v1();

//        getSHMPoints();
//        filterLUXPoints();
//        transPoints();
//
//        loadPoints();
//        scanGrids();

        if(interact_obstacleList) {
            setSHMOccupyGrids();
            getSHMObstacles();
            writeObstacles();
        } else {
            writeOccupyGrids();
        }

		//refreshGrids();
        setSHMSubmap();
    } else {
        processTest();
    }
}

bool RasterMap::processTest()
{
    clearGridsTest();
    getSHMNaviData();
    getLocation();
    setGrids();
    setDetectGrids();
    setWhiteGrids();

    getSHMPoints();
	filterLUXPoints();
    transPoints();

    loadPoints();
    scanGridsTest();
    clusterOccpuyGrids();
    setObjectList();
    writeObstaclesTest();

    return true;
}

bool RasterMap::getSHMBasePoint()
{
	m_data_active_pt.type = module::MarkerData::MARKER_ACTIVEPOINTINROUTE;
//    module::shm::SHARED_OBJECTS.GetMarker(&m_data_active_pt);

	base_pose.x = m_data_active_pt.value.v_activepointinroute.pre_pt.x;
    base_pose.y = m_data_active_pt.value.v_activepointinroute.pre_pt.y;
    base_pose.eulr = m_data_active_pt.value.v_activepointinroute.pre_pt.eulr;

    base_pose.x = dgps_pose.x;
    base_pose.y = dgps_pose.y;
    base_pose.eulr = dgps_pose.eulr;

	base_pose.x = 0;
    base_pose.y = 0;
    base_pose.eulr = 0;

	cout << "Base_pose: " << " x: " << base_pose.x << " y: " << base_pose.y << " eulr: " << base_pose.eulr << endl;

	frozen_pose.x = dgps_pose.x - base_pose.x;
    frozen_pose.y = dgps_pose.y - base_pose.y;
    frozen_pose.eulr = 0;
	cout << "Frozen_pose: " << " x: " << frozen_pose.x << " y: " << frozen_pose.y << " eulr: " << frozen_pose.eulr << endl;
	return true;
}

bool RasterMap::readGlobalMap_3b()//read from .3b
{
    //string GlobalMapFile = "/home/denggroup/UGV/bin/map/changshu.3b";
    string GlobalMapFile = "/home/denggroup/UGV/bin/map/kunchenghu.3b";
    //string GlobalMapFile = "./map/changshu.3b";
    //string GlobalMapFile = "./map/kunchenghu.3b";
    inGlobalMap.open(GlobalMapFile.c_str(), ifstream::binary);
    if(!inGlobalMap) {
        cerr << "error: unable to open global map file: "
            << inGlobalMap << endl;
        return false;
    } else {
        cout << "read the global map!" << endl;
        for(int i = 0; i < GLOBAL_MAP_ROWS; i++) {
            for(int j = 0; j < GLOBAL_MAP_COLS; j++) {
                inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[0]), sizeof(unsigned char));
				inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[1]), sizeof(unsigned char));
                inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[2]), sizeof(unsigned char));
//                unsigned char test = g_map[i][j].p[0];
//                test >> 6;
//                cout << "p[0]: " << (int)g_map[i][j].p[0] << "p[1]: " << (int)g_map[i][j].p[1]
//                    << "p[2]: " << (int)g_map[i][j].p[2] << endl;
            }
        }
    }
    return true;
}

bool RasterMap::readGlobalMap() //read from .png
{
	//string GlobalMapFile = "/home/denggroup/UGV/bin/map/yuanboyuan.png";
	// string GlobalMapFile = "/home/wangzhy/Workspace/Run/RasterMap/map_visualize.png";
	//string GlobalMapFile = "/home/wangzhy/Workspace/Run/RasterMap/map_final.3b.png";
	//string GlobalMapFile = "/home/denggroup/UGV/bin/map/map_final.3b.png";
	string GlobalMapFile = "./map_final.3b.png";
	inGlobalMap.open(GlobalMapFile.c_str(), ifstream::binary);
    if(!inGlobalMap) {
        cerr << "error: unable to open global map file: "
            << inGlobalMap << endl;
        return false;
    } else {
        cout << "read the global map!" << endl;
        IplImage* ipl = cvLoadImage(GlobalMapFile.c_str());
        for(int i = 0; i < GLOBAL_MAP_ROWS; i++) {
            for(int j = 0; j < GLOBAL_MAP_COLS; j++) {
            	uchar* ptr = (uchar*) (ipl->imageData + (GLOBAL_MAP_ROWS - i - 1) * ipl->widthStep );
            	g_map[i][j].p[0] = ptr[3*j];
            	g_map[i][j].p[1] = ptr[3*j+1];
            	g_map[i][j].p[2] = ptr[3*j+2];
//                unsigned char test = g_map[i][j].p[0];
//                test >> 6;
//                cout << "p[0]: " << (int)g_map[i][j].p[0] << "p[1]: " << (int)g_map[i][j].p[1]
//                    << "p[2]: " << (int)g_map[i][j].p[2] << endl;
            }
        }
    }
    return true;
}

bool RasterMap::readGlobalMap_old()  //read from .gmm
{
    //center_x = 194.6, center_y = 794.9;
    cout << "read the global map!" << endl;
    string GlobalMapFile = "./map/yuanboyuan.gmm";
    //string GlobalMapFile = "./map/wangqiuchang.gmm";
    inGlobalMap.open(GlobalMapFile.c_str(), ifstream::binary);
    if(!inGlobalMap) {
         cerr << "error: unable to open global map file: "
             << inGlobalMap << endl;
        return false;
	} else {
//        inGlobalMap.read(reinterpret_cast<char *>(&pose_num), sizeof(int));
//        cout << "pose number: " << pose_num << endl;
//        map_pos car_pos[pose_num];
//        inGlobalMap.read(reinterpret_cast<char *>(car_pos), sizeof(map_pos) * pose_num);
        for(int i = 0; i < GLOBAL_MAP_ROWS; i++) {
            for(int j = 0; j < GLOBAL_MAP_COLS; j++) {
                inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[0]), sizeof(unsigned char));
				//inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[1]), sizeof(unsigned char));
				g_map[i][j].p[1] = 0;
				g_map[i][j].p[2] = 0;
                //inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].p[2]), sizeof(unsigned char));
                //g_map[i][j].type = 0;
                //inGlobalMap.read(reinterpret_cast<char *>(&g_map[i][j].type), sizeof(unsigned char));
            }
        }
        cout << "read the global map successfully!" << endl;
	}
	inGlobalMap.close();
	return true;
}

double RasterMap::getfx(int global_row, int global_col)
{
    double diff_x = (global_col - GLOBAL_MAP_COLS / 2) * 0.2;
    double fx = frozen_pose.x + diff_x;
    return fx;
}

double RasterMap::getfy(int global_row, int global_col)
{
    double diff_y = (global_row - GLOBAL_MAP_ROWS / 2) * 0.2;
    double fy = frozen_pose.y + diff_y;
    return fy;
}

double RasterMap::getProbability(unsigned char p)
{
	//this function is used to get the probability

//	unsigned char b = 0xC0;
//	unsigned char result = b & p;
//	double r;
//    if(result == 0x00)
//        r = 0.0;
//    else if(result == 0x40)
//        r = 1.0;
//    else if(result == 0x80)
//        r = 0.5;
//	return r;
    p = p >> 6;
    //cout << (int)p << endl;
    if(p == 0)
        return 0.5;
    else if(p == 1)
        return 0.0;
    else if(p == 2)
        return 1.0;
    else
        return 0.5;
}

int RasterMap::getObstacleAttr(unsigned char p)
{
    p = p << 2;
    p = p >> 5;
    if(p == 0)
        return 0;
    else if(p == 1)
        return 1;
    else if(p == 2)
        return 2;
    else
        return 3;
}

unsigned char RasterMap::getGlobalGridChar(bool IsObject, bool IsDynamic, unsigned char p)
{
	unsigned char b, result;
	if(IsObject) {
		if(IsDynamic) {
			b = 0xF7;
			result = b | p;
		} else {
			b = 0xD7;
			result = b | p;
		}
	}
	return result;
}

unsigned char RasterMap::getGridChar(bool IsObject, bool IsDynamic, unsigned char p)
{
	unsigned char b, result;
	if(IsObject) {
		if(IsDynamic) {
			//dynamic object
			//binary: 110 --> 00110000
			//b = 0xF7;
			b = 0x30;
			result = b | p;
		} else {
			//static object
			//binary: 010 --> 00010000
			//b = 0xD7;
			b = 0x10;
			result = b | p;
		}

	} else {
		//no object
		//binary: 000 --> 00000000
		//b = 0xC7;
		b = 0x00;
		result = b | p;
	}
	return result;
}

bool RasterMap::getSHMNaviData()
{
    m_data_local_navi.type = module::MetaData::META_LOCAL_NAVIGATION;
//	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_local_navi);
    current_pose.x =  m_data_local_navi.value.v_navi_local.UGVtoFCF[0];
    current_pose.y =  m_data_local_navi.value.v_navi_local.UGVtoFCF[1];
    current_pose.eulr =  m_data_local_navi.value.v_navi_local.UGVtoFCF[2];

	//add to debug
	current_pose.x =  dgps_pose.x;
    current_pose.y =  dgps_pose.y;
    current_pose.eulr =  dgps_pose.eulr;

	current_pose.x += delta_pose.x;
    current_pose.y += delta_pose.y;
    current_pose.eulr += delta_pose.eulr;

    cout << "Delta_pose: " << " x: " << delta_pose.x << " y: " << delta_pose.y << " eulr: " << delta_pose.eulr << endl;
    cout << "Current_pose: " << " x: " << current_pose.x << " y : " << current_pose.y << " eulr: " << current_pose.eulr << endl;
    return true;
}

bool RasterMap::getSHMGlobalPoints()
{
	//typedef struct RecoGlobalPts{
	//	int num;
	//	enum{
	//	GlobalPtsNum = 5000
	//	};
	//	Pose_t pts[GlobalPtsNum];
	//}RecoGlobalPts_t;

	//input:
	//	module::Pose_t current_pose;
	//output:
	//	module::RecoData_t m_recodata_global_points;
	//	module::RecoGlobalPts_t m_global_points;
	m_recodata_global_points.type = module::RecoData::RT_GLOBALPTS;
//	module::shm::SHARED_OBJECTS.GetRecoData(&m_recodata_global_points);
	m_global_points = m_recodata_global_points.value.v_globalpts;
	if(m_global_points.num > 0) {
		double closest_dist = 100000.0;
		closest_global_pt = m_global_points.pts[0];
		for(int i = 0; i < m_global_points.num; i++) {
			double current_global_dist = getPoseDist(current_pose, m_global_points.pts[i]);
			if(current_global_dist < closest_dist) {
				closest_global_pt = m_global_points.pts[i];
				closest_dist = current_global_dist;
			}
		}
		if(closest_dist >= 30) {
			closest_global_pt = current_pose;
			cout << "NOT select global points!" << endl;
		} else {
			cout << "Select global points!" << endl;
		}
	} else {
		closest_global_pt = current_pose;
	}

	//add to set closest_global_pt
	closest_global_pt = current_pose;


	cout << "Closest global point:   x: " << closest_global_pt.x << "  " << closest_global_pt.y << endl;
	return true;
}

double RasterMap::getPoseDist(module::Pose_t p1, module::Pose_t p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool RasterMap::getLocation()
{
	//This function is used to locate the vehicle on the global map
    //input:  1. Navigation data: current_pose
	//        2. Closest global point: closest_global_pt
    //        3. Global map center on frozen coordinate: frozen_pose
    //output: 1. ugv_row, ugv_col
	//		  2. global_pt_row, global_pt_col
	//	      3. diff_x, diff_y
	//		  4. diff_rows, diff_cols

	diff_x = diff_y = 0;
	diff_rows = diff_cols = 0;

	double local_diff_x  = current_pose.x - frozen_pose.x;
	double local_diff_y  = current_pose.y - frozen_pose.y;
	double output_diff_x = closest_global_pt.x - frozen_pose.x;
	double output_diff_y = closest_global_pt.y - frozen_pose.y;

	int local_diff_row  = local_diff_y / 0.20;
	int local_diff_col  = local_diff_x / 0.20;
	int output_diff_row = output_diff_y / 0.20;
	int output_diff_col = output_diff_x / 0.20;

	//ugv_row = (GLOBAL_MAP_ROWS / 2) + local_diff_row;
    //ugv_col = (GLOBAL_MAP_COLS / 2) + local_diff_col;
	//global_pt_row = (GLOBAL_MAP_ROWS / 2) + output_diff_row;
	//global_pt_col = (GLOBAL_MAP_COLS / 2) + output_diff_col;

	ugv_pos.row = (GLOBAL_MAP_ROWS / 2) + local_diff_row;
	ugv_pos.col = (GLOBAL_MAP_COLS / 2) + local_diff_col;
	global_pos.row = (GLOBAL_MAP_ROWS / 2) + output_diff_row;
	global_pos.col = (GLOBAL_MAP_COLS / 2) + output_diff_col;

	diff_x = closest_global_pt.x - current_pose.x;
	diff_y = closest_global_pt.y - current_pose.y;
	//diff_rows = global_pt_row - ugv_row;
	//diff_cols = global_pt_col - ugv_col;
	diff_rows = global_pos.row - ugv_pos.row;
	diff_cols = global_pos.col - ugv_pos.col;

	cout << "UGV row: " << ugv_pos.row << " col: " << ugv_pos.col << endl;
	cout << "Global row: " << global_pos.row << " col: " << global_pos.col << endl;
	return true;
}

bool RasterMap::getLocation_v1()
{
    //This function is used to locate the vehicle on the global map
    //input:  1. Navigation data: current_pose
    //        2. Center GPS value on global map: center_x, center_y
    //output: ugv_row, ugv_col
    //          array position

    double diff_x = current_pose.x - frozen_pose.x;
    double diff_y = current_pose.y - frozen_pose.y;
    int diff_row = diff_y / 0.2;
    int diff_col = diff_x / 0.2;
    ugv_pos.row = (GLOBAL_MAP_ROWS / 2) + diff_row;
    ugv_pos.col = (GLOBAL_MAP_COLS / 2) + diff_col;

    int ugv_y = (ugv_pos.row - GLOBAL_MAP_ROWS / 2) * 0.2;
    int ugv_x = (ugv_pos.col - GLOBAL_MAP_COLS / 2) * 0.2;

    cout << "UGV Location: " << " row: " << ugv_pos.row << " col: " << ugv_pos.col
        << " x: " << ugv_x << " y: " << ugv_y  << endl;
    return true;
}

bool RasterMap::setGrids()
{
	//input:
	//	1. ugv_row, ugv_col
	//	2. global_pt_row, global_pt_col
	int local_beg_row  = ugv_pos.row - OUTPUT_MAP_ROWS / 2;
	int local_beg_col  = ugv_pos.col - OUTPUT_MAP_COLS / 2;
	int output_beg_row = global_pos.row - OUTPUT_MAP_ROWS / 2;
	int output_beg_col = global_pos.col - OUTPUT_MAP_COLS / 2;

	int local_gmap_row = 0, local_gmap_col = 0;
	int output_gmap_row = 0, output_gmap_col = 0;

	for(int i = 0; i < OUTPUT_MAP_ROWS; i++) {
        local_gmap_row  = local_beg_row + i;
		output_gmap_row = output_beg_row + i;
        for(int j = 0; j < OUTPUT_MAP_COLS; j++) {
            local_gmap_col  = local_beg_col + j;
			output_gmap_col = output_beg_col + j;

			//set the 'g'
			g[i][j].row = local_gmap_row;
            g[i][j].col = local_gmap_col;
			if((local_gmap_row >= 0) &&  (local_gmap_col >= 0) &&  (local_gmap_row < GLOBAL_MAP_ROWS) &&  (local_gmap_col < GLOBAL_MAP_COLS)) {
                g[i][j].global_p = getProbability(g_map[local_gmap_row][local_gmap_col].p[0]);
				//g[i][j].global_p = 0.0;     //added 1102
			} else {
				g[i][j].global_p = 0.5;
			}
			//set the out_map
			if((output_gmap_row >= 0) && (output_gmap_col >= 0) && (output_gmap_row < GLOBAL_MAP_ROWS) && (output_gmap_col < GLOBAL_MAP_COLS)) {
				out_map[i][j].p[0] = g_map[output_gmap_row][output_gmap_col].p[0];
                out_map[i][j].p[1] = g_map[output_gmap_row][output_gmap_col].p[1];
                out_map[i][j].p[2] = g_map[output_gmap_row][output_gmap_col].p[2];
				//add 1102
				//out_map[i][j].p[0] = 0x40;
                //out_map[i][j].p[1] = 0x00;
                //out_map[i][j].p[2] = 0x00;
			} else {
				out_map[i][j].p[0] = 0x80;
                out_map[i][j].p[1] = 0x00;
                out_map[i][j].p[2] = 0x00;
				//add 1102
				//out_map[i][j].p[0] = 0x40;
                //out_map[i][j].p[1] = 0x00;
                //out_map[i][j].p[2] = 0x00;
			}
		}
	}
	return true;
}

bool RasterMap::setGrids_v1()
{
    //This function is used to set the global probability in 'g'

    int beg_row = ugv_pos.row - OUTPUT_MAP_ROWS / 2;
    int beg_col = ugv_pos.col - OUTPUT_MAP_COLS / 2;
    int gmap_row = 0;
    int gmap_col = 0;

    for(int i = 0; i < OUTPUT_MAP_ROWS; i++) {
        gmap_row = beg_row + i;
        for(int j = 0; j < OUTPUT_MAP_COLS; j++) {
            gmap_col = beg_col + j;
            g[i][j].row = gmap_row;
            g[i][j].col = gmap_col;
            if((gmap_row >= 0) && (gmap_col >= 0) && (gmap_row < GLOBAL_MAP_ROWS) && (gmap_col < GLOBAL_MAP_COLS)) {
                g[i][j].global_p = getProbability(g_map[gmap_row][gmap_col].p[0]);
                out_map[i][j].p[0] = g_map[gmap_row][gmap_col].p[0];
                out_map[i][j].p[1] = g_map[gmap_row][gmap_col].p[1];
                out_map[i][j].p[2] = g_map[gmap_row][gmap_col].p[2];
            } else {
                g[i][j].global_p = 0.5;
                out_map[i][j].p[0] = 0x80;
                out_map[i][j].p[1] = 0x00;
                out_map[i][j].p[2] = 0x00;
            }
        }
    }

//	for(int i = 0; i < OUTPUT_MAP_ROWS; i++) {
//        for(int j = 0; j < OUTPUT_MAP_COLS; j++) {
//			//g[i][j].global_p = 0.5;
//			//cout << g[i][j].global_p << endl;
//		}
//	}
	return true;
}

int RasterMap::getRow(double x, double y) const
{
    int row = y / 0.2;
    return row + OUTPUT_MAP_ROWS / 2;
}

int RasterMap::getCol(double x, double y) const
{
    int col = x / 0.2;
    return col + OUTPUT_MAP_COLS / 2;
}

double RasterMap::getx(int row, int col) const
{
    return (col -  OUTPUT_MAP_COLS / 2) * 0.2;
}

double RasterMap::gety(int row, int col) const
{
    return (row -  OUTPUT_MAP_ROWS / 2) * 0.2;
}

int RasterMap::getLocalRow(int global_row, int global_col)
{
    return global_row - ugv_pos.row + OUTPUT_MAP_ROWS / 2;
}


int RasterMap::getLocalCol(int global_row, int global_col)
{
    return global_col - ugv_pos.col + OUTPUT_MAP_COLS / 2;
}

//int RasterMap::getGlobalRow(int local_row, int local_col)
//{
//    return local_row;
//}
//
//int RasterMap::getGlobalCol(int local_row, int local_col)
//{
//    return local_col;
//}

bool RasterMap::setDetectGrids()
{
     //set the center of detect box
    box_pt.setPoint((detect_left + detect_right)/2 , (detect_down + detect_up)/2);
	//cout << "box: " << " x: " << box.x << " y: " << box.y << endl;
	box_pt.transToFrozen(current_pose);

    ugv_pt.setPoint(0, 0);
    ugv_pt.transToFrozen(current_pose);

    //set the 4 vertex of detect box
    //left col, right col, front row, back row
    box_ld.setPoint(detect_left, detect_down);
    box_rd.setPoint(detect_right, detect_down);
    box_lu.setPoint(detect_left, detect_up);
    box_ru.setPoint(detect_right, detect_up);
//    cout << "(" << detect_left << ", " << detect_back << "), " << "(" << detect_right << ", " << detect_back << "), "
//            << "(" << detect_left << ", " << detect_front << "), " <<"(" << detect_right << ", " << detect_front << ")" << endl;
//    cout << "Before transform: " << endl;
//    cout << "left down point: " << "(" << pt_ld.x << ", " << pt_ld.y << "), "
//            << "right down point: "<< " (" << pt_rd.x << ", " << pt_rd.y << "), "
//            << "left up point: "   << "(" << pt_lu.x << ", " << pt_lu.y << "), "
//            << "right up point: "  << "(" << pt_ru.x << ", " << pt_ru.y << ") " << endl;

    //set the 4 vertex of ugv
    ugv_ld.setPoint(-1.2, -2.5);
    ugv_rd.setPoint(1.2,  -2.5);
    ugv_lu.setPoint(-1.2,  2.5);
    ugv_ru.setPoint(1.2,   2.5);

    box_ld.transToFrozen(current_pose);
    box_rd.transToFrozen(current_pose);
    box_lu.transToFrozen(current_pose);
    box_ru.transToFrozen(current_pose);
//    cout << "After transform: " << endl;
//    cout << "left down point: " << "(" << pt_ld.x << ", " << pt_ld.y << "), "
//            << "right down point: "<< " (" << pt_rd.x << ", " << pt_rd.y << "), "
//            << "left up point: "   << "(" << pt_lu.x << ", " << pt_lu.y << "), "
//            << "right up point: "  << "(" << pt_ru.x << ", " << pt_ru.y << ") " << endl;
    ugv_ld.transToFrozen(current_pose);
    ugv_rd.transToFrozen(current_pose);
    ugv_lu.transToFrozen(current_pose);
    ugv_ru.transToFrozen(current_pose);


    //get the scan range
    int left_col  = getSmallestCol(getCol(box_ld.x, box_ld.y), getCol(box_lu.x, box_lu.y), getCol(box_rd.x, box_rd.y), getCol(box_ru.x, box_ru.y));
    int right_col = getBiggestCol(getCol(box_ld.x, box_ld.y), getCol(box_lu.x, box_lu.y), getCol(box_rd.x, box_rd.y), getCol(box_ru.x, box_ru.y));
    int down_row  = getSmallestRow(getRow(box_ld.x, box_ld.y), getRow(box_lu.x, box_lu.y), getRow(box_rd.x, box_rd.y), getRow(box_ru.x, box_ru.y));
    int up_row    = getBiggestRow(getRow(box_ld.x, box_ld.y), getRow(box_lu.x, box_lu.y), getRow(box_rd.x, box_rd.y), getRow(box_ru.x, box_ru.y));

    arr_lc = (left_col > 0) ? left_col : 0;
    arr_rc = (right_col < OUTPUT_MAP_COLS - 1) ? right_col : OUTPUT_MAP_COLS - 1;
    arr_dr = (down_row > 0) ? down_row : 0;
    arr_ur = (up_row < OUTPUT_MAP_ROWS - 1) ? up_row : OUTPUT_MAP_ROWS - 1;

    cout << "Scan ROW range: " << arr_dr << " ~ " << arr_ur << endl;
    cout << "Scan COL range: " << arr_lc << " ~ " << arr_rc << endl;
    //cout << "We need to scan " << (arr_rc - arr_lc + 1) * (arr_ur - arr_dr + 1) << " grids!" << endl;
    //set the 4 line of detect box
    left_line.setLine(box_ld, box_lu);
    right_line.setLine(box_rd, box_ru);
    down_line.setLine(box_ld, box_rd);
    up_line.setLine(box_lu, box_ru);

    int box_grids = 0;
    for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            double x = getx(i, j);
            double y = gety(i, j);
            Point pt;
            pt.setPoint(x, y);
            if((left_line.getPointDist(pt, box_pt) >= 0) && (right_line.getPointDist(pt, box_pt) >= 0)
                    && (up_line.getPointDist(pt, box_pt) >= 0)  && (down_line.getPointDist(pt, box_pt)) >= 0) {
                g[i][j].IsInDetectBox = true;
                g[i][j].IsWhite = false;
                box_grids ++;
            } else {
                g[i][j].IsInDetectBox = false;
                g[i][j].IsWhite = false;
            }
        }
    }
    cout << "There are " << box_grids << " grids in detect box!" << endl;
    return true;
}

bool RasterMap::getSHMPoints()
{
    lux_pts.clear(), hdl_pts.clear();
    if(getSHMLUXPoints())
        cout << lux_pts.size() << " LUX points in detect box! "<<endl;
    if(getSHMHDLPoints())
        cout << hdl_pts.size() << " HDL points in detect box! "<<endl;
    return true;
}

bool RasterMap::getSHMPoints_v1()
{
	lux_pts.clear(), hdl_pts.clear();
    if(getSHMLUXPoints_v1())
        cout << lux_pts.size() << " LUX points in detect box! "<<endl;
    if(getSHMHDLPoints_v1())
        cout << hdl_pts.size() << " HDL points in detect box! "<<endl;
    return true;
}

bool RasterMap::getSHMLUXPoints()
{
    lux_pts.clear();
	lux_filter.clear();
	//determine the data type
	m_data_lux_points.type = module::MetaData::META_LUX_POINTS;
//	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_lux_points);
    m_lux_points = m_data_lux_points.value.v_luxPts;
	unsigned int pt_num = 0;
    for(unsigned int i = 0; i < m_lux_points.m_uPoints; i++) {
        LUXPoint tmp_pt(m_lux_points.m_pts[i].m_channel, \
                        m_lux_points.m_pts[i].m_dist, \
                        m_lux_points.m_pts[i].m_x, \
                        m_lux_points.m_pts[i].m_y);
		//modify the LUX points data
//		tmp_pt.x += -1.5207;
//		tmp_pt.y -= 0.2915;

		if(IsOnRoad(tmp_pt)) {
			unsigned int i = tmp_pt.y;
			lux_filter[i].push_back(tmp_pt);
			pt_num++;
		}
    }
	cout << "Before filter: " << pt_num << " lux pts!" << endl;
	return true;
}

bool RasterMap::getSHMHDLPoints()
{
    m_data_hdl_points.type = module::MetaData::META_LASER_HDL;
//	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_hdl_points);
    m_hdl_points = m_data_hdl_points.value.v_laserHdl;
    for(int i = 0; i < m_hdl_points.pts_count; i++) {
        HDLPoint tmp_pt(m_hdl_points.pts[i].x, \
                        m_hdl_points.pts[i].y, \
                        m_hdl_points.pts[i].z, \
                        m_hdl_points.pts[i].dist, \
                        m_hdl_points.pts[i].rot, \
                        m_hdl_points.pts[i].i, \
                        m_hdl_points.pts[i].c);
        //modify the HDL points data
        if(IsOnRoad(tmp_pt))
            hdl_pts.push_back(tmp_pt);
    }
	//cout << "There are " << hdl_pts.size() << " hdl points in detect box!" << endl;
    return true;
}

bool RasterMap::IsOnRoad(Point pt)
{
	pt.transToFrozen(current_pose);
	int local_row = getRow(pt.x, pt.y);
	int local_col = getCol(pt.x, pt.y);
	if(g[local_row][local_col].IsWhite) {
		return true;
	} else {
		return false;
	}
}

bool RasterMap::getSHMLUXPoints_v1()
{
    lux_pts.clear();
	lux_filter.clear();
	//determine the data type
	m_data_lux_points.type = module::MetaData::META_LUX_POINTS;
//	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_lux_points);
    m_lux_points = m_data_lux_points.value.v_luxPts;
	//read the points from shared memory
	//cout << "read points data from shared memory!" << endl;
	//cout << "points number:" << m_lux_points.m_uPoints << endl;
	unsigned int pt_num = 0;
    for(unsigned int i = 0; i < m_lux_points.m_uPoints; i++) {
        LUXPoint tmp_pt(m_lux_points.m_pts[i].m_channel, \
                        m_lux_points.m_pts[i].m_dist, \
                        m_lux_points.m_pts[i].m_x, \
                        m_lux_points.m_pts[i].m_y);
		//modify the LUX points data
//		tmp_pt.x += -1.5207;
//		tmp_pt.y -= 0.2915;

		//cout << "i:" << i
		//	<< " dist:" <<  m_points.m_pts[i].m_dist;

//		if(tmp_pt.IsInDetectBox())
//            lux_pts.push_back(tmp_pt);

		if(tmp_pt.IsInFilterBox()) {
				unsigned int i = tmp_pt.y;
				lux_filter[i].push_back(tmp_pt);
				pt_num++;
		}
    }
	cout << "Before filter: " << pt_num << " lux pts!" << endl;
	return true;
}

bool RasterMap::getSHMHDLPoints_v1()
{
    m_data_hdl_points.type = module::MetaData::META_LASER_HDL;
//	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_hdl_points);
    m_hdl_points = m_data_hdl_points.value.v_laserHdl;
    for(int i = 0; i < m_hdl_points.pts_count; i++) {
        HDLPoint tmp_pt(m_hdl_points.pts[i].x, \
                        m_hdl_points.pts[i].y, \
                        m_hdl_points.pts[i].z, \
                        m_hdl_points.pts[i].dist, \
                        m_hdl_points.pts[i].rot, \
                        m_hdl_points.pts[i].i, \
                        m_hdl_points.pts[i].c);
        //modify the HDL points data
        if(tmp_pt.IsInDetectBox())
            hdl_pts.push_back(tmp_pt);
    }
	//cout << "There are " << hdl_pts.size() << " hdl points in detect box!" << endl;
    return true;
}

bool RasterMap::filterLUXPoints()
{
	unsigned int pt_num = 0;
	for(map<unsigned int, vector<LUXPoint> >::iterator filter_iter = lux_filter.begin();
			filter_iter != lux_filter.end(); filter_iter++) {
		//bool diff_color = false;
		unsigned int red_points_num, blue_points_num, green_points_num, yellow_points_num;
		red_points_num = 0;
		blue_points_num = 0;
		green_points_num = 0;
		yellow_points_num = 0;
		if(filter_iter->second.size() >= 4) {
			for(vector<LUXPoint>::iterator pt_iter = filter_iter->second.begin();
					pt_iter != filter_iter->second.end(); pt_iter++) {
				if(pt_iter->channel == 0)
					red_points_num++;
				if(pt_iter->channel == 1)
					blue_points_num++;
				if(pt_iter->channel == 2)
					green_points_num++;
				if(pt_iter->channel == 3)
					yellow_points_num++;
			}
			//if(red_points_num == filter_iter->second.size() || blue_points_num == filter_iter->second.size()) {
			//	filter_iter->second.clear();
			if(HaveDifferentColors(red_points_num, blue_points_num, green_points_num, yellow_points_num)) {
				for(vector<LUXPoint>::iterator pt_iter = filter_iter->second.begin();
						pt_iter != filter_iter->second.end(); pt_iter++) {
					if(pt_iter->IsInDetectBox()) {
						lux_pts.push_back(*pt_iter);
						pt_num++;
					}
				}
			}
		}
	}
	cout << "After filter: " << pt_num << " lux points!" << endl;
	return true;
}

bool RasterMap::HaveDifferentColors(int red, int blue, int green, int yellow)
{
    if((red >= 3 && blue >=3) || (blue >=2 && green >=2) || (green >= 2 && yellow >=2))
		//|| (blue >= 3) || (green >= 3) )
        return true;
    else
        return false;
}

bool RasterMap::transPoints()
{
    module::Pose_t modify_pose;
	//modify_pose.x = current_pose.x;
	//modify_pose.y = current_pose.y;
	//modify_pose.eulr = current_pose.eulr;// + 2.000 * pi / 180.0;


	modify_pose.x = current_pose.x + modify_delta_pose.x;
	modify_pose.y = current_pose.y + modify_delta_pose.y;
	modify_pose.eulr = current_pose.eulr + modify_delta_pose.eulr;

	//cout << "********** Modify eulr: " << modify_pose.eulr << "***********" << endl;

	//transform LUX points and HDL points to frozen coordinate
    for(vector<LUXPoint>::iterator iter = lux_pts.begin();
            iter != lux_pts.end(); ++iter) {
        (*iter).transToFrozen(modify_pose);
    }
    //cout << "LUX points have been transformed to frozen coordinate!" << endl;
    for(vector<HDLPoint>::iterator iter = hdl_pts.begin();
            iter != hdl_pts.end(); ++iter) {
        (*iter).transToFrozen(modify_pose);
    }
    //cout << "HDL points have been transformed to frozen coordinate!" << endl;
    return true;
}

int RasterMap::getBiggestRow(int row1, int row2, int row3, int row4)
{
    int biggest_row = row1;
    if(biggest_row < row2)
        biggest_row = row2;
    if(biggest_row < row3)
        biggest_row = row3;
    if(biggest_row < row4)
        biggest_row = row4;
    return biggest_row;
}

int RasterMap::getSmallestRow(int row1, int row2, int row3, int row4)
{
    int smallest_row = row1;
    if(smallest_row > row2)
        smallest_row = row2;
    if(smallest_row > row3)
        smallest_row = row3;
    if(smallest_row > row4)
        smallest_row = row4;
    return smallest_row;
}

int RasterMap::getBiggestCol(int col1, int col2, int col3, int col4)
{
    int biggest_col = col1;
    if(biggest_col < col2)
        biggest_col = col2;
    if(biggest_col < col3)
        biggest_col = col3;
    if(biggest_col < col4)
        biggest_col = col4;
    return biggest_col;
}

int RasterMap::getSmallestCol(int col1, int col2, int col3, int col4)
{
    int smallest_col = col1;
    if(smallest_col > col2)
        smallest_col = col2;
    if(smallest_col > col3)
        smallest_col = col3;
    if(smallest_col > col4)
        smallest_col = col4;
    return smallest_col;
}

bool RasterMap::setWhiteGrids(float end_y, float stretch_width)
{
    //vector<Line> stretch_lines;
	vector<pos> white_pos;
	//Line lead_line = setLeadLine(end_y);      //used for real process

	lead_line = setLeadLine(end_y);             //used for test

    //cout << "Lead Line: " << " a: " << lead_line.a << " b: " << lead_line.b << " c: " << lead_line.c << endl;
	setStretchLines(lead_line, stretch_lines);
	//cout << "Stretch lines " << stretch_lines.size() << endl;
	for(vector<Line>::const_iterator line_iter = stretch_lines.begin();
			line_iter != stretch_lines.end(); line_iter++ ) {
		getWhiteGridsOnLine(*line_iter, white_pos, stretch_width);
	}
	//cout << "White pos number: " << white_pos.size() << endl;
	for(vector<pos>::const_iterator pos_iter = white_pos.begin();
			pos_iter != white_pos.end(); pos_iter++) {
        //cout << "row: " << pos_iter->row << " col: " << pos_iter->col << endl;
		g[pos_iter->row][pos_iter->col].IsWhite = true;
	}
	return true;
}

//set the leading line
Line RasterMap::setLeadLine(float end_y, float rotateAngle)
{
	Point start_pt;
	Point end_pt;
	start_pt.setPoint(0.0, 0.0);
	end_pt.setPoint(0.0, end_y);

	//add to rotate an angle
	module::Pose_t origin_pose;
	origin_pose.x = current_pose.x;
	origin_pose.y = current_pose.y;
	origin_pose.eulr = current_pose.eulr + (rotateAngle * pi / 180);

	start_pt.transToFrozen(current_pose);
	end_pt.transToFrozen(current_pose);
	Line lead_line;
	lead_line.setLine(start_pt, end_pt);
	return lead_line;
}

//set the stretch lines on leading line
void RasterMap::setStretchLines(const Line& leadLine, vector<Line>& stretch_lines)
{
    lead_line_pts.clear();
	stretch_lines.clear();
    stretch_pts.clear();

    //vector<Point> lead_line_pts;
    //vector<Point> stretch_pts;

	if(leadLine.getk()) {
		//set the increment
		//cout << "Increment variable X! " << endl;
		//cout << "Lead line pt_down: ( " << leadLine.pt_down.x << " , " << leadLine.pt_down.y << " ) "
        //    << " pt_up: ( " << leadLine.pt_up.x << " , " << leadLine.pt_up.y << " ) " << endl;
		float delta_x = 0;
		if(leadLine.pt_down.x <= leadLine.pt_up.x) {
			delta_x = 0.10;
			//set the stretch points
            for(float cx = leadLine.pt_down.x; cx <= leadLine.pt_up.x; cx += delta_x) {
                float cy = leadLine.getY(cx);
                pos tp;
                tp.row = getRow(cx, cy);
                tp.col = getCol(cx, cy);
                if(collide(tp)) {  //collide
                    if(lead_line_pts.size() >= 2) {
                        lead_line_pts.pop_back();
                        lead_line_pts.pop_back();
                    } else if(lead_line_pts.size() == 1) {
                        lead_line_pts.pop_back();
                    }
                    break;
                } else {
                    Point strch_pt;
                    strch_pt.setPoint(cx, cy);
                    lead_line_pts.push_back(strch_pt);
                }
            }
		} else {
			delta_x = -0.10;
			//set the stretch points
            for(float cx = leadLine.pt_down.x; cx >= leadLine.pt_up.x; cx += delta_x) {
                float cy = leadLine.getY(cx);
                pos tp;
                tp.row = getRow(cx, cy);
                tp.col = getCol(cx, cy);
                if(collide(tp)) {  //collide
                    if(lead_line_pts.size() >= 2) {
                        lead_line_pts.pop_back();
                        lead_line_pts.pop_back();
                    } else if(lead_line_pts.size() == 1) {
                        lead_line_pts.pop_back();
                    }
                    break;
                } else {
                    Point strch_pt;
                    strch_pt.setPoint(cx, cy);
                    lead_line_pts.push_back(strch_pt);
                }
            }
		}

	} else {
		//set the increment
        //cout << "Increment variable Y! " << endl;
        //cout << "Lead line pt_down: ( " << leadLine.pt_down.x << " , " << leadLine.pt_down.y << " ) "
        //    << " pt_up: ( " << leadLine.pt_up.x << " , " << leadLine.pt_up.y << " ) " << endl;
		float delta_y = 0;
		if(leadLine.pt_down.y <= leadLine.pt_up.y) {
			delta_y = 0.10;
            //set the stretch point
            for(float cy = leadLine.pt_down.y; cy <= leadLine.pt_up.y; cy += delta_y) {
                float cx = leadLine.getX(cy);
                pos tp;
                tp.row = getRow(cx, cy);
                tp.col = getCol(cx, cy);
                if(collide(tp)) {  //collide
                    if(lead_line_pts.size() >= 2) {
                        lead_line_pts.pop_back();
                        lead_line_pts.pop_back();
                    } else if(lead_line_pts.size() == 1) {
                        lead_line_pts.pop_back();
                    }
                    break;
                } else {
                    Point strch_pt;
                    strch_pt.setPoint(cx, cy);
                    lead_line_pts.push_back(strch_pt);
                }
            }
		} else {
			delta_y = -0.10;
            //set the stretch point
            for(float cy = leadLine.pt_down.y; cy >= leadLine.pt_up.y; cy += delta_y) {
                float cx = leadLine.getX(cy);
                pos tp;
                tp.row = getRow(cx, cy);
                tp.col = getCol(cx, cy);
                if(collide(tp)) {  //collide
                    if(lead_line_pts.size() >= 2) {
                        lead_line_pts.pop_back();
                        lead_line_pts.pop_back();
                    } else if(lead_line_pts.size() == 1) {
                        lead_line_pts.pop_back();
                    }
                    break;
                } else {
                    Point strch_pt;
                    strch_pt.setPoint(cx, cy);
                    lead_line_pts.push_back(strch_pt);
                }
            }
		}

	}
    //cout << lead_line_pts.size() << " lead line points! " << endl;
	//set the stretch lines
	//stretch_line: a = -leadLine.b, b = leadLine.a, c = ?
	for(vector<Point>::const_iterator pt_iter = lead_line_pts.begin();
			pt_iter != lead_line_pts.end(); pt_iter++) {
        //cout << "Stretch points: " << pt_iter->x << "  " << pt_iter->y << endl;
		Line stch_line(leadLine.b, -leadLine.a, *pt_iter);
		stretch_lines.push_back(stch_line);
	}
}

void RasterMap::getWhiteGridsOnLine(const Line& strch_line, vector<pos>& white_pos, float stretch_width)
{
	if(strch_line.getk()) {
		//set the increment
		float delta_x = 0.1;
		Point sp1 = strch_line.pt_down;
		while(getPointsDist(strch_line.pt_down, sp1) <= stretch_width) {
			pos tp;
			tp.row = getRow(sp1.x, sp1.y);
			tp.col = getCol(sp1.x, sp1.y);
			stretch_pts.push_back(sp1);
			if(collide(tp)) {  //collide

				break;
			} else {						     //no collide
				white_pos.push_back(tp);
				sp1.x -= delta_x;
				sp1.y = strch_line.getY(sp1.x);
			}
		}

		Point sp2 = strch_line.pt_down;
		while(getPointsDist(strch_line.pt_down, sp2) <= stretch_width) {
			pos tp;
			tp.row = getRow(sp2.x, sp2.y);
			tp.col = getCol(sp2.x, sp2.y);
			stretch_pts.push_back(sp2);
			if(collide(tp)) {  //collide

				break;
			} else {						     //no collide
				white_pos.push_back(tp);
				sp2.x += delta_x;
				sp2.y = strch_line.getY(sp2.x);
			}
		}

	} else {
		float delta_y = 0.1;
		Point sp1 = strch_line.pt_down;
		while(getPointsDist(strch_line.pt_down, sp1) <= stretch_width) {
			pos tp;
			tp.row = getRow(sp1.x, sp1.y);
			tp.col = getCol(sp1.x, sp1.y);
			stretch_pts.push_back(sp1);
			if(collide(tp)) {  //collide

				break;
			} else {						     //no collide
				white_pos.push_back(tp);
				sp1.y -= delta_y;
				sp1.x = strch_line.getX(sp1.y);
			}
		}

		Point sp2 = strch_line.pt_down;
		while(getPointsDist(strch_line.pt_down, sp2) <= stretch_width) {
			pos tp;
			tp.row = getRow(sp2.x, sp2.y);
			tp.col = getCol(sp2.x, sp2.y);
			stretch_pts.push_back(sp2);
			if(collide(tp)) {  //collide

				break;
			} else {						     //no collide
				white_pos.push_back(tp);
				sp2.y += delta_y;
				sp2.x = strch_line.getX(sp2.y);
			}
		}
	}
}

float RasterMap::getPointsDist(const Point& pt1, const Point& pt2)
{
	return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

bool RasterMap::collide(pos tp)
{
    if(g[tp.row][tp.col].global_p > 0.8
            || g[tp.row + 1][tp.col].global_p > 0.8 || g[tp.row - 1][tp.col].global_p > 0.8
            || g[tp.row][tp.col + 1].global_p > 0.8 || g[tp.row][tp.col - 1].global_p > 0.8)
        return true;
    else
        return false;
}

bool RasterMap::getWhiteGrids_v1()
{
     //cout << "detect white grids " << endl;
	sb.exist = false;
    //get line
//    g[1300][1220].gp = 1;
//    g[1400][1260].gp = 1;
    sb = getsbLine();
//    sb.exist = true;
//    sb.a = 0;
//    sb.b = 1;
//    sb.c = -30;

    //cout << "line: " << sb.a << "x + " << sb.b << "y + " << sb.c << " = 0" << endl;

    unsigned big_box = 0, small_box = 0;

    if(sb.exist) {
        //cout << "sb line exist!" << endl;
        for(int i = arr_dr; i <= arr_ur; i++) {
            for(int j = arr_lc; j <= arr_rc; j++) {
                if(g[i][j].IsInDetectBox) {
                    double x = getx(i, j);
                    double y = gety(i, j);
                    Point pt;
                    pt.setPoint(x, y);
                    if(sb.getPointDist(pt, ugv_pt) >= 0) {
                        g[i][j].IsWhite = true;
                        //cout << "set white grids!" << endl;
                        small_box++;
                    } else {
                        g[i][j].IsWhite = false;
                    }
                    big_box++;
                }
            }
        }
    } else {
		//cout << "sb line not exist!" << endl;
		for(int i = arr_dr; i <= arr_ur; i++) {
            for(int j = arr_lc; j <= arr_rc; j++) {
                if(g[i][j].IsInDetectBox) {
                    double x = getx(i, j);
                    double y = gety(i, j);
                    Point pt;
                    pt.setPoint(x, y);
					g[i][j].IsWhite = true;
                    small_box++;
                }
            }
        }
	}
    //cout << "White grids: " << small_box << endl;
    return true;
}

Line RasterMap::getsbLine()
{
    vector<Line> lines;
    Line sb = Line();

    double delta_x, delta_y;
    int num = (abs(detect_left) + abs(detect_right)) / 0.2;
    num++;
    //cout << "delta_width: " << detect_width << " deltaLength: " << deltaLength << endl;
    //cout << "num: " << num << endl;
    delta_x = (box_rd.x - box_ld.x) / (num);
    delta_y = (box_rd.y - box_ld.y) / (num);
    Point down, up;
    //get a group of lines
    for(int i = 0; i < num; i++)
    {
        down.x = box_ld.x + i*delta_x;
        down.y = box_ld.y + i*delta_y;
        up.x = box_lu.x + i*delta_x;
        up.y = box_lu.y + i*delta_y;
        Line tmp_line(down, up);
        lines.push_back(tmp_line);
    }
    //cout << "line number: " << lines.size() << endl;
    //scan the lines from left to right
    //vector<Point> line_pts;
    line_pts.clear();

    for(vector<Line>::iterator line_iter = lines.begin();
            line_iter != lines.end(); line_iter++) {
		//cout << "a: " << line_iter->a << " b: " << line_iter->b << " c: " << line_iter->c << endl;
        Point pt = getTailPoint(*line_iter);
        if(pt.x != 100000)
            line_pts.push_back(pt);
    }

//    cout << "tail points: " << line_pts.size() << endl;
//    for(vector<Point>::const_iterator pt_iter = line_pts.begin();
//            pt_iter != line_pts.end(); pt_iter++) {
//        cout << "tail point: (" << pt_iter->x << ", " << pt_iter->y << ")" << endl;
//    }
      //replace 20140821
      if(line_pts.size() >=2) {
            Point ran_pts[ line_pts.size() ];
            Line best_line;
            Line tmp_line;
            int best_in_num = 0;
            //int best_out_num = 0;
            int tmp_in_num = 0;
            //int tmp_out_num = 0;
            double set_distance = 0.5;
            double tmp_distance;
            int i = 0, j = 0, k = 0;
            for(vector<Point>::iterator pt_iter = line_pts.begin(); pt_iter != line_pts.end(); pt_iter ++)  {
                ran_pts[i] = *pt_iter;
                i ++;
            }
            sb.setLine( ran_pts[0], ran_pts[line_pts.size()-1] );
            best_line = sb;
            for(i = 0; i < line_pts.size(); i ++) {
                tmp_distance = sb.getPointDist(ran_pts[i], ugv_pt);
                if( abs(tmp_distance) <= set_distance )
                    best_in_num ++;
            }
			//cout<<"best_in_num = "<<best_in_num<<endl;

			int cnt_best_line = 0;
            for(i = 0; i < line_pts.size()-1 && tmp_in_num < line_pts.size()-1; i ++)
            {

				for(j = i+1; j < line_pts.size() && tmp_in_num < line_pts.size()-1; j ++)
                {
                    tmp_in_num = 0;
					sb.setLine( ran_pts[i], ran_pts[j] );
                    tmp_line = sb;
                    for(k = 0; k < line_pts.size(); k ++)
                    {
                        tmp_distance = sb.getPointDist(ran_pts[k], ugv_pt);
                        if( abs(tmp_distance) <= set_distance )
                            tmp_in_num  ++;
                    }//k
                    if( tmp_in_num >= best_in_num ) {
                        best_line = tmp_line;
						best_in_num = tmp_in_num;
                        left_point = ran_pts[i];
                        right_point = ran_pts[j];
						cnt_best_line++;
						//cout << "change best line!" << endl;
                    }


                }//j

            }//i
		//cout << "change times: " << cnt_best_line << endl;
        //cout << "inlier points number: " << best_in_num << endl;
        sb = best_line;
        Point chosen_pt = left_point;
        double dist_chosen = sb.getPointDist(left_point, ugv_pt);
        double dist_tmp;
        for(vector<Point>::iterator pt_iter = line_pts.begin(); pt_iter != line_pts.end(); pt_iter ++)
        {
            dist_tmp = sb.getPointDist(*pt_iter, ugv_pt);
            if( dist_tmp >= dist_chosen && abs(dist_tmp) <= set_distance)
            {
                dist_chosen = dist_tmp;
                chosen_pt.x = pt_iter->x;
                chosen_pt.y = pt_iter->y;
            }
        }
        sb.c = -1 * ( sb.a * chosen_pt.x + sb.b * chosen_pt.y );
        sb.exist = true;
        line_pts.push_back(chosen_pt);
    } else {
        sb.a = 0;
		sb.b = 0;
		sb.c = 0;
        sb.exist = false;
    }
	return sb;
}

Point RasterMap::getTailPoint(Line ln)
{
    if(ln.getk()) {   //k<=1, +-x
        if(ln.pt_down.x <= ln.pt_up.x) {
            for(double lx = ln.pt_down.x; lx <= ln.pt_up.x; lx += 0.2) {
                double ly = ln.getY(lx);
                int row = getRow(lx, ly);
                int col = getCol(lx, ly);
                //cout << " row: " << row << "  col: " << col << endl;
                //if(g[row][col].global_p < 0.2) {
                if(collide(g[row][col].global_p)) {
                    //cout << "   global_p: " << g[row][col].global_p << endl;
					//cout << "find the tail point!" << endl;
                    Point pt;
                    pt.setPoint(lx, ly);
                    return pt;
                }
            }
            Point pt;
            pt.setPoint(100000, 100000);
            return pt;
        } else {
            for(double lx = ln.pt_down.x; lx >= ln.pt_up.x; lx -= 0.2) {
                double ly = ln.getY(lx);
                int row = getRow(lx, ly);
                int col = getCol(lx, ly);
                //cout << " row: " << row << "  col: " << col << endl;
                //if(g[row][col].global_p < 0.2) {
                if(collide(g[row][col].global_p)) {
                    //cout << "   global_p: " << g[row][col].global_p << endl;
					//cout << "find the tail point!" << endl;
                    Point pt;
                    pt.setPoint(lx, ly);
                    return pt;
                }
            }
			Point pt;
			pt.setPoint(100000, 100000);
			return pt;
        }
    } else {            //k>1, +-y
        if(ln.pt_down.y <= ln.pt_up.y) {
            for(double ly = ln.pt_down.y; ly <= ln.pt_up.y; ly += 0.2) {
                double lx = ln.getX(ly);
                int row = getRow(lx, ly);
                int col = getCol(lx, ly);
                //cout << " row: " << row << "  col: " << col << endl;
                //if(g[row][col].global_p < 0.2) {
                if(collide(g[row][col].global_p)) {
                    //cout << "   global_p: " << g[row][col].global_p << endl;
                    Point pt;
                    pt.setPoint(lx, ly);
                    return pt;
                }
            }
            Point pt;
            pt.setPoint(100000, 100000);
            return pt;
        } else {
            for(double ly = ln.pt_down.y; ly >= ln.pt_up.y; ly -= 0.2) {
                double lx = ln.getX(ly);
                int row = getRow(lx, ly);
                int col = getCol(lx, ly);
                //cout << " row: " << row << "  col: " << col << endl;
                //if(g[row][col].global_p < 0.2) {
                if(collide(g[row][col].global_p)) {
                    //cout << "   global_p: " << g[row][col].global_p << endl;
                    Point pt;
                    pt.setPoint(lx, ly);
                    return pt;
                }
            }
            Point pt;
            pt.setPoint(100000, 100000);
            return pt;
        }
    }
    Point pt;
    pt.setPoint(100000, 100000);
    return pt;
}


bool RasterMap::loadPoints()
{
    if(loadLUXPoints() && loadHDLPoints())
        return true;
    else
        return false;
}

bool RasterMap::loadLUXPoints()
{
    lux_pos.clear();
    for(vector<LUXPoint>::const_iterator iter = lux_pts.begin();
            iter != lux_pts.end(); ++iter) {
        int row = getRow(iter->x, iter->y);
        int col = getCol(iter->x, iter->y);
        if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
            g[row][col].HaveLUXPoint = true;
            g[row][col].lux_pts.push_back(*iter);
        }
    }
    for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            if(g[i][j].HaveLUXPoint) {
//                pos_t tp;
//                tp.row = i;
//                tp.col = j;
                pos tp;
                tp.row = i;
                tp.col = j;
                lux_pos.push_back(tp);
            }
        }
    }
    return true;
}

bool RasterMap::loadHDLPoints()
{
    hdl_pos.clear();
    for(vector<HDLPoint>::const_iterator iter = hdl_pts.begin();
            iter != hdl_pts.end(); ++iter) {
        int row = getRow(iter->x, iter->y);
        int col = getCol(iter->x, iter->y);
        if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
            g[row][col].HaveHDLPoint = true;
            g[row][col].hdl_pts.push_back(*iter);
        }
    }
    for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            if(g[i][j].HaveHDLPoint) {
//                pos_t tp;
//                tp.row = i;
//                tp.col = j;
                pos tp;
                tp.row = i;
                tp.col = j;
                hdl_pos.push_back(tp);
            }
        }
    }
    return true;
}

bool RasterMap::scanGrids()
{
    convex_pos.clear();
    current_pos.clear();
    for(vector<pos>::const_iterator pos_iter = lux_pos.begin();
            pos_iter != lux_pos.end(); ++pos_iter) {
        if(g[pos_iter->row][pos_iter->col].lux_pts.size() > 0) {
            g[pos_iter->row][pos_iter->col].lux_occupy = true;
        } else {
            g[pos_iter->row][pos_iter->col].lux_occupy = false;
        }
    }

    double z_max, z_min;
    for(vector<pos>::const_iterator pos_iter = hdl_pos.begin();
            pos_iter != hdl_pos.end(); ++pos_iter) {
        z_max = -10000.0, z_min = 10000.0;
        for(vector<HDLPoint>::iterator pt_iter = g[pos_iter->row][pos_iter->col].hdl_pts.begin();
                pt_iter != g[pos_iter->row][pos_iter->col].hdl_pts.end(); pt_iter++) {
            if(pt_iter->z > z_max)
                z_max = pt_iter->z;
            if(pt_iter->z < z_min)
                z_min = pt_iter->z;
        }
        double z_diff = z_max - z_min;
        if(z_diff >= 0.35) {
            g[pos_iter->row][pos_iter->col].hdl_occupy = true;
        } else {
            g[pos_iter->row][pos_iter->col].hdl_occupy = false;
        }
    }

//      unsigned int convex_cnt = 0;
//                cluster_pos tp;
//                tp.id = convex_cnt;
//                tp.row = i;
//                tp.col = j;
//                convex_pos.push_back(tp);
//                convex_cnt++;
    for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            if(g[i][j].lux_occupy || g[i][j].hdl_occupy) {
                pos tp;
//                tp.row = i;
//                tp.col = j;
                tp.row = g[i][j].row; //- GLOBAL_MAP_ROWS / 2;
                tp.col = g[i][j].col; //- GLOBAL_MAP_COLS / 2;
                current_pos.push_back(tp);
            }
        }
    }

    cout << "Current_pos number: " << current_pos.size() << endl;
//    for(vector<pos>::const_iterator pos_iter = current_pos.begin();
//            pos_iter != current_pos.end(); pos_iter++) {
//        cout << "       " <<  " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
//    }

    updatePosQueue(current_pos, convex_pos);

    cout << "convex number: " << convex_pos.size() << endl;
//    for(vector<cluster_pos_t>::const_iterator pos_iter = convex_pos.begin();
//            pos_iter != convex_pos.end(); pos_iter++) {
//        cout << "       " << " id: " << pos_iter->id << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
//    }

    return true;
}

bool RasterMap::setSHMOccupyGrids()
{
	m_recodata_occupyGrids.type = module::RecoData_t::RT_OCCUPY_GRIDS;
	m_recodata_occupyGrids.value.v_occupy_grids.interactive_flag = true;
	m_recodata_occupyGrids.value.v_occupy_grids.num = convex_pos.size();
	unsigned int pos_cnt = 0;
	for(vector<cluster_pos_t>::const_iterator pos_iter = convex_pos.begin();
			pos_iter != convex_pos.end(); pos_iter++) {
		m_recodata_occupyGrids.value.v_occupy_grids.occupy_grids[pos_cnt].row = pos_iter->row;
		m_recodata_occupyGrids.value.v_occupy_grids.occupy_grids[pos_cnt].col = pos_iter->col;
		pos_cnt++;
	}
//	module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_occupyGrids);
	return true;
}

bool RasterMap::writeOccupyGrids()
{
    for(vector<cluster_pos_t>::const_iterator pos_iter = convex_pos.begin();
            pos_iter != convex_pos.end(); pos_iter++) {
		//cout << "global_row: " << pos_iter->row << "  global_col: "
		//	<< pos_iter->col << endl;
        int row = getLocalRow(pos_iter->row, pos_iter->col);
        int col = getLocalCol(pos_iter->row, pos_iter->col);
		//cout << "local_row: " << row  << "  local_col: " << col << endl;
		if((row >= 0) && (col >= 0) && (row < OUTPUT_MAP_ROWS) && (col < OUTPUT_MAP_COLS)) {
			out_map[row][col].p[0] = getGridChar(true, false, out_map[row][col].p[0]);
			//cout << "out_map p: " << (int)out_map[row][col].p[0] << endl;
		}
		else
			cout << "Local row or col error!" << endl;
    }
    return true;
}

bool RasterMap::getSHMObstacles()
{
//	typedef struct obstacle
//    {
//        unsigned char type;			//ÀàÐÍ
//        gridFXY_t anchor_pt;			//Ãªµã
//        gridFXY_t bounding_box[4];		//°üÂçºÐµÄËÄ¸ö¶¥µã
//        gridFXY_t velocity;				//ËÙ¶È
//    } obstacle_t;
//
//    typedef struct RecoObstacleList
//    {
//        bool interactive_flag;
//        unsigned int num;			//ÕÏ°­ÎïµÄ¸öÊý
//        obstacle_t obs[MAX_OBSTACLE_NUM];	//ÕÏ°­ÎïÊý×é
//    } RecoObstacleList_t;

	m_recodata_obstacleList.type = module::RecoData::RT_OBSTACLE_LIST;
//	module::shm::SHARED_OBJECTS.GetRecoData(&m_recodata_obstacleList);
	m_obstacleList = m_recodata_obstacleList.value.v_obstacle_list;
	if(m_obstacleList.interactive_flag) {
		obs.clear();
        //the obstacles list is the newest
        for(unsigned int i = 0; i < m_obstacleList.num; i++) {
            module::obstacle_t tmp_obstacle = m_obstacleList.obs[i];
			cout << "lalala: " << tmp_obstacle.anchor_pt.x << " " << tmp_obstacle.anchor_pt.x << endl;
            obs.push_back(tmp_obstacle);
        }
        m_recodata_obstacleList.value.v_obstacle_list.interactive_flag = false;
//        module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_obstacleList);
        return true;
	} else {
	    //the obstacles list is not the newest
	    m_recodata_obstacleList.value.v_obstacle_list.interactive_flag = false;
//        module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_obstacleList);
        return false;
	}
}

bool RasterMap::setSHMSubmap()
{
	m_recodata_submap.type = module::RecoData_t::RT_SUB_MAP;
	//m_recodata_submap.value.v_sub_map.left_down_grid.x = getfx(g[0][0].row + diff_rows, g[0][0].col + diff_cols);
	//m_recodata_submap.value.v_sub_map.left_down_grid.y = getfy(g[0][0].row + diff_rows, g[0][0].col + diff_cols);

	float x = closest_global_pt.x - 80.0;
	float y = closest_global_pt.y - 80.0;
	m_recodata_submap.value.v_sub_map.left_down_grid.x = x;
	m_recodata_submap.value.v_sub_map.left_down_grid.y = y;
	//cout << "left_down_grid(1): " << m_recodata_submap.value.v_sub_map.left_down_grid.x
	//	<< " , " << m_recodata_submap.value.v_sub_map.left_down_grid.y << endl
	//	<< "left_down_grid(2): " << x << " , " << y << endl;
	for(int i = 0; i < OUTPUT_MAP_ROWS; i++) {
		for(int j = 0; j < OUTPUT_MAP_COLS; j++) {
			m_recodata_submap.value.v_sub_map.submap[i][j].p[0] = out_map[i][j].p[0];
			m_recodata_submap.value.v_sub_map.submap[i][j].p[1] = out_map[i][j].p[1];
			m_recodata_submap.value.v_sub_map.submap[i][j].p[2] = out_map[i][j].p[2];
		}
	}
//	module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_submap);
    return true;
}

bool RasterMap::clearGrids()
{
    //cout << "Begin Clear grids " << endl;
	//cout << "Arr" << arr_dr << " " << arr_ur << " " << arr_lc << " " << arr_rc << endl;
	for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            //cout << "clear lux points" << endl;
			g[i][j].lux_pts.clear();
			//cout << "clear hdl points" << endl;
            g[i][j].hdl_pts.clear();
			//cout << "clear have lux point" << endl;
            g[i][j].HaveLUXPoint = false;
			//cout << "clear have hdl point" << endl;
            g[i][j].HaveHDLPoint = false;
			//cout << "clear in detect box " << endl;
            g[i][j].IsInDetectBox = false;
			//cout << "clear is white" << endl;
            g[i][j].IsWhite = false;
			//cout << "clear lux occupy" << endl;
            g[i][j].lux_occupy = false;
			//cout << "clear hdl occupy" << endl;
            g[i][j].hdl_occupy = false;
            g[i][j].HaveObstacle = false;
        }
    }
	//cout << "Clear done!" << endl;
    return true;
}

bool RasterMap::updatePosQueue(vector<pos>& current_pos, vector<cluster_pos_t>& convex_pos, unsigned int queue_size)
{
    //input:
    //  queue<vector<pos> > pos_queue;
    //  map<pos, unsigned int> pos_counter;

    // 1. scan current_pos and update pos_counter
    for(vector<pos>::const_iterator pos_iter = current_pos.begin();
            pos_iter != current_pos.end(); pos_iter++) {
        pos_counter[*pos_iter]++;
    }

    // 2. push current_pos to the deque
    pos_queue.push(current_pos);

    // 3. pop deque to head_pos
    vector<pos> head_pos;
    if(pos_queue.size() > queue_size) {
        head_pos = pos_queue.front();
        pos_queue.pop();
    }

    // 4. scan the head_pos and update pos_counter
    for(vector<pos>::const_iterator pos_iter = head_pos.begin();
            pos_iter != head_pos.end(); pos_iter++) {
        pos_counter[*pos_iter]--;
        if(pos_counter[*pos_iter] == 0) {
            pos_counter.erase(*pos_iter);
        }
    }
    scanPosQueue(convex_pos);
    return true;
}

bool RasterMap::scanPosQueue(vector<cluster_pos_t>& convex_pos, unsigned int occupy_times)
{
    unsigned int convex_pos_cnt = 0;
    for(map<pos, unsigned int>::const_iterator map_iter = pos_counter.begin();
            map_iter != pos_counter.end(); map_iter++) {
        if(map_iter->second >= occupy_times) {
            int global_row = map_iter->first.row;
            int global_col = map_iter->first.col;
            int local_row = getLocalRow(global_row, global_col);
            int local_col = getLocalCol(global_row, global_col);
            if(g[local_row][local_col].IsWhite) {
                cluster_pos_t tmp_pos;
                tmp_pos.id = convex_pos_cnt;
                tmp_pos.row = global_row - GLOBAL_MAP_ROWS / 2;;
                tmp_pos.col = global_col - GLOBAL_MAP_COLS / 2;;

                convex_pos.push_back(tmp_pos);
                convex_pos_cnt++;
            }
        }
    }
    return true;
}

Point RasterMap::getLocalPoint(Point global_pt)
{
	Point local_pt;
	float ugv_y = (ugv_pos.row - GLOBAL_MAP_ROWS / 2) * 0.2;
    float ugv_x = (ugv_pos.col - GLOBAL_MAP_COLS / 2) * 0.2;
	local_pt.setPoint(global_pt.x - ugv_x, global_pt.y - ugv_y);
	return local_pt;

	//Point local_pt;
    //local_pt.setPoint(global_pt.x - ugv_pt.x, global_pt.y - ugv_pt.y);
    //return local_pt;
}

boundingBox RasterMap::getLocalBoundingBox(const boundingBox& global_bbx)
{
	boundingBox local_bbx;
	local_bbx.vertex[0] = getLocalPoint(global_bbx.vertex[0]);
	local_bbx.vertex[1] = getLocalPoint(global_bbx.vertex[1]);
	local_bbx.vertex[2] = getLocalPoint(global_bbx.vertex[2]);
	local_bbx.vertex[3] = getLocalPoint(global_bbx.vertex[3]);
	return local_bbx;
}

bool RasterMap::writeObstacles()
{
	//typedef struct obstacle
	//{
	//	unsigned char type;			//类型
	//	gridFXY_t anchor_pt;			//锚点
	//	gridFXY_t bounding_box[4];		//包络盒的四个顶点
	//	gridFXY_t velocity;				//速度
	//} obstacle_t;

	//set<pos> box_grids;

	box_grids.clear();
	cout << "Get " << obs.size() << " obstacles!" << endl;

	unsigned int obj_num = 0;
    for(vector<module::obstacle_t>::const_iterator obj_iter = obs.begin();
            obj_iter != obs.end(); obj_iter++) {

		boundingBox bbx, local_bbx;
        Point global_anchor_pt, local_anchor_pt;

		global_anchor_pt.setPoint(obj_iter->anchor_pt.x, obj_iter->anchor_pt.y);
		local_anchor_pt = getLocalPoint(global_anchor_pt);

		bbx.vertex[0].x = obj_iter->bounding_box[0].x;
        bbx.vertex[0].y = obj_iter->bounding_box[0].y;
        bbx.vertex[1].x = obj_iter->bounding_box[1].x;
        bbx.vertex[1].y = obj_iter->bounding_box[1].y;
        bbx.vertex[2].x = obj_iter->bounding_box[2].x;
        bbx.vertex[2].y = obj_iter->bounding_box[2].y;
        bbx.vertex[3].x = obj_iter->bounding_box[3].x;
        bbx.vertex[3].y = obj_iter->bounding_box[3].y;
        local_bbx = getLocalBoundingBox(bbx);

		cout << "Obstacle " << obj_num << endl;
		cout << "     " << "Glocal Anchor Point: ( " << global_anchor_pt.x << " , " << global_anchor_pt.y << " ) " << endl;
		cout << "     " << "Local Anchor Point: ( " << local_anchor_pt.x << " , " << local_anchor_pt.y << " ) " << endl;

		obj_num++;

		vector<Line> box_lines;
		if(local_anchor_pt.x >= -80 && local_anchor_pt.x <= 80 && local_anchor_pt.y > -80 && local_anchor_pt.y < 80) {
			generateLines(local_bbx, box_lines);
			cout << "Available obstacle!" << endl;
		} else {
			cout << "Unavailable obstacle!" << endl;
			continue;
		}

        for(vector<Line>::const_iterator line_iter = box_lines.begin();
                line_iter != box_lines.end(); line_iter++) {
            getLineGrids(*line_iter, box_grids[obj_num]);
        }
    }

	//cout << "Get line grids DONE DONE DONE " << endl;
	unsigned int map_num = 0;
	for(map<unsigned int, set<pos> >::const_iterator map_iter = box_grids.begin();
			map_iter != box_grids.end(); map_iter++) {

		bool isDynamic = false;
		unsigned char ttype = obs[map_num].type;
		if(ttype == 1) {
			isDynamic = false;
			cout << "Static obstacle!" << endl;
		}
		else {
			isDynamic = true;
			cout << "Dynamic obstacle!" << endl;
		}
		for(set<pos>::const_iterator set_iter = map_iter->second.begin();
				set_iter != map_iter->second.end(); set_iter++) {
			int local_row = set_iter->row;
			int local_col = set_iter->col;
			int global_row  = g[local_row][local_col].row;
			int global_col = g[local_row][local_col].col;
			int out_row = local_row - diff_rows;
			int out_col = local_col - diff_cols;
			//write back to g_map
			//unsigned char RasterMap::getGridChar(bool IsObject, bool IsDynamic, unsigned char p)

			if(!isDynamic)
				g_map[global_row][global_col].p[0] = getGlobalGridChar(true, isDynamic, g_map[global_row][global_col].p[0]);
			out_map[out_row][out_col].p[0] = getGridChar(true, isDynamic, out_map[out_row][out_col].p[0]);
			//cout << (int)(out_map[out_row][out_col].p[0] & 0x38) << endl;
		}
	}


    //for(set<pos>::const_iterator pos_iter = box_grids.begin();
    //        pos_iter != box_grids.end(); pos_iter++) {
    //    int row = pos_iter->row;
    //    int col = pos_iter->col;
    //    if((row >= 0) && (col >= 0) && (row < OUTPUT_MAP_ROWS) && (col < OUTPUT_MAP_COLS)) {
	//		//g[row[col].]
	//		out_map[row][col].p[0] = getGridChar(true, false, out_map[row][col].p[0]);
	//	}
    //}
	//cout << ".........The End.........." << endl;
	return true;
}

bool RasterMap::writeObstacles_v1()
{
    //typedef struct obstacle
	//{
	//	unsigned char type;			//类型
	//	gridFXY_t anchor_pt;			//锚点
	//	gridFXY_t bounding_box[4];		//包络盒的四个顶点
	//	gridFXY_t velocity;				//速度
	//} obstacle_t;

	set<pos> box_grids;
	cout << "Get " << obs.size() << " obstacles!" << endl;
	unsigned int obj_num = 0;
    for(vector<module::obstacle_t>::const_iterator obj_iter = obs.begin();
            obj_iter != obs.end(); obj_iter++) {
        Point global_anchor_pt, local_anchor_pt;
		global_anchor_pt.setPoint(obj_iter->anchor_pt.x, obj_iter->anchor_pt.y);
		local_anchor_pt = getLocalPoint(global_anchor_pt);

		cout << "Obstacle " << obj_num << endl;
		cout << "     " << "Anchor Point: ( " << local_anchor_pt.x << " , " << local_anchor_pt.y << " ) " << endl;
		obj_num++;

		vector<Line> box_lines;
        boundingBox bbx;
        bbx.vertex[0].x = obj_iter->bounding_box[0].x;
        bbx.vertex[0].y = obj_iter->bounding_box[0].y;
        bbx.vertex[1].x = obj_iter->bounding_box[1].x;
        bbx.vertex[1].y = obj_iter->bounding_box[1].y;
        bbx.vertex[2].x = obj_iter->bounding_box[2].x;
        bbx.vertex[2].y = obj_iter->bounding_box[2].y;
        bbx.vertex[3].x = obj_iter->bounding_box[3].x;
        bbx.vertex[3].y = obj_iter->bounding_box[3].y;

		/*
		cout << "Before transform: " << endl;
		cout << "    Vertex[0]: " << "( " << bbx.vertex[0].x << " , " << bbx.vertex[0].y << " ) " << endl;
		cout << "    Vertex[1]: " << "( " << bbx.vertex[1].x << " , " << bbx.vertex[1].y << " ) " << endl;
		cout << "    Vertex[2]: " << "( " << bbx.vertex[2].x << " , " << bbx.vertex[2].y << " ) " << endl;
		cout << "    Vertex[3]: " << "( " << bbx.vertex[3].x << " , " << bbx.vertex[3].y << " ) " << endl;
		*/
		/*
		cout << "After transform: " << endl;
		cout << "    Vertex[0]: " << "( " << bbx.vertex[0].x << " , " << bbx.vertex[0].y << " ) " << endl;
		cout << "    Vertex[1]: " << "( " << bbx.vertex[1].x << " , " << bbx.vertex[1].y << " ) " << endl;
		cout << "    Vertex[2]: " << "( " << bbx.vertex[2].x << " , " << bbx.vertex[2].y << " ) " << endl;
		cout << "    Vertex[3]: " << "( " << bbx.vertex[3].x << " , " << bbx.vertex[3].y << " ) " << endl;
        */

		boundingBox local_bbx;
        local_bbx = getLocalBoundingBox(bbx);
		if(local_anchor_pt.x >= -80 && local_anchor_pt.x <= 80 && local_anchor_pt.y > -80 && local_anchor_pt.y < 80) {
			generateLines(local_bbx, box_lines);
			cout << "Available obstacle!" << endl;
		} else {
			cout << "Unavailable obstacle!" << endl;
			continue;
		}

        for(vector<Line>::const_iterator line_iter = box_lines.begin();
                line_iter != box_lines.end(); line_iter++) {
            getLineGrids(*line_iter, box_grids);
        }
    }


    for(set<pos>::const_iterator pos_iter = box_grids.begin();
            pos_iter != box_grids.end(); pos_iter++) {
        int row = pos_iter->row;
        int col = pos_iter->col;
        if((row >= 0) && (col >= 0) && (row < OUTPUT_MAP_ROWS) && (col < OUTPUT_MAP_COLS)) {
			//g[row[col].]
			out_map[row][col].p[0] = getGridChar(true, false, out_map[row][col].p[0]);
		}
    }
	return true;
}

void RasterMap::getLineGrids(Line grid_line, set<pos>& line_grids)
{
    if(grid_line.getk()) {  //k<=1, +-x
        //cout << "we will +-x!" << endl;
        if(grid_line.pt_down.x <= grid_line.pt_up.x) {
            for(double lx = grid_line.pt_down.x; lx <= grid_line.pt_up.x; lx += 0.1) {
                double ly = grid_line.getY(lx);
                pos tp;
                tp.row = getRow(lx, ly);
                tp.col = getCol(lx, ly);
//                cout << "lx: " << lx << " ly: " << ly
//                    << "  row: " << tp.row << " col: " << tp.col <<  endl;

                //line_grids.push_back(tp);
                line_grids.insert(tp);
            }
        } else {
            for(double lx = grid_line.pt_down.x; lx >= grid_line.pt_up.x; lx -= 0.1) {
                double ly = grid_line.getY(lx);
                pos tp;
                tp.row = getRow(lx, ly);
                tp.col = getCol(lx, ly);
//                cout << "lx: " << lx << " ly: " << ly
//                    << "  row: " << tp.row << " col: " << tp.col <<  endl;
                //line_grids.push_back(tp);
                line_grids.insert(tp);
            }
        }
    } else {
        //cout << "we will +-y!" << endl;
        if(grid_line.pt_down.y <= grid_line.pt_up.y) {
            for(double ly = grid_line.pt_down.y; ly <= grid_line.pt_up.y; ly += 0.1) {
                double lx = grid_line.getX(ly);
                pos tp;
                tp.row = getRow(lx, ly);
                tp.col = getCol(lx, ly);
//                cout << "lx: " << lx << " ly: " << ly
//                    << "  row: " << tp.row << " col: " << tp.col <<  endl;
                //line_grids.push_back(tp);
                line_grids.insert(tp);
            }
        } else {
            for(double ly = grid_line.pt_down.y; ly >= grid_line.pt_up.y; ly -= 0.1) {
                double lx = grid_line.getX(ly);
                pos tp;
                tp.row = getRow(lx, ly);
                tp.col = getCol(lx, ly);
//                cout << "lx: " << lx << " ly: " << ly
//                    << "  row: " << tp.row << " col: " << tp.col <<  endl;
                //line_grids.push_back(tp);
                line_grids.insert(tp);
            }
        }
    }
}

//void RasterMap::transBoundingBox(boundingBox& bbx)
//{
//    //ugv + result = box
//    float ugv_y = (ugv_row - GLOBAL_MAP_ROWS / 2) * 0.2;
//    float ugv_x = (ugv_col - GLOBAL_MAP_COLS / 2) * 0.2;
//    cout << "ugv_y: " << ugv_y << " ugv_x: " << ugv_x << endl;
//    for(int i = 0; i < 4; i++) {
//        bbx.vertex[i].x = bbx.vertex[i].x - ugv_x;
//        bbx.vertex[i].y = bbx.vertex[i].y - ugv_y;
//    }
//}

void RasterMap::generateLines(boundingBox& bbx, vector<Line>& box_lines)
{
    Point pt_down1 = bbx.vertex[0], pt_down2 = bbx.vertex[3];
    Point pt_up1 = bbx.vertex[1],   pt_up2 = bbx.vertex[2];

    Line down_line, up_line;
    float down_dist = sqrt(pow(pt_down1.x - pt_down2.x, 2) + pow(pt_down1.y - pt_down2.y, 2));
    float up_dist = sqrt(pow(pt_up1.x - pt_up2.x, 2) + pow(pt_up1.y - pt_up2.y, 2));
    int line_number = (down_dist / 0.2) + 1;

    if(line_number > 1) {
        float delta_x = (pt_down2.x - pt_down1.x) / (line_number - 1);
        float delta_y = (pt_down2.y - pt_down1.y) / (line_number - 1);
        cout << "down dist: " << down_dist << " up dist: " << up_dist << endl;
        cout << "line number: " << line_number << endl;
        cout << "delta_x: " << delta_x << " delta_y: " << delta_y << endl;

        //down_line.setLine(pt_down1, pt_down2);
        //up_line.setLine(pt_up1, pt_up2);

        Point pt_down, pt_up;
        float down_x = pt_down1.x, up_x = pt_up1.x;
        float down_y = pt_down1.y, up_y = pt_up1.y;
        for(int i = 0; i < (line_number * 2); i++) {
            down_x = pt_down1.x + i * delta_x / 2, up_x = pt_up1.x + i * delta_x / 2;
            down_y = pt_down1.y + i * delta_y / 2, up_y = pt_up1.y + i * delta_y / 2;
            //cout << "down_x: " << down_x << " down_y: " << down_y
            //    << " up_x: " << up_x << " up_y: " << up_y << endl;
            pt_down.setPoint(down_x, down_y);
            pt_up.setPoint(up_x, up_y);
            Line fuck_line;
            fuck_line.setLine(pt_down, pt_up);
            box_lines.push_back(fuck_line);
        }
    } else {
        Line fuck_line;
        fuck_line.setLine(pt_down1, pt_up1);
        box_lines.push_back(fuck_line);
    }
}

bool RasterMap::clusterOccpuyGrids()
{
    //input: vector<cluster_pos_t> convex_pos;
    //output: map<unsigned int, vector<cluster_pos_t> > clusters;

    clusters.clear();
    //convex_pos.clear();

    unsigned int id1 = 0, id2 = 0;
    //unsigned int sid = big = 0;
    for(vector<cluster_pos_t>::iterator pos_iter = convex_pos.begin();
            pos_iter != convex_pos.end(); pos_iter++) {
        clusters[pos_iter->id].push_back(*pos_iter);
    }

    if(convex_pos.size() > 1) {
        for(vector<cluster_pos_t>::iterator pos_iter1 = convex_pos.begin();
                pos_iter1 != convex_pos.end()-1; pos_iter1 ++) {
            for(vector<cluster_pos_t>::iterator pos_iter2 = pos_iter1+1;
                    pos_iter2 != convex_pos.end(); pos_iter2++) {
                if(IsAdjacent(*pos_iter1, *pos_iter2)) {
                    id1 = pos_iter1->id;
                    id2 = pos_iter2->id;
                    if(id1 != id2) {
                        if(id1 > id2) {
                            pos_iter1->id = id2;
                            for(vector<cluster_pos>::iterator cpos_iter = clusters[id1].begin();
                                    cpos_iter != clusters[id1].end(); cpos_iter++) {
                                cpos_iter->id = id2;
                                clusters[id2].push_back(*cpos_iter);
                            }
                            clusters.erase(id1);
                        } else {
                            pos_iter2->id = id1;
                            for(vector<cluster_pos>::iterator cpos_iter = clusters[id2].begin();
                                    cpos_iter != clusters[id2].end(); cpos_iter++) {
                                cpos_iter->id = id1;
                                clusters[id1].push_back(*cpos_iter);
                            }
                            clusters.erase(id2);
                        }
                    }
                }
            }
        }
    }

    for(map<unsigned int, vector<cluster_pos_t> >::const_iterator clus_iter = clusters.begin();
            clus_iter != clusters.end(); clus_iter++) {
        cout << "cluster " << clus_iter->first << endl;
        for(vector<cluster_pos_t>::const_iterator pos_iter = clus_iter->second.begin();
                pos_iter != clus_iter->second.end(); pos_iter++) {
            cout << "   " << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
        }
    }

//    objs.clear();
//    for(map<unsigned int, vector<cluster_pos_t> >::const_iterator clus_pos = clusters.begin();
//            clus_pos != clusters.end(); clus_pos++) {
//        //double xmin, xmax, ymin, ymax;
//        int left_col, right_col, down_row, up_row;
//        left_col = GLOBAL_MAP_COLS, right_col = -GLOBAL_MAP_COLS;
//        down_row = GLOBAL_MAP_ROWS, up_row = -GLOBAL_MAP_ROWS;
//        for(vector<cluster_pos_t>::const_iterator pos_iter = clus_pos->second.begin();
//                pos_iter != clus_pos->second.end(); pos_iter++) {
//            int current_row = pos_iter->row;
//            int current_col = pos_iter->col;
//            if(current_row > up_row)
//                up_row = current_row;
//            if(current_row < down_row)
//                down_row = current_row;
//            if(current_col > right_col)
//                right_col = current_col;
//            if(current_col < left_col)
//                left_col = current_col;
//        }
//        cout << "left col: " << left_col << "  right col: " << right_col
//            << "  down row:  " << down_row << "  up_row:  " << up_row << endl;
//
//        objs[clus_pos->first].xmin = getx(0, getLocalCol(0, left_col));
//        objs[clus_pos->first].xmax = getx(0, getLocalCol(0, right_col));
//        objs[clus_pos->first].ymin = gety(getLocalRow(down_row, 0), 0);
//        objs[clus_pos->first].ymax = gety(getLocalRow(up_row, 0),   0);
//    }
//
//    for(map<unsigned int, object_t>::const_iterator obj_iter = objs.begin();
//            obj_iter != objs.end(); obj_iter++) {
//        cout << "obj : " << " xmin: " << obj_iter->second.xmin << " xmax: " << obj_iter->second.xmax
//            << " ymin: " << obj_iter->second.ymin << " ymax: " << obj_iter->second.ymax << endl;
//    }

    return true;
}

bool RasterMap::IsAdjacent(const cluster_pos_t& cp1, const cluster_pos_t& cp2)
{
    int row_dist = abs(cp1.row - cp2.row);
    int col_dist = abs(cp1.col - cp2.col);
    if((row_dist == 0) && (col_dist == 0)) {
        return false;
    } else {
        if((row_dist <= 2) && (col_dist <= 2))
            return true;
        else
            return false;
    }
}

bool RasterMap::setObjectList()
{
    //input: map<unsigned int, vector<cluster_pos_t> > clusters;
    //output: map<unsigned int, object_t> objs;

    objs.clear();
    unsigned int obj_cnt = 0;


    cluster_pts.clear();
    unsigned int cluster_cnt = 0;
    for(map<unsigned int, vector<cluster_pos_t> >::const_iterator map_iter = clusters.begin();
            map_iter != clusters.end(); map_iter ++) {
        //cout << "     cluster " << map_iter->first << " contains grids: " << endl;
        for(vector<cluster_pos_t>::const_iterator pos_iter = map_iter->second.begin();
                pos_iter != map_iter->second.end(); pos_iter ++ ) {
            //cout << "          " << " id: " << pos_iter->id << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
			Point tmp_pt;
			//tmp_pt.x = getx(0, getLocalCol(pos_iter->row, pos_iter->col));
			//tmp_pt.y = gety(getLocalRow(pos_iter->row, pos_iter->col), 0);
			tmp_pt.x = pos_iter->col * 0.2;
			tmp_pt.y = pos_iter->row * 0.2;
			cluster_pts[cluster_cnt].push_back(tmp_pt);
		}
		cluster_cnt++;
    }

    for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
            map_iter != cluster_pts.end(); map_iter++) {
        cout << "cluster " << map_iter->first << endl;
        for(vector<Point>::const_iterator pt_iter = map_iter->second.begin();
                pt_iter != map_iter->second.end(); pt_iter++) {
            cout << "   " << " x: " << pt_iter->x << " y: " << pt_iter->y << endl;
        }
    }

    for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
            map_iter != cluster_pts.end(); map_iter++) {
        //if the number of points in such cluster is more than 'cluster_min_size',
        //  the cluster will be set to a new object
        if(map_iter->second.size() > 1) {
            objs[obj_cnt] = setNewObject(map_iter->second);
            //transBoundingBox(objs[obj_cnt].bbx);
            objs[obj_cnt].bbx = getLocalBoundingBox(objs[obj_cnt].bbx);
            obj_cnt++;
        }
    }
    //print new object list info:
	///*
    cout << "Object list: " << endl;
    for(map<unsigned int, object_t>::const_iterator map_iter = objs.begin();
            map_iter != objs.end(); map_iter++) {
        cout << "    Object " << map_iter->first << endl;
        cout << "         vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
        cout << "         vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
        cout << "         vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
        cout << "         vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
    }
	//*/
    return true;
}

object_t RasterMap::setNewObject(const vector<Point>& pts, int line_pairs)
{
    //int line_pairs = 6;
    double delta_angle = (pi / 2) / line_pairs;     //15
    //cout << "delta angle: " << delta_angle / pi * 180 << endl;

    double area_min = 10000.0;
    double length, width;
    int select_line = 0;

    Point long_pt[2], short_pt[2];
    //double a_max, b_max, c_max;
    //a_max = b_max = c_max = 0;

    //select the line pair
    for(int i = 0; i < line_pairs; i++) {
        double angle[2];
        //double tanValue[2];
        double a[2], b[2], c[2];
        angle[0] = i * delta_angle;
        angle[1] = angle[0] + pi / 2;
        //cout << "--------------------------------------------------------" << endl;
        //cout << "angle 0: " <<  angle[0] / pi * 180 << " angle 1: " <<  angle[1] / pi * 180 << endl;
        if(i == 0) {
            a[0] = 0;
            b[0] = 1;
            a[1] = 1;
            b[1] = 0;
        } else {
            a[0] = tan(angle[0]);
            b[0] = -1;
            a[1] = tan(angle[1]);
            b[1] = -1;
        }
        Point head_pt = *(pts.begin());

        c[0] = - a[0]*head_pt.x - b[0]*head_pt.y;

        //Here, pt_set1 and pt_set2 are point sets. Their elements are intersection points
        //First element is the origin point. Second element is the intersection point
        //vector<Point> pt_set1;
        vector<pair<Point, Point> > pt_set1;
        for(vector<Point>::const_iterator pt_iter = pts.begin();
                pt_iter != pts.end(); pt_iter++) {
            double c1 = - a[1] * pt_iter->x - b[1] * pt_iter->y;
            Point tmp_pt = getIntersectPoint(a[0], b[0], c[0], a[1], b[1], c1);
            //pt_set1.push_back(tmp_pt);
            pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
            pt_set1.push_back(point_pair);
        }

        c[1] = - a[1] * head_pt.x - b[1] * head_pt.y;
        //vector<Point> pt_set2;
        vector<pair<Point, Point> > pt_set2;
        for(vector<Point>::const_iterator pt_iter = pts.begin();
                pt_iter != pts.end(); pt_iter++) {
            double c0 = - a[0] * pt_iter->x - b[0] * pt_iter->y;
            Point tmp_pt = getIntersectPoint(a[0], b[0], c0, a[1], b[1], c[1]);
            //pt_set2.push_back(tmp_pt);
            pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
            pt_set2.push_back(point_pair);
        }

        double max_dist1 = 0;
        Point raw_point_set1[2], insect_point_set1[2];
        for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set1.begin();
                pair_iter1 != pt_set1.end(); pair_iter1++) {
            for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set1.begin();
                    pair_iter2 != pt_set1.end(); pair_iter2++) {
                double dist1 = getPointDist(pair_iter1->second, pair_iter2->second);
                if(dist1 >= max_dist1) {
                    max_dist1 = dist1;
                    raw_point_set1[0] = pair_iter1->first;
                    raw_point_set1[1] = pair_iter2->first;
                    insect_point_set1[0] = pair_iter1->second;
                    insect_point_set1[1] = pair_iter2->second;
                }
            }
        }

        double max_dist2 = 0;
        Point raw_point_set2[2], insect_point_set2[2];
        for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set2.begin();
                pair_iter1 != pt_set2.end(); pair_iter1++) {
            for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set2.begin();
                    pair_iter2 != pt_set2.end(); pair_iter2++) {
                double dist2 = getPointDist(pair_iter1->second, pair_iter2->second);
                if(dist2 >= max_dist2) {
                    max_dist2 = dist2;
                    raw_point_set2[0] = pair_iter1->first;
                    raw_point_set2[1] = pair_iter2->first;
                    insect_point_set2[0] = pair_iter1->second;
                    insect_point_set2[1] = pair_iter2->second;
                }
            }
        }

        double area = max_dist1 * max_dist2;
		/*
        cout << "selected two points in set1 are: " << endl;
        cout << "   origin point1: " << "  x:  " << raw_point_set1[0].x << "  y:  " << raw_point_set1[0].y << endl;
        cout << "   origin point2: " << "  x:  " << raw_point_set1[1].x << "  y:  " << raw_point_set1[1].y << endl;
        cout << "   insect point1: " << "  x:  " << insect_point_set1[0].x << "  y:  " << insect_point_set1[0].y << endl;
        cout << "   insect point2: " << "  x:  " << insect_point_set1[1].x << "  y:  " << insect_point_set1[1].y << endl;

        cout << "selected two points in set2 are: " << endl;
        cout << "   origin point1: " << "  x:  " << raw_point_set2[0].x << "  y:  " << raw_point_set2[0].y << endl;
        cout << "   origin point2: " << "  x:  " << raw_point_set2[1].x << "  y:  " << raw_point_set2[1].y << endl;
        cout << "   insect point1: " << "  x:  " << insect_point_set2[0].x << "  y:  " << insect_point_set2[0].y << endl;
        cout << "   insect point2: " << "  x:  " << insect_point_set2[1].x << "  y:  " << insect_point_set2[1].y << endl;
		*/
        if(area < area_min) {
            area_min = area;
            if(max_dist1 >= max_dist2) {
                select_line = i;
                long_pt[0] = raw_point_set1[0];
                long_pt[1] = raw_point_set1[1];
                short_pt[0] = raw_point_set2[0];
                short_pt[1] = raw_point_set2[1];
                length = max_dist1;
                width = max_dist2;
            } else {
                select_line = (pi / 2) / delta_angle + i;
                long_pt[0] = raw_point_set2[0];
                long_pt[1] = raw_point_set2[1];
                short_pt[0] = raw_point_set1[0];
                short_pt[1] = raw_point_set1[1];
                length = max_dist2;
                width = max_dist1;
            }
        }
    }
//    cout << "The select lines are " << " angle[0]: "  << select_line * delta_angle / pi * 180
//            << " angle[1]: " << (pi / 2 + select_line * delta_angle) / pi * 180 << endl;
	/*
    cout << "The select lines are " << " Lone line angle:  " << select_line * delta_angle / pi * 180
        << "  Short line angle: " << ((select_line + (int)((pi/2) / delta_angle)) % (int)(pi / delta_angle)) * delta_angle / pi * 180 << endl;

    cout << "long line points: " << " ( " << long_pt[0].x << " , " << long_pt[0].y << " ) "
            << " ( " << long_pt[1].x << " , " << long_pt[1].y << " ) " << endl;
    cout << "short line points: " << " ( " << short_pt[0].x << " , " << short_pt[0].y << " ) "
            << " ( " << short_pt[1].x << " , " << short_pt[1].y << " ) " << endl;
    cout << "Area: " << area_min << "  Length:  " << length << "  Width:  " << width << endl;
	*/

    //set the object
    //Object obj;
    object_t obj;
    obj.bbx = getBoxVertex(select_line, long_pt[0], long_pt[1], short_pt[0], short_pt[1]);
    //obj.belief = 1;
    //obj.length = length;
    //obj.width = width;
    //obj.anchor_pt = getAnchorPoint(obj.bbx);
    return obj;
}

Point RasterMap::getIntersectPoint(double a1, double b1, double c1, double a2, double b2, double c2)
{
    Point pt;
    double x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
    double y = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1);
    pt.setPoint(x, y);
    return pt;
}

double RasterMap::getPointDist(const Point& pt1, const Point& pt2)
{
    return sqrt(pow((pt1.x - pt2.x),2) + pow((pt1.y - pt2.y),2));
}

boundingBox RasterMap::getBoxVertex(int long_line, Point long_pt1, Point long_pt2, Point short_pt1, Point short_pt2, int line_pairs)
{
    double la, lb;
    double sa, sb;
    if(long_line == 0) {
        la = 0, lb = 1;
        sa = 1, sb = 0;
    } else if(long_line == line_pairs) {
        la = 1, lb = 0;
        sa = 0, sb = 1;
    } else {
        double angle[2];
        double delta_angle = (pi / 2) / line_pairs;
        angle[0] = long_line * delta_angle, angle[1] = ((long_line + line_pairs) % (line_pairs * 2)) * delta_angle;
        //cout << "Get box vertex: " << "angle 0: " << angle[0] / pi * 180 << " angle 1: " << angle[1] / pi * 180 << endl;
        la = tan(angle[0]), lb = -1;
        sa = tan(angle[1]), sb = -1;
    }
    double lc[2], sc[2];

    //demo: c[0] = - a[0]*head_pt.x - b[0]*head_pt.y;
    lc[0] = - la*short_pt1.x - lb*short_pt1.y;
    lc[1] = - la*short_pt2.x - lb*short_pt2.y;
    sc[0] = - sa*long_pt1.x - sb*long_pt1.y;
    sc[1] = - sa*long_pt2.x - sb*long_pt2.y;

    //cout << "la: " << la << " lb: " << lb << " lc0: " << lc[0] << " lc1: " << lc[1] << endl;
    //cout << "sa: " << sa << " sb: " << sb << " sc0: " << sc[0] << " sc1: " << sc[1] << endl;

    boundingBox bbx;
    bbx.vertex[0] = getIntersectPoint(la, lb, lc[0], sa, sb, sc[0]);
    bbx.vertex[1] = getIntersectPoint(la, lb, lc[0], sa, sb, sc[1]);
    bbx.vertex[2] = getIntersectPoint(la, lb, lc[1], sa, sb, sc[1]);
    bbx.vertex[3] = getIntersectPoint(la, lb, lc[1], sa, sb, sc[0]);
    //sortPoints(bbx);
    return bbx;
}

bool RasterMap::scanGridsTest()
{
    unsigned int convex_cnt = 0;
    g[OUTPUT_MAP_ROWS / 2][OUTPUT_MAP_COLS / 2].test_occpy = false;
    convex_pos.clear();
    for(int i = arr_dr; i <= arr_ur; i++) {
        for(int j = arr_lc; j <= arr_rc; j++) {
            if(g[i][j].test_occpy && g[i][j].IsWhite) {
                cluster_pos tp;
                tp.id = convex_cnt;
                tp.row = g[i][j].row - GLOBAL_MAP_ROWS / 2;
                tp.col = g[i][j].col - GLOBAL_MAP_COLS / 2;
                convex_pos.push_back(tp);
                convex_cnt++;
            }
        }
    }
    //clearGridsTest();
    cout << "Convex_pos number: " << convex_pos.size() << endl;

    for(vector<cluster_pos_t>::const_iterator pos_iter = convex_pos.begin();
            pos_iter != convex_pos.end(); pos_iter++) {
        cout << "       " << " id: " << pos_iter->id << " row: " << pos_iter->row << " col: " << pos_iter->col
            << " Local row: " << getLocalRow(pos_iter->row + GLOBAL_MAP_ROWS / 2, pos_iter->col + GLOBAL_MAP_COLS / 2)
            << " Local col: " << getLocalCol(pos_iter->row + GLOBAL_MAP_ROWS / 2, pos_iter->col + GLOBAL_MAP_COLS / 2)
            << endl;
    }
    return true;
}

bool RasterMap::writeObstaclesTest()
{
    set<pos> box_grids;

    for(map<unsigned int, object_t>::const_iterator obj_iter = objs.begin();
            obj_iter != objs.end(); obj_iter++) {
        //vector<Line> box_lines;
        boundingBox bbx = obj_iter->second.bbx;
//        bbx.vertex[0].x = obj_iter->bounding_box[0].x;
//        bbx.vertex[0].y = obj_iter->bounding_box[0].y;
//        bbx.vertex[1].x = obj_iter->bounding_box[1].x;
//        bbx.vertex[1].y = obj_iter->bounding_box[1].y;
//        bbx.vertex[2].x = obj_iter->bounding_box[2].x;
//        bbx.vertex[2].y = obj_iter->bounding_box[2].y;
//        bbx.vertex[3].x = obj_iter->bounding_box[3].x;
//        bbx.vertex[3].y = obj_iter->bounding_box[3].y;
//        transBoundingBox(bbx);
        generateLines(bbx, box_lines);
        for(vector<Line>::const_iterator line_iter = box_lines.begin();
                line_iter != box_lines.end(); line_iter++) {
            getLineGrids(*line_iter, box_grids);
        }
    }
    cout << "Occupied Grids: " << endl;
    for(set<pos>::const_iterator pos_iter = box_grids.begin();
            pos_iter != box_grids.end(); pos_iter++) {
        int row = pos_iter->row;
        int col = pos_iter->col;
        cout << "     " << " row: " << row << " col: " << col << endl;
        if((row >= 0) && (col >= 0) && (row < OUTPUT_MAP_ROWS) && (col < OUTPUT_MAP_COLS)) {
			//out_map[row][col].p[0] = getGridChar(true, false, out_map[row][col].p[0]);
			g[row][col].HaveObstacle = true;
		}

    }
    return true;
}

bool RasterMap::clearGridsTest(bool my_mode)
{
    if(my_mode) {
        for(int i = arr_dr; i <= arr_ur; i++) {
            for(int j = arr_lc; j <= arr_rc; j++) {
                g[i][j].HaveObstacle = false;
                box_lines.clear();
                //g[i][j].test_occpy = false;
            }
        }
    } else {
        for(int i = arr_dr; i <= arr_ur; i++) {
            for(int j = arr_lc; j <= arr_rc; j++) {
                //g[i][j].HaveObstacle = false;
                g[i][j].test_occpy = false;
                box_lines.clear();
            }
        }

    }
    return true;
}

void RasterMap::clearGlobalAttr(int global_row, int global_col)
{
    //unsigned char p = g_map[global_row][global_col].p[0];
    //binary: 000 --> 11000111
    //unsigned char b = 0xC7;
    //unsigned result = b & p;
    //g_map[global_row][global_col].p[0] = result;

	g_map[global_row][global_col].p[0] = g_map[global_row][global_col].p[0] & 0xC7;

}

void RasterMap::refreshGrids()
{
	/*
	module::MarkerData_t m_markerdata;
    m_markerdata.type = module::MarkerData_t::MARKER_OBJCLEAR;
	//m_markerdata.value.v_objclear.mark = false;
	module::shm::SHARED_OBJECTS.GetMarker(&m_markerdata);
	if(m_markerdata.value.v_objclear.mark) {
		for(int i = arr_dr; i <= arr_ur; i++) {
			for(int j = arr_lc; j <= arr_rc; j++) {
				int global_row = g[i][j].row;
				int global_col = g[i][j].col;
				clearGlobalAttr(global_row, global_col);
			}
		}
	}
	m_markerdata.type = module::MarkerData_t::MARKER_OBJCLEAR;
	m_markerdata.value.v_objclear.mark = false;
	module::shm::SHARED_OBJECTS.SetMarker(m_markerdata);
	 */
}

void RasterMap::refreshGrids_bigMap()
{
    /*
	module::MarkerData_t m_markerdata;
    m_markerdata.type = module::MarkerData_t::MARKER_OBJCLEAR;
	//m_markerdata.value.v_objclear.mark = false;
	module::shm::SHARED_OBJECTS.GetMarker(&m_markerdata);
	if(m_markerdata.value.v_objclear.mark) {
		for(int i = 0; i <= OUTPUT_MAP_ROWS; i++) {
			for(int j = 0; j <= OUTPUT_MAP_COLS; j++) {
				int global_row = g[i][j].row;
				int global_col = g[i][j].col;
				clearGlobalAttr(global_row, global_col);
			}
		}
	}
	m_markerdata.type = module::MarkerData_t::MARKER_OBJCLEAR;
	m_markerdata.value.v_objclear.mark = false;
	module::shm::SHARED_OBJECTS.SetMarker(m_markerdata);
	*/
}

//void RasterMap::writeTimeLog()
//{
//    long msNumber[11];
//    for(int i = 0; i < 10; i++) {
//        msNumber[i] = (tv[i+1].tv_sec - tv[i].tv_sec) * 1000.0 + (tv[i+1].tv_usec - tv[i].tv_usec) / 1000.0;
//        cout << msNumber[i] << endl;
//        timelog << msNumber[i] << " ";
//    }
//    msNumber[10] = (tend.tv_sec - tbeg.tv_sec) * 1000.0 + (tend.tv_usec - tbeg.tv_usec) / 1000.0;
//    timelog << msNumber[10];
//    timelog << endl;
//}

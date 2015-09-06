#ifndef DRAW_H_INCLUDED
#define DRAW_H_INCLUDED

#include <iostream>
#include <opencv2/opencv.hpp>

#include <module/shm.h>
#include <module/module.h>

using namespace std;

class Draw
{
public:
    Draw()
    {
        //global_map_rows = 0;
        //global_map_cols = 0;
        global_map_center.x = 0;
        global_map_center.y = 0;
        global_map_center.eulr = 0;
    };

    ~Draw()
    {
        global_map.release();
        resize_map.release();
        ugv.release();
    };

    void showGlobalMap();
    void setGlobalMapCenter(module::Pose_t map_center);
    bool readGlobalMap(string name);
    void drawUGV(module::Pose_t ugv_pose);

    //variables
    cv::Mat global_map, resize_map;
    cv::Mat ugv;
    module::Pose_t global_map_center;

    //int global_map_rows, global_map_cols;
};


#endif // DRAW_H_INCLUDED

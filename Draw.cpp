#include "Draw.h"

bool Draw::readGlobalMap(string name)
{
    global_map = cv::imread(name, CV_LOAD_IMAGE_COLOR);
    if(!global_map.data) {
        cout << "Unable to open the global map!" << endl;
        return false;
    } else {
        //cv::Size global_size = global_map.size();
        //cout << "Global size: " << global_size.width << " " << global_size.height << endl;
//        global_map.resize(1000);
//        cout << "Global map " << " Rows: " << global_map.rows << " Cols: " << global_map.cols << endl;

        cout << "Global map " << " Rows: " << global_map.rows << " Cols: " << global_map.cols << endl;
//        cout << "Resize map " << " Rows: " << resize_map.rows << " Cols: " << resize_map.cols << endl;
    }
    return true;
}

void Draw::setGlobalMapCenter(module::Pose_t map_center)
{
    global_map_center = map_center;
}

void Draw::showGlobalMap()
{
    if(global_map.rows > 800) {
        float scale = 800.0 / global_map.rows;
        cv::Size sz;
        sz.height = 800;
        sz.width = global_map.cols * scale;
        cv::resize(global_map, resize_map, sz);
    } else {
        resize_map = global_map;
    }
    cv::namedWindow("Global Map", CV_WINDOW_AUTOSIZE);
    cv::imshow("Global Map", resize_map);
    //cv::waitKey(0);
}

void Draw::drawUGV(module::Pose_t ugv_pose)
{
    // calculate the position of ugv on global map
    module::Pose_t delta_pose;
    delta_pose.x = ugv_pose.x - global_map_center.x;
    delta_pose.y = ugv_pose.y - global_map_center.y;

    int delta_rows = delta_pose.y / 0.2;
    int delta_cols = delta_pose.x / 0.2;
    int ugv_rows = global_map.rows / 2 + delta_rows;
    int ugv_cols = global_map.cols / 2 + delta_cols;
    if(ugv_rows >= global_map.rows || ugv_cols >= global_map.cols || ugv_rows < 0 || ugv_cols < 0) {
        cout << "UGV outside!!!" << endl;
    } else {
        //calculate the pixel of ugv to display
        cout << "UGV inside!!!" << endl;
        cv::Point ugv_pixel;
        ugv_pixel.x = global_map.cols / 2 + delta_cols;
        ugv_pixel.y = global_map.rows / 2 - delta_rows;
        cv::Scalar ugv_color = cv::Scalar(0, 0, 255);

        cv::circle(global_map, ugv_pixel, 30, ugv_color, -1);

    }
}


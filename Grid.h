#ifndef GRID_H_INCLUDED
#define GRID_H_INCLUDED

#include <vector>

#include "LUXPoint.h"
#include "HDLPoint.h"

using namespace std;
class Grid
{
public:
    //variables
    int row, col;
    float global_p;
    bool IsWhite, IsInDetectBox;
    bool HaveLUXPoint, HaveHDLPoint;
    bool lux_occupy, hdl_occupy;

    bool test_occpy;

    bool IsHorWhite;
    bool IsInHorDetectBox;
    bool HaveObstacle;

    vector<LUXPoint> lux_pts;
    vector<HDLPoint> hdl_pts;

    //functions
    Grid(int r = 0, int c = 0, float g_p = 0, bool white = false, bool inDetectBox = false, \
         bool have_luxpt = false, bool have_hdlpt = false, bool lux_occ = false, bool hdl_occ = false, bool test_occ = false, \
         bool horwhite = false, bool inHorDetectBox = false, bool haveObs = false)
        : row(r), col(c), global_p(g_p), IsWhite(white), IsInDetectBox(inDetectBox), \
            HaveLUXPoint(have_luxpt), HaveHDLPoint(have_hdlpt), lux_occupy(lux_occ), hdl_occupy(hdl_occ),  test_occpy(test_occ), \
            IsHorWhite(horwhite), IsInHorDetectBox(inHorDetectBox), HaveObstacle(haveObs) {}


    //Grid(const Grid &g) : row(g.row), col(g.col) {}
    //~Grid();

//    inline bool
//    operator==(const Grid &lg, const Grid &rg)
//    {
//        return lg.row == rg.row &&
//            lg.col == rg.col;
//    }
//
//    inline bool
//    operator!=(const Grid &lg, const Grid &rg)
//    {
//        return !(lg == rg);
//    }

private:


};

#endif // GRID_H_INCLUDED

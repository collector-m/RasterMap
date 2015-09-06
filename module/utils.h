#pragma once

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "types.h"

namespace module
{
namespace utils
{

class ConvertUtils
{
public:
	/**
	 * convert camera data to opencv's IplImage structure
	 */
	static IplImage* Convert(const MetaCameraBW_t& data);
	static IplImage* Convert(const MetaCameraC_t& data);
	static void cal_ldad(Pose_t startPt,Pose_t endPt,double &ld,double &ad);
//	static IplImage* Convert(const MarkerTaillightImage_t& data);
};
}
}

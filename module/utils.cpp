#include "utils.h"

namespace module
{
namespace utils
{
IplImage* ConvertUtils::Convert(const MetaCameraBW_t& data)
{
	IplImage* img = cvCreateImage(cvSize(BWCAM_IMAGE_WIDTH, BWCAM_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
	memcpy(img->imageData, data.mdata, img->imageSize);
	return img;
}

IplImage* ConvertUtils::Convert(const MetaCameraC_t& data)
{
	IplImage* img = cvCreateImage(cvSize(CCAM_IMAGE_WIDTH, CCAM_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
	memcpy(img->imageData, data.mdata, img->imageSize);
	return img;
}
void ConvertUtils::cal_ldad(Pose_t startPt,Pose_t endPt,double &ld,double &ad){
	double x0 = startPt.x;
	double y0 = startPt.y;
	double phi0 = startPt.eulr;
	double x1 = endPt.x;
	double y1 = endPt.y;
	double phi1 = endPt.eulr;
	double L,alpha = 0,delta_phi;

	L = sqrt((x1-x0) * (x1-x0) + (y1-y0) * (y1-y0));

	//Calculate alpha
	if (y1 > y0)
	{
		alpha = atan((x1-x0) / (y1-y0));
	}
	if (y1 == y0)
	{
		if (x1 >= x0)
			alpha = M_PI / 2;
		else
			alpha = M_PI * 3 / 2;
	}
	if (y1 < y0)
	{
		alpha = M_PI + atan((x1-x0) / (y1-y0));
	}

	//if (phi0 >= M_PI) phi0 -= 2 * M_PI;

	delta_phi = phi1 - phi0;

	if (delta_phi >= M_PI)
		delta_phi -= 2 * M_PI;
	if (delta_phi < -M_PI)
		delta_phi += 2 * M_PI;

	ad = -delta_phi * 180 / M_PI;
	ld = L * sin(phi0 - alpha);
	return ;
}
//IplImage* ConvertUtils::Convert(const MarkerTaillightImage_t& data)
//{
//	IplImage* img = cvCreateImage(cvSize(data.width, data.height), IPL_DEPTH_8U, 3);
//	memcpy(img->imageData, data.imgdata, img->imageSize);
//	return img;
//}
}
}

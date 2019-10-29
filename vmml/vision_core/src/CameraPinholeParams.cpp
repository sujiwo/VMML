/*
 * CameraPinholeParams.cpp
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include "CameraPinholeParams.h"


using namespace Eigen;
using namespace std;


namespace Vmml {

Matrix<double,3,4> CameraPinholeParams::toMatrix() const
{
	Matrix<double,3,4> K = Matrix<double,3,4>::Zero();
	K(2,2) = 1.0;
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
	return K;
}


Eigen::Matrix3d
CameraPinholeParams::toMatrix3() const
{
	Matrix3d K = Matrix3d::Identity();
	K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
	return K;
}


cv::Mat
CameraPinholeParams::toCvMat() const
{
	cv::Mat K = cv::Mat::zeros(3,3,CV_32F);
	K.at<float>(2,2) = 1.0;
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
	return K;
}


/*
CameraPinholeParams
CameraPinholeParams::loadCameraParamsFromFile(const string &f)
{
	CameraPinholeParams c;
	INIReader cameraParser(f);
	c.fx = cameraParser.GetReal("", "fx", 0);
	c.fy = cameraParser.GetReal("", "fy", 0);
	c.cx = cameraParser.GetReal("", "cx", 0);
	c.cy = cameraParser.GetReal("", "cy", 0);
	c.width = cameraParser.GetInteger("", "width", 0);
	c.height = cameraParser.GetInteger("", "height", 0);

	return c;
}
*/


CameraPinholeParams
CameraPinholeParams::operator* (const float r) const
{
	CameraPinholeParams n = *this;
	n.width *= r;
	n.height *= r;
	n.fx *= r;
	n.fy *= r;
	n.cx *= r;
	n.cy *= r;

	if (!mask.empty()) {
		cv::resize(mask, n.mask, cv::Size(), r, r);
	}

	return n;
}


#include <cmath>

float
CameraPinholeParams::getHorizontalFoV() const
{
	double tanT = cx / fx;
	return 2 * atan(tanT);
}

float
CameraPinholeParams::getVerticalFoV() const
{
	double tanT = cy / fy;
	return 2 * atan(tanT);
}

} /* namespace Vmml */

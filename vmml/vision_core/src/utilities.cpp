/*
 * utilities.cpp
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#include "utilities.h"

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


using Eigen::VectorXd;
using namespace std;

namespace Vmml {

#define dPrecision 3

const ptime epoch(boost::gregorian::date(1970,1,1));


VectorXd
cdf (const cv::Mat &grayImage, const cv::Mat &mask)
{
	VectorXd rcdf = VectorXd::Zero(256);
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
	cv::calcHist (&grayImage, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);
	// cumulative sum
	rcdf[0] = hist.at<float>(0);
	for (int i=1; i<histSize; i++) {
		rcdf[i] = rcdf[i-1] + hist.at<float>(i);
	}

	rcdf = rcdf / cv::sum(hist)[0];
	return rcdf;
}


void debugMsg(const string &s, double is_error)
{
	if (!is_error)
		cerr << s << endl << flush;
	else
		cout << s << endl << flush;
}


double toSeconds (const ptime &pt)
{
	tduration td = pt - epoch;
	return td_seconds(td);
}


ptime fromSeconds (const double s)
{
	long micro_ts = s*1e6;
	tduration td = boost::posix_time::microseconds(micro_ts);
	return epoch + td;
}

}		// namespace Vmml

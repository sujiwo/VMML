/*
 * ImageBag.cpp
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImageBag.h"
#include "vmml/utilities.h"


using namespace std;


namespace Vmml {


ImageBag::ImageBag(const rosbag::Bag &bag, const std::string &imageTopic) :
	RandomAccessBag(bag, imageTopic)
{}


ImageBag::~ImageBag()
{}


cv::Mat
ImageBag::at(unsigned int position, bool raw)
{
	auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);

	cv_bridge::CvImagePtr imgPtr;
	if (raw==true)
		imgPtr = cv_bridge::toCvCopy(*bImageMsg);
	else
		imgPtr = cv_bridge::toCvCopy(*bImageMsg, sensor_msgs::image_encodings::BGR8);
	return imgPtr->image;
}


cv::Mat
ImageBag::getGrayscale(unsigned int position)
{
	auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);
	auto imgPtr = cv_bridge::toCvCopy(bImageMsg, sensor_msgs::image_encodings::MONO8);
	return imgPtr->image;
}


cv::Mat
ImageBag::at(const ptime &t)
{
	return at(ros::Time::fromBoost(t));
}


cv::Mat
ImageBag::at(const ros::Time &t)
{
	uint N = getPositionAtTime(t);
	return at(N);
}



bool
ImageBag::save(unsigned int position, const string &filename, bool raw)
{
	auto image = at(position, raw);
	return cv::imwrite(filename, image);
}


void
ImageBag::getImageDimensions(uint &width, uint &height)
{
	auto image0 = at(0, true);
	width = image0.cols;
	height = image0.rows;
}


cv::Size
ImageBag::getImageDimensions()
{
	uint width, height;
	return cv::Size(width, height);
}


/*
 * XXX: Desampling algorithm should be improved, as it returns lower-than-requested
 */
void
ImageBag::desample(const float newFreq, std::vector<uint64> &messagePosList) const
{
	// Must be lower than current frequency
	assert(newFreq < hz());

	const double lengthInSeconds = (getBagStopTime()-getBagStartTime()).toSec();

	uint posWk = 0, nextWk;
	const double tIntrv = 1.0 / newFreq;
	for (double twork=0.0; twork<lengthInSeconds; twork+=1.0) {
		double tMax = min(twork+1.0, lengthInSeconds);
		double tm = twork+tIntrv;
		while (tm < tMax) {
			uint p = getPositionAtDurationSecond(tm);
			messagePosList.push_back(p);
			tm += tIntrv;
		}
	}
}

} /* namespace Vmml */

/*
 * ImageBag.cpp
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImageBag.h"
#include "vmml/utilities.h"


using namespace std;


namespace Vmml {


ImageBag::ImageBag(const rosbag::Bag &bag, const std::string &imageTopic, float zoom) :
	RandomAccessBag(bag, imageTopic),
	zoomRatio(zoom),
	imgPreps(ImagePreprocessor::ProcessMode::AGC)
{
	setGammaMeteringMask();
}


ImageBag::~ImageBag()
{}


cv::Mat
ImageBag::at(unsigned int position, bool raw)
{
	auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);

	if (raw==true) {
		auto enc = bImageMsg->encoding;
		auto imgPtr = cv_bridge::toCvCopy(bImageMsg);
		return imgPtr->image;
	}

	else {
		auto imgPtr = cv_bridge::toCvCopy(bImageMsg, sensor_msgs::image_encodings::BGR8);
		cv::Mat imageRes;
		cv::resize(imgPtr->image, imageRes, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
		if (imgPreps.maskIsEmpty()==false)
			imgPreps.preprocess(imageRes);
		return imageRes;
	}
}


cv::Mat
ImageBag::getGrayscale(unsigned int position)
{
	auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);
	auto imgPtr = cv_bridge::toCvCopy(bImageMsg, sensor_msgs::image_encodings::MONO8);
	cv::Mat imageRes;
	cv::resize(imgPtr->image, imageRes, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
	return imageRes;
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
	auto image = at(position);
	return cv::imwrite(filename, image);
}


void
ImageBag::setGammaMeteringMask(const cv::Mat &mask)
{
	cv::Mat preprocessMeteringMask;
	cv::resize(mask, preprocessMeteringMask, cv::Size(), zoomRatio, zoomRatio);

	// Test sizes of bag and mask
	cv::Mat img0 = getGrayscale(0);
	if (img0.rows!=preprocessMeteringMask.rows or img0.cols!=preprocessMeteringMask.cols)
		throw length_error("Mask and bag images have mismatched sizes");
	imgPreps.setMask(preprocessMeteringMask);
}


void
ImageBag::setGammaMeteringMask(const std::string &p)
{
	string realPath;

	if (p.empty()) {
		Path maskPath(ros::package::getPath("vision_core"));
		maskPath /= "car_mask_meter.png";
		realPath = maskPath.string();
	}
	else realPath = p;

	cv::Mat mask = cv::imread(realPath, cv::IMREAD_GRAYSCALE);
	return setGammaMeteringMask(mask);
}


void
ImageBag::getImageDimensions(uint &width, uint &height)
{
	uint origW, origH;
	getOriginalImageDimensions(origW, origH);
	width = origW * zoomRatio;
	height = origH * zoomRatio;
}


void
ImageBag::getOriginalImageDimensions(uint &width, uint &height)
{
	auto image = at(0, true);
	width = image.cols;
	height = image.rows;
}


cv::Mat
ImageBag::equalizeGamma(const cv::Mat &src) const
{

}


} /* namespace Vmml */

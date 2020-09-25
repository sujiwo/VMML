/*
 * ImageBag.cpp
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#include <exception>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "ImageBag.h"


using namespace std;


namespace PlaceRecognizer {


ImageBag::ImageBag(const rosbag::Bag &bag, const std::string &imageTopic) :
	RandomAccessBag(bag, imageTopic)
{
	if (messageType()!="sensor_msgs/Image" and messageType()!="sensor_msgs/CompressedImage")
		throw runtime_error("Requested topic is not recognized image");
	isCompressed = messageType()=="sensor_msgs/CompressedImage";
}


ImageBag::~ImageBag()
{}


cv::Mat
ImageBag::at(unsigned int position, bool raw)
{
	cv_bridge::CvImagePtr imgPtr;

	if (!isCompressed) {
		auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);
		if (raw==true)
			imgPtr = cv_bridge::toCvCopy(*bImageMsg);
		else
			imgPtr = cv_bridge::toCvCopy(*bImageMsg, sensor_msgs::image_encodings::BGR8);
	}

	// Raw does not apply here
	else {
		auto cImageMsg = RandomAccessBag::at<sensor_msgs::CompressedImage>(position);
		imgPtr = cv_bridge::toCvCopy(*cImageMsg, sensor_msgs::image_encodings::BGR8);
	}

	return imgPtr->image;
}


sensor_msgs::ImageConstPtr ImageBag::getMessage(uint position)
{
	if (isCompressed) {
		auto cImageMsg = RandomAccessBag::at<sensor_msgs::CompressedImage>(position);
		return cv_bridge::toCvCopy(*cImageMsg)->toImageMsg();
	}
	else {
		return RandomAccessBag::at<sensor_msgs::Image>(position);
	}
}


cv::Mat
ImageBag::getGrayscale(unsigned int position)
{
	auto bImageMsg = RandomAccessBag::at<sensor_msgs::Image>(position);
	auto imgPtr = cv_bridge::toCvCopy(bImageMsg, sensor_msgs::image_encodings::MONO8);
	return imgPtr->image;
}


cv::Mat
ImageBag::at(const ros::Time &t, double *timeDiff)
{
	uint N = getPositionAtTime(t);
	auto tExact = timeAt(N);

	if (timeDiff!=nullptr)
		*timeDiff = (tExact-t).toSec();

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
	getImageDimensions(width, height);
	return cv::Size(width, height);
}


std::string
ImageBag::suggestTopic(const rosbag::Bag &bag)
{
	auto vTopicList = RandomAccessBag::getTopicList(bag);
	for (auto pr: vTopicList) {
		if (pr.second=="sensor_msgs/Image" or pr.second=="sensor_msgs/CompressedImage") {
			return pr.first;
		}
		else continue;
	}
	throw runtime_error("No suitable image topic found");
}


} /* namespace Vmml */

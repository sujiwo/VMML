/*
 * test_rgb_filters.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d.hpp>
#include "vmml/ImagePreprocessor.h"


using namespace std;


image_transport::Publisher imagePub1, imagePub2;
//auto detector = cv::AKAZE::create();
auto detector = cv::ORB::create(3000);
const float alpha = 0.3975;


void imageHandlerAutoGamma(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgBgr = cv_bridge::toCvShare(imgMsg, "bgr8");
	auto imagePrep = ImagePreprocessor::autoAdjustGammaRGB(imgBgr->image);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
		imagePrep,
		cv::Mat(),
		kpList,
		descriptors);
	cv::drawKeypoints(imagePrep, kpList, drawFrameKeypts, cv::Scalar(0,255,0));
//	cerr << "# features: " << kpList.size() << endl;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = drawFrameKeypts;
	cvImg.header.stamp = ros::Time::now();

	imagePub1.publish(cvImg.toImageMsg());
}


void imageHandlerIlluminatiInvariant(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgRaw = cv_bridge::toCvShare(imgMsg);
	auto imagePrep = ImagePreprocessor::toIlluminatiInvariant(imgRaw->image, alpha);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
		imagePrep,
		cv::Mat(),
		kpList,
		descriptors);
	cv::drawKeypoints(imagePrep, kpList, drawFrameKeypts, cv::Scalar(0,255,0));
//	cerr << "# features: " << kpList.size() << endl;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = drawFrameKeypts;
	cvImg.header.stamp = ros::Time::now();

	imagePub2.publish(cvImg.toImageMsg());
}


void imageHandlerGrayWorld(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgBgr = cv_bridge::toCvShare(imgMsg, "bgr8");
	auto imagePrep = ImagePreprocessor::GrayWorld(imgBgr->image);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
		imagePrep,
		cv::Mat(),
		kpList,
		descriptors);
	cv::drawKeypoints(imagePrep, kpList, drawFrameKeypts, cv::Scalar(0,255,0));
//	cerr << "# features: " << kpList.size() << endl;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = drawFrameKeypts;
	cvImg.header.stamp = ros::Time::now();

	imagePub1.publish(cvImg.toImageMsg());

}


void imageHandler(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	thread handler1([&] {
		imageHandlerGrayWorld(imgMsg);
	});

	thread handler2([&] {
		imageHandlerIlluminatiInvariant(imgMsg);
	});

	handler1.join();
	handler2.join();
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_rgb_filters");
	ros::NodeHandle mNode;

	image_transport::ImageTransport iTrans(mNode);
	imagePub1 = iTrans.advertise("/front_rgb/gray_world", 1);
	imagePub2 = iTrans.advertise("/front_rgb/illuminati", 1);

	ros::Subscriber imgSub = mNode.subscribe("/front_rgb/image_raw", 1, imageHandler);
	ros::spin();

	return 0;
}

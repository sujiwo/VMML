/*
 * test_rgb_filters.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d.hpp>
#include "vmml/ImagePreprocessor.h"


using namespace std;


image_transport::Publisher imagePub;
//auto detector = cv::AKAZE::create();
auto detector = cv::ORB::create(6000);


void imageHandler(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgRaw = cv_bridge::toCvShare(imgMsg, "bgr8");
	auto imagePrep = ImagePreprocessor::retinaHdr(imgRaw->image);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
		imagePrep,
		cv::Mat(),
		kpList,
		descriptors);
	cv::drawKeypoints(imagePrep, kpList, drawFrameKeypts);
	cerr << "# features: " << kpList.size() << endl;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = drawFrameKeypts;
	cvImg.header.stamp = ros::Time::now();

	imagePub.publish(cvImg.toImageMsg());
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_rgb_filters");
	ros::NodeHandle mNode;

	image_transport::ImageTransport iTrans(mNode);
	imagePub = iTrans.advertise("/front_rgb/image_illuminati", 1);

	ros::Subscriber imgSub = mNode.subscribe("/front_rgb/image_raw", 1, imageHandler);
	ros::spin();

	return 0;
}

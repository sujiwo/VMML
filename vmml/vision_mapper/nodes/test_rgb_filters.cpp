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


image_transport::Publisher imagePub;
auto orbDet = cv::ORB::create(1500);


void imageHandler(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgRaw = cv_bridge::toCvShare(imgMsg);
	auto imagePrep = ImagePreprocessor::toIlluminatiInvariant(imgRaw->image, 0.3975);

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::MONO8;
	cvImg.image = imagePrep;
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

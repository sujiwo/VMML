/*
 * ime_ros.cpp
 *
 *  Created on: Aug 18, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "im_enhance.h"


using namespace std;



class ImageContrastEnhancer
{
public:

ImageContrastEnhancer(ros::NodeHandle &nh) :
	handle(nh),
	imageHandler(nh)
{
	subscriber = imageHandler.subscribe("image", 1, &ImageContrastEnhancer::imageCallback, this);
	publisher = imageHandler.advertise("image_processed", 1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	auto image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
	auto imageProcessed = ice::multiScaleRetinexCP(image);

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = imageProcessed;
	cvImg.header.stamp = ros::Time::now();
	cvImg.header.frame_id = "camera";
	auto imgMsg = cvImg.toImageMsg();

	publisher.publish(*imgMsg);
}

private:
	ros::NodeHandle &handle;
	image_transport::ImageTransport imageHandler;
	image_transport::Subscriber subscriber;
	image_transport::Publisher publisher;
};




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ime_ros");
	ros::NodeHandle roshdl;

	ImageContrastEnhancer ice(roshdl);

	ros::spin();

	return 0;
}
